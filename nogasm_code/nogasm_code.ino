#pragma GCC push_options
//#pragma GCC optimize ("Os")
//#pragma GCC optimize ("O2")
//#pragma GCC optimize ("Od")

#define DEBUG_LOG_ENABLED 1
#define PROFILING_ENABLED 0

//defines NOGASM_WIFI_SSID & NOGASM_WIFI_PASS:
#include "nogasm_wifi.h"
#include "bufferedwificlient.h"

//#define CMD_DEBUG 1

// Protogasm Code, forked from Nogasm Code Rev. 3
/* Drives a vibrator and uses changes in pressure of an inflatable buttplug
 * to estimate a user's closeness to orgasm, and turn off the vibrator
 * before that point.
 * A state machine updating at 60Hz creates different modes and option menus
 * that can be identified by the color of the LEDs, especially the RGB LED
 * in the central button/encoder knob.
 * 
 * [Red]    Vibrator Control Test
 * [Blue]   Automatic vibrator edging, knob adjusts orgasm detection sensitivity
 * [Green]  Setting menu for maximum vibrator speed in automatic mode
 * [White]  Debubbing menu to show data from the pressure sensor ADC
 * [Off]    While still plugged in, holding the button down for >3 seconds turns
 *          the whole device off, until the button is pressed again.
 * 
 * Settings like edging sensitivity, or maximum motor speed are stored in EEPROM,
 * so they are saved through power-cycling.
 * 
 * In the automatic edging mode, the vibrator speed will linearly ramp up to full
 * speed (set in the green menu) over 30 seconds. If a near-orgasm is detected,
 * the vibrator abruptly turns off for 15 seconds, then begins ramping up again.
 * 
 * The motor will beep during power on/off, and if the plug pressure rises above
 * the maximum the board can read - this condition could lead to a missed orgasm 
 * if unchecked. The analog gain for the sensor is adjustable via a trimpot to
 * accomidate different types of plugs that have higher/lower resting pressures.
 * 
 * Motor speed, current pressure, and average pressure are reported via USB serial
 * at 115200 baud. Timestamps can also be enabled, from the main loop.
 * 
 * There is some framework for more features like an adjustable "cool off" time 
 * other than the default 15 seconds, and options for LED brightness and enabling/
 * disabling beeps.
 * 
 * Note - Do not set all the LEDs to white at full brightness at once
 * (RGB 255,255,255) It may overheat the voltage regulator and cause the board 
 * to reset.
 */
//=======Libraries===============================
#include <Arduino.h>
//#define ENCODER_DO_NOT_USE_INTERRUPTS //for encoder.h
#include <Encoder.h>
#include <EEPROM.h>
#include "FastLED.h"
#include <MovingAveragePlus.h> //using custom version from https://github.com/trilithus/MovingAveragePlus
#include <AceRoutine.h>
#include "commandmanager.h"
#include "sharedcommands.h"
#include <avr/pgmspace.h>
#include <WiFiS3.h>
#include <CircularBuffer.h>
#include <MemoryFree.h>
using namespace ace_routine;

//Running pressure average array length and update frequency
#define RA_HIST_SECONDS 15 //25
#define RA_FREQUENCY 10
#define RA_TICK_PERIOD (FREQUENCY / RA_FREQUENCY)

MovingAveragePlus<uint16_t, uint32_t, RA_FREQUENCY*RA_HIST_SECONDS> g_raOverallPressure;
MovingAveragePlus<uint16_t, uint32_t, 8> g_raImmediatePressure;

//LCD
#define U8X8_HAVE_HW_I2C
#include <Arduino.h>
#include <U8g2lib.h>
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif



#if 1
#define font_13pt u8g2_font_6x13_tr
#define font_24pt u8g2_font_logisoso24_tn//u8g2_font_inb24_mr 
#else
#define font_13pt u8g2_font_6x13_tr
#define font_24pt u8g2_font_6x13_tr 
#define font_15pt u8g2_font_6x13_tr 
#define font_symbols u8g2_font_unifont_t_weather
#endif

//=======Hardware Setup===============================
//LEDs
#define NUM_LEDS 24
#define LED_PIN 10
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB
#define BRIGHTNESS 75 //Subject to change, limits current that the LEDs draw

//Encoder
#define ENC_SW   5 //Pushbutton on the encoder
Encoder* myEnc = nullptr; //Quadrature inputs
#define ENC_SW_UP   HIGH
#define ENC_SW_DOWN LOW

//Motor
constexpr uint8_t PIN_VIBRATOR_ONOFF = 12;
constexpr uint8_t PIN_VIBRATOR_DOWN = 8;
constexpr uint8_t PIN_VIBRATOR_UP = 13;
constexpr uint8_t VIBRATOR_PUSH_DELAY = 150;

//Pressure Sensor Analog In
#define BUTTPIN A0
// Sampling 4x and not dividing keeps the samples from the Arduino Uno's 10 bit
// ADC in a similar range to the Teensy LC's 12-bit ADC.  This helps ensure the
// feedback algorithm will behave similar to the original.
#define OVERSAMPLE 4
#define ADC_MAX 1023

//=======Software/Timing options=====================
#define FREQUENCY 60 //Update frequency in Hz
#define LONG_PRESS_MS 600 //ms requirements for a long press, to move to option menus
#define V_LONG_PRESS_MS 2500 //ms for a very long press, which turns the device off
#define EDGEALGORITHM 1

//Update/render period
#define period (1000/FREQUENCY)
#define longBtnCount (LONG_PRESS_MS / period)

int sensitivity = 0; //orgasm detection sensitivity, persists through different states

//=======State Machine Modes=========================
enum class MachineStates
{
  UNDEFINED,
  menu,
  manual_motor_control,
  manual_edge_control,
  auto_edge_control,
  read_variable,
};


//Button states - no press, short press, long press
enum class PhysBtnState
{
  None,
  Short,
  Long,
  VLong,
};

//=======Global Settings=============================
#define MOT_MAX 255 // Motor PWM maximum
//#define MOT_MIN 20  // Motor PWM minimum.  It needs a little more than this to start.

CRGB leds[NUM_LEDS];

int16_t g_currentPressure = 0;
int16_t g_avgPressure = 0; //Running 25 second average pressure
//int bri =100; //Brightness setting
int16_t rampTimeS = 150; //120; //Ramp-up time, in seconds
float s_cooldownTime = 0.40f;//0.5f;
#define DEFAULT_PLIMIT 900
int16_t pLimit = DEFAULT_PLIMIT; //Limit in change of pressure before the vibrator turns off
int16_t pLimitLast = DEFAULT_PLIMIT;
int16_t g_MaxSpeed = 255; //maximum speed the motor will ramp up to in automatic mode
float g_motSpeed = 0; //Motor speed, 0-255 (float to maintain smooth ramping to low speeds)
uint16_t edgeCount = 0;
auto g_lastEdgeCountChange = 0;
static uint32_t g_tick = 0;

//Menu Items:
enum class MenuItemsEnum : uint8_t
{
  INVALID = 0,
  test,
  manual,
  automatic,
  automatic_start,
  automatic_continue,
  automatic_input_time,
  automatic_input_min,
  automatic_input_max,
  automatic_back,
  toggle_online,
  resetSession,
  select_host,
  back,
  LAST,
};

MenuItemsEnum g_rootMenu[] {
  MenuItemsEnum::INVALID,
  MenuItemsEnum::automatic,
  MenuItemsEnum::manual,
  MenuItemsEnum::test,
  MenuItemsEnum::toggle_online,
  MenuItemsEnum::select_host,
  MenuItemsEnum::resetSession,
  MenuItemsEnum::LAST,
};

MenuItemsEnum g_automaticMenu[] {
  MenuItemsEnum::INVALID,
  MenuItemsEnum::automatic_start,
  MenuItemsEnum::automatic_continue,
  MenuItemsEnum::toggle_online,
  MenuItemsEnum::automatic_input_time,
  MenuItemsEnum::automatic_input_min,
  MenuItemsEnum::automatic_input_max,
  MenuItemsEnum::automatic_back,
  MenuItemsEnum::LAST,
};

//Utility
template <typename T>
T clamp(const T val, const T minVal, const T maxVal) { return (val<minVal) ? minVal : (val>maxVal ? maxVal : val); };

void spareDebugPrint(const char* pStr)
{
  static auto lastPrint = millis();
  if (millis()>lastPrint+1000)
  {
    Serial.print(pStr);
  }
}

void spareDebugPrintln(const char* pStr)
{
  static auto lastPrint = millis();
  if (millis()>lastPrint+1000)
  {
    Serial.println(pStr);
  }
}

//======= DEBUG ===============================
#if defined(DEBUG_LOG_ENABLED) && DEBUG_LOG_ENABLED!=0
#define DEBUG_ONLY(...) __VA_ARGS__
#else
#define DEBUG_ONLY(...) void(0)
#endif


//=======EEPROM Addresses============================
//128b available on teensy LC
#define EEPROM_ONLINEMODE_ADDR   0*4
#define EEPROM_MAX_SPEED_ADDR    1*4
#define EEPROM_SENSITIVITY_ADDR  2*4
#define AUTOEDGE_MIN_ADDR        3*4
#define AUTOEDGE_MAX_ADDR        4*4
#define AUTOEDGE_TIME_ADDR       5*4
#define EEPROM_SELECTEDHOST_ADDR 6*4

//#define RAMPSPEED_ADDR    4 //For now, ramp speed adjustments aren't implemented
#if 1
//=======Hardware Coroutines=========================
class Logic : Coroutine
{
  protected:
    void run_menu();
    void run_read_variable();
    void run_automatic_edge_control();
    void run_edge_control();
    void run_manual_motor_control();
    void update_current_status();
    void log_session();
    PhysBtnState check_button();
    void run_state_machine(MachineStates state);
    MachineStates change_state(MachineStates newState);
    void resetSession();
    void clearSession();
  public:
    void Setup();
    MachineStates getState() const { return _state; }
    MenuItemsEnum getMenuItem() const { return *_currentMenuItem; }
    bool isMenuItemLast() const { return _currentMenuItemState.isLast; }
    bool isMenuItemFirst() const { return _currentMenuItemState.isFirst; }
    int16_t getAutomaticInputMax() const { return _automatic_edge_maxtarget; }
    int16_t getAutomaticInputMin() const { return _automatic_edge_mintarget; }
    int16_t getAutomaticInputTimeMinutes() const { return _automatic_edge_time_minutes; }
    virtual int runCoroutine() override;
  private:
    static int sampleTick;

    MenuItemsEnum* _currentMenuItem = nullptr;
    struct 
    {
      bool isFirst;
      bool isLast;
    } _currentMenuItemState;

    MachineStates _state = MachineStates::UNDEFINED;
    PhysBtnState _physBtnState = PhysBtnState::None;
    int32_t _menuKnobState = 0;

    int16_t* _readVariable = nullptr;
    int16_t _readVariableMin = 0;
    int16_t _readVariableMax = 1000;

    int16_t _automatic_edge_maxtarget = 0;
    int16_t _automatic_edge_mintarget = 0;
    int16_t _automatic_edge_time_minutes = 0;
    int16_t _autoEdgeTargetOffset = 0;

    unsigned long _autoTimeStart = 0;

    bool _hasSession = false;
    #if (defined(EDGEALGORITHM) && EDGEALGORITHM==1)
    float _cooldown = 0.0f;
    unsigned long _lastEdgeTime=0;
    unsigned long _lastMotorEnableTimeMs=0;
    public:
      float GetCooldown() const { return _cooldown; }
    #endif
} g_logic;

class Sensor : Coroutine
{
public:
    void Setup();
    virtual int runCoroutine() override;
} g_sensor;

class Vibrator : Coroutine
{
  struct
  {
    bool powered = false;
    uint8_t mode = 0;
  } _currentState, _targetState;
  static constexpr uint8_t MAX_LEVEL = 6;
  public:
    void Setup();
    void Update();
    void vibrator_up();
    void vibrator_down();
    void vibrator_off();
    void vibrator_on();
    void vibrator_reset();
    void vibrator_to(const int mode);
    void set_vibrator_mode(int fromStrength, int toStrength);
    bool isVibratorOn() { return _currentState.powered; }
    int map_motspeed_to_vibrator();

    virtual int runCoroutine() override;
  protected:
    int8_t getStateDiff() const 
    { 
        const int8_t fromStrength = clamp<int8_t>(static_cast<int8_t>(_currentState.mode), 0, MAX_LEVEL);
        const int8_t toStrength = clamp<int8_t>(static_cast<int8_t>(_targetState.mode), 0, MAX_LEVEL);
        return toStrength - fromStrength; 
      }
} g_vibrator;

class LEDs : Coroutine
{
  public:
    void Setup();
    void Update();

    virtual int runCoroutine() override 
    {
      COROUTINE_LOOP()
      {
        Update();
        COROUTINE_YIELD();
      }
    }
} g_leds;

class LCD : public Coroutine
{
  private:
    static U8G2_SSD1306_128X32_UNIVISION_1_HW_I2C u8g2;

    typedef void (*lcdrender_t)();
    static lcdrender_t lcdrender_activefunc;
  protected:
    void run_lcd_status(MachineStates state);
  public:
    void Setup();
    void Update();
    void nextMode() { 
      _mode++; 
      _lastChange=millis(); 
      g_lastEdgeCountChange=_lastTargetChange=0;
    }

    virtual int runCoroutine() override;

    template <typename T, typename I, int idx = 0, typename... Iremainder>
    static void setLCDFunc(T func, I arg, Iremainder... args);

    static void lcdfunc_renderMenu();
    static void lcdfunc_edgeCount();
    static void lcdfunc_renderText();
    static void lcdfunc_renderTarget();
    static void lcdfunc_renderPressure();

    static void RenderPanic(const String _str);
  public:
    static unsigned long _lastTargetChange;
  private:
    union lcdrenderdata_t
    {
      int32_t integer32;
      const char *stringptr;

      int32_t operator=(const int32_t arg)
      {
        integer32 = arg;
        return arg;
      }

      const char *operator=(const char *arg)
      {
        stringptr = arg;
        return arg;
      }
    } static lcdrenderdata[2];
    int8_t _mode = 0;
    unsigned long _lastChange = millis();
} g_lcd;
LCD::lcdrenderdata_t LCD::lcdrenderdata[2]{};
unsigned long LCD::_lastTargetChange = 0;
LCD::lcdrender_t LCD::lcdrender_activefunc = nullptr;
U8G2_SSD1306_128X32_UNIVISION_1_HW_I2C LCD::u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

struct LogData
{
  int32_t param1{};
  int32_t param2{};
  int32_t param3{};
  int32_t param4{};
  int32_t param5{};
  int32_t param6{};
};

class WIFI : public Coroutine
{
  private:
    struct internalLog
    {
      uint64_t time=0;
      LogData data{};
    };
    std::string _session;
    CircularBuffer<internalLog, 60> _buffer;
    uint64_t _sessionStartTime=0;
    uint64_t _wantsToBeOnlineStartTime = 0;
    int16_t _selectedHost = 0;
    uint8_t _onlineMode = 1;
    bool _connected = false;
    bool _resetSession = false;
    bool _wantsToBeOnline = false;
  private:
    bool connect();
    bool internalRequestSession();
    void internalRequestSessionResponse();
    void internalSendLog(const internalLog& logData);
  public:
    void Setup();
    void Update();

    virtual int runCoroutine() override;

    void LogBeginSession() { _session.clear(); _resetSession = true; }
    void LogEndSession();
    void LogSessionTick(unsigned long long, const LogData& logData);

    void ClearSession() { _session.clear(); _resetSession = false; }
    void FlagDisconnected() { _connected=false; };
    bool IsConnected() const { return _connected; }

    bool GetOnlineMode() const { return (_onlineMode!=0); }
    void SetOnlineMode(bool pMode, bool pPersist);
    bool ShouldConnect() const {  return (_onlineMode!=0) && _wantsToBeOnline; }

    bool GetWantsToBeOnline() const { return _wantsToBeOnline; }
    void SetWantsToBeOnline() { 
      Serial.println("want to be online"); 
      if (!_wantsToBeOnline)
      {
        _wantsToBeOnlineStartTime=millis(); 
        _wantsToBeOnline = true; }
      }
    void SetWantsToBeOffline() { Serial.println("stay offline"); _wantsToBeOnline = false; }
    uint64_t GetWantsToBeOnlineStartTime() const { return _wantsToBeOnlineStartTime; }

    int16_t GetSelectedHost() const { return _selectedHost; }
    int16_t* GetSelectedHostPtr() { return &_selectedHost; }
  } g_wifi;

class WIFIWriter : public Coroutine
{
private:
  BufferedWiFiClient<64> _client;
public:
  void Setup() {};
  void Update() {};

  virtual int runCoroutine() override;
  BufferedWiFiClient<64>& GetClient() { return _client; }
} g_WifiWriter;

/////////////////////////////////////////////////////
// LCD

void getMenuItemLabels(MenuItemsEnum pValue, String& output)
{
  switch(pValue)
  {
    case MenuItemsEnum::test: output = F("Mode: Test"); return;
    case MenuItemsEnum::manual: output = F("Mode: Manual Edging"); return;
    case MenuItemsEnum::automatic: output = F("Mode: Automatic"); return;
    case MenuItemsEnum::automatic_start: output = F("Start"); return;
    case MenuItemsEnum::automatic_continue: output = F("Continue"); return;
    case MenuItemsEnum::automatic_input_time: output = F("Duration: "); return;
    case MenuItemsEnum::automatic_input_min: output = F("Start treshold: "); return;
    case MenuItemsEnum::automatic_input_max: output = F("End treshold: "); return;
    case MenuItemsEnum::automatic_back: output = F("[ Back ]"); return;
    case MenuItemsEnum::select_host: output = "Host: "; output += NOGASM_LOGGER_HOST[g_wifi.GetSelectedHost()]; return;
    case MenuItemsEnum::toggle_online: output = (g_wifi.GetOnlineMode() ? F("Go offline") : F("Go online")); return;
    case MenuItemsEnum::resetSession: output = F("Reset Session"); return;
    case MenuItemsEnum::LAST: output = F("!ERROR!"); return;
  }
}

int LCD::runCoroutine()
{
  COROUTINE_LOOP()
  {
    Update();
    COROUTINE_YIELD();
  }
}

void LCD::Setup()
{
  u8g2.begin();
}

void LCD::Update()
{
  //u8g2.setI2CAddress(0x3C*2);
  if (lcdrender_activefunc != nullptr)
  {
    lcdrender_activefunc();
    if (!u8g2.nextPage())
    {
      memset(&lcdrenderdata[0], 0, sizeof(lcdrenderdata));
      lcdrender_activefunc = nullptr;
    }
  }
  else
  {
    run_lcd_status(g_logic.getState());
  }
}

void LCD::RenderPanic(const String _str)
{
  Serial.println(_str);
  Serial.flush();
  LCD::setLCDFunc(LCD::lcdfunc_renderText, _str.c_str());
  do
  {
    lcdrender_activefunc();
  } while (u8g2.nextPage());
  pinMode(LED_BUILTIN, OUTPUT);
  while (true)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
  }
}

void LCD::run_lcd_status(MachineStates state)
{
  if (state == MachineStates::menu)
  {
    setLCDFunc(lcdfunc_renderMenu, 0);
  }
  else
  {
    const auto now = millis();

    if ( (now - _lastChange) > 5000 )
    {
      _mode++;
      _lastChange = now;
    }

    if ( (now-_lastTargetChange) < 15000 )
    {
      _mode = 1;
    }

    if ( (now - g_lastEdgeCountChange) < 5000 )
    {
      _mode = 2;
    }

    switch (_mode % 3)
    {
      case 0:
        {
          setLCDFunc(lcdfunc_renderPressure, g_currentPressure, g_avgPressure);
        } break;
      case 1:
        {
          setLCDFunc(lcdfunc_renderTarget, max(0, g_currentPressure - g_avgPressure), max(0, pLimit));
        } break;
      case 2:
      {
        setLCDFunc(lcdfunc_edgeCount, edgeCount);
      } break;
    }
  }
}

template<typename T, typename I, int idx, typename ...Iremainder>
void LCD::setLCDFunc(T func, I arg, Iremainder... args)
{
  if (lcdrender_activefunc == nullptr)
  {
    if (idx < sizeof(lcdrenderdata)/sizeof(lcdrenderdata[0]))
    {
      lcdrenderdata[idx] = arg;
    }
    if constexpr(idx < sizeof...(args))
    {
      setLCDFunc<T,I,idx+1>(func, args...);
    }
    if constexpr(idx == 0)
    {
      lcdrender_activefunc = func;
      u8g2.firstPage();
    }
  }
}

void LCD::lcdfunc_renderMenu()
{
#if 0
  u8g2.setFont(font_13pt);
  u8g2.setCursor(0, 20);

  static String text;
  text = g_MenuItemLabels[static_cast<uint8_t>(g_logic.getMenuItem())];
  u8g2.print(text);
#else
  u8g2.setFont(font_13pt);
  
  uint8_t left = 2;
  uint8_t top = 5;
  uint8_t bottom = u8g2.getDisplayHeight() - 5;
  uint8_t right = u8g2.getDisplayWidth() - 2;
  u8g2.drawFrame(left, top, right-left, bottom-top);

  static String text;

  static char buffer[64];
  getMenuItemLabels(g_logic.getMenuItem(), text);
     
  switch(g_logic.getMenuItem())
  {
    case MenuItemsEnum::automatic_input_time:
    text += g_logic.getAutomaticInputTimeMinutes();
    text += F("m");
    break;
    case MenuItemsEnum::automatic_input_min:
    text += g_logic.getAutomaticInputMin();
    break; 
    case MenuItemsEnum::automatic_input_max:
    text += g_logic.getAutomaticInputMax();
    break; 
  }

  const auto textWidth = u8g2.getStrWidth(text.c_str());
  u8g2.setCursor((u8g2.getDisplayWidth()/2) - (textWidth/2), 20);
  u8g2.print(text);
  
  if (!g_logic.isMenuItemFirst())
  {
    u8g2.drawLine(left, 0, left, top-2);
    u8g2.drawLine(left, top-2, right, top-2);
    u8g2.drawLine(right, top-2, right, 0);
  }

  if (!g_logic.isMenuItemLast())
  {
    u8g2.drawLine(left, bottom+2, left, u8g2.getDisplayHeight());
    u8g2.drawLine(left, bottom+2, right, bottom+2);
    u8g2.drawLine(right, bottom+2, right, u8g2.getDisplayHeight());
  }
#endif
}

//static auto lastPressureChange = millis();
void LCD::lcdfunc_renderPressure()
{
  u8g2.setFont(font_13pt);
  u8g2.setCursor(0, 20);
  u8g2.print(F("P"));
  u8g2.setFont(font_24pt);
  u8g2.setCursor(10, u8g2.getMaxCharHeight());
  u8g2.print(lcdrenderdata[0].integer32);
  u8g2.setCursor(50, u8g2.getMaxCharHeight());
  u8g2.setFont(font_13pt);
  u8g2.print(F("vs"));
  u8g2.setFont(font_24pt);
  u8g2.setCursor(60, u8g2.getMaxCharHeight());
  u8g2.print(lcdrenderdata[1].integer32);
}

void LCD::lcdfunc_renderTarget()
{
  u8g2.setFont(font_13pt);
  u8g2.setCursor(0, 20);
  u8g2.print(F("T "));
  u8g2.setCursor(10, u8g2.getMaxCharHeight());
  u8g2.print(lcdrenderdata[0].integer32);
  u8g2.print(F(" / "));
  u8g2.print(lcdrenderdata[1].integer32);

  uint8_t left = 10;
  uint8_t top = u8g2.getMaxCharHeight() + 2;
  uint8_t bottom = u8g2.getDisplayHeight();
  uint8_t right = u8g2.getDisplayWidth();
  
  u8g2.setDrawColor(1);
  u8g2.drawBox(left, top, right-left, bottom-top);

  left += 1;
  top += 1;
  bottom -= 1;
  right -= 1;
  u8g2.setDrawColor(0);
  u8g2.drawBox(left, top, (right-left), (bottom-top));

  left += 1;
  top += 1;
  bottom -= 1;
  right -= 1;
  if (lcdrenderdata[1].integer32 > 0)
  {
    double p = double(lcdrenderdata[0].integer32 * 100) / double(lcdrenderdata[1].integer32 * 100);
    right = max(left, int ((double(right)-double(left)) * p));
    u8g2.setDrawColor(1);
    u8g2.drawBox(left, top, (right-left), (bottom-top));
  }

  //restore
  u8g2.setDrawColor(1);
}

void LCD::lcdfunc_renderText()
{
  u8g2.setFont(lcdrenderdata[1].integer32 == 0 ? font_13pt : font_24pt);
  int y = u8g2.getMaxCharHeight();

  if (lcdrenderdata[0].stringptr != nullptr)
  {
    u8g2.setCursor(10, y);
    u8g2.print(lcdrenderdata[0].stringptr);
    y += u8g2.getMaxCharHeight();
  }
}

namespace bitmaps
{
  static const uint8_t heart_bits[] PROGMEM = {
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x0c, 0xf8, 0x3e, 0xf8, 0x3f,
      0xf8, 0x3f, 0xf8, 0x3f, 0xf8, 0x3f, 0xf0, 0x1f, 0xc0, 0x07, 0x80, 0x03,
      0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

  static const uint8_t heart_bits_crossed[] PROGMEM = {
      0x01, 0x00, 0x02, 0x80, 0x04, 0x40, 0x68, 0x2c, 0x98, 0x32, 0x28, 0x29,
      0x48, 0x24, 0x88, 0x22, 0x18, 0x21, 0xb0, 0x12, 0x40, 0x04, 0xa0, 0x0a,
      0x10, 0x11, 0x08, 0x20, 0x04, 0x40, 0x02, 0x80};

  static const uint8_t connected_bits[] PROGMEM = {
      0x00, 0x00, 0xf0, 0x07, 0xf0, 0x07, 0xf0, 0x03, 0x30, 0x00, 0xb0, 0x07,
      0xbf, 0xff, 0xff, 0xff, 0xbf, 0xff, 0xb0, 0x07, 0x30, 0x00, 0xf0, 0x03,
      0xf0, 0x03, 0xf0, 0x03, 0x00, 0x00, 0x00, 0x00};

  static const uint8_t wifi_on_bits[] PROGMEM = {
      0x00, 0x00, 0x00, 0x00, 0xF8, 0x1F, 0xFE, 0x7F, 0x0F, 0xF0, 0xE3, 0xC7, 
      0xF9, 0x9F, 0x1C, 0x38, 0xCC, 0x33, 0xE0, 0x07, 0x70, 0x0E, 0x30, 0x0C, 
      0x80, 0x01, 0x80, 0x01, 0x80, 0x01, 0x00, 0x00
  };

  static const uint8_t wifi_off_bits[] PROGMEM = {
      0x00, 0xC0, 0x00, 0xE0, 0x00, 0x70, 0xFC, 0x3F, 0x0E, 0x7C, 0x03, 0xCE, 
      0xF1, 0x8F, 0x98, 0x1B, 0xCC, 0x31, 0xE0, 0x03, 0x70, 0x06, 0x38, 0x0C, 
      0x1C, 0x00, 0x8E, 0x01, 0x87, 0x01, 0x03, 0x00
  };
}

  void LCD::lcdfunc_edgeCount()
  {
    //u8g2.setFont(font_24pt);
    u8g2.setCursor(0, u8g2.getMaxCharHeight() - 4);

    u8g2.setFont(font_13pt);
    u8g2.print(F("EDGES "));

    u8g2.setFont(font_24pt);
    u8g2.print(lcdrenderdata[0].integer32);

  #define heart_width  16
  #define heart_height 16
  #if 1

#if !defined(max)
  static constexpr uint8_t buffersz = std::max(std::max(sizeof(bitmaps::connected_bits), sizeof(bitmaps::heart_bits)),
                                                                       sizeof(bitmaps::heart_bits_crossed));
#else
  static constexpr uint8_t buffersz = max(max(sizeof(bitmaps::connected_bits), sizeof(bitmaps::heart_bits)),
                                                                       sizeof(bitmaps::heart_bits_crossed));
#endif

  uint8_t realBuffer[buffersz];
  uint8_t* buffer = &realBuffer[0]; //(uint8_t*)malloc(buffersz);

  //vibe active or not:
  for(uint8_t i=0; i < sizeof(bitmaps::heart_bits); i++)
  {
    buffer[i] = pgm_read_byte_near((g_motSpeed >= 0.01f ? bitmaps::heart_bits : bitmaps::heart_bits_crossed) +i);
  }
  u8g2.drawXBM(u8g2.getDisplayWidth() - (heart_width * 2) - heart_width/3, 0, heart_width, heart_height, buffer);


  for(uint8_t i=0; i < sizeof(bitmaps::wifi_on_bits); i++)
  {
    buffer[i] = pgm_read_byte_near((g_wifi.IsConnected() ? bitmaps::wifi_on_bits : bitmaps::wifi_off_bits) +i);
  }
  u8g2.drawXBM(u8g2.getDisplayWidth() - heart_width, 0, heart_width, heart_height, buffer);


  //free(buffer);
  #else
  #define heart_width  16
  #define heart_height 16
  static unsigned char heart_bits[] = {
  0x00,0x00,0x00,0x00,0x00,0x00,0x60,0x0c,0xf8,0x3e,0xf8,0x3f,
  0xf8,0x3f,0xf8,0x3f,0xf8,0x3f,0xf0,0x1f,0xc0,0x07,0x80,0x03,
  0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00};
  u8g2.drawXBM(u8g2.getDisplayWidth() - heart_width, 0, heart_width, heart_height, &heart_bits[0]);
  #endif
}

/////////////////////////////////////////////////////
// Vibrator
int Vibrator::runCoroutine()
{
  static int8_t i = 0;
  static int8_t diff = 0;
  COROUTINE_LOOP()
  {
    diff = getStateDiff();

    if (_currentState.powered == false && _targetState.powered == true)
    {
      //turn on:
      _currentState.powered = true;
      digitalWrite(PIN_VIBRATOR_ONOFF, HIGH);
      COROUTINE_DELAY(VIBRATOR_PUSH_DELAY);
      digitalWrite(PIN_VIBRATOR_ONOFF, LOW);
      COROUTINE_DELAY(VIBRATOR_PUSH_DELAY);
      DEBUG_ONLY(Serial.println(F("[VIBRATOR]\tTurned on")));
    }
    else if (_currentState.powered == true && _targetState.powered == false)
    {
      //turn off:
      _currentState.powered = false;
      digitalWrite(PIN_VIBRATOR_ONOFF, HIGH);
      COROUTINE_DELAY(VIBRATOR_PUSH_DELAY);
      digitalWrite(PIN_VIBRATOR_ONOFF, LOW);
      COROUTINE_DELAY(VIBRATOR_PUSH_DELAY);
      DEBUG_ONLY(Serial.println(F("[VIBRATOR]\tTurned off")));
    }

    if (_currentState.powered)
    {
      if (diff != 0)
      {
        DEBUG_ONLY(Serial.print(F("[VIBRATOR] ")));
        DEBUG_ONLY(Serial.print(_currentState.mode));
        DEBUG_ONLY(Serial.print(F(" -> ")));
        DEBUG_ONLY(Serial.println(_targetState.mode));
      }
      if (diff > 0)
      {
        for (i = 0; i < diff && _currentState.powered; i++)
        {
          //turn up:
          _currentState.mode++;
          digitalWrite(PIN_VIBRATOR_UP, HIGH);
          COROUTINE_DELAY(VIBRATOR_PUSH_DELAY);
          digitalWrite(PIN_VIBRATOR_UP, LOW);
          COROUTINE_DELAY(VIBRATOR_PUSH_DELAY);
          DEBUG_ONLY(Serial.println(F("[VIBRATOR]\tTurned UP")));
        }
      }
      else if (diff < 0)
      {
        for (i = 0; i < -diff && _currentState.powered; i++)
        {
          //turn down:
          _currentState.mode--;
          digitalWrite(PIN_VIBRATOR_DOWN, HIGH);
          COROUTINE_DELAY(VIBRATOR_PUSH_DELAY);
          digitalWrite(PIN_VIBRATOR_DOWN, LOW);
          COROUTINE_DELAY(VIBRATOR_PUSH_DELAY);
          DEBUG_ONLY(Serial.println(F("[VIBRATOR]\tTurned DOWN")));
        }
      }
    }

    COROUTINE_YIELD();
  }
}

void Vibrator::Setup()
{
  pinMode(PIN_VIBRATOR_ONOFF, OUTPUT);
  pinMode(PIN_VIBRATOR_UP, OUTPUT);
  pinMode(PIN_VIBRATOR_DOWN, OUTPUT);

  //Make sure the motor is off
  vibrator_off();
}

void Vibrator::Update()
{

}

void Vibrator::vibrator_up() { if (_targetState.mode<MAX_LEVEL) {  _targetState.mode++; } }
void Vibrator::vibrator_down() { if (_targetState.mode>0) { _targetState.mode--; } }
void Vibrator::vibrator_off() { _targetState.powered = false; }
void Vibrator::vibrator_reset() { _targetState.mode=0; _currentState.mode=MAX_LEVEL; }
void Vibrator::vibrator_on() { if (!_targetState.powered) { _targetState.powered = true; } }
void Vibrator::vibrator_to(const int mode) 
{ 
  if(mode > 0)
  {
    _targetState.mode = clamp<int>(mode-1, 0, MAX_LEVEL); 
  }
}

int Vibrator::map_motspeed_to_vibrator()
{
  static constexpr uint8_t pct[] = {uint8_t(0.35f * 255.0f),
                                    uint8_t(0.21f * 255.0f), 
                                    uint8_t(0.17f * 255.0f), 
                                    uint8_t(0.10f * 255.0f), 
                                    uint8_t(0.10f * 255.0f), 
                                    uint8_t(0.07f * 255.0f)};
  static constexpr uint8_t speedCount = (sizeof(pct)/sizeof(pct[0]));
  const uint8_t speed = (uint8_t)clamp<float>(g_motSpeed, 0.0f, 255.0f);
  if (speed <= 1)
  {
    return 0;
  } else if(speed >= g_MaxSpeed)
  {
    //Go back, as to not overstimulate.
    g_motSpeed = 2.0f;
  }

  //Map motSpeed to index:
  uint8_t lastValue = 1;
  
  //1,2,3,4 speed settinsg:
  for(auto i=0; i < speedCount; i++)
  {
    if (speed > lastValue &&  speed <= lastValue + pct[i])
    {
      return i+1;
    }
    lastValue += pct[i];
  }
  return -1;
};
/////////////////////////////////////////////////////
// LEDs

void LEDs::Setup()
{
   FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  // limit power draw to .6A at 5v... Didn't seem to work in my FastLED version though
  //FastLED.setMaxPowerInVoltsAndMilliamps(5,DEFAULT_PLIMIT);
  FastLED.setBrightness(BRIGHTNESS);
}

void LEDs::Update()
{
}

/////////////////////////////////////////////////////
// Sensor
void Sensor::Setup()
    {
      analogReference(AR_EXTERNAL);//(EXTERNAL);
      pinMode(BUTTPIN, INPUT); //default is 10 bit resolution (1024), 0-3.3
      g_raOverallPressure.clear(); 
      g_raImmediatePressure.clear();
    }

    int Sensor::runCoroutine() 
    {
      static auto s_lastTime = micros();
      static int32_t s_tickaccumulator = 0;
      static uint32_t s_lastTick = 0;

      COROUTINE_LOOP()
      {
        static auto time = micros();
        static auto timeMillis = millis();
        static auto elapsed = 0ul;
        time = micros();
        timeMillis = micros();
        elapsed = [&]()
        {
          if (time >= s_lastTime)
          {
            return time - s_lastTime;
          }
          else
          {
            return (ULONG_MAX - s_lastTime) + time;
          }
        }();
        s_lastTime = time;

        s_tickaccumulator += elapsed;
        if (s_tickaccumulator >= 1000000 / RA_FREQUENCY)
        {
          s_tickaccumulator -= 1000000 / RA_FREQUENCY;
          g_tick++;
        }

        auto pressure = analogRead(BUTTPIN)*8;
        g_raImmediatePressure.push(uint16_t(pressure));
        g_currentPressure = g_raImmediatePressure.get();

        if (s_lastTick != g_tick)
        {
          s_lastTick = g_tick;

          //Only update our overall presure if it's not being influenced by the vibe.
          //Or, if it lowers the average pressure...
          if (!g_vibrator.isVibratorOn() || g_currentPressure < g_raOverallPressure.get())
          {
            g_raOverallPressure.push(uint16_t(g_currentPressure));
          }
          g_avgPressure = g_raOverallPressure.get();
          COROUTINE_YIELD();
        } else {
          COROUTINE_DELAY(1);
        }
      }
    }
#if 0
    int Sensor::runCoroutine() 
    {
      static auto s_lastTime = micros();
      static uint32_t g_tick = 0;
      static int32_t s_tickaccumulator = 0;
      static uint32_t s_lastTick = 0;

      COROUTINE_LOOP()
      {
        static auto time = micros();
        static auto timeMillis = millis();
        static auto elapsed = 0ul;
        time = micros();
        timeMillis = micros();
        elapsed = [&]()
        {
          if (time >= s_lastTime)
          {
            return time - s_lastTime;
          }
          else
          {
            return (ULONG_MAX - s_lastTime) + time;
          }
        }();
        s_lastTime = time;

        s_tickaccumulator += elapsed;
        if (s_tickaccumulator >= 1000000 / 60)
        {
          s_tickaccumulator -= 1000000 / 60;
          g_tick++;
        }

        if (s_lastTick != g_tick)
        {
          s_lastTick = g_tick;
          if (g_tick % RA_TICK_PERIOD == 0)
          {
            raPressure.addValue(g_pressure);
            avgPressure = raPressure.getAverage();
          }

          static int l_pressure = 0;
          l_pressure = 0;
          static uint8_t i = 0;
          for (i = OVERSAMPLE; i; --i)
          {
            l_pressure += analogRead(BUTTPIN);
            if (i)
            {           // Don't delay after the last sample
              COROUTINE_DELAY(1); // Spread samples out a little
            }
          }
          g_pressure = l_pressure;
        }
        COROUTINE_YIELD();
      }
    }
#endif
/////////////////////////////////////////////////////
// Logic
int Logic::sampleTick = 0;
int Logic::runCoroutine()
    {
      COROUTINE_LOOP()
      {              
        fadeToBlackBy(leds,NUM_LEDS,20); //Create a fading light effect. LED buffer is not otherwise cleared
        _physBtnState = check_button();
        if (_physBtnState == PhysBtnState::Long && _state != MachineStates::menu)
        {
          change_state(MachineStates::menu);
        }

        run_state_machine(_state);
        FastLED.show(); //Update the physical LEDs to match the buffer in software

        log_session();

        COROUTINE_YIELD();
      }
    }

void Logic::Setup()
{
  pinMode(ENC_SW,   INPUT_PULLUP); //Pin to read when encoder is pressed
  digitalWrite(ENC_SW, HIGH); // Encoder switch pullup

  //Storage:
  sensitivity = EEPROM.read(EEPROM_SENSITIVITY_ADDR);
  g_MaxSpeed = 254;//min(EEPROM.read(EEPROM_MAX_SPEED_ADDR), MOT_MAX); //Obey the MOT_MAX the first power  cycle after chaning it.

  _automatic_edge_time_minutes = clamp(EEPROM.get<int16_t>(AUTOEDGE_TIME_ADDR, _automatic_edge_time_minutes), int16_t(0), int16_t(1800));
  _automatic_edge_mintarget = clamp(EEPROM.get<int16_t>(AUTOEDGE_MIN_ADDR, _automatic_edge_mintarget), int16_t(0), int16_t(1000));
  _automatic_edge_maxtarget = clamp(EEPROM.get<int16_t>(AUTOEDGE_MAX_ADDR, _automatic_edge_maxtarget), int16_t(0), int16_t(1000));
  Serial.print("[SETUP] edge time in minutes:");
  Serial.println(_automatic_edge_time_minutes);

  _currentMenuItem = &g_rootMenu[1];
  _currentMenuItemState.isFirst = true;
  _currentMenuItemState.isLast = false;
  change_state(MachineStates::menu);
}

#define DEFAULT_COOLDOWN_s 45.0f
#define DEFAULT_COOLDOWN_ADJUSTMENT_FASTER_s 3.0f
#define DEFAULT_COOLDOWN_ADJUSTMENT_SLOWER_s 8.0f
#define DEFAULT_EDGETIME_TARGET_s 27.0f
#define DEFAULT_EDGETIME_TARGET_MIN_S 2.0f
#define DEFAULT_EDGETIME_TARGET_MAX_s (DEFAULT_EDGETIME_TARGET_s+13.0f)
#define DEFAULT_MIN_COOLDOWN_s 20.0f
#define DEFAULT_MAX_COOLDOWN_s 45.0f
void Logic::resetSession()
{
#if (defined(EDGEALGORITHM) && EDGEALGORITHM == 1)
  _cooldown = DEFAULT_COOLDOWN_s;
  _lastEdgeTime = millis();
#endif
  _autoTimeStart = millis();

  if (_hasSession) {
    g_wifi.LogEndSession();
    delay(100);
  }

  g_wifi.LogBeginSession();
  _hasSession = true;
}

void Logic::clearSession()
{
  g_wifi.ClearSession();
  _hasSession = false;
}

void Logic::run_menu()
{
  if (_physBtnState == PhysBtnState::Short)
  {
      switch (*_currentMenuItem)
      {
        case MenuItemsEnum::test: { change_state(MachineStates::manual_motor_control); } break;
        case MenuItemsEnum::manual: { resetSession(); change_state(MachineStates::manual_edge_control); } break;
        case MenuItemsEnum::automatic: { _currentMenuItem = &g_automaticMenu[1]; } break;
        case MenuItemsEnum::automatic_start: { resetSession(); change_state(MachineStates::auto_edge_control); } break;
        case MenuItemsEnum::automatic_continue: { if (!_hasSession) { resetSession(); }; change_state(MachineStates::auto_edge_control); } break;
        case MenuItemsEnum::automatic_input_time: { 
          _readVariable = &_automatic_edge_time_minutes;
          _readVariableMin = 5;
          _readVariableMax = 120;
          change_state(MachineStates::read_variable); 
        } break;
        case MenuItemsEnum::automatic_input_min: { 
          _readVariable = &_automatic_edge_mintarget;
          _readVariableMin = 0;
          _readVariableMax = 1500;
          change_state(MachineStates::read_variable); 
        } break;
        case MenuItemsEnum::automatic_input_max: { 
          _readVariable = &_automatic_edge_maxtarget;
          _readVariableMin = 0;
          _readVariableMax = 1500;
          change_state(MachineStates::read_variable); 
        } break;
        case MenuItemsEnum::select_host: { 
          _readVariable = g_wifi.GetSelectedHostPtr();
          _readVariableMin = 0;
          _readVariableMax = 1;
          change_state(MachineStates::read_variable); 
        } break;
        case MenuItemsEnum::automatic_back: { _currentMenuItem = &g_rootMenu[1]; } break;
        case MenuItemsEnum::toggle_online: { 
          Serial.println(g_wifi.GetOnlineMode() ? "toggle_online, onlinemode = true" : "toggle_online, onlinemode = false");
          volatile const bool newMode = !(g_wifi.GetOnlineMode());
          if (newMode)
          {
            g_wifi.SetWantsToBeOnline();
          }
          else
          {
            g_wifi.SetWantsToBeOffline();
          }
          Serial.print("writing to eeprom: ");
          Serial.println(newMode ? "true" : "false");
          Serial.flush();
          g_wifi.SetOnlineMode(newMode, true);
        } break;
        case MenuItemsEnum::resetSession: { resetSession(); }; break;
      }
  }

  _currentMenuItemState.isFirst = (*_currentMenuItem == MenuItemsEnum::INVALID) || (*(_currentMenuItem-1) == MenuItemsEnum::INVALID);
  _currentMenuItemState.isLast = *(_currentMenuItem+1) == MenuItemsEnum::LAST;

  const auto knob = myEnc->read();
  const auto delta = knob - _menuKnobState;
  if (delta >= 4)
  {
    if (!_currentMenuItemState.isLast)
    {
      _currentMenuItem++;
    }
    _menuKnobState = knob;
  } else if (delta <= -4)
  {
    if (!_currentMenuItemState.isFirst)
    {
      _currentMenuItem--;
    }
    _menuKnobState = knob;
  }
}

void Logic::run_read_variable()
{
  auto& var = *_readVariable;
  static String displayTxt;
  displayTxt = F("> ");
  displayTxt += var;
  displayTxt += F(" <");
  LCD::setLCDFunc(LCD::lcdfunc_renderText, displayTxt.c_str());

  const auto knob = myEnc->read();
  const auto delta = knob - _menuKnobState;

  if (delta >= 4)
  {
    var = clamp<int16_t>(++var, _readVariableMin, _readVariableMax);
    _menuKnobState = knob;
  } else if (delta <= -4)
  {
    var = clamp<int16_t>(--var, _readVariableMin, _readVariableMax);
    _menuKnobState = knob;
  }

  //confirm, go back to menu.
  if (_physBtnState == PhysBtnState::Short)
  {
    change_state(MachineStates::menu);
  }
}

// Manual vibrator control mode (red), still shows orgasm closeness in background
void Logic::run_manual_motor_control() 
{
  //In manual mode, only allow for 13 cursor positions, for adjusting motor speed.
  const int knob = encLimitRead(0,NUM_LEDS-1);
  g_motSpeed = map(knob, 0, NUM_LEDS - 1, 0., (float)MOT_MAX);

  const auto setting = g_vibrator.map_motspeed_to_vibrator();
  if (setting > 0)
  {
    g_vibrator.vibrator_on();
    g_vibrator.vibrator_to(setting);
  }
  else
  {
    g_vibrator.vibrator_off();
    g_vibrator.vibrator_reset();
  }

  //gyrGraphDraw(avgPressure, 0, 4 * 3 * NUM_LEDS);
  const int presDraw = map(constrain(g_currentPressure - g_avgPressure, 0, pLimit),0,pLimit,0,NUM_LEDS*3);
  draw_bars_3(presDraw, CRGB::Green,CRGB::Yellow,CRGB::Red);
  draw_cursor(knob, CRGB::Red);
}

void Logic::run_automatic_edge_control()
{
  const auto now = millis();
  const auto elapsedms = uint32_t(now - _autoTimeStart);
  const auto timespanms = uint32_t(_automatic_edge_time_minutes) * uint32_t(60) * uint32_t(1000);
  const float progress = float(elapsedms) / float(timespanms);
  _autoEdgeTargetOffset = _automatic_edge_mintarget + ((float)(_automatic_edge_maxtarget-_automatic_edge_mintarget) * progress);
  _autoEdgeTargetOffset = clamp<float>(_autoEdgeTargetOffset, _automatic_edge_mintarget, _automatic_edge_maxtarget);
}

// Automatic edging mode, knob adjust sensitivity.
void Logic::run_edge_control() 
{
  unsigned long now = millis();
  static float motIncrement = 0.0;
  const float oldMotSpeed = g_motSpeed;
  motIncrement = ((float)g_MaxSpeed / ((float)FREQUENCY * (float)rampTimeS));

  const int knob = encLimitRead(0,(3*NUM_LEDS)-1);
  sensitivity = knob*4; //Save the setting if we leave and return to this state
  //Reverse "Knob" to map it onto a pressure limit, so that it effectively adjusts sensitivity 
  pLimit = max(0, map(knob, 0, 3 * (NUM_LEDS - 1), DEFAULT_PLIMIT, 1)); //set the limit of delta pressure before the vibrator turns off
  pLimit += (int)_autoEdgeTargetOffset; 

  if (pLimit != pLimitLast)
  {
    pLimitLast = pLimit;
    LCD::_lastTargetChange = now;
  }

  //Temporarily raise the limit:
  //* if we haven't edged in the past DEFAULT_MIN_COOLDOWN_s/2 seconds
  //* are within the DEFAULT_EDGETIME_TARGET_MIN_S after the motor starts again.
  int16_t limitOffset=0;
  if ( !(now < _lastEdgeTime + static_cast<uint16_t>(DEFAULT_MIN_COOLDOWN_s*0.5f*1000.0f)) && 
        (now < _lastMotorEnableTimeMs + static_cast<uint16_t>(DEFAULT_EDGETIME_TARGET_MIN_S*1.1f*1000.0f)) )
  {
    limitOffset = (_automatic_edge_maxtarget - pLimit)*0.5f;
  }

  //When someone clenches harder than the pressure limit
  if (g_currentPressure - g_avgPressure > (pLimit+limitOffset)) 
  {
    if (g_motSpeed > 0.0f)
    {
      #if (defined(EDGEALGORITHM) && EDGEALGORITHM==1)
      //Adjust cooldown
      const float edgeTime = static_cast<float>(now-_lastEdgeTime) / 1000.0f;
      const float diff = edgeTime - DEFAULT_EDGETIME_TARGET_s;

      DEBUG_ONLY(Serial.print(F("Adjusted cooldown: ")));
      DEBUG_ONLY(Serial.print(_cooldown));
      DEBUG_ONLY(Serial.print(F("s -> ")));
      if (diff >= 0.0f)
      { //Too slow
        const float x = clamp<float>(diff / DEFAULT_EDGETIME_TARGET_MAX_s, 0.0f, 1.0f);
        const float adjustment = ((pow(x-1.0f, 3.0f))+1.0f) * DEFAULT_COOLDOWN_ADJUSTMENT_FASTER_s;
        _cooldown -= adjustment;
      } else if (diff < 0.0f)
      { //Too fast!
        const float x = clamp<float>(1.0f - (edgeTime/DEFAULT_EDGETIME_TARGET_s), 0.0f, 1.0f);
        _cooldown += (((pow(x-1.0f, 3.0f))+1.0f) * DEFAULT_COOLDOWN_ADJUSTMENT_SLOWER_s);
      }
      _cooldown = clamp<float>(_cooldown, DEFAULT_EDGETIME_TARGET_MIN_S, DEFAULT_EDGETIME_TARGET_MAX_s);
      _lastEdgeTime = millis();
      DEBUG_ONLY(Serial.print(_cooldown));
      DEBUG_ONLY(Serial.println(F("s.")));
      DEBUG_ONLY(Serial.flush());
      #endif
      g_vibrator.vibrator_off();
      g_vibrator.vibrator_reset();
      edgeCount++;
      g_lastEdgeCountChange = millis();
    }

    #if !defined(EDGEALGORITHM) || EDGEALGORITHM==0
    g_motSpeed = -s_cooldownTime*(float)rampTimeS*((float)FREQUENCY*motIncrement);//Stay off for a while (half the ramp up time)
    #elif (defined(EDGEALGORITHM) && EDGEALGORITHM==1)
    g_motSpeed = -_cooldown*((float)FREQUENCY*motIncrement);
    #endif
  } else if (g_motSpeed < static_cast<float>(g_MaxSpeed)) {
    g_motSpeed += motIncrement;
  }

  if(oldMotSpeed <= 0.0f && g_motSpeed > 0.0f)
  {
    _lastMotorEnableTimeMs = now;
  }

  const auto setting = g_vibrator.map_motspeed_to_vibrator();
  if (setting > 0)
  {
    g_vibrator.vibrator_on();
    g_vibrator.vibrator_to(setting);
  }
  else
  {
    g_vibrator.vibrator_off();
    g_vibrator.vibrator_reset();
  }
  
  int presDraw = map(constrain(g_currentPressure - g_avgPressure, 0, pLimit), 0, pLimit, 0, NUM_LEDS * 3);
  draw_bars_3(presDraw, CRGB::Green, CRGB::Yellow, CRGB::Red);
  draw_cursor_3(knob, CRGB(50, 50, 200), CRGB::Blue, CRGB::Purple);
}

//Poll the knob click button, and check for long/very long presses as well
PhysBtnState Logic::check_button()
{
  static PhysBtnState btnState = PhysBtnState::None;
  btnState = PhysBtnState::None;

  static bool lastBtn = ENC_SW_UP;
  static unsigned long keyDownTime = 0;
  static bool thisBtn = 0;
  thisBtn = digitalRead(ENC_SW);

  //Detect single presses, no repeating, on keyup
  if(thisBtn == ENC_SW_DOWN && lastBtn == ENC_SW_UP){
    keyDownTime = millis();
  }

  if (thisBtn == ENC_SW_UP && lastBtn == ENC_SW_DOWN)
  { //there was a keyup
    if ((millis() - keyDownTime) >= V_LONG_PRESS_MS)
    {
      btnState = PhysBtnState::VLong;
      keyDownTime = 0;
    }
    else if ((millis() - keyDownTime) >= LONG_PRESS_MS)
    {
      btnState = PhysBtnState::Long;
      keyDownTime = 0;
    }
    else
    {
      btnState = PhysBtnState::Short;
      keyDownTime = 0;
    }
  }

  lastBtn = thisBtn;
  return btnState;
}

void Logic::update_current_status()
{
  if (_physBtnState == PhysBtnState::Short)
  {
    g_lcd.nextMode();
  }
}

void Logic::log_session()
{
  static unsigned long lastTick = 0;

  auto now = millis();
  if(_hasSession && (lastTick+1000 < now))
  {
    lastTick = now;
    g_wifi.LogSessionTick(
        now, 
        {int32_t(g_motSpeed), 
        int32_t(g_currentPressure), 
        int32_t(g_avgPressure), 
        int32_t(g_currentPressure - g_avgPressure), 
        int32_t(pLimit), 
        int32_t(g_logic.GetCooldown()*1000.0f)}
    );
  }
}

//run the important/unique parts of each state. Also, set button LED color.
void Logic::run_state_machine(MachineStates state)
{
  switch (state) {
    case MachineStates::manual_motor_control:
      update_current_status();
      run_manual_motor_control();
      break;
    case MachineStates::manual_edge_control:
      update_current_status();
      run_edge_control();
      break;
    case MachineStates::auto_edge_control:
    {
      update_current_status();
      run_automatic_edge_control();
      run_edge_control();
      break;
    }
    case MachineStates::menu: 
      run_menu(); 
      break;
    case MachineStates::read_variable:
    {
      run_read_variable();
    } break;
    default:
      break;
  }
}

//Switch between state machine states, and reset the encoder position as necessary
//Returns the next state to run. Very long presses will turn the system off (sort of)
MachineStates Logic::change_state(MachineStates newState)
{
  auto oldState = _state;

  if (oldState != newState)
  {
    switch(oldState)
    {
      case MachineStates::UNDEFINED: {} break;
      case MachineStates::menu: {} break;
      case MachineStates::read_variable: {} break;
      case MachineStates::manual_motor_control: 
      {
      } break;
      case MachineStates::manual_edge_control: 
      {
        EEPROM.update(EEPROM_SENSITIVITY_ADDR, sensitivity);
      } break;
      case MachineStates::auto_edge_control: 
      {
        EEPROM.update(EEPROM_SENSITIVITY_ADDR, sensitivity);
        _autoEdgeTargetOffset = 0;
      } break;
      default: g_lcd.RenderPanic(F("State not implemented!")); break;
    }

    switch(newState)
    {
      case MachineStates::UNDEFINED: {} break;
      case MachineStates::menu: 
      {
        g_wifi.SetWantsToBeOffline();
        clearSession();
        _menuKnobState = myEnc->read();
        g_vibrator.vibrator_off();
      } break;
      case MachineStates::read_variable: {} break;
      case MachineStates::manual_motor_control: 
      {
        g_wifi.SetWantsToBeOffline();
        g_vibrator.vibrator_reset();
        g_vibrator.vibrator_on();
      } break;
      case MachineStates::manual_edge_control: 
      {
        //Restore last sensitivity value:
        g_wifi.SetWantsToBeOnline();
        sensitivity = EEPROM.read(EEPROM_SENSITIVITY_ADDR);
        myEnc->write(sensitivity);
        g_vibrator.vibrator_reset();
        g_vibrator.vibrator_on();
      } break;
      case MachineStates::auto_edge_control: 
      {
        //Store target settings in eeprom:
        g_wifi.SetWantsToBeOnline();
        EEPROM.put<int16_t>(AUTOEDGE_TIME_ADDR, _automatic_edge_time_minutes);
        EEPROM.put<int16_t>(AUTOEDGE_MIN_ADDR, _automatic_edge_mintarget);
        EEPROM.put<int16_t>(AUTOEDGE_MAX_ADDR, _automatic_edge_maxtarget);
        Serial.print("_automatic_edge_time_minutes = ");
        Serial.println(_automatic_edge_time_minutes);
        if (_autoTimeStart==0)
        {
          _autoTimeStart = millis();
        }

        //Restore last sensitivity value:
        sensitivity = EEPROM.read(EEPROM_SENSITIVITY_ADDR);
        myEnc->write(sensitivity);
        g_vibrator.vibrator_reset();
        g_vibrator.vibrator_on();
      } break;
      default: g_lcd.RenderPanic(F("State not implemented!")); break;
    }

    _state = newState;
    return _state;
  }

  #if 0
  if(btnState == BTN_NONE){
    return state;
  }
  if(btnState == BTN_V_LONG){
    //Turn the device off until woken up by the button
    Serial.println(F("power off"));
    fill_gradient_RGB(leds,0,CRGB::Black,NUM_LEDS-1,CRGB::Black);//Turn off LEDS
    FastLED.show();
    while(!digitalRead(ENC_SW))delay(1);
    return MachineStates::manual_motor_control;
  }
  else if(btnState == BTN_SHORT){
    switch(state){
      case MachineStates::manual_motor_control:
        myEnc->write(sensitivity);//Whenever going into auto mode, keep the last sensitivity
        g_motSpeed = 0; //Also reset the motor speed to 0
        return MachineStates::manual_edge_control;
      case MachineStates::manual_edge_control:
        myEnc->write(0);//Whenever going into manual mode, set the speed to 0.
        g_motSpeed = 0;
        EEPROM.update(EEPROM_SENSITIVITY_ADDR, sensitivity);
        return MachineStates::manual_motor_control;
    }
  }
  else if(btnState == BTN_LONG){
    switch (state) {
      case MachineStates::manual_motor_control:
        myEnc->write(map(g_MaxSpeed, 0, 255, 0, 4 * (NUM_LEDS))); //start at saved value
        return MachineStates::manual_edge_control;//OPT_SPEED;
      case MachineStates::manual_edge_control:
        myEnc->write(map(g_MaxSpeed, 0, 255, 0, 4 * (NUM_LEDS))); //start at saved value
        return MachineStates::manual_motor_control;//OPT_SPEED;
    }
  }
  else return MachineStates::manual_motor_control;
  #endif
}

//=======LED Drawing Functions=================

//Draw a "cursor", one pixel representing either a pressure or encoder position value
//C1,C2,C3 are colors for each of 3 revolutions over the 13 LEDs (39 values)
void draw_cursor_3(int pos,CRGB C1, CRGB C2, CRGB C3){
  pos = constrain(pos,0,NUM_LEDS*3-1);
  const int colorNum = pos/NUM_LEDS; //revolution number
  const int cursorPos = pos % NUM_LEDS; //place on circle, from 0-12
  switch(colorNum){
    case 0:
      leds[cursorPos] = C1;
      break;
    case 1:
      leds[cursorPos] = C2;
      break;
    case 2:
      leds[cursorPos] = C3;
      break;
  }
}

//Draw a "cursor", one pixel representing either a pressure or encoder position value
void draw_cursor(int pos,CRGB C1){
  pos = constrain(pos,0,NUM_LEDS-1);
  leds[pos] = C1;
}

//Draw 3 revolutions of bars around the LEDs. From 0-39, 3 colors
void draw_bars_3(int pos,CRGB C1, CRGB C2, CRGB C3){
  pos = constrain(pos,0,NUM_LEDS*3-1);
  const int colorNum = pos/NUM_LEDS; //revolution number
  const int barPos = pos % NUM_LEDS; //place on circle, from 0-12
  switch(colorNum){
    case 0:
      fill_gradient_RGB(leds,0,C1,barPos,C1);
      //leds[barPos] = C1;
      break;
    case 1:
      fill_gradient_RGB(leds,0,C1,barPos,C2);
      break;
    case 2:
      fill_gradient_RGB(leds,0,C2,barPos,C3);
      break;
  }
}

//Provide a limited encoder reading corresponting to tacticle clicks on the knob.
//Each click passes through 4 encoder pulses. This reduces it to 1 pulse per click
int encLimitRead(int minVal, int maxVal){
  if(myEnc->read()>maxVal*4)myEnc->write(maxVal*4);
  else if(myEnc->read()<minVal*4) myEnc->write(minVal*4);
  return constrain(myEnc->read()/4,minVal,maxVal);
}

#if defined(PROFILING_ENABLED) && PROFILING_ENABLED==1
using ProfiledFunc = void (*)();
void profile(int storageID, ProfiledFunc profiledFunc)
{
  static constexpr uint8_t profilecount = 10;
  static uint16_t times[profilecount]={};
  static uint16_t maxTime[profilecount]={};
  const auto now = millis();
  profiledFunc();
  const auto diff = millis()-now;
  times[storageID] += diff;
  times[storageID] /= 2;
  maxTime[storageID] = max(maxTime[storageID], min(diff,0xFFFFul));

  static uint32_t lastReport = now;
  if (now > lastReport+1000)
  {
    lastReport = now;
    Serial.println(F("== times =="));
    for (auto i=0; i < profilecount; i++)
    {
      Serial.print(F("["));
      Serial.print(i);
      Serial.print(F("] avg: "));
      Serial.print(times[i]);
      Serial.print(F(" max: "));
      Serial.println(maxTime[i]);
      if (maxTime[i]>0)
      {
        maxTime[i] -= std::min(uint16_t(maxTime[i]/10), maxTime[i]);
      }
    }
  }
}
#endif
#endif

#define ProfileBlock(pTreshold, pMsg)                                                                 \
    struct TimerLogger##__LINE__                                                                      \
    {                                                                                                 \
      struct inner                                                                                    \
      {                                                                                               \
        uint64_t _start;                                                                              \
        inner() : _start(millis()) {};                                                                \
        ~inner() { if((millis()-_start)>pTreshold) { Serial.println("Timer triggered " pMsg); } }     \
      } _inner;                                                                                       \
    } _timer##__LINE__;

/////////////////////////////////////////////////////
// WIFI control
int WIFI::runCoroutine()
{
  auto checkTimeout = [&](){
      if (ShouldConnect())
      {
        if(!IsConnected())
        {
          const auto now = millis();
          if (now > GetWantsToBeOnlineStartTime()+(1000*15))
          {
            SetOnlineMode(false, false);
          }
        } else
        {
          _wantsToBeOnlineStartTime = millis();
        }
      }
  };

  COROUTINE_LOOP()
  {
      if(!ShouldConnect())
      {
        return 0;
      }

      checkTimeout();

      #if 1
      static uint64_t timeout = 1000;
      static auto status = wl_status_t::WL_DISCONNECTED;
      while (status != WL_CONNECTED && status != WL_IDLE_STATUS)
      {
        ProfileBlock(200, "Reconnection");
        WiFi.setTimeout(timeout);
        WiFi.begin(NOGASM_WIFI_SSID, NOGASM_WIFI_PASS);

        static auto start = millis();
        start = millis();
        while (ShouldConnect() && WiFi.status() != WL_CONNECTED && millis()<(start+timeout))
        {
          COROUTINE_DELAY(15);
        }
        status = static_cast<wl_status_t>(WiFi.status());
        if(status != WL_CONNECTED)
        {
          WiFi.end();
          Serial.print("Wifi connection failed... error code: ");
          Serial.print(status);
          Serial.println(", retrying");
          timeout += 1000;
          COROUTINE_DELAY(1000*1);
        }
        else
        {
          Serial.println("Wifi connected!");
          COROUTINE_DELAY(1000*1);
        }
      }
    #endif

    //refresh status, make sure we haven't disconnected...
    static uint64_t lastTick = millis();
    if (millis() > lastTick+1000)
    {
      lastTick = millis();
      ProfileBlock(200, "Wifi Status");
      status = static_cast<wl_status_t>(WiFi.status());
      if(status != WL_CONNECTED && status != WL_IDLE_STATUS)
      {
        ProfileBlock(200, "Wifi End");
        WiFi.end();
        Serial.print("Lost wifi connection, status: ");
        Serial.println(status);
      }

      auto& client = g_WifiWriter.GetClient();
      _connected = !client.hasError() && client.connected();
      if (!client.connected())
      {
        Serial.println("client disconnected");
      }

      if (client.hasError())
      {
        Serial.println("client has error");
      }

      if (!_connected)
      {
        Serial.println("server disconnected, reconnecting");
        connect();
      }
    }

    if (status == WL_CONNECTED && g_wifi.ShouldConnect())
    {
      if (_resetSession)
      {
        static bool success;
        success = internalRequestSession();
        if (!success)
        {
          Serial.println("request failed");
          COROUTINE_DELAY(1000 * 60);
        }
        else
        {
          while(ShouldConnect() && !g_WifiWriter.GetClient().available())
          {
            spareDebugPrintln("waiting for data to become available...");
            COROUTINE_DELAY(10);
          }
          internalRequestSessionResponse();
        }
        COROUTINE_YIELD();
      }

      {
        while(ShouldConnect() && WiFi.status() == WL_CONNECTED && !_buffer.isEmpty())
        {
          if (connect())
          {
            ProfileBlock(200, "Wifi send log");
            internalSendLog(_buffer.shift());
          }
          checkTimeout();
          COROUTINE_YIELD();
        }
      }
    }
    
    COROUTINE_YIELD();
  }
}

void WIFI::Setup()
{
  _onlineMode = EEPROM.get(EEPROM_ONLINEMODE_ADDR, _onlineMode);
  Serial.println((_onlineMode!=0) ? "[STARTUP] We want to be online" : "[STARTUP] We want to be offline");

  _selectedHost = std::clamp(EEPROM.get(EEPROM_SELECTEDHOST_ADDR, _selectedHost), int16_t(0), int16_t(std::size(NOGASM_LOGGER_HOST)));
  Serial.print("[STARTUP] SelectedHost=");
  Serial.println(_selectedHost); 
}

void WIFI::Update()
{}

bool WIFI::internalRequestSession() 
{
#if 1
  //_client.stop();
  //_client.flush();
  Serial.println("requesting session");
  auto& client = g_WifiWriter.GetClient();
  if (connect())
  {
    client.println("GET /session HTTP/1.1");
    client.print("Host: "); client.println(NOGASM_LOGGER_HOST[g_wifi.GetSelectedHost()]);
    client.println("User-Agent: ArduinoWiFi/1.1");
    client.println("Connection: close");
    client.println();
    client.flushOut(-1);
    Serial.println("session requested, awaiting answer");
    return true;
  }
  else
  {
    Serial.println("connection failed");
  }
#endif
  return false;
};

void WIFI::internalRequestSessionResponse()
{
    Serial.println("reading answer");
    auto& client = g_WifiWriter.GetClient();
    auto answer =  client.readString();
    std::string s{answer.c_str()};
    if (s.find("200 OK")>=0)
    {
      Serial.println("http answer");
      s.erase(0, s.find_last_of("\n\n")+2);
      _session = s.c_str();
      _sessionStartTime = millis();
      Serial.println(_session.c_str());
      _resetSession = false;
      client.stop();
      _connected = false;
    }
    else
    {
      _resetSession = true;
    }
}

void WIFI::internalSendLog(const internalLog& logData)
{
  auto& client = g_WifiWriter.GetClient();
  

  arduino::String s;

  //body:
  s+="{"
                    "\"time\": "; s+=static_cast<unsigned long>(logData.time-_sessionStartTime); s+=","
                    "\"param1\": "; s+=logData.data.param1; s+=","
                    "\"param2\": "; s+=logData.data.param2; s+=","
                    "\"param3\": "; s+=logData.data.param3; s+=","
                    "\"param4\": "; s+=logData.data.param4; s+=","
                    "\"param5\": "; s+=logData.data.param5; s+=","
                    "\"param6\": "; s+=logData.data.param6;             
  s+="}";
  uint16_t c=s.length();

  client.print("POST /session/"); client.print(_session.c_str()); client.println("/log HTTP/1.1");
  client.print("Host: "); client.println(NOGASM_LOGGER_HOST[g_wifi.GetSelectedHost()]);
  client.println("User-Agent: ArduinoWiFi/1.1");
  client.println("Connection: keep-alive");
  client.println("Keep-Alive: timeout=5, max=5000");
  //client.println("Connection: close");
  client.print("Content-Length: "); client.println(c);
  client.println();
  client.println(s);
  //client.flush(); //don't care about any previous responses
  /*
  if (c>500)
  {
    Serial.println("[ERROR] c>500");
    while(true) { delay(1); }
  }

  for(int i=0; i < 500-c; i++)
  {
    client.print(" ");
  }
  */

  //Serial.println(&lbuffer[0]);
}

void WIFI::LogEndSession() 
{
   _buffer.clear();
   _session.clear();
}

void WIFI::LogSessionTick(unsigned long long tick, const LogData& logData) 
{
   if (!_buffer.isFull() && !_session.empty())
   {
     internalLog log;
     log.time = tick;
     log.data = logData;
     _buffer.push(std::move(log));
   }
   else
   {
    //Serial.println(!_buffer.isFull() ? "buffer was not full" : "fail, buffer full");
    //Serial.println(!_session.empty() ? "session was set" : "fail, session not set");
   }
}

void WIFI::SetOnlineMode(volatile bool pMode, bool pPersist)
{
  Serial.println((_onlineMode!=0) ? "Online mode was true" : "Online mode was false");
  _onlineMode = (pMode ? 1 : 0);
  Serial.println((_onlineMode!=0) ? "Online mode is now true" : "Online mode is now false");
  
  if (pPersist)
  {
    EEPROM.put(EEPROM_ONLINEMODE_ADDR, pMode);
  }
}

bool WIFI::connect()
{
  if(!GetOnlineMode())
  {
    return false;
  }

  auto& client = g_WifiWriter.GetClient();
  if (!_connected)
  {
    client.stop();
    client.clearError();
    client.clear();

    // print the SSID of the network you're attached to:
    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());


    // print your board's IP address:

    IPAddress ip = WiFi.localIP();
    Serial.print("IP Address: ");
    Serial.println(ip);


    // print the received signal strength:

    long rssi = WiFi.RSSI();
    Serial.print("signal strength (RSSI):");
    Serial.print(rssi);
    Serial.println(" dBm");

    _connected = client.connect(NOGASM_LOGGER_HOST[g_wifi.GetSelectedHost()], NOGASM_LOGGER_PORT)==1;
    if (_connected)
    {
      //always:
      EEPROM.put(EEPROM_SELECTEDHOST_ADDR, _selectedHost);
      Serial.print("Set selected host to ");
      Serial.println(_selectedHost);
    }
    return _connected;
  }
  return true;
}

/////////////////////////////////////////////////////
// WIFI writer
int WIFIWriter::runCoroutine()
{
  COROUTINE_LOOP()
  {
    static auto start = 0ll;
    start = millis();

    if (_client.HasDataToWriteOut())
    {
      while(millis() < start+33)
      {
        while(_client.available())
        {
          _client.read();
        }
        auto wroteBytes = _client.flushOut();
        if (wroteBytes==0)
        {
            COROUTINE_DELAY(50);
            break;
        }
      }
    }
    else
    {
      COROUTINE_DELAY(200);
    }
    
    COROUTINE_YIELD();
  }
}
//=======Setup=======================================
void setup() 
{
  myEnc = new Encoder(3, 2);

  Serial.begin(9600); delay(100); DEBUG_ONLY(Serial.println(F("Serial up"))); Serial.flush();
  #if 1
  g_sensor.Setup();  delay(100); DEBUG_ONLY(Serial.println(F("Sensor up"))); Serial.flush();
  g_logic.Setup();  delay(100); DEBUG_ONLY(Serial.println(F("Logic up"))); Serial.flush();
  g_lcd.Setup();  delay(100); DEBUG_ONLY(Serial.println(F("LCD up"))); Serial.flush();
  g_leds.Setup();  delay(100); DEBUG_ONLY(Serial.println(F("LEDS up"))); Serial.flush();
  g_vibrator.Setup();  delay(100); DEBUG_ONLY(Serial.println(F("Vibrator up"))); Serial.flush();
  g_wifi.Setup();  delay(100); DEBUG_ONLY(Serial.println(F("Wifi up"))); Serial.flush();
  g_WifiWriter.Setup();  delay(100); DEBUG_ONLY(Serial.println(F("WifiWriter up"))); Serial.flush();
  #endif
  Serial.println(F("Started up"));
}

//=======Main Loop=============================
void loop() 
{
  static bool rendered = false;
#if 1
#if defined(PROFILING_ENABLED) && PROFILING_ENABLED==1
  profile(0, [](){ g_sensor.runCoroutine(); });
  profile(1, [](){ g_logic.runCoroutine(); });
  profile(2, [](){ g_lcd.runCoroutine(); });
  profile(3, [](){ g_leds.runCoroutine(); });
  profile(4, [](){ g_vibrator.runCoroutine(); });
  profile(5, [](){ g_wifi.runCoroutine(); });
  profile(7, [](){ g_WifiWriter.runCoroutine(); });
#else
  g_sensor.runCoroutine(); 
  g_logic.runCoroutine();
  g_lcd.runCoroutine();
  g_leds.runCoroutine();
  g_vibrator.runCoroutine();
  g_wifi.runCoroutine();
  g_WifiWriter.runCoroutine();
#endif
#endif
}