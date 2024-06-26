#pragma GCC push_options
//#pragma GCC optimize ("Os")
//#pragma GCC optimize ("O2")
//#pragma GCC optimize ("Od")

#define DEBUG_LOG_ENABLED 1
#define PROFILING_ENABLED 0

//defines NOGASM_WIFI_SSID & NOGASM_WIFI_PASS:
#include "nogasm_wifi.h"

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
#define RA_VARHIST_SECONDS 5
#define RA_FREQUENCY 10
#define RA_TICK_PERIOD (FREQUENCY / RA_FREQUENCY)

MovingAveragePlus<uint16_t, uint32_t, RA_FREQUENCY*RA_HIST_SECONDS> g_raOverallPressure;
MovingAveragePlus<float, float, RA_FREQUENCY*RA_VARHIST_SECONDS> g_raOverallPressureVariance;
MovingAveragePlus<uint16_t, uint32_t, 8> g_raImmediatePressure;
float g_PressureVariance = 0.0f;
float g_PressureVarianceMaximum = 15.0f;
//LCD
#define U8X8_HAVE_HW_I2C
#include <Arduino.h>
#include <U8g2lib.h>
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif



#if 1
#define font_8pt  u8g2_font_5x8_tr
#define font_13pt u8g2_font_6x13_tr
#define font_15pt u8g2_font_9x15_tr
#define font_18pt u8g2_font_9x18_tr
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
  stabilize,
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

#define DEFAULT_COOLDOWN_s 45.0f
#define DEFAULT_COOLDOWN_ADJUSTMENT_FASTER_s 3.0f
#define DEFAULT_COOLDOWN_ADJUSTMENT_SLOWER_s 8.0f
#define DEFAULT_EDGETIME_TARGET_s 27.0f
#define DEFAULT_EDGETIME_TARGET_MIN_S 2.0f
#define DEFAULT_EDGETIME_TARGET_MAX_s (DEFAULT_EDGETIME_TARGET_s+13.0f)
#define DEFAULT_MIN_COOLDOWN_s 20.0f
#define DEFAULT_MAX_COOLDOWN_s 45.0f

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
  automatic_input_stabilization_maximum,
  automatic_input_edge_INITIALCOOLDOWN_s,
  automatic_input_edge_COOLDOWN_ADJUSTMENT_FASTER_s,
  automatic_input_edge_COOLDOWN_ADJUSTMENT_SLOWER_s,
  automatic_input_edge_EDGETIME_TARGET_s,
  automatic_input_edge_EDGETIME_TARGET_MIN_S,
  automatic_input_edge_EDGETIME_TARGET_MAX_s,
  automatic_input_edge_MIN_COOLDOWN_s,
  automatic_input_edge_MAX_COOLDOWN_s,
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
  MenuItemsEnum::automatic_input_stabilization_maximum,
  MenuItemsEnum::automatic_input_edge_INITIALCOOLDOWN_s,
  MenuItemsEnum::automatic_input_edge_COOLDOWN_ADJUSTMENT_FASTER_s,
  MenuItemsEnum::automatic_input_edge_COOLDOWN_ADJUSTMENT_SLOWER_s,
  MenuItemsEnum::automatic_input_edge_EDGETIME_TARGET_s,
  MenuItemsEnum::automatic_input_edge_EDGETIME_TARGET_MIN_S,
  MenuItemsEnum::automatic_input_edge_EDGETIME_TARGET_MAX_s,
  MenuItemsEnum::automatic_input_edge_MIN_COOLDOWN_s,
  MenuItemsEnum::automatic_input_edge_MAX_COOLDOWN_s,
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
#define EEPROM_ONLINEMODE_ADDR                              0*4
#define EEPROM_MAX_SPEED_ADDR                               1*4
#define EEPROM_SENSITIVITY_ADDR                             2*4
#define AUTOEDGE_MIN_ADDR                                   3*4
#define AUTOEDGE_MAX_ADDR                                   4*4
#define AUTOEDGE_TIME_ADDR                                  5*4
#define EEPROM_SELECTEDHOST_ADDR                            6*4
#define PRESSUREVARMAX_ADDR                                 7*4
#define EEPROM_DEFAULT_COOLDOWN_s_ADDR                      8*4
#define EEPROM_DEFAULT_COOLDOWN_ADJUSTMENT_FASTER_s_ADDR    9*4
#define EEPROM_DEFAULT_COOLDOWN_ADJUSTMENT_SLOWER_s_ADDR   10*4
#define EEPROM_DEFAULT_EDGETIME_TARGET_s_ADDR              11*4
#define EEPROM_DEFAULT_EDGETIME_TARGET_MIN_S_ADDR          12*4
#define EEPROM_DEFAULT_EDGETIME_TARGET_MAX_s_ADDR          13*4
#define EEPROM_DEFAULT_MIN_COOLDOWN_s_ADDR                 14*4
#define EEPROM_DEFAULT_MAX_COOLDOWN_s_ADDR                 15*4


//#define RAMPSPEED_ADDR    4 //For now, ramp speed adjustments aren't implemented
#if 1
class CustomCoroutine : public Coroutine {
private:
  uint8_t _suspendCount = 0;
public:
  void Suspend() { _suspendCount++; }
  void Resume() { _suspendCount--; }
  virtual bool IsSuspended() const { return _suspendCount!=0; }
};

//=======Hardware Coroutines=========================
class Logic : public CustomCoroutine
{
  using super = CustomCoroutine;
  protected:
    void run_menu();
    void run_read_variable();
    void run_automatic_edge_control();
    void run_edge_control();
    void run_manual_motor_control();
    void run_stabilize();
    void update_current_status();
    void log_session();
    PhysBtnState check_button();
    void run_state_machine(MachineStates state);
    MachineStates change_state(MachineStates newState);
    void resetSession();
    void clearSession();
    void SaveSettings();
  public:
    void Setup();
    void PushState(MachineStates newState);
    void PopState();
    MachineStates getState() const { return *_state; }
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

    MachineStates _stateStack[8]{};
    MachineStates* _state = &_stateStack[0];
    PhysBtnState _physBtnState = PhysBtnState::None;
    int32_t _menuKnobState = 0;

    typedef void (*writeVariableCallback)(const int16_t* pWrittenVariable);
    int16_t* _readVariable = nullptr;
    int16_t _readVariableMin = 0;
    int16_t _readVariableMax = 1000;
    writeVariableCallback _writeVariableCallback = nullptr;

    int16_t _automatic_edge_maxtarget = 0;
    int16_t _automatic_edge_mintarget = 0;
    int16_t _automatic_edge_time_minutes = 0;
    int16_t _autoEdgeTargetOffset = 0;
    unsigned long _autoTimeStart = 0;
    int16_t _automatic_edge_INITIALCOOLDOWN_s = DEFAULT_COOLDOWN_s;
    int16_t _automatic_edge_COOLDOWN_ADJUSTMENT_FASTER_s = DEFAULT_COOLDOWN_ADJUSTMENT_FASTER_s;
    int16_t _automatic_edge_COOLDOWN_ADJUSTMENT_SLOWER_s = DEFAULT_COOLDOWN_ADJUSTMENT_SLOWER_s;
    int16_t _automatic_edge_EDGETIME_TARGET_s = DEFAULT_EDGETIME_TARGET_s;
    int16_t _automatic_edge_EDGETIME_TARGET_MIN_S = DEFAULT_EDGETIME_TARGET_MIN_S;
    int16_t _automatic_edge_EDGETIME_TARGET_MAX_s = DEFAULT_EDGETIME_TARGET_MAX_s;
    int16_t _automatic_edge_MIN_COOLDOWN_s = DEFAULT_MIN_COOLDOWN_s;
    int16_t _automatic_edge_MAX_COOLDOWN_s = DEFAULT_MAX_COOLDOWN_s;

    bool _hasSession = false;
    #if (defined(EDGEALGORITHM) && EDGEALGORITHM==1)
    float _cooldown = 0.0f;
    unsigned long _lastEdgeTime=0;
    unsigned long _lastMotorEnableTimeMs=0;
    public:
      float GetCooldown() const { return _cooldown; }
    #endif
} g_logic;

class Sensor : public CustomCoroutine
{
  using super = CustomCoroutine;
public:
    void Setup();
    virtual int runCoroutine() override;
} g_sensor;

class Vibrator : public CustomCoroutine
{
  using super = CustomCoroutine;
  struct
  {
    bool powered = false;
    uint8_t mode = 0;
    uint32_t lastChange = 0;
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

    bool IsChanging() const { 
      return (_currentState.powered != _targetState.powered) || getStateDiff()!=0;
    }

    virtual int runCoroutine() override;
  protected:
    int8_t getStateDiff() const 
    { 
        const int8_t fromStrength = clamp<int8_t>(static_cast<int8_t>(_currentState.mode), 0, MAX_LEVEL);
        const int8_t toStrength = clamp<int8_t>(static_cast<int8_t>(_targetState.mode), 0, MAX_LEVEL);
        return toStrength - fromStrength; 
      }
} g_vibrator;

class LEDs : public CustomCoroutine
{
  using super = CustomCoroutine;
  public:
    void Setup();
    void Update();

    //virtual bool IsSuspended();
    virtual int runCoroutine() override 
    {
      COROUTINE_LOOP()
      {
        Update();
        COROUTINE_YIELD();
      }
    }
} g_leds;

class LCD : public CustomCoroutine
{
  using super = CustomCoroutine;
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

    virtual bool IsSuspended() const override;
    virtual int runCoroutine() override;

    template <typename T, typename I, int idx = 0, typename... Iremainder>
    static void setLCDFunc(T func, I arg, Iremainder... args);

    static void lcdfunc_renderMenu();
    static void lcdfunc_edgeCount();
    static void lcdfunc_renderText();
    static void lcdfunc_renderTarget();
    static void lcdfunc_renderPressure();
    static void lcdfunc_renderStabilization();

    static void RenderPanic(const String _str);
  public:
    static unsigned long _lastTargetChange;
  private:
    struct lcdrenderdata_t
    {
      int32_t integer32;
      const char *stringptr;

      int32_t operator=(const int32_t arg)
      {
        integer32 = arg;
        return arg;
      }

      const char* operator=(const char *arg)
      {
        stringptr = arg;
        return arg;
      }
    } static lcdrenderdata[8];
    int8_t _mode = 0;
    unsigned long _lastChange = millis();
} g_lcd;
LCD::lcdrenderdata_t LCD::lcdrenderdata[8];
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
  int32_t param7{};
  int32_t param8{};
};

class WIFI : public CustomCoroutine
{
  using super = CustomCoroutine;
  private:
    struct internalLog
    {
      uint64_t time=0;
      int8_t  channel=0;
      LogData data{};
    };
    uint64_t _lastPing=0;
    std::string _session;
    CircularBuffer<internalLog, 60> _buffer;
    WiFiUDP _udpclient;
    uint64_t _sessionStartTime=0;
    uint64_t _wantsToBeOnlineStartTime = 0;
    int16_t _selectedHost = 0;
    uint8_t _onlineMode = 1;
    bool _connected = false;
    bool _resetSession = false;
    bool _wantsToBeOnline = false;
  private:
    bool connect();
    void internalRequestSession();
    void internalRequestSessionResponse();
    void internalSendLog(const internalLog& logData);
  public:
    void Setup();
    void Update();

    virtual bool IsSuspended() const;

    virtual int runCoroutine() override;

    void LogBeginSession() { _session.clear(); _resetSession = true; }
    void LogEndSession();
    void LogSessionTick(const LogData& logData);
    void LogMotorChange(bool pNewMotorState, uint32_t pRequestTime, uint8_t pMotorLevel);
    void LogEdgeChange(uint16_t pEdgeCount);

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
    case MenuItemsEnum::automatic_input_stabilization_maximum: output = F("Stab. target: "); return;
    case MenuItemsEnum::automatic_input_edge_INITIALCOOLDOWN_s: output = F("edge-initial-cooldown(s)"); return;
    case MenuItemsEnum::automatic_input_edge_COOLDOWN_ADJUSTMENT_FASTER_s: output = F("edge-adjust-faster(s)"); return;
    case MenuItemsEnum::automatic_input_edge_COOLDOWN_ADJUSTMENT_SLOWER_s: output = F("edge-adjust-slower(s)"); return;
    case MenuItemsEnum::automatic_input_edge_EDGETIME_TARGET_s: output = F("edge-time-targ(s)"); return;
    case MenuItemsEnum::automatic_input_edge_EDGETIME_TARGET_MIN_S: output = F("edge-time-min(s)"); return;
    case MenuItemsEnum::automatic_input_edge_EDGETIME_TARGET_MAX_s: output = F("edge-time-max(s)"); return;
    case MenuItemsEnum::automatic_input_edge_MIN_COOLDOWN_s: output = F("edge-cooldown-min(s)"); return;
    case MenuItemsEnum::automatic_input_edge_MAX_COOLDOWN_s: output = F("edge-cooldown-max(s)"); return;
    case MenuItemsEnum::automatic_back: output = F("[ Back ]"); return;
    case MenuItemsEnum::select_host: output = "Host: "; output += NOGASM_LOGGER_HOST[g_wifi.GetSelectedHost()].toString(); return;
    case MenuItemsEnum::toggle_online: output = (g_wifi.GetOnlineMode() ? F("Go offline") : F("Go online")); return;
    case MenuItemsEnum::resetSession: output = F("Reset Session"); return;
    case MenuItemsEnum::LAST: output = F("!ERROR!"); return;
  }
}

bool LCD::IsSuspended() const
{
  return g_vibrator.IsChanging() || super::IsSuspended();
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
      //memset(&lcdrenderdata[0], 0, sizeof(lcdrenderdata));
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
  else if (state == MachineStates::stabilize)
  {
    setLCDFunc(lcdfunc_renderStabilization, g_PressureVariance, g_PressureVarianceMaximum);
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
          setLCDFunc(lcdfunc_renderPressure, g_currentPressure, g_avgPressure, (int16_t)g_PressureVariance, (int16_t)g_PressureVarianceMaximum, 1337);
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
void LCD::setLCDFunc(T func, volatile I arg, Iremainder... args)
{
  if (lcdrender_activefunc == nullptr)
  {
    if constexpr(idx < std::size(lcdrenderdata))
    {
      LCD::lcdrenderdata[idx] = arg;
    }
    if constexpr(sizeof...(args)>0)
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
    case MenuItemsEnum::automatic_input_stabilization_maximum:
    text += g_PressureVarianceMaximum;
    break;
  }

  auto textWidth = u8g2.getStrWidth(text.c_str());
  if (textWidth > u8g2.getDisplayWidth())
  {
    u8g2.setFont(font_8pt);
    textWidth = u8g2.getStrWidth(text.c_str());
  }
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
}

//static auto lastPressureChange = millis();
void LCD::lcdfunc_renderPressure()
{
  u8g2.setFont(font_13pt);
  u8g2.setCursor(0, u8g2.getMaxCharHeight());
  u8g2.print(F("P"));
  u8g2.setFont(font_15pt);
  u8g2.setCursor(10, u8g2.getMaxCharHeight());
  u8g2.print(lcdrenderdata[0].integer32);
  u8g2.setCursor(50, u8g2.getMaxCharHeight());
  u8g2.setFont(font_13pt);
  u8g2.print(F("vs"));
  u8g2.setFont(font_15pt);
  u8g2.setCursor(65, u8g2.getMaxCharHeight());
  u8g2.print(lcdrenderdata[1].integer32);

  u8g2.setFont(font_13pt);
  u8g2.setCursor(0, 2+(u8g2.getMaxCharHeight()*2));
  u8g2.print(F("V"));
  u8g2.setFont(font_15pt);
  u8g2.setCursor(10, 2+(u8g2.getMaxCharHeight()*2));
  u8g2.print(lcdrenderdata[2].integer32);
  u8g2.setCursor(50, 2+(u8g2.getMaxCharHeight()*2));
  u8g2.setFont(font_13pt);
  u8g2.print(F("vs"));
  u8g2.setFont(font_15pt);
  u8g2.setCursor(65, 2+(u8g2.getMaxCharHeight()*2));
  u8g2.print(lcdrenderdata[3].integer32);
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

void LCD::lcdfunc_renderStabilization()
{
  u8g2.setFont(font_18pt);
  u8g2.setCursor(0, u8g2.getMaxCharHeight());
  u8g2.print(F("Edged!"));
  u8g2.setFont(font_15pt);
  u8g2.setCursor(0, 2+(u8g2.getMaxCharHeight()*2));
  u8g2.print(F("v"));
  u8g2.setCursor(10, 2+(u8g2.getMaxCharHeight()*2));
  u8g2.print(uint32_t(lcdrenderdata[0].integer32));
  u8g2.setCursor(50, 2+(u8g2.getMaxCharHeight()*2));
  u8g2.print(F("vs"));
  u8g2.setCursor(70, 2+(u8g2.getMaxCharHeight()*2));
  u8g2.print(uint32_t(lcdrenderdata[1].integer32));
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
      g_wifi.Suspend();
      {
        digitalWrite(PIN_VIBRATOR_ONOFF, HIGH);
        COROUTINE_DELAY(VIBRATOR_PUSH_DELAY);
        digitalWrite(PIN_VIBRATOR_ONOFF, LOW);
        g_wifi.LogMotorChange(true, _targetState.lastChange, _targetState.mode);
        COROUTINE_DELAY(VIBRATOR_PUSH_DELAY);
      }
      g_wifi.Resume();

      DEBUG_ONLY(Serial.println(F("[VIBRATOR]\tTurned on")));
    }
    else if (_currentState.powered == true && _targetState.powered == false)
    {
      //turn off:
      _currentState.powered = false;
      g_wifi.Suspend();
      {
        digitalWrite(PIN_VIBRATOR_ONOFF, HIGH);
        COROUTINE_DELAY(VIBRATOR_PUSH_DELAY);
        digitalWrite(PIN_VIBRATOR_ONOFF, LOW);
        g_wifi.LogMotorChange(false, _targetState.lastChange, _targetState.mode);
        COROUTINE_DELAY(VIBRATOR_PUSH_DELAY);
      }
      g_wifi.Resume();

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
          g_wifi.Suspend();
          {
            digitalWrite(PIN_VIBRATOR_UP, HIGH);
            COROUTINE_DELAY(VIBRATOR_PUSH_DELAY);
            digitalWrite(PIN_VIBRATOR_UP, LOW);
            g_wifi.LogMotorChange(true, _targetState.lastChange, _targetState.mode);
            COROUTINE_DELAY(VIBRATOR_PUSH_DELAY);
          }
          g_wifi.Resume();
          DEBUG_ONLY(Serial.println(F("[VIBRATOR]\tTurned UP")));
        }
      }
      else if (diff < 0)
      {
        for (i = 0; i < -diff && _currentState.powered; i++)
        {
          //turn down:
          _currentState.mode--;
          g_wifi.Suspend();
          {
            digitalWrite(PIN_VIBRATOR_DOWN, HIGH);
            COROUTINE_DELAY(VIBRATOR_PUSH_DELAY);
            digitalWrite(PIN_VIBRATOR_DOWN, LOW);
            g_wifi.LogMotorChange(true, _targetState.lastChange, _targetState.mode);
            COROUTINE_DELAY(VIBRATOR_PUSH_DELAY);
          }
          g_wifi.Resume();
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

void Vibrator::vibrator_up() { if (_targetState.mode<MAX_LEVEL) {  _targetState.mode++; _targetState.lastChange=millis(); } }
void Vibrator::vibrator_down() { if (_targetState.mode>0) { _targetState.mode--;  _targetState.lastChange=millis();} }
void Vibrator::vibrator_off() { _targetState.powered = false;  _targetState.lastChange=millis(); }
void Vibrator::vibrator_reset() { _targetState.mode=0; _currentState.mode=MAX_LEVEL;  _targetState.lastChange=millis(); }
void Vibrator::vibrator_on() { if (!_targetState.powered) { _targetState.powered = true;  _targetState.lastChange=millis(); } }
void Vibrator::vibrator_to(const int mode) 
{ 
  if(mode > 0)
  {
    _targetState.mode = clamp<int>(mode-1, 0, MAX_LEVEL); 
    _targetState.lastChange=millis();
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
      g_raOverallPressureVariance.clear();
      g_PressureVariance = 0.0f;
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

          //update std deviation
          g_raOverallPressureVariance.push(g_currentPressure);
          const auto raOverallPressureVarianceCount = g_raOverallPressureVariance.size();
          const float raOverallPressureVarianceAvg = g_raOverallPressureVariance.get();
          float pressureVariance = 0.0f;
          if (raOverallPressureVarianceCount >= 2)
          {
            for (int i=0; i < raOverallPressureVarianceCount; i++)
            {
              pressureVariance += pow(g_raOverallPressureVariance[i] - raOverallPressureVarianceAvg, 2);
            }
            g_PressureVariance = sqrt(pressureVariance/float(raOverallPressureVarianceCount-1));
          }

          //Only update our overall presure if it's not being influenced by the vibe.
          //Or, if it lowers the average pressure...
           if ((!g_vibrator.isVibratorOn() && g_logic.getState() != MachineStates::stabilize) || g_currentPressure < g_raOverallPressure.get())
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
/////////////////////////////////////////////////////
// Logic
int Logic::sampleTick = 0;
int Logic::runCoroutine()
    {
      COROUTINE_LOOP()
      {              
        fadeToBlackBy(leds,NUM_LEDS,20); //Create a fading light effect. LED buffer is not otherwise cleared
        _physBtnState = check_button();
        if (_physBtnState == PhysBtnState::Long && getState() != MachineStates::menu)
        {
          change_state(MachineStates::menu);
        }

        run_state_machine(getState());
        FastLED.show(); //Update the physical LEDs to match the buffer in software

        log_session();

        COROUTINE_YIELD();
      }
    }

void Logic::SaveSettings()
{
  EEPROM.put<int16_t>(AUTOEDGE_TIME_ADDR, _automatic_edge_time_minutes);
  EEPROM.put<int16_t>(AUTOEDGE_MIN_ADDR, _automatic_edge_mintarget);
  EEPROM.put<int16_t>(AUTOEDGE_MAX_ADDR, _automatic_edge_maxtarget);
  EEPROM.put<int16_t>(EEPROM_DEFAULT_COOLDOWN_s_ADDR, _automatic_edge_INITIALCOOLDOWN_s);
  EEPROM.put<int16_t>(EEPROM_DEFAULT_COOLDOWN_ADJUSTMENT_FASTER_s_ADDR, _automatic_edge_COOLDOWN_ADJUSTMENT_FASTER_s);
  EEPROM.put<int16_t>(EEPROM_DEFAULT_COOLDOWN_ADJUSTMENT_SLOWER_s_ADDR, _automatic_edge_COOLDOWN_ADJUSTMENT_SLOWER_s);
  EEPROM.put<int16_t>(EEPROM_DEFAULT_EDGETIME_TARGET_s_ADDR, _automatic_edge_EDGETIME_TARGET_s);
  EEPROM.put<int16_t>(EEPROM_DEFAULT_EDGETIME_TARGET_MIN_S_ADDR, _automatic_edge_EDGETIME_TARGET_MIN_S);
  EEPROM.put<int16_t>(EEPROM_DEFAULT_EDGETIME_TARGET_MAX_s_ADDR, _automatic_edge_EDGETIME_TARGET_MAX_s);
  EEPROM.put<int16_t>(EEPROM_DEFAULT_MIN_COOLDOWN_s_ADDR, _automatic_edge_MIN_COOLDOWN_s);
  EEPROM.put<int16_t>(EEPROM_DEFAULT_MAX_COOLDOWN_s_ADDR, _automatic_edge_MAX_COOLDOWN_s);
}

void Logic::PushState(MachineStates newState)
{
  //Don't call change_state for push/pop state!
   _state++; 
   *_state = newState; 
}

void Logic::PopState() 
 { 
  _state--; 
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
  g_PressureVarianceMaximum = clamp(EEPROM.get<float>(PRESSUREVARMAX_ADDR, g_PressureVarianceMaximum), float(0.0f), float(50.0f));
  _automatic_edge_INITIALCOOLDOWN_s = clamp(EEPROM.get<int16_t>(EEPROM_DEFAULT_COOLDOWN_s_ADDR, _automatic_edge_INITIALCOOLDOWN_s), int16_t(0), int16_t(120));
  _automatic_edge_COOLDOWN_ADJUSTMENT_FASTER_s = clamp(EEPROM.get<int16_t>(EEPROM_DEFAULT_COOLDOWN_ADJUSTMENT_FASTER_s_ADDR, _automatic_edge_COOLDOWN_ADJUSTMENT_FASTER_s), int16_t(0), int16_t(120));
  _automatic_edge_COOLDOWN_ADJUSTMENT_SLOWER_s = clamp(EEPROM.get<int16_t>(EEPROM_DEFAULT_COOLDOWN_ADJUSTMENT_SLOWER_s_ADDR, _automatic_edge_COOLDOWN_ADJUSTMENT_SLOWER_s), int16_t(0), int16_t(120));
  _automatic_edge_EDGETIME_TARGET_s = clamp(EEPROM.get<int16_t>(EEPROM_DEFAULT_EDGETIME_TARGET_s_ADDR, _automatic_edge_EDGETIME_TARGET_s), int16_t(0), int16_t(120));
  _automatic_edge_EDGETIME_TARGET_MIN_S = clamp(EEPROM.get<int16_t>(EEPROM_DEFAULT_EDGETIME_TARGET_MIN_S_ADDR, _automatic_edge_EDGETIME_TARGET_MIN_S), int16_t(0), int16_t(120));
  _automatic_edge_EDGETIME_TARGET_MAX_s = clamp(EEPROM.get<int16_t>(EEPROM_DEFAULT_EDGETIME_TARGET_MAX_s_ADDR, _automatic_edge_EDGETIME_TARGET_MAX_s), int16_t(0), int16_t(120));
  _automatic_edge_MIN_COOLDOWN_s = clamp(EEPROM.get<int16_t>(EEPROM_DEFAULT_MIN_COOLDOWN_s_ADDR, _automatic_edge_MIN_COOLDOWN_s), int16_t(0), int16_t(120));
  _automatic_edge_MAX_COOLDOWN_s = clamp(EEPROM.get<int16_t>(EEPROM_DEFAULT_MAX_COOLDOWN_s_ADDR, _automatic_edge_MAX_COOLDOWN_s), int16_t(0), int16_t(120));

  _currentMenuItem = &g_rootMenu[1];
  _currentMenuItemState.isFirst = true;
  _currentMenuItemState.isLast = false;
  change_state(MachineStates::menu);
}

void Logic::resetSession()
{
#if (defined(EDGEALGORITHM) && EDGEALGORITHM == 1)
  _cooldown = _automatic_edge_INITIALCOOLDOWN_s;
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
          _writeVariableCallback = [](const int16_t* pWrittenVariable){ g_logic.SaveSettings(); };
          change_state(MachineStates::read_variable); 
        } break;
        case MenuItemsEnum::automatic_input_min: { 
          _readVariable = &_automatic_edge_mintarget;
          _readVariableMin = 0;
          _readVariableMax = 1500;
          _writeVariableCallback = [](const int16_t* pWrittenVariable){ g_logic.SaveSettings(); };
          change_state(MachineStates::read_variable); 
        } break;
        case MenuItemsEnum::automatic_input_max: { 
          _readVariable = &_automatic_edge_maxtarget;
          _readVariableMin = 0;
          _readVariableMax = 1500;
          _writeVariableCallback = [](const int16_t* pWrittenVariable){ g_logic.SaveSettings(); };
          change_state(MachineStates::read_variable); 
        } break;
        case MenuItemsEnum::automatic_input_edge_INITIALCOOLDOWN_s: {
          _readVariable = &_automatic_edge_INITIALCOOLDOWN_s;
          _readVariableMin = 0;
          _readVariableMax = 120;
          _writeVariableCallback = [](const int16_t* pWrittenVariable){ g_logic.SaveSettings(); };
          change_state(MachineStates::read_variable);
        } break;
        case MenuItemsEnum::automatic_input_edge_COOLDOWN_ADJUSTMENT_FASTER_s: {
          _readVariable = &_automatic_edge_COOLDOWN_ADJUSTMENT_FASTER_s;
          _readVariableMin = 0;
          _readVariableMax = 120;
          _writeVariableCallback = [](const int16_t* pWrittenVariable){ g_logic.SaveSettings(); };
          change_state(MachineStates::read_variable);
        } break;
        case MenuItemsEnum::automatic_input_edge_COOLDOWN_ADJUSTMENT_SLOWER_s: {
          _readVariable = &_automatic_edge_COOLDOWN_ADJUSTMENT_SLOWER_s;
          _readVariableMin = 0;
          _readVariableMax = 120;
          _writeVariableCallback = [](const int16_t* pWrittenVariable){ g_logic.SaveSettings(); };
          change_state(MachineStates::read_variable);
        } break;
        case MenuItemsEnum::automatic_input_edge_EDGETIME_TARGET_s: {
          _readVariable = &_automatic_edge_EDGETIME_TARGET_s;
          _readVariableMin = 0;
          _readVariableMax = 120;
          _writeVariableCallback = [](const int16_t* pWrittenVariable){ g_logic.SaveSettings(); };
          change_state(MachineStates::read_variable);
        } break;
        case MenuItemsEnum::automatic_input_edge_EDGETIME_TARGET_MIN_S: {
          _readVariable = &_automatic_edge_EDGETIME_TARGET_MIN_S;
          _readVariableMin = 0;
          _readVariableMax = 120;
          _writeVariableCallback = [](const int16_t* pWrittenVariable){ g_logic.SaveSettings(); };
          change_state(MachineStates::read_variable);
        } break;
        case MenuItemsEnum::automatic_input_edge_EDGETIME_TARGET_MAX_s: {
          _readVariable = &_automatic_edge_EDGETIME_TARGET_MAX_s;
          _readVariableMin = 0;
          _readVariableMax = 120;
          _writeVariableCallback = [](const int16_t* pWrittenVariable){ g_logic.SaveSettings(); };
          change_state(MachineStates::read_variable);
        } break;
        case MenuItemsEnum::automatic_input_edge_MIN_COOLDOWN_s: {
          _readVariable = &_automatic_edge_MIN_COOLDOWN_s;
          _readVariableMin = 0;
          _readVariableMax = 120;
          _writeVariableCallback = [](const int16_t* pWrittenVariable){ g_logic.SaveSettings(); };
          change_state(MachineStates::read_variable);
        } break;
        case MenuItemsEnum::automatic_input_edge_MAX_COOLDOWN_s: {
          _readVariable = &_automatic_edge_MAX_COOLDOWN_s;
          _readVariableMin = 0;
          _readVariableMax = 120;
          _writeVariableCallback = [](const int16_t* pWrittenVariable){ g_logic.SaveSettings(); };
          change_state(MachineStates::read_variable);
        } break;
        case MenuItemsEnum::automatic_input_stabilization_maximum: {
          static int16_t dummy;
          dummy=(int16_t)g_PressureVarianceMaximum;
          _readVariable = &dummy;
          _readVariableMin = 0;
          _readVariableMax = 1500;
          _writeVariableCallback = [](const int16_t* pWrittenVariable){ 
            g_PressureVarianceMaximum=(float)*pWrittenVariable; 
            EEPROM.put<float>(PRESSUREVARMAX_ADDR, g_PressureVarianceMaximum);
            Serial.println("Stored PRESSUREVARMAX_ADDR");
          };
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
    if (_writeVariableCallback != nullptr)
    {
      _writeVariableCallback(&var);
    }
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
  if ( !(now < _lastEdgeTime + static_cast<uint16_t>(_automatic_edge_MIN_COOLDOWN_s*0.5f*1000.0f)) && 
        (now < _lastMotorEnableTimeMs + static_cast<uint16_t>(_automatic_edge_EDGETIME_TARGET_MIN_S*1.1f*1000.0f)) )
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
      const float diff = edgeTime - _automatic_edge_EDGETIME_TARGET_s;

      DEBUG_ONLY(Serial.print(F("Adjusted cooldown: ")));
      DEBUG_ONLY(Serial.print(_cooldown));
      DEBUG_ONLY(Serial.print(F("s -> ")));
      if (diff >= 0.0f)
      { //Too slow
        const float x = clamp<float>(diff / _automatic_edge_EDGETIME_TARGET_MAX_s, 0.0f, 1.0f);
        const float adjustment = ((pow(x-1.0f, 3.0f))+1.0f) * _automatic_edge_COOLDOWN_ADJUSTMENT_FASTER_s;
        _cooldown -= adjustment;
      } else if (diff < 0.0f)
      { //Too fast!
        const float x = clamp<float>(1.0f - (edgeTime/_automatic_edge_EDGETIME_TARGET_s), 0.0f, 1.0f);
        _cooldown += (((pow(x-1.0f, 3.0f))+1.0f) * _automatic_edge_COOLDOWN_ADJUSTMENT_SLOWER_s);
      }
      _cooldown = clamp<float>(_cooldown, _automatic_edge_EDGETIME_TARGET_MIN_S, _automatic_edge_EDGETIME_TARGET_MAX_s);
      _lastEdgeTime = millis();
      DEBUG_ONLY(Serial.print(_cooldown));
      DEBUG_ONLY(Serial.println(F("s.")));
      DEBUG_ONLY(Serial.flush());
      #endif
      g_vibrator.vibrator_off();
      g_vibrator.vibrator_reset();
      edgeCount++;
      g_lastEdgeCountChange = millis();
      g_wifi.LogEdgeChange(edgeCount);
      PushState(MachineStates::stabilize);
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

//wait for our running average to stabilize
void Logic::run_stabilize()
{
  const auto now = millis();
  if (g_PressureVariance <= g_PressureVarianceMaximum && (now-_lastEdgeTime) > _automatic_edge_EDGETIME_TARGET_MIN_S*1000)
  {
    PopState();
  }
  else
  {
    g_vibrator.vibrator_off();
  }
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
  if(_hasSession && (lastTick+250 < now))
  {
    lastTick = now;
    g_wifi.LogSessionTick(
        {int32_t(g_motSpeed), 
        int32_t(g_currentPressure), 
        int32_t(g_avgPressure), 
        int32_t(g_currentPressure - g_avgPressure), 
        int32_t(pLimit), 
        int32_t(g_logic.GetCooldown()*1000.0f),
        int32_t(g_PressureVariance),
        int32_t(g_PressureVarianceMaximum)}
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
    case MachineStates::stabilize:
    {
      update_current_status();
      run_stabilize();
    } break;
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
  auto oldState = getState();

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
      case MachineStates::stabilize:
      {
        //handled in push/pop state
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
      case MachineStates::stabilize:
      {
        //handled in push/pop state
      } break;
      default: g_lcd.RenderPanic(F("State not implemented!")); break;
    }

    *_state = newState;
    return *_state;
  }
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
  COROUTINE_LOOP()
  {
    Update();

    if(!ShouldConnect())
    {
      return 0;
    }

    //Make sure we're always connected to the WIFI AP
    #if 1
    static uint64_t timeout = 1000;
    static auto status = wl_status_t::WL_DISCONNECTED;
    {
      while (status != WL_CONNECTED)
      {
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
          
          COROUTINE_DELAY(1000*1);
        }
      }
    }
    #endif

    status = static_cast<wl_status_t>(WiFi.status());
    if (status == WL_CONNECTED)
    {
      if (_resetSession)
      {
        internalRequestSession();

        //wait until we receive session packet (there are no other packets)
        static auto beginTime = millis();
        beginTime = millis();
        
        _connected = false;
        while(millis() <= beginTime+(1*1000))
        {
          int packetSize = _udpclient.parsePacket();
          if (packetSize>0 && _resetSession && _udpclient.remotePort()==NOGASM_LOGGER_PORT)
          {
            _session = _udpclient.readString().c_str();
            _resetSession = false;
            _connected = true;
            break;
          }
          COROUTINE_YIELD();
        }
      }
      else
      {
        //send log data:
        while(!_buffer.isEmpty())
        {
          internalSendLog(_buffer.shift());
          COROUTINE_YIELD();
        }

        int packetSize = _udpclient.parsePacket();
        if (packetSize > 0)
        {
          _lastPing = millis();
          _udpclient.flush();
          COROUTINE_YIELD();
        }
        _connected = (_lastPing+5000) > millis();
      }
    }
    
    COROUTINE_YIELD();
  }
}

bool WIFI::IsSuspended() const
{
  return g_vibrator.IsChanging() || super::IsSuspended();
}

void WIFI::Setup()
{
  _onlineMode = EEPROM.get(EEPROM_ONLINEMODE_ADDR, _onlineMode);
  Serial.println((_onlineMode!=0) ? "[STARTUP] We want to be online" : "[STARTUP] We want to be offline");

  _selectedHost = std::clamp(EEPROM.get(EEPROM_SELECTEDHOST_ADDR, _selectedHost), int16_t(0), int16_t(std::size(NOGASM_LOGGER_HOST)));
  Serial.print("[STARTUP] SelectedHost=");
  Serial.println(_selectedHost); 

  _udpclient.begin(NOGASM_SESSION_PORT);
}

void WIFI::Update()
{
  static auto lastSelectedHost = _selectedHost;
  if (lastSelectedHost != _selectedHost)
  {
    EEPROM.put(EEPROM_SELECTEDHOST_ADDR, _selectedHost);
    lastSelectedHost = _selectedHost;
  }
}

void WIFI::internalRequestSession() 
{
  if (_udpclient.beginPacket(NOGASM_LOGGER_HOST[g_wifi.GetSelectedHost()], NOGASM_LOGGER_PORT) == 1)
  {
    _udpclient.write(1);
    if (_udpclient.endPacket() != 1) {
      Serial.println("packet failed to send");
    }
  } 
  else
  {
    Serial.print("error: could not resolve udp host/port ");
    Serial.print(NOGASM_LOGGER_HOST[g_wifi.GetSelectedHost()].toString());
    Serial.print(":");
    Serial.print(NOGASM_LOGGER_PORT);
    Serial.print(", wifi code: ");
    Serial.println(WiFi.status());
    Serial.println(_udpclient.available() ? "udpclient is available" : "udpclient is unavailable");
    Serial.println(_udpclient.availableForWrite() ? "udpclient is write-available" : "udpclient is not write available");
    Serial.print("write error: "); Serial.println(_udpclient.getWriteError());
    _udpclient.flush();
    _udpclient.clearWriteError();
    _udpclient.stop();
    Serial.println(_udpclient.begin(51337)==1 ? "UDPClient reset" : "UDPClient Reset Failed!!");
  }
};

void WIFI::internalRequestSessionResponse()
{
}

void WIFI::internalSendLog(const internalLog& logData)
{
  ProfileBlock(50, "Wifi sendlog");
  
  static arduino::String body = [](){
    arduino::String staticString;
    staticString.reserve(2000);
    return staticString;
  }();

  static arduino::String s = [](){
    arduino::String staticString;
    staticString.reserve(2000);
    return staticString;
  }();

  body="{"
                    "\"channel\": "; body += logData.channel; body += ","
                    "\"session\": \""; body += _session.c_str(); body += "\","
                    "\"time\": "; body+=static_cast<unsigned long>(logData.time-_sessionStartTime); body+=","
                    "\"param1\": "; body+=logData.data.param1; body+=","
                    "\"param2\": "; body+=logData.data.param2; body+=","
                    "\"param3\": "; body+=logData.data.param3; body+=","
                    "\"param4\": "; body+=logData.data.param4; body+=","
                    "\"param5\": "; body+=logData.data.param5; body+=","
                    "\"param6\": "; body+=logData.data.param6; body+=","
                    "\"param7\": "; body+=logData.data.param7; body+=","
                    "\"param8\": "; body+=logData.data.param8;
  body+="}";

  if (_udpclient.beginPacket(NOGASM_LOGGER_HOST[g_wifi.GetSelectedHost()], NOGASM_SESSION_PORT) == 1)
  {
    _udpclient.write(body.c_str());
    if (_udpclient.endPacket() != 1) {
      Serial.println("packet failed to send");
    }
  } 
  else
  {
    Serial.print("error: could not resolve udp host/port ");
    Serial.print(NOGASM_LOGGER_HOST[g_wifi.GetSelectedHost()].toString());
    Serial.print(":");
    Serial.print(NOGASM_LOGGER_PORT);
    Serial.print(", wifi code: ");
    Serial.println(WiFi.status());
    Serial.println(_udpclient.available() ? "udpclient is available" : "udpclient is unavailable");
    Serial.println(_udpclient.availableForWrite() ? "udpclient is write-available" : "udpclient is not write available");
    Serial.print("write error: "); Serial.println(_udpclient.getWriteError());
    _udpclient.flush();
    _udpclient.clearWriteError();
    _udpclient.stop();
    Serial.println(_udpclient.begin(51337)==1 ? "UDPClient reset" : "UDPClient Reset Failed!!");
  }
}

void WIFI::LogEndSession() 
{
   _buffer.clear();
   _session.clear();
}

void WIFI::LogSessionTick(const LogData& logData) 
{
   if (!_buffer.isFull() && !_session.empty())
   {
     internalLog log;
     log.time = millis();
     log.data = logData;
     log.channel = 0;
     _buffer.push(std::move(log));
   }
}

void WIFI::LogMotorChange(bool pNewMotorState, uint32_t pRequestTime, uint8_t pMotorLevel)
{
   if (!_buffer.isFull() && !_session.empty())
   {
     const int32_t now{millis()};
     internalLog log;
     log.time = now;
     log.data.param1 = (pNewMotorState ? 1 : 0);
     log.data.param2 = pMotorLevel;
     log.data.param3 = (now - static_cast<int32_t>(pRequestTime));
     log.channel = 1;
     _buffer.push(std::move(log));
   }
}

void WIFI::LogEdgeChange(uint16_t pEdgeCount)
{
   if (!_buffer.isFull() && !_session.empty())
   {
     internalLog log;
     log.time = millis();
     log.data.param1 = pEdgeCount;
     log.channel = 2;
     _buffer.push(std::move(log));
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

  return true;
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
#else
  if (!g_sensor.IsSuspended()) g_sensor.runCoroutine(); 
  if (!g_logic.IsSuspended()) g_logic.runCoroutine();
  if (!g_lcd.IsSuspended()) g_lcd.runCoroutine();
  if (!g_leds.IsSuspended()) g_leds.runCoroutine();
  if (!g_vibrator.IsSuspended()) g_vibrator.runCoroutine();
  if (!g_wifi.IsSuspended()) g_wifi.runCoroutine();
#endif
#endif
}