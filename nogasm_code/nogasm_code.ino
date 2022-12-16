#pragma GCC push_options
#pragma GCC optimize ("Os")

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
#include <Encoder.h>
#include <EEPROM.h>
#include "FastLED.h"
#include <MovingAveragePlus.h> //using custom version from https://github.com/trilithus/MovingAveragePlus
#include <Arduino.h>
#include <AceRoutine.h>
#include "commandmanager.h"
#include "sharedcommands.h"
#include <avr/pgmspace.h>
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
Encoder myEnc(3, 2); //Quadrature inputs
#define ENC_SW_UP   HIGH
#define ENC_SW_DOWN LOW

//Motor
constexpr uint8_t PIN_VIBRATOR_ONOFF = 12;
constexpr uint8_t PIN_VIBRATOR_DOWN = 8;
constexpr uint8_t PIN_VIBRATOR_UP = 13;
constexpr uint8_t VIBRATOR_PUSH_DELAY = 150;

//Storage
#define I2C_PROTOCOL 0

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

int g_currentPressure = 0;
int g_avgPressure = 0; //Running 25 second average pressure
//int bri =100; //Brightness setting
int rampTimeS = 150; //120; //Ramp-up time, in seconds
float s_cooldownTime = 0.40f;//0.5f;
#define DEFAULT_PLIMIT 900
int pLimit = DEFAULT_PLIMIT; //Limit in change of pressure before the vibrator turns off
int pLimitLast = DEFAULT_PLIMIT;
int g_MaxSpeed = 255; //maximum speed the motor will ramp up to in automatic mode
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
  resetSession,
  back,
  LAST,
};

MenuItemsEnum g_rootMenu[] {
  MenuItemsEnum::INVALID,
  MenuItemsEnum::test,
  MenuItemsEnum::manual,
  MenuItemsEnum::automatic,
  MenuItemsEnum::resetSession,
  MenuItemsEnum::LAST,
};

MenuItemsEnum g_automaticMenu[] {
  MenuItemsEnum::INVALID,
  MenuItemsEnum::automatic_start,
  MenuItemsEnum::automatic_continue,
  MenuItemsEnum::automatic_input_time,
  MenuItemsEnum::automatic_input_min,
  MenuItemsEnum::automatic_input_max,
  MenuItemsEnum::automatic_back,
  MenuItemsEnum::LAST,
};

const __FlashStringHelper& getMenuItemLabels(MenuItemsEnum pValue, String& output)
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
    case MenuItemsEnum::resetSession: output = F("Reset Session"); return;
    case MenuItemsEnum::LAST: output = F("!ERROR!"); return;
  }
}

//Utility
template <typename T>
T clamp(const T val, const T minVal, const T maxVal) { return (val<minVal) ? minVal : (val>maxVal ? maxVal : val); };

//======= DEBUG ===============================
#define DEBUG_LOG_ENABLED 0
#define PROFILING_ENABLED 0

#if defined(DEBUG_LOG_ENABLED) && DEBUG_LOG_ENABLED!=0
#define DEBUG_ONLY(...) __VA_ARGS__
#else
#define DEBUG_ONLY(...) void(0)
#endif
void logI2C(String& output)
{
  /*
  output = "";
  output += g_tick;//(millis() / 1000.0); //Timestamp (s)
  output += (",");
  output += (g_motSpeed); //Motor speed (0-255)
  output += (",");
  output += (g_pressure); //(Original ADC value - 12 bits, 0-4095)
  output += (",");
  output += (avgPressure); //Running average of (default last 25 seconds) pressure
  output += (",");
  output += (g_pressure - avgPressure);
  output += (",");
  output += (pLimit);
  output += (F(",\r\n"));
  */
}

//=======EEPROM Addresses============================
//128b available on teensy LC
#define BEEP_ADDR         1
#define MAX_SPEED_ADDR    2
#define SENSITIVITY_ADDR  3
#define AUTOEDGE_MIN_ADDR 4 //4,5
#define AUTOEDGE_MAX_ADDR 6 //6,7
#define AUTOEDGE_TIME_ADDR 8 //8,9

//#define RAMPSPEED_ADDR    4 //For now, ramp speed adjustments aren't implemented

//=======Hardware Coroutines=========================
class I2C : public Coroutine
{
  public:
    void Setup();
    void Update();

    virtual int runCoroutine() override;
    void SendI2CCommand(Command* cmd);

    uint32_t GetLastPingValue() const {return _lastPingValue; }
    unsigned long GetLastPing() const { return _lastPing; }
  private:
    uint32_t _lastPingValue = 0;
    unsigned long _lastPing = 0;
} g_i2c;

class Logic : Coroutine
{
  protected:
    void run_menu();
    void run_read_variable();
    void run_automatic_edge_control();
    void run_edge_control();
    void run_manual_motor_control();
    void update_current_status();
    PhysBtnState check_button();
    void run_state_machine(MachineStates state);
    MachineStates Logic::change_state(MachineStates newState);
    void resetSession();
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


/////////////////////////////////////////////////////
// LCD
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

  static constexpr uint8_t buffersz = max(max(sizeof(bitmaps::connected_bits), sizeof(bitmaps::heart_bits)),
                                                                       sizeof(bitmaps::heart_bits_crossed));

  uint8_t realBuffer[buffersz];
  uint8_t* buffer = &realBuffer[0]; //(uint8_t*)malloc(buffersz);

  //vibe active or not:
  for(uint8_t i=0; i < sizeof(bitmaps::heart_bits); i++)
  {
    buffer[i] = pgm_read_byte_near((g_motSpeed >= 0.01f ? bitmaps::heart_bits : bitmaps::heart_bits_crossed) +i);
  }
  u8g2.drawXBM(u8g2.getDisplayWidth() - heart_width, 0, heart_width, heart_height, buffer);

  if (millis() < g_i2c.GetLastPing()+1000 && g_i2c.GetLastPingValue()>0)
  {
    for(uint8_t i=0; i < sizeof(bitmaps::connected_bits); i++)
    {
      buffer[i] = pgm_read_byte_near(bitmaps::connected_bits + i);
    }
    u8g2.drawXBM(u8g2.getDisplayWidth() - heart_width, heart_height, heart_width, heart_height, buffer);
  }

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
      analogReference(INTERNAL);//(EXTERNAL);
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
        COROUTINE_YIELD();
      }
    }

void Logic::Setup()
{
  pinMode(ENC_SW,   INPUT); //Pin to read when encoder is pressed
  digitalWrite(ENC_SW, HIGH); // Encoder switch pullup

  //Storage:
  sensitivity = EEPROM.read(SENSITIVITY_ADDR);
  g_MaxSpeed = min(EEPROM.read(MAX_SPEED_ADDR), MOT_MAX); //Obey the MOT_MAX the first power  cycle after chaning it.

    

  _automatic_edge_time_minutes = clamp(EEPROM.get<int16_t>(AUTOEDGE_TIME_ADDR, _automatic_edge_time_minutes), int16_t(0), int16_t(1800));
  _automatic_edge_mintarget = clamp(EEPROM.get<int16_t>(AUTOEDGE_MIN_ADDR, _automatic_edge_mintarget), int16_t(0), int16_t(1000));
  _automatic_edge_maxtarget = clamp(EEPROM.get<int16_t>(AUTOEDGE_MAX_ADDR, _automatic_edge_maxtarget), int16_t(0), int16_t(1000));

  _currentMenuItem = &g_rootMenu[1];
  _currentMenuItemState.isFirst = true;
  _currentMenuItemState.isLast = false;
  change_state(MachineStates::menu);
  resetSession();
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
    SessionBeginEndCmd cmd;
    cmd.SetValue(0);
    g_i2c.SendI2CCommand(&cmd);
    delay(100);
  }

  SessionBeginEndCmd cmd;
  cmd.SetValue(1);
  g_i2c.SendI2CCommand(&cmd);
  _hasSession = true;
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
        case MenuItemsEnum::automatic_back: { _currentMenuItem = &g_rootMenu[1]; } break;
        case MenuItemsEnum::resetSession: { resetSession(); }; break;
      }
  }

  _currentMenuItemState.isFirst = (*_currentMenuItem == MenuItemsEnum::INVALID) || (*(_currentMenuItem-1) == MenuItemsEnum::INVALID);
  _currentMenuItemState.isLast = *(_currentMenuItem+1) == MenuItemsEnum::LAST;

  const auto knob = myEnc.read();
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

  const auto knob = myEnc.read();
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
  static float motIncrement = 0.0;
  motIncrement = ((float)g_MaxSpeed / ((float)FREQUENCY * (float)rampTimeS));

  const int knob = encLimitRead(0,(3*NUM_LEDS)-1);
  sensitivity = knob*4; //Save the setting if we leave and return to this state
  //Reverse "Knob" to map it onto a pressure limit, so that it effectively adjusts sensitivity 
  pLimit = max(0, map(knob, 0, 3 * (NUM_LEDS - 1), DEFAULT_PLIMIT, 1)); //set the limit of delta pressure before the vibrator turns off
  pLimit += (int)_autoEdgeTargetOffset; 

  if (pLimit != pLimitLast)
  {
    pLimitLast = pLimit;
    LCD::_lastTargetChange = millis();
  }
  //When someone clenches harder than the pressure limit
  if (g_currentPressure - g_avgPressure > pLimit) {
    if (g_motSpeed > 0.0f)
    {
      #if (defined(EDGEALGORITHM) && EDGEALGORITHM==1)
      //Adjust cooldown
      const float edgeTime = static_cast<float>(millis()-_lastEdgeTime) / 1000.0f;
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
    
  }
  else if (g_motSpeed < (float)g_MaxSpeed) {
    g_motSpeed += motIncrement;
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
        EEPROM.update(SENSITIVITY_ADDR, sensitivity);
      } break;
      case MachineStates::auto_edge_control: 
      {
        EEPROM.update(SENSITIVITY_ADDR, sensitivity);
        _autoEdgeTargetOffset = 0;
      } break;
      default: g_lcd.RenderPanic(F("State not implemented!")); break;
    }

    switch(newState)
    {
      case MachineStates::UNDEFINED: {} break;
      case MachineStates::menu: 
      {
        _menuKnobState = myEnc.read();
        g_vibrator.vibrator_off();
      } break;
      case MachineStates::read_variable: {} break;
      case MachineStates::manual_motor_control: 
      {
        g_vibrator.vibrator_reset();
        g_vibrator.vibrator_on();
      } break;
      case MachineStates::manual_edge_control: 
      {
        //Restore last sensitivity value:
        sensitivity = EEPROM.read(SENSITIVITY_ADDR);
        myEnc.write(sensitivity);
        g_vibrator.vibrator_reset();
        g_vibrator.vibrator_on();
      } break;
      case MachineStates::auto_edge_control: 
      {
        //Store target settings in eeprom:
        EEPROM.put<int16_t>(AUTOEDGE_TIME_ADDR, _automatic_edge_time_minutes);
        EEPROM.put<int16_t>(AUTOEDGE_MIN_ADDR, _automatic_edge_mintarget);
        EEPROM.put<int16_t>(AUTOEDGE_MAX_ADDR, _automatic_edge_maxtarget);
        if (_autoTimeStart==0)
        {
          _autoTimeStart = millis();
        }

        //Restore last sensitivity value:
        sensitivity = EEPROM.read(SENSITIVITY_ADDR);
        myEnc.write(sensitivity);
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
        myEnc.write(sensitivity);//Whenever going into auto mode, keep the last sensitivity
        g_motSpeed = 0; //Also reset the motor speed to 0
        return MachineStates::manual_edge_control;
      case MachineStates::manual_edge_control:
        myEnc.write(0);//Whenever going into manual mode, set the speed to 0.
        g_motSpeed = 0;
        EEPROM.update(SENSITIVITY_ADDR, sensitivity);
        return MachineStates::manual_motor_control;
    }
  }
  else if(btnState == BTN_LONG){
    switch (state) {
      case MachineStates::manual_motor_control:
        myEnc.write(map(g_MaxSpeed, 0, 255, 0, 4 * (NUM_LEDS))); //start at saved value
        return MachineStates::manual_edge_control;//OPT_SPEED;
      case MachineStates::manual_edge_control:
        myEnc.write(map(g_MaxSpeed, 0, 255, 0, 4 * (NUM_LEDS))); //start at saved value
        return MachineStates::manual_motor_control;//OPT_SPEED;
    }
  }
  else return MachineStates::manual_motor_control;
  #endif
}

// I2C

void I2C::Setup()
{
  pinMode(11, OUTPUT);
  digitalWrite(11, LOW);
  Wire.begin();
  Wire.setClock(3400000);
  //pinMode(11, LOW);
  //Wire.setTimeout(1000);
  //Wire.setWireTimeout(1000, false);
}

void I2C::Update()
{
}

int I2C::runCoroutine()
{
  #if 1
  COROUTINE_LOOP()
  {
    static CSVLogCommand *cmd = nullptr;
    cmd = new CSVLogCommand();
    if (cmd != nullptr)
    {
      cmd->LogCSV(
        g_tick, 
        int32_t(g_motSpeed), 
        int32_t(g_currentPressure), 
        int32_t(g_avgPressure), 
        int32_t(g_currentPressure - g_avgPressure), 
        int32_t(pLimit), 
        int32_t(g_logic.GetCooldown()*1000.0f)
      );
      g_i2c.SendI2CCommand(cmd);
    }
    delete (cmd);
    cmd=nullptr;
    #if 0
    static String i2cstring;
    logI2C(i2cstring);
    #if defined(DEBUG_LOG_ENABLED) && DEBUG_LOG_ENABLED==1
    Serial.print(i2cstring);
    #endif

    static PrintCmd *cmd = nullptr;
    cmd = new PrintCmd();
    if (cmd != nullptr)
    {
      cmd->SetString(i2cstring.c_str());
      i2cstring = "";
      g_i2c.SendI2CCommand(cmd);

      delete (cmd);
      cmd = nullptr;
    }
    else
    {
      Serial.println(F("Out of Memory!"));
    }
    #endif


    COROUTINE_DELAY(250);
    COROUTINE_YIELD();
  }
  #endif
}

void I2C::SendI2CCommand(Command* cmd)
{
  digitalWrite(11, HIGH);
  #if defined(I2C_PROTOCOL) && I2C_PROTOCOL==0
  //Request a data to allow the logger to be ready, and then make sure
  //we wait.
  //Wire.setClock(35);
  Wire.setClock(3400000);
  auto wired = Wire.requestFrom(0x43, 1);
  if (wired == 0)
  {
    return;
  }
  while(Wire.available())
  {
    _lastPingValue = Wire.read();
    _lastPing = millis();
  }
  #endif

  //Start submitting the command:
  //Wire.setClock(16000000);
  Wire.beginTransmission(0x43); /* begin with device address 8 */
  int32_t sz = cmd->GetBufferSize();
  #if defined(I2C_PROTOCOL) && I2C_PROTOCOL==1
  Wire.write(0xDEul);
  Wire.write(0xADul);
  Wire.write(0xBEul);
  Wire.write(0xEFul);
  #endif
  Wire.write(static_cast<unsigned int>(cmd->cmdId));
  Wire.write(reinterpret_cast<const uint8_t*>(&sz), sizeof(sz));
  Wire.endTransmission(); 
  const uint8_t* buffer = reinterpret_cast<const uint8_t*>(cmd->GetBuffer());
  do
  {
    Wire.beginTransmission(0x43); 
    const uint8_t written = Wire.write(buffer, clamp<int32_t>(sz, 0, 10));
    sz -= written;
    buffer += written;
    Wire.endTransmission();    /* stop transmitting */
  } while(sz>0);
  digitalWrite(11, LOW);
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
  if(myEnc.read()>maxVal*4)myEnc.write(maxVal*4);
  else if(myEnc.read()<minVal*4) myEnc.write(minVal*4);
  return constrain(myEnc.read()/4,minVal,maxVal);
}

#if defined(PROFILING_ENABLED) && PROFILING_ENABLED==1
using ProfiledFunc = void (*)();
void profile(int storageID, ProfiledFunc profiledFunc)
{
  static constexpr uint8_t profilecount = 6;
  static uint8_t times[profilecount]={};
  static uint8_t maxTime[profilecount]={};
  const auto now = millis();
  profiledFunc();
  const auto diff = millis()-now;
  times[storageID] += diff;
  times[storageID] /= 2;
  maxTime[storageID] = max(maxTime[storageID], min(diff,255));

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
    }
  }
}
#endif

//=======Setup=======================================
void setup() 
{
  Wire.setWireTimeout(50000, true);
  Serial.begin(230400); delay(100); DEBUG_ONLY(Serial.println(F("Serial up"))); Serial.flush();
  g_sensor.Setup();  delay(100); DEBUG_ONLY(Serial.println(F("Sensor up"))); Serial.flush();
  g_logic.Setup();  delay(100); DEBUG_ONLY(Serial.println(F("Logic up"))); Serial.flush();
  g_lcd.Setup();  delay(100); DEBUG_ONLY(Serial.println(F("LCD up"))); Serial.flush();
  g_leds.Setup();  delay(100); DEBUG_ONLY(Serial.println(F("LEDS up"))); Serial.flush();
  g_vibrator.Setup();  delay(100); DEBUG_ONLY(Serial.println(F("Vibrator up"))); Serial.flush();
  g_i2c.Setup();  delay(100); DEBUG_ONLY(Serial.println(F("i2C up"))); Serial.flush();
  Serial.println(F("Started up"));
}

//=======Main Loop=============================
void loop() 
{
  static bool rendered = false;
#if 0
  profile(0, [](){ g_sensor.runCoroutine(); });
  profile(1, [](){ g_logic.runCoroutine(); });
  profile(2, [](){ g_lcd.runCoroutine(); });
  profile(3, [](){ g_leds.runCoroutine(); });
  profile(4, [](){ g_vibrator.runCoroutine(); });
  profile(5, [](){ g_i2c.runCoroutine(); });
#else
  g_sensor.runCoroutine(); 
  g_logic.runCoroutine();
  g_lcd.runCoroutine();
  g_leds.runCoroutine();
  g_vibrator.runCoroutine();
  g_i2c.runCoroutine();
#endif
}