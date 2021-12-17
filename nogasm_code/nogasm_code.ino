#pragma GCC push_options
#pragma GCC optimize ("Os")

// Protogasm Code, forked from Nogasm Code Rev. 3
/* Drives a vibrator and uses changes in pressure of an inflatable buttplug
 * to estimate a user's closeness to orgasm, and turn off the vibrator
 * before that point.
 * A state machine updating at 60Hz creates different modes and option menus
 * that can be identified by the color of the LEDs, especially the RGB LED
 * in the central button/encoder knob.
 * 
 * [Red]    Manual Vibrator Control
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
#include "RunningAverageT.h"
#include <Arduino.h>
#include <AceRoutine.h>
#include "commandmanager.h"
#include "sharedcommands.h"
using namespace ace_routine;

//Running pressure average array length and update frequency
#define RA_HIST_SECONDS 25 //25
#define RA_FREQUENCY 6
#define RA_TICK_PERIOD (FREQUENCY / RA_FREQUENCY)
RunningAverageT<unsigned short, RA_FREQUENCY*RA_HIST_SECONDS> raPressure;

//LCD
#define U8X8_HAVE_HW_I2C
#include <Arduino.h>
#include <U8g2lib.h>
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif



#if 1
#define font_13pt u8g2_font_6x13_tr
#define font_24pt u8g2_font_inb24_mr 
#define font_15pt u8g2_font_6x13_tr 
#define font_symbols u8g2_font_unifont_t_weather
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
#define BRIGHTNESS 10 //Subject to change, limits current that the LEDs draw

//Encoder
#define ENC_SW   5 //Pushbutton on the encoder
Encoder myEnc(3, 2); //Quadrature inputs
#define ENC_SW_UP   HIGH
#define ENC_SW_DOWN LOW

//Motor
constexpr uint8_t PIN_VIBRATOR_ONOFF = 12;
constexpr uint8_t PIN_VIBRATOR_DOWN = 8;
constexpr uint8_t PIN_VIBRATOR_UP = 13;
constexpr uint8_t VIBRATOR_PUSH_DELAY = 100;



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

//Update/render period
#define period (1000/FREQUENCY)
#define longBtnCount (LONG_PRESS_MS / period)

int sensitivity = 0; //orgasm detection sensitivity, persists through different states

//=======State Machine Modes=========================
#define MANUAL      1
#define AUTO        2

//Button states - no press, short press, long press
#define BTN_NONE   0
#define BTN_SHORT  1
#define BTN_LONG   2
#define BTN_V_LONG 3


uint8_t state = MANUAL;
//=======Global Settings=============================
#define MOT_MAX 255 // Motor PWM maximum
//#define MOT_MIN 20  // Motor PWM minimum.  It needs a little more than this to start.

CRGB leds[NUM_LEDS];

int g_pressure = 0;
int avgPressure = 0; //Running 25 second average pressure
//int bri =100; //Brightness setting
int rampTimeS = 120; //Ramp-up time, in seconds
float s_cooldownTime = 0.40f;//0.5f;
#define DEFAULT_PLIMIT 900
int pLimit = DEFAULT_PLIMIT; //Limit in change of pressure before the vibrator turns off
int pLimitLast = DEFAULT_PLIMIT;
int maxSpeed = 255; //maximum speed the motor will ramp up to in automatic mode
float motSpeed = 0; //Motor speed, 0-255 (float to maintain smooth ramping to low speeds)
uint16_t edgeCount = 0;
auto lastEdgeCountChange = millis();

//Utility
static int clamp(const int val, const int minVal, const int maxVal) { return (val<minVal) ? minVal : (val>maxVal ? maxVal : val); };
void beep_motor(int f1, int f2, int f3) {
  /*
    analogWrite(MOTPIN, 0);
    tone(MOTPIN, f1);
    delay(250);
    tone(MOTPIN, f2);
    delay(250);
    tone(MOTPIN, f3);
    delay(250);
    noTone(MOTPIN);
    analogWrite(MOTPIN, motSpeed);
  */
}

//======= DEBUG ===============================
#define DEBUG_LOG_ENABLED 0
void logDebug()
{
#if defined(DEBUG_LOG_ENABLED) && DEBUG_LOG_ENABLED==1
  static bool printed_header = false;
  if (printed_header == false)
  {
    printed_header = true;
    Serial.println(F("Time,Motor,Pressure,Avg Pressure"));
  }
#if defined(DEBUG_OPT_LOGTIMESTAMP) && DEBUG_OPT_LOGTIMESTAMP==1
  Serial.print(millis() / 1000.0); //Timestamp (s)
  Serial.print(F(","));
#endif
  Serial.print(motSpeed); //Motor speed (0-255)
  Serial.print(F(","));
  Serial.print(g_pressure); //(Original ADC value - 12 bits, 0-4095)
  Serial.print(F(","));
  Serial.print(avgPressure); //Running average of (default last 25 seconds) pressure
  Serial.print(F(","));
  Serial.print(g_pressure - avgPressure);
  Serial.print(F(","));
  Serial.print(pLimit);
  Serial.println(F(","));
  
#endif
}

void logI2C(String& output)
{
  output = "";
  output += (millis() / 1000.0); //Timestamp (s)
  output += (F(","));
  output += (motSpeed); //Motor speed (0-255)
  output += (F(","));
  output += (g_pressure); //(Original ADC value - 12 bits, 0-4095)
  output += (F(","));
  output += (avgPressure); //Running average of (default last 25 seconds) pressure
  output += (F(","));
  output += (g_pressure - avgPressure);
  output += (F(","));
  output += (pLimit);
  output += (F(",\r\n"));
}

//=======EEPROM Addresses============================
//128b available on teensy LC
#define BEEP_ADDR         1
#define MAX_SPEED_ADDR    2
#define SENSITIVITY_ADDR  3
//#define RAMPSPEED_ADDR    4 //For now, ramp speed adjustments aren't implemented

//=======Hardware Coroutines=========================
class I2C : public Coroutine
{
    public:
    void Setup();
    void Update();

    virtual int runCoroutine() override;
    void SendI2CCommand(Command* cmd);
} g_i2c;

class Logic : Coroutine
{
  protected:
    void run_auto();
    void run_manual();
    uint8_t check_button();
    void run_state_machine(uint8_t state);
    uint8_t set_state(uint8_t btnState, uint8_t state);
  public:
    void Setup();
    void Update();

    virtual int runCoroutine() override;
  private:
    static int sampleTick;
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
    void vibrator_to(const int mode);
    void set_vibrator_mode(int fromStrength, int toStrength);
    int map_motspeed_to_vibrator();

    virtual int runCoroutine() override;
  protected:
    int8_t getStateDiff() const 
    { 
        int8_t fromStrength = clamp(static_cast<int8_t>(_currentState.mode), 0, MAX_LEVEL);
        int8_t toStrength = clamp(static_cast<int8_t>(_targetState.mode), 0, MAX_LEVEL);
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
    void run_lcd_status(uint8_t state);
  public:
    void Setup();
    void Update();
    virtual int runCoroutine() override;

    template <typename T, typename I, int idx = 0, typename... Iremainder>
    static void setLCDFunc(T func, I arg, Iremainder... args);
    static void lcdfunc_edgeCount();
    static void lcdfunc_renderText();
    static void lcdfunc_renderTarget();
    static void lcdfunc_renderPressure();

    static void RenderPanic(const char* _str);
  public:
    static unsigned long lastTargetChange;
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
} g_lcd;
LCD::lcdrenderdata_t LCD::lcdrenderdata[2]{};
unsigned long LCD::lastTargetChange = millis();
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
  #if 0
  u8g2.setI2CAddress(0x3C*2);
  u8g2.setBusClock(12500);
  Wire.setClock(12500);
  #if 0
  #if 0
  u8g2.setBusClock(35);//200000);
  //Wire.setTimeout(100);
  Wire.setWireTimeout(50000, true);
  #endif

  Wire.setWireTimeout(10000, false);
  #endif
  #endif
  u8g2.begin();

  setLCDFunc(lcdfunc_renderText, __DATE__, __TIME__);
  do {
    lcdrender_activefunc();
  } while( u8g2.nextPage() );
  lcdrender_activefunc = nullptr;
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
    run_lcd_status(state);
  }
}

void LCD::RenderPanic(const char* _str)
{
  LCD::setLCDFunc(LCD::lcdfunc_renderText, _str);
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

void LCD::run_lcd_status(uint8_t state)
{
  static int8_t mode = 0;
  static auto lastChange = millis();
  const auto now = millis();

  if ( (now - lastChange) > 5000 )
  {
    mode++;
    lastChange = now;
  }

  if ( (now-lastTargetChange) < 15000 )
  {
    mode = 1;
  }

  if ( (now - lastEdgeCountChange) < 5000 )
  {
    mode = 2;
  }
  
  switch (mode % 3)
  {
    case 0:
      {
        setLCDFunc(lcdfunc_renderPressure, avgPressure);
      } break;
    case 1:
      {
        setLCDFunc(lcdfunc_renderTarget, max(0, g_pressure - avgPressure), max(0, pLimit));
      } break;
    case 2:
    {
      setLCDFunc(lcdfunc_edgeCount, edgeCount);
    } break;
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

//static auto lastPressureChange = millis();
void LCD::lcdfunc_renderPressure()
{
  u8g2.setFont(font_13pt);
  u8g2.setCursor(0, 20);
  u8g2.print(F("P"));
  u8g2.setFont(font_24pt);
  u8g2.setCursor(10, u8g2.getMaxCharHeight());
  u8g2.print(lcdrenderdata[0].integer32);
}

void LCD::lcdfunc_renderTarget()
{
  u8g2.setFont(font_13pt);
  u8g2.setCursor(0, 20);
  u8g2.print(F("T "));
  u8g2.setFont(font_15pt );
  u8g2.setCursor(10, u8g2.getMaxCharHeight());
  u8g2.print(lcdrenderdata[0].integer32);
  u8g2.setFont(font_13pt);
  u8g2.print(F(" / "));
  u8g2.setFont(font_15pt);
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
  u8g2.setFont(font_13pt);
  int y = u8g2.getMaxCharHeight();
  for (const auto& data : lcdrenderdata)
  {
    if (data.stringptr != nullptr)
    {
      u8g2.setCursor(10, y);
      u8g2.print(data.stringptr);
      y += u8g2.getMaxCharHeight();
    }
  }
}

void LCD::lcdfunc_edgeCount()
{
  u8g2.setFont(font_24pt);
  u8g2.setCursor(0, u8g2.getMaxCharHeight()-4);

  u8g2.setFont(font_13pt);
  u8g2.print(F("EDGES "));
  
  u8g2.setFont(font_24pt);
  u8g2.print(lcdrenderdata[0].integer32);

  if(motSpeed < 0.01f)
  {
    constexpr char symbol[2] = {32 + 12, 0x00};
    u8g2.setFont(font_symbols);
    u8g2.setCursor(u8g2.getDisplayWidth() - u8g2.getMaxCharWidth() - 4, u8g2.getMaxCharHeight()+8);
    u8g2.print(symbol);
  }
  else 
  {
    constexpr char symbol[2] = {32 + 13, 0x00};
    u8g2.setFont(font_symbols);
    u8g2.setCursor(u8g2.getDisplayWidth() - u8g2.getMaxCharWidth() - 4, u8g2.getMaxCharHeight()+8);
    u8g2.print(symbol);    
  }
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
      Serial.println(F("[VIBRATOR]\tTurned on"));
    }
    else if (_currentState.powered == true && _targetState.powered == false)
    {
      //turn off:
      _currentState.powered = false;
      digitalWrite(PIN_VIBRATOR_ONOFF, HIGH);
      COROUTINE_DELAY(VIBRATOR_PUSH_DELAY);
      digitalWrite(PIN_VIBRATOR_ONOFF, LOW);
      COROUTINE_DELAY(VIBRATOR_PUSH_DELAY);
      Serial.println(F("[VIBRATOR]\tTurned off"));
    }

    if (_currentState.powered)
    {
      if (diff != 0)
      {
        Serial.print(F("[VIBRATOR] "));
        Serial.print(_currentState.mode);
        Serial.print(F(" -> "));
        Serial.println(_targetState.mode);
      }
      if (diff > 0)
      {
        for (i = 0; i < diff; i++)
        {
          //turn up:
          _currentState.mode++;
          digitalWrite(PIN_VIBRATOR_UP, HIGH);
          COROUTINE_DELAY(VIBRATOR_PUSH_DELAY);
          digitalWrite(PIN_VIBRATOR_UP, LOW);
          COROUTINE_DELAY(VIBRATOR_PUSH_DELAY);
          Serial.println(F("[VIBRATOR]\tTurned UP"));
        }
      }
      else if (diff < 0)
      {
        for (i = 0; i < -diff; i++)
        {
          //turn down:
          _currentState.mode--;
          digitalWrite(PIN_VIBRATOR_DOWN, HIGH);
          COROUTINE_DELAY(VIBRATOR_PUSH_DELAY);
          digitalWrite(PIN_VIBRATOR_DOWN, LOW);
          COROUTINE_DELAY(VIBRATOR_PUSH_DELAY);
          Serial.println(F("[VIBRATOR]\tTurned DOWN"));
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
void Vibrator::vibrator_on() { if (!_targetState.powered) { _targetState.powered = true; } }
void Vibrator::vibrator_to(const int mode) 
{ 
  if(mode > 0)
  {
    _targetState.mode = clamp(mode-1, 0, MAX_LEVEL); 
  }
}

int Vibrator::map_motspeed_to_vibrator()
{
  const float pct[] = { 0.25, 0.21, 0.17, 0.13, 0.13, 0.11 };
  static constexpr uint8_t speedCount = (sizeof(pct)/sizeof(pct[0]));
  static uint8_t ranges[speedCount+1][2];
  static bool initialized = false;
  if(!initialized)
  {
    initialized = true;

    //Off:
    ranges[0][0] = 0;
    ranges[0][1] = 1;
    unsigned int lastValue = 1;
    
    //1,2,3,4 speed settinsg:
    for(auto i=0; i < speedCount; i++)
    {
      ranges[1+i][0] = lastValue;
      ranges[1+i][1] = static_cast<uint8_t>(clamp(lastValue + static_cast<int>(pct[i] * 255.0f), 0, 255));
      lastValue = ranges[1+i][1];
    }
#if 0
    for(auto i=0; i < speedCount+1; i++)
    {
      Serial.print(F("["));
      Serial.print(i);
      Serial.print(F("] "));
      Serial.print(ranges[i][0]);
      Serial.print(F("-"));
      Serial.println(ranges[i][1]);
    }
#endif
  }

  if(motSpeed >= maxSpeed)
  {
    //Go back, as to not overstimulate.
    motSpeed = ranges[2][0];
  }

  //Map motSpeed to index:
  for (auto i=0; i < speedCount+1; i++)
  {
    if (motSpeed >= ranges[i][0] && motSpeed < ranges[i][1])
    {
      return i;
    }
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
      analogReference(EXTERNAL);
      pinMode(BUTTPIN, INPUT); //default is 10 bit resolution (1024), 0-3.3
      raPressure.clear();      //Initialize a running pressure average

      //Check memory, if size is 0, we failed to allocate...
      if (raPressure.getSize() == 0)
      {
        LCD::RenderPanic(reinterpret_cast<const char*>(F("OUT OF MEM")));
      }
    }

    int Sensor::runCoroutine() 
    {
      static auto s_lastTime = micros();
      static uint32_t s_tick = 0;
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
        if (s_tickaccumulator >= 1e6 / 60)
        {
          s_tickaccumulator -= 1e6 / 60;
          s_tick++;
        }

        if (s_lastTick != s_tick)
        {
          s_lastTick = s_tick;
          if (s_tick % RA_TICK_PERIOD == 0)
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

/////////////////////////////////////////////////////
// Logic
int Logic::sampleTick = 0;

int Logic::runCoroutine()
    {
      COROUTINE_LOOP()
      {       
        fadeToBlackBy(leds,NUM_LEDS,20); //Create a fading light effect. LED buffer is not otherwise cleared
        static uint8_t l_btnState = 0;
        l_btnState = check_button();

        static auto l_newState = state;
        l_newState = set_state(l_btnState,state); //Set the next state based on this state and button presses

        if (l_newState != state)
        { //state changed
          state = l_newState;
          if (state == AUTO || state == MANUAL)
          {
            SessionBeginEndCmd cmd;
            cmd.SetValue((state == AUTO) ? true : false);
            g_i2c.SendI2CCommand(&cmd);
          }
        }
        run_state_machine(l_newState);
        FastLED.show(); //Update the physical LEDs to match the buffer in software

        //Alert that the Pressure voltage amplifier is railing, and the trim pot needs to be adjusted
        if(g_pressure > 4030)beep_motor(2093,2093,2093); //Three high beeps
        COROUTINE_YIELD();
      }
    }

void Logic::Setup()
{
  pinMode(ENC_SW,   INPUT); //Pin to read when encoder is pressed
  digitalWrite(ENC_SW, HIGH); // Encoder switch pullup

  //Storage:
  sensitivity = EEPROM.read(SENSITIVITY_ADDR);
  maxSpeed = min(EEPROM.read(MAX_SPEED_ADDR), MOT_MAX); //Obey the MOT_MAX the first power  cycle after chaning it.
  beep_motor(1047,1396,2093); //Power on beep
}

void Logic::Update()
{

    //Run this section at the update frequency (default 60 Hz)
  if (millis() % period == 0) {   
    //Report pressure and motor data over USB for analysis / other uses. timestamps disabled by default
    logDebug();
  }
}

// Manual vibrator control mode (red), still shows orgasm closeness in background
void Logic::run_manual() 
{
  //In manual mode, only allow for 13 cursor positions, for adjusting motor speed.
  const int knob = encLimitRead(0,NUM_LEDS-1);
  motSpeed = map(knob, 0, NUM_LEDS - 1, 0., (float)MOT_MAX);

  const auto setting = g_vibrator.map_motspeed_to_vibrator();
  if (setting > 0)
  {
    g_vibrator.vibrator_on();
    g_vibrator.vibrator_to(setting);
  }
  else
  {
    g_vibrator.vibrator_off();
    g_vibrator.vibrator_to(0);
  }

  //gyrGraphDraw(avgPressure, 0, 4 * 3 * NUM_LEDS);
  const int presDraw = map(constrain(g_pressure - avgPressure, 0, pLimit),0,pLimit,0,NUM_LEDS*3);
  draw_bars_3(presDraw, CRGB::Green,CRGB::Yellow,CRGB::Red);
  draw_cursor(knob, CRGB::Red);
}

// Automatic edging mode, knob adjust sensitivity.
void Logic::run_auto() 
{
  static float motIncrement = 0.0;
  motIncrement = ((float)maxSpeed / ((float)FREQUENCY * (float)rampTimeS));

  int knob = encLimitRead(0,(3*NUM_LEDS)-1);
  sensitivity = knob*4; //Save the setting if we leave and return to this state
  //Reverse "Knob" to map it onto a pressure limit, so that it effectively adjusts sensitivity
  pLimit = map(knob, 0, 3 * (NUM_LEDS - 1), DEFAULT_PLIMIT, 1); //set the limit of delta pressure before the vibrator turns off
  if (pLimit != pLimitLast)
  {
    pLimitLast = pLimit;
    LCD::lastTargetChange = millis();
  }
  //When someone clenches harder than the pressure limit
  if (g_pressure - avgPressure > pLimit) {
    if (motSpeed > 0.0f)
    {
      g_vibrator.vibrator_off();
      g_vibrator.vibrator_to(0);
      edgeCount++;
      lastEdgeCountChange = millis();
    }
    motSpeed = -s_cooldownTime*(float)rampTimeS*((float)FREQUENCY*motIncrement);//Stay off for a while (half the ramp up time)
    
  }
  else if (motSpeed < (float)maxSpeed) {
    motSpeed += motIncrement;
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
    g_vibrator.vibrator_to(0);
  }
  
  int presDraw = map(constrain(g_pressure - avgPressure, 0, pLimit), 0, pLimit, 0, NUM_LEDS * 3);
  draw_bars_3(presDraw, CRGB::Green, CRGB::Yellow, CRGB::Red);
  draw_cursor_3(knob, CRGB(50, 50, 200), CRGB::Blue, CRGB::Purple);
}

//Poll the knob click button, and check for long/very long presses as well
uint8_t Logic::check_button()
{
  static uint8_t btnState = BTN_NONE;
  btnState = BTN_NONE;

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
      btnState = BTN_V_LONG;
      keyDownTime = 0;
    }
    else if ((millis() - keyDownTime) >= LONG_PRESS_MS)
    {
      btnState = BTN_LONG;
      keyDownTime = 0;
    }
    else
    {
      btnState = BTN_SHORT;
      keyDownTime = 0;
    }
  }

  lastBtn = thisBtn;
  return btnState;
}

//run the important/unique parts of each state. Also, set button LED color.
void Logic::run_state_machine(uint8_t state)
{
  switch (state) {
    case MANUAL:
      run_manual();
      break;
    case AUTO:
      run_auto();
      break;
    default:
      run_manual();
      break;
  }
}

//Switch between state machine states, and reset the encoder position as necessary
//Returns the next state to run. Very long presses will turn the system off (sort of)
uint8_t Logic::set_state(uint8_t btnState, uint8_t state)
{
  if(btnState == BTN_NONE){
    return state;
  }
  if(btnState == BTN_V_LONG){
    //Turn the device off until woken up by the button
    Serial.println(F("power off"));
    fill_gradient_RGB(leds,0,CRGB::Black,NUM_LEDS-1,CRGB::Black);//Turn off LEDS
    FastLED.show();
    g_vibrator.vibrator_off();
    //analogWrite(MOTPIN, 0);
    beep_motor(2093,1396,1047);
    //analogWrite(MOTPIN, 0); //Turn Motor off
    g_vibrator.vibrator_off();
    while(!digitalRead(ENC_SW))delay(1);
    beep_motor(1047,1396,2093);
    return MANUAL ;
  }
  else if(btnState == BTN_SHORT){
    switch(state){
      case MANUAL:
        myEnc.write(sensitivity);//Whenever going into auto mode, keep the last sensitivity
        motSpeed = 0; //Also reset the motor speed to 0
        return AUTO;
      case AUTO:
        myEnc.write(0);//Whenever going into manual mode, set the speed to 0.
        motSpeed = 0;
        EEPROM.update(SENSITIVITY_ADDR, sensitivity);
        return MANUAL;
    }
  }
  else if(btnState == BTN_LONG){
    switch (state) {
      case MANUAL:
        myEnc.write(map(maxSpeed, 0, 255, 0, 4 * (NUM_LEDS))); //start at saved value
        return AUTO;//OPT_SPEED;
      case AUTO:
        myEnc.write(map(maxSpeed, 0, 255, 0, 4 * (NUM_LEDS))); //start at saved value
        return MANUAL;//OPT_SPEED;
    }
  }
  else return MANUAL;
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
    COROUTINE_DELAY(250);
    static String i2cstring;
    logI2C(i2cstring);

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
    COROUTINE_YIELD();
  }
  #endif
}

void I2C::SendI2CCommand(Command* cmd)
{
  #define I2C_PROTOCOL 1

  #if defined(I2C_PROTOCOL) && I2C_PROTOCOL==0
  //Request a data to allow the logger to be ready, and then make sure
  //we wait.
  //Wire.setClock(35);
  Wire.requestFrom(0x43, 1);
  while(Wire.available())
  {
    Serial.println(Wire.read());
  }
  #endif

  //Start submitting the command:
  //Wire.setClock(16000000);
  Wire.setClock(3400000);
  digitalWrite(11, HIGH);
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
    uint8_t written = Wire.write(buffer, clamp(sz, 0, 10));
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
  int colorNum = pos/NUM_LEDS; //revolution number
  int cursorPos = pos % NUM_LEDS; //place on circle, from 0-12
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
  int colorNum = pos/NUM_LEDS; //revolution number
  int barPos = pos % NUM_LEDS; //place on circle, from 0-12
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

//typedef void (*ProfiledFunc)();
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

//=======Setup=======================================
void setup() 
{
  Wire.setWireTimeout(50000, true);

  Serial.begin(115200); delay(100); Serial.println(F("Serial up")); Serial.flush();
  g_sensor.Setup();  delay(100); Serial.println(F("Sensor up")); Serial.flush();
  g_logic.Setup();  delay(100); Serial.println(F("Logic up")); Serial.flush();
  g_lcd.Setup();  delay(100); Serial.println(F("LCD up")); Serial.flush();
  g_leds.Setup();  delay(100); Serial.println(F("LEDS up")); Serial.flush();
  g_vibrator.Setup();  delay(100); Serial.println(F("Vibrator up")); Serial.flush();
  g_i2c.Setup();  delay(100); Serial.println(F("i2C up")); Serial.flush();
  
  Serial.println("Started up");
}

//=======Main Loop=============================
void loop() 
{
  static uint8_t state = MANUAL;
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
