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
#define font_15pt u8g2_font_VCR_OSD_mr 
#define font_symbols u8g2_font_unifont_t_weather
#else
#define font_13pt u8g2_font_6x13_tr
#define font_24pt u8g2_font_6x13_tr 
#define font_15pt u8g2_font_6x13_tr 
#define font_symbols u8g2_font_unifont_t_weather
#endif

U8G2_SSD1306_128X32_UNIVISION_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);   // Adafruit ESP8266/32u4/ARM Boards + FeatherWing OLED

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

int pressure = 0;
int avgPressure = 0; //Running 25 second average pressure
//int bri =100; //Brightness setting
int rampTimeS = 60; //Ramp-up time, in seconds
#define DEFAULT_PLIMIT 600
int pLimit = DEFAULT_PLIMIT; //Limit in change of pressure before the vibrator turns off
int pLimitLast = DEFAULT_PLIMIT;
int maxSpeed = 255; //maximum speed the motor will ramp up to in automatic mode
float motSpeed = 0; //Motor speed, 0-255 (float to maintain smooth ramping to low speeds)
uint16_t edgeCount = 0;
auto lastEdgeCountChange = millis();

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
  Serial.print(pressure); //(Original ADC value - 12 bits, 0-4095)
  Serial.print(F(","));
  Serial.print(avgPressure); //Running average of (default last 25 seconds) pressure
  Serial.print(F(","));
  Serial.print(pressure - avgPressure);
  Serial.print(F(","));
  Serial.print(pLimit);
  Serial.println(F(","));
  
#endif
}

//=======EEPROM Addresses============================
//128b available on teensy LC
#define BEEP_ADDR         1
#define MAX_SPEED_ADDR    2
#define SENSITIVITY_ADDR  3
//#define RAMPSPEED_ADDR    4 //For now, ramp speed adjustments aren't implemented

//=======LCD logic=============================
typedef void (*lcdrender_t)();
static lcdrender_t lcdrender_activefunc = nullptr;
union lcdrenderdata_t
{
  int32_t integer32;
  const char* stringptr;

  operator =(const int32_t arg) {
    integer32 = arg;
  }

  operator =(const char* arg) {
    stringptr = arg;
  }
} lcdrenderdata[2]{};

template<typename T, typename I, int idx=0, typename ...Iremainder>
void setLCDFunc(T func, I arg, Iremainder... args)
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
static void lcdfunc_renderPressure()
{
  u8g2.setFont(font_13pt);
  u8g2.setCursor(0, 20);
  u8g2.print("P");
  u8g2.setFont(font_24pt);
  u8g2.setCursor(10, u8g2.getMaxCharHeight());
  u8g2.print(lcdrenderdata[0].integer32);
}

static auto lastTargetChange = millis();
static void lcdfunc_renderTarget()
{
  u8g2.setFont(font_13pt);
  u8g2.setCursor(0, 20);
  u8g2.print("T ");
  u8g2.setFont(font_15pt );
  u8g2.setCursor(10, u8g2.getMaxCharHeight());
  u8g2.print(lcdrenderdata[0].integer32);
  u8g2.setFont(font_13pt);
  u8g2.print(" / ");
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

static void lcdfunc_renderText()
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

static void lcdfunc_edgeCount()
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

//=======Setup=======================================
//Beep out tones over the motor by frequency (1047,1396,2093) may work well
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
void setup() {
  u8g2.begin();
  /*
  u8g2.setFont(u8g2_font_ncenB14_tr);
  u8g2.firstPage();
  do {
    u8g2.setCursor(0, 20);
    u8g2.print(F("Hello World!"));
  } while ( u8g2.nextPage() );*/
  setLCDFunc(lcdfunc_renderText, __DATE__, __TIME__);
  do {
    lcdrender_activefunc();
  } while( u8g2.nextPage() );
  lcdrender_activefunc = nullptr;
  pinMode(ENC_SW,   INPUT); //Pin to read when encoder is pressed
  digitalWrite(ENC_SW, HIGH); // Encoder switch pullup

  analogReference(EXTERNAL);

  // Classic AVR based Arduinos have a PWM frequency of about 490Hz which
  // causes the motor to whine.  Change the prescaler to achieve 31372Hz.
  //sbi(TCCR1B, CS10);
  //cbi(TCCR1B, CS11);
  //cbi(TCCR1B, CS12);
  //pinMode(MOTPIN, OUTPUT); //Enable "analog" out (PWM)

  pinMode(BUTTPIN, INPUT); //default is 10 bit resolution (1024), 0-3.3
  raPressure.clear(); //Initialize a running pressure average

  pinMode(PIN_VIBRATOR_ONOFF, LOW);
  pinMode(PIN_VIBRATOR_UP, LOW);
  pinMode(PIN_VIBRATOR_DOWN, LOW);

  //Check memory, if size is 0, we failed to allocate...
  if (raPressure.getSize() == 0)
  {
    setLCDFunc(lcdfunc_renderText, F("OUT OF MEM"));
    do {
      lcdrender_activefunc();
    } while( u8g2.nextPage() );
    pinMode(LED_BUILTIN, OUTPUT);
    while(true)
    {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
      delay(100);
    }
  }

  //Make sure the motor is off
  vibrator_off();

  delay(3000); // 3 second delay for recovery

  Serial.begin(115200);

  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  // limit power draw to .6A at 5v... Didn't seem to work in my FastLED version though
  //FastLED.setMaxPowerInVoltsAndMilliamps(5,DEFAULT_PLIMIT);
  FastLED.setBrightness(BRIGHTNESS);

  //Recall saved settings from memory
  sensitivity = EEPROM.read(SENSITIVITY_ADDR);
  maxSpeed = min(EEPROM.read(MAX_SPEED_ADDR), MOT_MAX); //Obey the MOT_MAX the first power  cycle after chaning it.
  beep_motor(1047,1396,2093); //Power on beep
}

//=======Vibrator Control=================
static int clamp(const int val, const int minVal, const int maxVal) { return max(min(val, maxVal), minVal); };
static uint8_t g_vibrator_mode = 0; //off
void set_vibrator_mode(int fromStrength, int toStrength)
{
  fromStrength = clamp(fromStrength, 0, 5);
  toStrength = clamp(toStrength, 0, 5);
  
  if (fromStrength == 0)
  {
    //turn on:
    digitalWrite(PIN_VIBRATOR_ONOFF, HIGH);
    delay(VIBRATOR_PUSH_DELAY);
    digitalWrite(PIN_VIBRATOR_ONOFF, LOW);
    delay(VIBRATOR_PUSH_DELAY);
    Serial.println(F("[VIBRATOR]\tTurned on"));
    //consume increase;
    fromStrength++;
  }

  int diff = toStrength-fromStrength;
  if (diff > 0)
  {
    for (int i=0; i < diff; i++)
    {
      //turn up:
      digitalWrite(PIN_VIBRATOR_UP, HIGH);
      delay(VIBRATOR_PUSH_DELAY);
      digitalWrite(PIN_VIBRATOR_UP, LOW);
      delay(VIBRATOR_PUSH_DELAY);
      Serial.println(F("[VIBRATOR]\tTurned UP"));
    }
  }
  else if (diff < 0)
  {
    for (int i=0; i < -diff; i++)
    {
      //turn down:
      digitalWrite(PIN_VIBRATOR_DOWN, HIGH);
      delay(VIBRATOR_PUSH_DELAY);
      digitalWrite(PIN_VIBRATOR_DOWN, LOW);
      delay(VIBRATOR_PUSH_DELAY);
      Serial.println(F("[VIBRATOR]\tTurned DOWN"));
    }    
  }

  if (toStrength == 0)
  {
    //turn off:
    digitalWrite(PIN_VIBRATOR_ONOFF, HIGH);
    delay(VIBRATOR_PUSH_DELAY);
    digitalWrite(PIN_VIBRATOR_ONOFF, LOW);
    delay(VIBRATOR_PUSH_DELAY);
    Serial.println(F("[VIBRATOR]\tTurned OFF"));
  }

  if (g_vibrator_mode != toStrength)
  {
    Serial.print(F("[VIBRATOR]\t Strength "));
    Serial.print(g_vibrator_mode);
    Serial.print(F(" -> "));
    Serial.println(toStrength);
  }
  g_vibrator_mode = toStrength;
}
static void vibrator_up() { set_vibrator_mode(g_vibrator_mode, g_vibrator_mode+1); }
static void vibrator_down() { set_vibrator_mode(g_vibrator_mode, g_vibrator_mode-1); }
static void vibrator_off() { if (g_vibrator_mode!=0) set_vibrator_mode(g_vibrator_mode, 0); }
static void vibrator_on() { if (g_vibrator_mode==0) set_vibrator_mode(g_vibrator_mode, 1); }
static void vibrator_to(const int mode) { if (g_vibrator_mode != mode) { set_vibrator_mode(g_vibrator_mode, mode); }; }

static int map_motspeed_to_vibrator()
{
  const float pct[] = { 0.3, 0.22, 0.27, 0.21 };
  static unsigned char ranges[5][2];
  static bool initialized = false;
  if(!initialized)
  {
    initialized = true;

    //Off:
    ranges[0][0] = 0;
    ranges[0][1] = 1;
    unsigned int lastValue = 1;
    
    //1,2,3,4 speed settinsg:
    for(auto i=0; i < 4; i++)
    {
      ranges[1+i][0] = lastValue;
      ranges[1+i][1] = lastValue + static_cast<char>(pct[i] * 255.0);
      lastValue = ranges[1+i][1];
    }
  }

  if(motSpeed >= maxSpeed)
  {
    //Go back, as to not overstimulate.
    motSpeed = ranges[2][0];
  }

  //Map motSpeed to index:
  for (auto i=0; i < 5; i++)
  {
    if (motSpeed >= ranges[i][0] && motSpeed < ranges[i][1])
    {
      return i;
    }
  }
  return -1;
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

//=======Program Modes/States==================

// Manual vibrator control mode (red), still shows orgasm closeness in background
void run_manual() {
  //In manual mode, only allow for 13 cursor positions, for adjusting motor speed.
  const int knob = encLimitRead(0,NUM_LEDS-1);
  motSpeed = map(knob, 0, NUM_LEDS - 1, 0., (float)MOT_MAX);

  const auto setting = map_motspeed_to_vibrator();
  if (setting >= 0)
  {
    vibrator_to(setting);
  }

  //gyrGraphDraw(avgPressure, 0, 4 * 3 * NUM_LEDS);
  const int presDraw = map(constrain(pressure - avgPressure, 0, pLimit),0,pLimit,0,NUM_LEDS*3);
  draw_bars_3(presDraw, CRGB::Green,CRGB::Yellow,CRGB::Red);
  draw_cursor(knob, CRGB::Red);
}

// Automatic edging mode, knob adjust sensitivity.
void run_auto() {
  static float motIncrement = 0.0;
  motIncrement = ((float)maxSpeed / ((float)FREQUENCY * (float)rampTimeS));

  int knob = encLimitRead(0,(3*NUM_LEDS)-1);
  sensitivity = knob*4; //Save the setting if we leave and return to this state
  //Reverse "Knob" to map it onto a pressure limit, so that it effectively adjusts sensitivity
  pLimit = map(knob, 0, 3 * (NUM_LEDS - 1), 600, 1); //set the limit of delta pressure before the vibrator turns off
  if (pLimit != pLimitLast)
  {
    pLimitLast = pLimit;
    lastTargetChange = millis();
  }
  //When someone clenches harder than the pressure limit
  if (pressure - avgPressure > pLimit) {
    if (motSpeed > 0.0f)
    {
      vibrator_off();
      edgeCount++;
      lastEdgeCountChange = millis();
    }
    motSpeed = -.5*(float)rampTimeS*((float)FREQUENCY*motIncrement);//Stay off for a while (half the ramp up time)
    
  }
  else if (motSpeed < (float)maxSpeed) {
    motSpeed += motIncrement;
  }

  const auto setting = map_motspeed_to_vibrator();
  if (setting >= 0)
  {
    vibrator_to(setting);
  }
  
  int presDraw = map(constrain(pressure - avgPressure, 0, pLimit), 0, pLimit, 0, NUM_LEDS * 3);
  draw_bars_3(presDraw, CRGB::Green, CRGB::Yellow, CRGB::Red);
  draw_cursor_3(knob, CRGB(50, 50, 200), CRGB::Blue, CRGB::Purple);
}

//Poll the knob click button, and check for long/very long presses as well
uint8_t check_button(){
  static bool lastBtn = ENC_SW_DOWN;
  static unsigned long keyDownTime = 0;
  uint8_t btnState = BTN_NONE;
  bool thisBtn = digitalRead(ENC_SW);

  //Detect single presses, no repeating, on keyup
  if(thisBtn == ENC_SW_DOWN && lastBtn == ENC_SW_UP){
    keyDownTime = millis();
  }
  
  if (thisBtn == ENC_SW_UP && lastBtn == ENC_SW_DOWN) { //there was a keyup
    if((millis()-keyDownTime) >= V_LONG_PRESS_MS){
      btnState = BTN_V_LONG;
    }
    else if((millis()-keyDownTime) >= LONG_PRESS_MS){
      btnState = BTN_LONG;
      }
    else{
      btnState = BTN_SHORT;
      }
    }

  lastBtn = thisBtn;
  return btnState;
}

//run the important/unique parts of each state. Also, set button LED color.
void run_state_machine(uint8_t state){
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

void run_lcd_status(uint8_t state)
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
        setLCDFunc(lcdfunc_renderTarget, max(0, pressure - avgPressure), max(0, pLimit));
      } break;
    case 2:
    {
      setLCDFunc(lcdfunc_edgeCount, edgeCount);
    } break;
  }

}

//Switch between state machine states, and reset the encoder position as necessary
//Returns the next state to run. Very long presses will turn the system off (sort of)
uint8_t set_state(uint8_t btnState, uint8_t state){
  if(btnState == BTN_NONE){
    return state;
  }
  if(btnState == BTN_V_LONG){
    //Turn the device off until woken up by the button
    Serial.println(F("power off"));
    fill_gradient_RGB(leds,0,CRGB::Black,NUM_LEDS-1,CRGB::Black);//Turn off LEDS
    FastLED.show();
    vibrator_off();
    //analogWrite(MOTPIN, 0);
    beep_motor(2093,1396,1047);
    //analogWrite(MOTPIN, 0); //Turn Motor off
    vibrator_off();
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

//=======Main Loop=============================
void loop() {
  static uint8_t state = MANUAL;
  static int sampleTick = 0;
  static bool rendered = false;

  //Run this section at the update frequency (default 60 Hz)
  if (millis() % period == 0) {
    delay(1);
    
    sampleTick++; //Add pressure samples to the running average slower than 60Hz
    if (sampleTick % RA_TICK_PERIOD == 0) {
      raPressure.addValue(pressure);
      avgPressure = raPressure.getAverage();
    }
    
    pressure=0;
    for(uint8_t i=OVERSAMPLE; i; --i) {
      pressure += analogRead(BUTTPIN);
      if(i) {      // Don't delay after the last sample
        delay(1);  // Spread samples out a little
      }
    }
    fadeToBlackBy(leds,NUM_LEDS,20); //Create a fading light effect. LED buffer is not otherwise cleared
    uint8_t btnState = check_button();
    const auto newState = set_state(btnState,state); //Set the next state based on this state and button presses
    if (newState != state)
    { //state changed
      if (state == AUTO)
      {
        //g_Storage.BeginSession();
      }
    }
    run_state_machine(newState);
    FastLED.show(); //Update the physical LEDs to match the buffer in software

    //Alert that the Pressure voltage amplifier is railing, and the trim pot needs to be adjusted
    if(pressure > 4030)beep_motor(2093,2093,2093); //Three high beeps

    //Report pressure and motor data over USB for analysis / other uses. timestamps disabled by default
    logDebug();

    run_lcd_status(state);
    rendered = false;
  }
  else if (!rendered)
  {
    if (lcdrender_activefunc != nullptr)
    {
      lcdrender_activefunc();
      if (!u8g2.nextPage())
      {
        memset(&lcdrenderdata[0], 0, sizeof(lcdrenderdata));
        lcdrender_activefunc = nullptr;
      }
      rendered = true;
    }
  }
}
