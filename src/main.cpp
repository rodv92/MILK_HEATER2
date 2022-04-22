#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#include <PID_v1.h>
#include <SpinTimer.h>
#include <LcdKeypad.h>

#include <VL6180X.h>

// LCD KEYPAD

LcdKeypad* myLcdKeypad = 0;

// TEMPERATURE SENSOR
#define ONE_WIRE_BUS 2

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

#define RELAY_PIN 3

bool program_init = false;

const PROGMEM uint16_t progdata[2][4] = {{30,1000,50,3000},{30,2000,60,2000}};
uint16_t current_program_data[4];
//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
// prev double Kp=300, Ki=5, Kd=1000;
// prev double Kp=300, Ki=7, Kd=1000; quite good... long settle time from above.

//Calculate heat capacity = vessel + measured liquid quantity. J/c (Q = mc*delta(T))
//Calculate total energy needed to bring to thermal equlibrium Q = mc*(Tfinal - Tsstart)
// error = Tfinal - Tstart
// rise = tfinal - tstart
// Q = mc*error
// P = mc*error/rise
// kp = mc/rise * 1.2
// Divide energy by required rise time to get power. 1 output = 1.2W
// steady state gain : 300 for 5.7l is quite ok

double Kp=300, Ki=20, Kd=-4000;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, P_ON_E, REVERSE);

int WindowSize = 1000;
uint8_t Duty = 0;
uint8_t Duty2 = 0;

unsigned long windowStartTime;
//bool program_started; // program is running

// Implement specific LcdKeypadAdapter in order to allow receiving key press events
class MyLcdKeypadAdapter : public LcdKeypadAdapter
{
private:
  LcdKeypad* m_lcdKeypad;
  uint8_t m_value; // program number
public:
  bool program_started; // program status
  uint8_t program_number; // public program number
  MyLcdKeypadAdapter(LcdKeypad* lcdKeypad)
  : m_lcdKeypad(lcdKeypad)
  , m_value(0)
  , program_number(1)
  , program_started(false)
  { }

  // Specific handleKeyChanged() method implementation - define your actions here
  void handleKeyChanged(LcdKeypad::Key newKey)
  {
    if (0 != m_lcdKeypad)
    {
      if (LcdKeypad::UP_KEY == newKey)
      {
        m_value++;
        m_value = constrain(m_value,1,5);
        program_number = m_value;
      }
      else if (LcdKeypad::DOWN_KEY == newKey)
      {
        m_value--;
        m_value = constrain(m_value,1,5);
        program_number = m_value;
      }
      else if (LcdKeypad::SELECT_KEY == newKey)
      {
        program_started = true;
      }

      m_lcdKeypad->setCursor(0, 1);            // position the cursor at beginning of the second line
      m_lcdKeypad->print(m_value,DEC);             // print the value on the second line of the display
      m_lcdKeypad->print("                ");  // wipe out characters behind the printed value
     
      // RGB colored backlight: set according to the current value
      // monochrome backlight: set backlight on or off according to the current value
      m_lcdKeypad->setBacklight(static_cast<LcdKeypad::LcdBacklightColor>(LcdKeypad::LCDBL_WHITE & m_value));
    }
  }
};

MyLcdKeypadAdapter* myLcdAdapter;


void setup()
{

  myLcdKeypad = new LcdKeypad();  // instantiate an object of the LcdKeypad class, using default parameters
  myLcdAdapter = new MyLcdKeypadAdapter(myLcdKeypad);
  // Attach the specific LcdKeypadAdapter implementation (dependency injection)
  myLcdKeypad->attachAdapter(myLcdAdapter);
  
  myLcdKeypad->setCursor(0, 0);   // position the cursor at beginning of the first line
  myLcdKeypad->print("PROGRAM:");   // print a Value label on the first line of the display

  pinMode(RELAY_PIN,OUTPUT);
  Serial.begin(9600);
  Serial.println("START");
  windowStartTime = millis();

  //initialize the variables we're linked to
  Setpoint = 40; // Degrees Celsius

  //tell the PID to range between 0 and the full window size
  myPID.SetOutputLimits(0, WindowSize);

  myPID.SetSampleTime(5000);

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
}


void loop() {
  // put your main code here, to run repeatedly:

  while(!myLcdAdapter->program_started)
  {}
  if(!program_init) 
  {
    uint8_t k;
    for(k=0;k<4;k++)
    {
      current_program_data[k] = pgm_read_word(&(progdata[myLcdAdapter->program_number][k]));
    }
  }

  sensors.requestTemperatures(); // Send the command to get temperatures

  Input = float(sensors.getTempCByIndex(0));

 
  myPID.Compute();

  /************************************************
   * turn the output pin on/off based on pid output
   ************************************************/
  if (millis() - windowStartTime > WindowSize)
  { //time to shift the Relay Window
    windowStartTime += WindowSize;

    Duty = int(100 *(Output/WindowSize)); 
    Duty2 = 128 - int(128*Duty/100);
    Serial.print("CT:");
    Serial.print(Input);
    Serial.print(" TT:");
    Serial.print(Setpoint);
    Serial.print(" D:");
    Serial.println(Duty);
    //Serial.print(" O:");
    //Serial.println(Output);
    

  }
  
  //Output = max(0,Output);
  //if (Output < millis() - windowStartTime) AnalogWrite(RELAY_PIN, HIGH);
  
  
  if (Output < millis() - windowStartTime) digitalWrite(RELAY_PIN, HIGH);
  else digitalWrite(RELAY_PIN, LOW);
  

}