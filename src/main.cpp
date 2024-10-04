#define VERSION "1.0.0.35"
#define DEBUGLEVEL 4

#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#include <PID_v1.h>
#include <SpinTimer.h>
#include <LcdKeypad.h>

#include <VL6180X.h>
#include <avr/wdt.h>

// LCD KEYPAD

LcdKeypad* myLcdKeypad = 0;

// TEMPERATURE SENSOR
#define ONE_WIRE_BUS 2

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
VL6180X range_sensor;


#define MOTOR_RELAY_PIN 3
#define HEATER_SSR_PIN 10
#define BUZZER_PIN 11

bool self_test = false;
bool debug = true;
bool program_init = false;
bool program_ended = false;
//bool program_paused = false;
bool stir_only = false;
bool stir_disabled = false;
bool stir_suspend = false;
uint8_t stir_duty_pct = 100;

bool open_rise = false;
uint8_t program_step = 0;

String msg_id[3] = {"SELECT_PROGRAM","PROGRAM_END","ADD_RENNET"};

enum state { initialization, start_program, end_step, end_program };

const PROGMEM int16_t progdata[5][20] = 
{
  

  //format_of_array 
  // {target_temp,rise_time,stir,pause_after,aural_temp_warn_delta
  // target_temp2,rise_time2,stir2,pause_after2,aural_temp_warn_delta_2 
  // etc...}
  
  // if rise_time == -1 then open rise (full power)
  // if target_temperature = -1 then disable heater
  // if stir !=0 or != 1 and <= 99 then it is intepreted as stir duty cycle in percent     
  
  // stir only program :
  // if program is an array of 0 values with 1 for stirring then it will only perform stirring without end.
  {0,0,1,0,0,
  0,0,1,0,0,
  0,0,1,0,0,
  0,0,1,0,0} 
  ,

  // program fromage suisse type emmental
 /*
  {32,1200,0,0,
  32,2400,0,1,
  32,2400,1,0,
  49,1800,1,0}
 */

//programme calibration 23.12°C à 33.12°C pour 10 litres avec isolation thermique, chauffage à puissance max.
// temps total 11 min 25 secondes. (68 sec par degré) overshoot jusqu'à 40.13 (soit 7deg pour 100%)
 {33,-1,0,1,0,
  0,0,0,0,0,
  0,0,0,0,0,
  0,0,0,0,0}
 //fin programme calibration

// TEST 5L of water from 23C to 32C with steering and without insulation : 31.55 after 1200 sec
// plateau overshoot +0.5C
 ,
 
//programme mantien temperature
 /*
 {34,7200,0,1,
  0,0,0,0,
  0,0,0,0,
  0,0,0,0}
 ,
*/

 // programme pasteurisation
 {72,1200,0,0,0,
  72,180,1,1,0,
  32,2400,1,0,0,
  32,2400,1,0,0}
 // fin programme pasteurisation

 ,
 
 // programme parmesan
 {32,1200,1,0,0,
  32,2400,1,1,0,
  37,1200,1,0,0,
  51,1600,1,0,0}
 // fin programme parmesan

 // programme confiture de lait step0 : 1h to 97, stir 10% duty, no pause after, warn 5°C under target
 // programme confiture de lait step1 : 5h at 99, stir 100% duty, pause after, warn 1°C over target
,
 {97,3600,10,0,-5,
  99,18000,1,1,1,
  0,0,0,0,0,
  0,0,0,0,0
  }
 // fin programme confiture de lait

 

};
int16_t current_program_data[5];
//Define Variables we'll be connecting to
double Setpoint, Input, Output = 0.0;


// Units S.I
double liquid_qty_litres;
double heat_transfer_rate;
double ambient_temperature;

const double vessel_radius = 0.1375f;
const double vessel_height = 0.245f;
const double vessel_area = vessel_height*vessel_radius*2.f*PI;
const double vessel_thickness = 0.001f;
const double vessel_thermal_conductivity = 25.f;
const double hotplate_max_power = 1500.f;
const double boost_factor = 1.8f;
const double vessel_volume_l = vessel_radius*vessel_radius*PI*vessel_height*1000.f;

const double liquid_convective_coefficient_vertical = 12.4f; //(milk / steel vessel interface from 50° to 35°)
const double air_convective_coefficient_vertical = 3.8f; //(from ambient temperature at 20°C and metal surface at 35°C)
 
 const double air_convective_coefficient_horizontal = 4.8f; //(milk / to air interface from 40° to 30°)
 const double liquid_convective_coefficient_horizontal = 13.6f; //( steel bottom to milk interface from 50°C to 35°C)
 
 const double milk_thermal_conductivity = 0.622f;
 const double milk_heat_capacity = 3970.f; // J/kg; (hot plate heat capacity estimated at 460 J/kg, not factored in)
 const double milk_density = 1.036f; // kg/l

 double milk_thickness;
 double vertical_surface_U_value;
 double horizontal_surface_U_value;
 double heat_transfer_rate_vertical;
 double heat_transfer_rate_horizontal;
 double total_heat_loss;
 double total_energy_required;

 double power_required;
 double temperature_rise_time;
 


uint8_t i;
uint8_t k;
uint16_t range;
double avgret;
double avgdist;
double crosstalk;
  

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

double Kp=300.f, Ki=20.f, Kd=-50000.f;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, P_ON_E, REVERSE);

uint16_t WindowSizeHeat = 1000; // heater duty period. a larger period reduces thermal stress on the plate, and reduces electrical switching effects
                                // a shorter period gives more uniform heating and less inertia, making PID work better. Beware SCR switching happens at a multiple of 1/(2*mains_frequency)
uint16_t WindowSizeStir = 60000; // stir motor duty period. Use a multiples of (60/RPM)/2 to keep the paddles at the same alignment at each turn off
                                  // duty cycle also has to be a multiple of (60/RPM)/2 and less than WindowSizestir for alignment conservation.
//uint16_t WindowSizeRise = 20000;
float EffectiveDuty = 0.f;
float Duty2 = 0.f;

uint32_t windowStartTimeHeat;
uint32_t windowStartTimeStir;
uint32_t programStartTime;
uint32_t programElapsedTime;
uint32_t programSwitchPoint;

void resetFunc() 
{
  wdt_disable();
  wdt_enable(WDTO_15MS); // 15 ms watchdog 
  while(true); // infinite loop without feeding the dog, should reset in 15ms
}

void dbg(const __FlashStringHelper* fsh , uint8_t debuglevel)
{
  if (debuglevel <= DEBUGLEVEL)
  {
    Serial.print(fsh);
  }
}

void dbg(uint8_t val , uint8_t debuglevel)
{
  if (debuglevel <= DEBUGLEVEL)
  {
    Serial.print(val);
  }
}

void dbg(uint16_t val , uint8_t debuglevel)
{
  if (debuglevel <= DEBUGLEVEL)
  {
    Serial.print(val);
  }
}


void dbg(int16_t val , uint8_t debuglevel)
{
  if (debuglevel <= DEBUGLEVEL)
  {
    Serial.print(val);
  }
}

void dbg(int32_t val , uint8_t debuglevel)
{
  if (debuglevel <= DEBUGLEVEL)
  {
    Serial.print(val);
  }
}


void dbg(float val , uint8_t debuglevel)
{
  if (debuglevel <= DEBUGLEVEL)
  {
    Serial.print(val);
  }
}


void dbg(double val , uint8_t debuglevel)
{
  if (debuglevel <= DEBUGLEVEL)
  {
    Serial.print(val);
  }
}



void dbgln(const __FlashStringHelper* fsh , uint8_t debuglevel)
{
  if (debuglevel <= DEBUGLEVEL)
  {
    Serial.println(fsh);
  }
}


void dbgln(uint8_t val , uint8_t debuglevel)
{
  if (debuglevel <= DEBUGLEVEL)
  {
    Serial.println(val);
  }
}


void dbgln(uint16_t val , uint8_t debuglevel)
{
  if (debuglevel <= DEBUGLEVEL)
  {
    Serial.println(val);
  }
}

void dbgln(uint32_t val , uint8_t debuglevel)
{
  if (debuglevel <= DEBUGLEVEL)
  {
    Serial.println(val);
  }
}


void dbgln(int16_t val , uint8_t debuglevel)
{
  if (debuglevel <= DEBUGLEVEL)
  {
    Serial.println(val);
  }
}



void dbgln(int32_t val , uint8_t debuglevel)
{
  if (debuglevel <= DEBUGLEVEL)
  {
    Serial.println(val);
  }
}



void dbgln(float val , uint8_t debuglevel)
{
  if (debuglevel <= DEBUGLEVEL)
  {
    Serial.println(val);
  }
}


void dbgln(double val , uint8_t debuglevel)
{
  if (debuglevel <= DEBUGLEVEL)
  {
    Serial.println(val);
  }
}



//bool program_started; // program is running

// Implement specific LcdKeypadAdapter in order to allow receiving key press events
class MyLcdKeypadAdapter : public LcdKeypadAdapter
{
private:
  LcdKeypad* m_lcdKeypad;
  uint8_t m_value; // program number
public:
  uint8_t program_number; // public program number
  bool program_paused;
  bool program_started; // program status
  uint8_t liquid_qty_decilitres;
  bool input_qty;
  bool input_ambtemp;
 
  MyLcdKeypadAdapter(LcdKeypad* lcdKeypad)
  : m_lcdKeypad(lcdKeypad)
  , m_value(0)
  , program_number(1)
  , program_paused(false)
  , program_started(false)
  , liquid_qty_decilitres(50)
  , input_qty(false)
  , input_ambtemp(true)
  { }

  // Specific handleKeyChanged() method implementation - define your actions here
  void handleKeyChanged(LcdKeypad::Key newKey)
  {
    if (0 != m_lcdKeypad)
    {
      if (LcdKeypad::UP_KEY == newKey)
      {
        if (self_test) {dbgln(F("UP"),1); return;}
        if (debug) {dbgln(F("UP"),1);}
        if (input_qty) 
        {
          liquid_qty_decilitres++;
          liquid_qty_decilitres = constrain(liquid_qty_decilitres,50,150);
        }
        
        if (input_ambtemp) 
        {
          ambient_temperature += 1.f;
          ambient_temperature = constrain(ambient_temperature,0.0,40.0);
        }
        
        if ((program_started) && !input_qty && !input_ambtemp) 
        { 
          digitalWrite(MOTOR_RELAY_PIN,LOW);
          stir_suspend = false; 
          dbgln(F("MOTOR_ENABLED"),1); 
          return;
        }
        else
        {
          m_value++;
          m_value = constrain(m_value,0,3);
          program_number = m_value;
        }
        
      }
      else if (LcdKeypad::DOWN_KEY == newKey)
      {
        if (self_test) {dbgln(F("DOWN"),1); return;}
        if (debug) {dbgln(F("DOWN"),1);}
        if (input_qty) 
        {
          liquid_qty_decilitres--;
          liquid_qty_decilitres = constrain(liquid_qty_decilitres,50,150);
        }
        
        if (input_ambtemp) 
        {
          ambient_temperature -= 1.f;
          ambient_temperature = constrain(ambient_temperature,0.0,40.0);
        }
        
        if ((program_started) && !input_qty && !input_ambtemp) 
        {
          digitalWrite(MOTOR_RELAY_PIN,HIGH);
          stir_suspend = true;
          dbgln(F("MOTOR_DISABLED"),1); 
          return;
        }
        else 
        {
          m_value--;
          m_value = constrain(m_value,0,3);
          program_number = m_value;
        }
      }
      else if (LcdKeypad::LEFT_KEY == newKey)
      {
        if (self_test) {dbgln(F("LEFT"),1); return;}
        if (debug) {dbgln(F("LEFT"),1);}
        if (program_started) { program_paused = !program_paused;}
  
      }
      else if (LcdKeypad::RIGHT_KEY == newKey)
      {
        if (self_test) {dbgln(F("RIGHT"),1); return;}
        if (debug) {dbgln(F("RIGHT"),1);}
        if (program_started) { program_step++;}
        
      }
  
     
      else if (LcdKeypad::SELECT_KEY == newKey)
      {
        if (self_test) {dbgln(F("SELECT"),1); return;}
        if (debug) {dbgln(F("SELECT"),1);}
        if(input_qty) 
        {
          dbgln(F("C1"),1);
          input_qty = false;
          
        }
        if(!program_started && !input_ambtemp && !input_qty) {
          dbgln(F("C3"),1);
          program_started = true;
          input_qty = true;
          }
        if(input_ambtemp) 
        {
          input_ambtemp = false; program_started = false;
          dbgln(F("C2"),1);
        }

        if(program_paused) {program_paused = false;}
      }

      m_lcdKeypad->setCursor(0, 1);            // position the cursor at beginning of the second line
      
      if(input_qty) 
      {
        m_lcdKeypad->print(liquid_qty_decilitres,DEC);             // print the value on the second line of the display
        m_lcdKeypad->print(F("                "));  // wipe out characters behind the printed value
      }
      
      if(input_ambtemp) 
      {
        m_lcdKeypad->print(int(floor(ambient_temperature + 0.5f)),DEC);             // print the value on the second line of the display
        m_lcdKeypad->print(F("                "));  // wipe out characters behind the printed value
      }
      
      if(!program_started)
      {
        m_lcdKeypad->print(m_value,DEC);             // print the value on the second line of the display
        m_lcdKeypad->print(F("                "));  // wipe out characters behind the printed value
      }
      
      // RGB colored backlight: set according to the current value
      // monochrome backlight: set backlight on or off according to the current value
      m_lcdKeypad->setBacklight(static_cast<LcdKeypad::LcdBacklightColor>(LcdKeypad::LCDBL_WHITE & m_value));
    }
  }
};

MyLcdKeypadAdapter* myLcdAdapter;

long readVcc() 
{
 
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1126400L / result; // Back-calculate AVcc in mV
  return result;

}


void PrintStatus(float dt)
{
  myLcdKeypad->clear();
  myLcdKeypad->setCursor(0, 0);   // position the cursor at beginning of the first line
  myLcdKeypad->print(F("CT:"));   // print a Value label on the first line of the display
  myLcdKeypad->setCursor(3, 0);   // position the cursor at char 7 of the first line
  myLcdKeypad->print(Input);   // print a Value label on the first line of the display
  myLcdKeypad->setCursor(9, 0);   // position the cursor at char 7 of the first line
  myLcdKeypad->print(F("TT:"));   // print a Value label on the first line of the display
  myLcdKeypad->setCursor(13, 0);   // position the cursor at char 7 of the first line
  myLcdKeypad->print(Setpoint);   // print a Value label on the first line of the display
  myLcdKeypad->setCursor(0, 1);   // position the cursor at beginning of the first line
  myLcdKeypad->print(F("SP:"));   // print a Value label on the first line of the display
  myLcdKeypad->setCursor(3, 1);   // position the cursor at beginning of the first line
  myLcdKeypad->print(program_step);   // print a Value label on the first line of the display
  
  myLcdKeypad->setCursor(5, 1);   // position the cursor at beginning of the first line
  myLcdKeypad->print(F("ST:"));   // print a Value label on the first line of the display
  
  myLcdKeypad->setCursor(8, 1);   // position the cursor at beginning of the first line
  myLcdKeypad->print(current_program_data[2]);

  myLcdKeypad->setCursor(9, 1);   // position the cursor at beginning of the first line
  myLcdKeypad->print(F("DU:"));   // print a Value label on the first line of the display
  
  myLcdKeypad->setCursor(12, 1);   // position the cursor at beginning of the first line
  myLcdKeypad->print(dt,DEC);
}


void buzz(state s)
{

  uint16_t freq = 880;
  uint16_t half_period_us = int(1.e6f/(2*freq));

  switch(s)
  {

    case initialization:

      for(uint16_t i=0; i < freq/10; i++)
      {
        //Serial.println("buzz");
        digitalWrite(BUZZER_PIN,LOW);
        delayMicroseconds(half_period_us);
        digitalWrite(BUZZER_PIN,HIGH);
        delayMicroseconds(half_period_us);
      }

    case start_program:

      for(uint16_t i=0; i < freq/5; i++)
      {
        //Serial.println("buzz");
        digitalWrite(BUZZER_PIN,LOW);
        delayMicroseconds(half_period_us);
        digitalWrite(BUZZER_PIN,HIGH);
        delayMicroseconds(half_period_us);
      }

     
      break;

    case end_step:


      for(uint16_t i=0; i < freq*5; i++)
      {
        digitalWrite(BUZZER_PIN,LOW);
        delayMicroseconds(half_period_us);
        digitalWrite(BUZZER_PIN,HIGH);
        delayMicroseconds(half_period_us);
      }


      break;

    case end_program:

      for(uint8_t j=0; j < 3; j++)
      {
      
        for(uint16_t i=0; i < freq/2; i++)
        {
          digitalWrite(BUZZER_PIN,LOW);
          delayMicroseconds(half_period_us);
          digitalWrite(BUZZER_PIN,HIGH);
          delayMicroseconds(half_period_us);
        }
      
        if(j==2) {break;}
        delay(500);

      }
      break;

  }
}



void setup()
{

  
  pinMode(MOTOR_RELAY_PIN,OUTPUT);
  digitalWrite(MOTOR_RELAY_PIN,HIGH);
  pinMode(HEATER_SSR_PIN, OUTPUT);
  pinMode(BUZZER_PIN,OUTPUT);
  digitalWrite(BUZZER_PIN,HIGH);

  Serial.begin(9600);

  dbgln(F("BUZZ INIT"),1);
  buzz(initialization);
  dbgln((int32_t)readVcc(),1);

  myLcdKeypad = new LcdKeypad();  // instantiate an object of the LcdKeypad class, using default parameters
  myLcdAdapter = new MyLcdKeypadAdapter(myLcdKeypad);
  // Attach the specific LcdKeypadAdapter implementation (dependency injection)
  myLcdKeypad->attachAdapter(myLcdAdapter);
  
  myLcdKeypad->setCursor(0, 0);   // position the cursor at beginning of the first line
  myLcdKeypad->print(VERSION);   // print a Value label on the first line of the display
  delay(2000);

  range_sensor.init();
  range_sensor.configureDefault();
  range_sensor.setTimeout(200);
  
  // result of calibration
  range_sensor.setScaling(1);
  // max convergence time
  range_sensor.writeReg(0x01C,32);
  // offset calibration_before_10
  range_sensor.writeReg(0x024,0);
  // crosstalk calibration_before_422
  range_sensor.writeReg16Bit(0x01E,0);

  myLcdKeypad->clear();
  myLcdKeypad->setCursor(0, 0);   // position the cursor at beginning of the first line
  myLcdKeypad->print(F("RANGE_INIT"));   // print a Value label on the first line of the display
  delay(2000);

  

  dbgln(F("START"),1);
  
  if(self_test)
  {
    dbgln(F("SELF_TEST"),1);
    dbg(F("TEMPERATURE_READ_TEST:"),1);
    sensors.requestTemperatures(); // Send the command to get temperatures
    Input = float(sensors.getTempCByIndex(0));
    dbgln(Input,1);
    dbg(F("RANGE_READ_TEST:"),1);
    //range_sensor.setScaling(1);

    
      dbg(F("(Scaling = "),1);
      dbg(range_sensor.getScaling(),1);
      dbgln(F("x "),1);

    dbg(F("CONV TIME:"),1);
    dbgln(range_sensor.readReg(0x01C),1);
    range_sensor.writeReg(0x01C,32);
   
    dbg(F("SYSRANGE__RANGE_CHECK_ENABLES_default_16:"),1);
    dbgln(range_sensor.readReg(0x02D),1);
    
    range_sensor.writeReg(0x02D,19);
    dbg(F("SYSRANGE__RANGE_CHECK_ENABLES_default_16:"),1);
    dbgln(range_sensor.readReg(0x02D),1);

     dbg(F("SYSRANGE__RANGE_IGNORE_VALID_HEIGHT:"),1);
    dbgln(range_sensor.readReg(0x025),1);
    range_sensor.writeReg(0x025,0);
    dbg(F("SYSRANGE__RANGE_IGNORE_VALID_HEIGHT:"),1);
    dbgln(range_sensor.readReg(0x025),1);
    

    dbg(F("SYSRANGE__RANGE_IGNORE_THRESHOLD_default_0:"),1);
    dbgln(range_sensor.readReg16Bit(0x026),1);
    range_sensor.writeReg16Bit(0x026,0);
    dbg(F("SYSRANGE__RANGE_IGNORE_THRESHOLD_default_0:"),1);
    dbgln(range_sensor.readReg16Bit(0x026),1);
    

    
    // crosstalk 5
    dbg(F("CONV TIME:"),1);
    dbgln(range_sensor.readReg(0x01C),1);
    
    dbg(F("CROSSTALK COMP:"),1);
    dbgln(range_sensor.readReg16Bit(0x01E),1);
    
    //range_sensor.writeReg(0x01E,0x00);
    //range_sensor.writeReg(0x01F,0x00);
    
  
    range_sensor.writeReg16Bit(0x01E,130);
    dbg(F("CROSSTALK COMP:"),1);
    dbgln(range_sensor.readReg16Bit(0x01E),1);
    dbg(F("OFFSET RANGE (prev, 13):"),1);
    dbgln(range_sensor.readReg(0x024),1);
    dbg(F("OFFSET RANGE (prev, 13):"),1);
    range_sensor.writeReg(0x024,36);
    dbg(F("OFFSET RANGE (prev, 13):"),1);
    dbgln(range_sensor.readReg(0x024),1);
    
    
    
    //Serial.println(range_sensor.readReg(0x01E));
    //Serial.println(range_sensor.readReg(0x01F));
    
    
    
    for(i=0;i<20;i++)
    {
      delay(200);
      //range = range_sensor.readReg(0x064);
      range = range_sensor.readRangeSingleMillimeters();
      
      dbg(F("RESULT_RANGE_STATUS:"),1);
      dbgln(range_sensor.readReg(0x04D),1);
      avgdist += range;
      avgret += range_sensor.readReg16Bit(0x066);
      dbg(F("RANGE:"),1);
      dbgln(range,1);
      dbg(F("RETURN RATE:"),1);
      dbgln(range_sensor.readReg16Bit(0x066),1);

      if (range_sensor.timeoutOccurred()) { dbgln(F(" TIMEOUT"),1); }
    }
    
    avgdist = avgdist/20.0;
    dbgln(F("AVGDIST:"),1);
    dbgln(avgdist,1);
   
    avgret = avgret/(20.0*128.0);
    dbgln(F("AVGRET:"),1);
    dbgln(avgret,1);
    
    crosstalk = avgret*(1.0 - avgdist/100.0);
    dbg(F("CROSSTALK:"),1);
    dbgln(crosstalk,1);

    dbg(F("THR LOW:"),1);
    dbgln(range_sensor.readReg(0x01A),1);
    

    dbg(F("THR HIGH:"),1);
    dbgln(range_sensor.readReg(0x019),1);
    

    dbgln(F("MOTOR_RELAY_TEST"),1);
    digitalWrite(MOTOR_RELAY_PIN,LOW);
    delay(5000);
    digitalWrite(MOTOR_RELAY_PIN,HIGH);
    dbgln(F("HEATER_SSR_TEST"),1);
    digitalWrite(HEATER_SSR_PIN,HIGH);
    delay(5000);
    digitalWrite(HEATER_SSR_PIN,LOW);
    dbgln(F("LCD_TEST"),1);

    // does not work : myLcdKeypad->setBackLightOn(false);
    
    myLcdKeypad->setCursor(0, 0);   // position the cursor at beginning of the first line
    myLcdKeypad->print(F("ABCDEFGHIJKLMNOP"));   // print a Value label on the first line of the display

    myLcdKeypad->setCursor(0, 1);   // position the cursor at beginning of the first line
    myLcdKeypad->print(F("ABCDEFGHIJKLMNOP"));   // print a Value label on the first line of the display
    //myLcdKeypad->noDisplay();
    dbgln(F("SELF_TEST_END"),1);
  }


  sensors.requestTemperatures(); // Send the command to get temperatures
  delay(1000);
  
  Input = float(sensors.getTempCByIndex(0));

  myLcdKeypad->clear();
  myLcdKeypad->setCursor(0, 0);   // position the cursor at beginning of the first line
  myLcdKeypad->print(F("TEMP:"));   // print a Value label on the first line of the display
  myLcdKeypad->setCursor(6, 0);   // position the cursor at beginning of the first line
  myLcdKeypad->print(Input);   // print a Value label on the first line of the display
  delay(2000); 

/*
  avgdist = 0.0;
  avgret = 0.0;
  
  for(i=0;i<20;i++)
  {
    delay(200);
    //range = range_sensor.readReg(0x064);
    myLcdKeypad->clear();
    myLcdKeypad->setCursor(0, 0);   // position the cursor at beginning of the first line
    myLcdKeypad->print("RANGE_I:");   // print a Value label on the first line of the display
    myLcdKeypad->setCursor(8, 0);   // position the cursor at beginning of the first line
    myLcdKeypad->print(i);   // print a Value label on the first line of the display
    //delay(2000); 

    range = range_sensor.readRangeSingleMillimeters();
    avgdist += range;
    avgret += range_sensor.readReg16Bit(0x066);
    Serial.println(range);
    //Serial.print("RETURN RATE:");
    //Serial.println();

    if (range_sensor.timeoutOccurred()) { Serial.println(" TIMEOUT"); }
  }
    
    avgdist = avgdist/20.0;
    Serial.println("AVGDIST:");
    Serial.println(avgdist);
   
    myLcdKeypad->clear();
    myLcdKeypad->setCursor(0, 0);   // position the cursor at beginning of the first line
    myLcdKeypad->print("AVGDIST:");   // print a Value label on the first line of the display
    myLcdKeypad->setCursor(8, 0);   // position the cursor at beginning of the first line
    myLcdKeypad->print(avgdist);   // print a Value label on the first line of the display
    delay(2000);  

    
  */


    // initialize ambient temperature based on liquid temperature probe.
    // Requires that milk is poured AFTER initialization to be effective.
    // Ideally a separate ambient temperature sensor is required, far from any source of heat.

    sensors.requestTemperatures();
    ambient_temperature = float(sensors.getTempCByIndex(0));


 
    myLcdKeypad->clear();
    myLcdKeypad->setCursor(0, 0);   // position the cursor at beginning of the first line
    myLcdKeypad->print(F("INPUT AMBT TEMP:"));   // print a Value label on the first line of the display
    // change to display temp at init
    myLcdKeypad->setCursor(0, 1);            // position the cursor at beginning of the second line
    myLcdKeypad->print(int(floor(ambient_temperature + 0.5f)),DEC);
    // end change to display temp at init    

    dbgln(F("INPUT AMBIENT TEMPERATURE"),1);

    myLcdAdapter->input_ambtemp = true;
    while(myLcdAdapter->input_ambtemp)
    {
      //delay(100);
      //Serial.println(readVcc());
      scheduleTimers();  // Get the timer(s) ticked, in particular the LcdKeypad dirver's keyPollTimer
    }

    //delay(1000);
    myLcdKeypad->clear();
    myLcdKeypad->setCursor(0, 0);   // position the cursor at beginning of the first line
    myLcdKeypad->print(F("SELECT PROGRAM:"));   // print a Value label on the first line of the display
    
  
  
    windowStartTimeHeat = millis();
    windowStartTimeStir = windowStartTimeHeat;
    dbgln(F("END SETUP"),1);
}



void loop() {

  static bool pidmode = false;
  // put your main code here, to run repeatedly:
  pinMode(HEATER_SSR_PIN, OUTPUT);
  pinMode(MOTOR_RELAY_PIN,OUTPUT);
  
  if (self_test) {return;}
  
  while(!myLcdAdapter->program_started)
  {
    //Serial.println("PROG START WAIT");
    scheduleTimers();  // Get the timer(s) ticked, in particular the LcdKeypad dirver's keyPollTimer
  }
  
  // program_started = true means the user has selected a program.
  // program_init = false means the selected program needs to be initialized
    
  scheduleTimers();  // Get the timer(s) ticked, in particular the LcdKeypad dirver's keyPollTimer
  
  /*
  while(program_paused)
  {
    scheduleTimers();
    
  }
  */

  if(!program_init) 
  {

    dbgln(F("SELECTED PROGRAM:"),1);
    dbgln(myLcdAdapter->program_number,1);
    dbgln(F("PROGRAM STEP:"),1);
    dbgln(program_step,1);

    bool empty_program_step = true;

    for(k=0;k<5;k++)
    {
      current_program_data[k] = pgm_read_word(&(progdata[myLcdAdapter->program_number][k+program_step*5]));    
      if (current_program_data[k] != 0) {empty_program_step = false;} // note : if program step is padded with zeros, it means program end.
      dbgln(F("READ PROGRAM DATA"),1);
      dbgln(current_program_data[k],1);
    }

    if (empty_program_step) 
    {
    
          digitalWrite(HEATER_SSR_PIN, LOW); // make sure heat is turned off
          digitalWrite(MOTOR_RELAY_PIN,HIGH);
          program_ended = true;
          dbgln(F("EMPTY PROGRAM STEP"),1);
          myLcdKeypad->clear();
          myLcdKeypad->setCursor(0, 0);   // position the cursor at beginning of the first line
          myLcdKeypad->print(F("PROGRAM END"));
          dbgln(F("BUZZ PROGRAM_END"),1);
          buzz(end_program);
          dbgln(F("PROGRAM_END"),1);
          while(true); // halt processing
   
    }



    /*
    for(i=0;i<20;i++)
      {
        //range = range_sensor.readReg(0x064);
        range = range_sensor.readRangeSingleMillimeters();
        avgdist += range;
        avgret += range_sensor.readReg16Bit(0x066);
        Serial.println(range);
        //Serial.print("RETURN RATE:"));
        //Serial.println();

        if (range_sensor.timeoutOccurred()) { Serial.println(" TIMEOUT")); }
      }
      
      avgdist = avgdist/20.0;
      Serial.println("AVG_DIST:"));
      Serial.println(avgdist);
      liquid_qty_litres = constrain(3.1416*1.375*1.375*(250 - avgdist)/100.0,0,15);
    
    
    */

    if (current_program_data[0] == 0) { stir_only = true; myLcdAdapter->input_qty = false;}
    
    if ((program_step == 0) && !stir_only)
    {

      myLcdAdapter->input_qty = true;
      myLcdKeypad->clear();
      myLcdKeypad->setCursor(0, 0);   // position the cursor at beginning of the first line
      myLcdKeypad->print(F("INPUT QTY dL:"));   // print a Value label on the first line of the display

      while(myLcdAdapter->input_qty)
      {
        scheduleTimers();
        delay(100);
        dbgln(F("WAIT"),1);
      }

      liquid_qty_litres = (myLcdAdapter->liquid_qty_decilitres)/10.0;

    }


    // override
    //liquid_qty_litres = 9.6;
    //ambient_temperature = 20.0;
    dbgln(F("LIQUID_QTY_LITRES:"),1);
    dbgln(liquid_qty_litres,1);
    
    
    //Kp = constrain(52*liquid_qty_litres,10,780);
    //Kp = current_program_data[2];

    dbgln(F("kp="),1);
    dbgln(Kp,1);

    
    if(!stir_only)
    {
      Setpoint = float(current_program_data[0]); // Degrees Celsius
      
      if ((Setpoint - Input) < 2.0) { temperature_rise_time = 600; } // We don't bother having a controlled rise if it is already close to target, so we can jump to closed loop already}
      else { temperature_rise_time = float(current_program_data[1]);}
      
      if(current_program_data[2] > 1)
      {
        stir_disabled = false;
        stir_duty_pct = constrain(current_program_data[2],2,100);
      }
      else
      {
        stir_disabled = !bool(current_program_data[2]);
        stir_duty_pct = current_program_data[2]*100;
      }
      
      dbgln(F("SETPOINT"),1);
      dbgln(Setpoint,1);
      dbgln(F("TEMP_RISE_TIME"),1);
      dbgln(temperature_rise_time,1);


      //https://www.tlv.com/global/TI/steam-theory/overall-heat-transfer-coefficient.html#:~:text=The%20overall%20heat%20transfer%20coefficient,ft2%C2%B0F)%5D.
      // milk density p = 1.035 kg/m2
      // milk viscosity v = 0.0015 N*s/m2
      // milk specific heat 3950 J/(Kg*K)
      // milk thermal conductivity =  0.622 W/(m*K)
      // milk thermal expansion coefficient (used water) .000210 1/K


      milk_thickness = (1000*liquid_qty_litres)/(PI*vessel_radius*vessel_radius); // metres
      
      vertical_surface_U_value = 1.0/(1.0/liquid_convective_coefficient_vertical + vessel_thickness/vessel_thermal_conductivity + 1.0/air_convective_coefficient_vertical);
      horizontal_surface_U_value = 1.0/(1.0/liquid_convective_coefficient_horizontal + milk_thickness/milk_thermal_conductivity + 1.0/air_convective_coefficient_horizontal);
      // OPEN LOOP CONTROL when rise time is specified and more than 2 degrees from target
      
      // added fill factor and its influence on exposed internal vertical steel surface to air. // "1 +" accounts for the external wall. 
      heat_transfer_rate_vertical =  (vertical_surface_U_value*vessel_area)*(1 + (vessel_volume_l - liquid_qty_litres)/vessel_volume_l)*(Input - ambient_temperature);
      
      heat_transfer_rate_horizontal = horizontal_surface_U_value*(PI*vessel_radius*vessel_radius)*(Input - ambient_temperature);
      total_heat_loss = heat_transfer_rate_vertical + heat_transfer_rate_horizontal;

      //Calculate total energy needed to bring to thermal equlibrium Q = mc*(Tfinal - Tsstart)
      // temperature_rise = Tfinal - Tstart
      // temperature_rise_time = tfinal - tstart

      // (P = Q/temperature_rise_time) + total_heat_loss
      // when duty is 100 percent = 1200W
      // Duty = P / 12 (12W per 1% duty cycle), assuming the hot plate is 1200W rated nominal power
      // steady state gain : 300 for 5.7l is quite ok
      total_energy_required = liquid_qty_litres*milk_density*milk_heat_capacity*(Setpoint - Input);
      
      if (temperature_rise_time <= programElapsedTime)
        {
          temperature_rise_time = float(current_program_data[1]);
        }

      power_required = boost_factor*((total_energy_required/(temperature_rise_time - programElapsedTime)) + total_heat_loss);
      //power_required = (total_energy_required/temperature_rise_time);
      
      // hot plate is 1500W max power
      power_required = constrain(power_required,0.f,hotplate_max_power);
      
      if (current_program_data[1] == -1) 
      { open_rise = true; 
        Duty2 = 100; 
        temperature_rise_time = 3600; //security limit step time for open temperature rise
        dbgln(F("OPEN_RISE"),1);
      } 
      else 
      {
        temperature_rise_time = float(current_program_data[1]);
        Duty2 = constrain(100.f*power_required/hotplate_max_power,0.f,100.f);
      } // restoring the value to get real program step time (maybe tweak was applied in case delta_t < 2.0)
      
      dbg(F("POWER_REQUIRED:"),1);
      dbgln(power_required,1);
      Kp = constrain(100.f*(power_required/5.f)/hotplate_max_power,0.f,100.f)*WindowSizeHeat/100.f;
      
      dbg(F("Duty2:"),1);
      dbgln(Duty2,1);

      //Duty2 = 100 - Duty;
      dbg(F("Kp="),1);
      dbgln(Kp,1);
      myPID.SetTunings(Kp,Ki,Kd);
      //initialize the variables we're linked to
  
      //tell the PID to range between 0 and the full window size
      myPID.SetOutputLimits(0, WindowSizeHeat);
      
      myPID.SetSampleTime(5000);
      //PID set to manual (duty not controlled by the PID algorithm)
      myPID.SetMode(MANUAL);
    }
    else // else stir_only == true 
    {
      PrintStatus(0);  // print a Value label on the first line of the display
    }
  // start stirring;
  
    if (stir_disabled)
    {
      dbgln(F("STIR_DISABLED"),1);
      digitalWrite(MOTOR_RELAY_PIN,HIGH);
    }
    else
    {
      dbgln(F("STIR_ENABLED"),1);
      digitalWrite(MOTOR_RELAY_PIN,LOW);
    }
    
    
    dbgln(F("STEP_INIT_END"),1);
    program_init = true;
    dbgln(F("BUZZ START PROGRAM"),1);
    buzz(start_program);
    programStartTime = millis();
  } //end if(!program_init)

  
  sensors.requestTemperatures(); // Send the command to get temperatures

  Input = float(sensors.getTempCByIndex(0));

  if (!stir_only) 
  {

    heat_transfer_rate_vertical =  (vertical_surface_U_value*vessel_area)*(1 + (vessel_volume_l - liquid_qty_litres)/vessel_volume_l)*(Input - ambient_temperature);
    heat_transfer_rate_horizontal = horizontal_surface_U_value*(PI*vessel_radius*vessel_radius)*(Input - ambient_temperature);
    total_heat_loss = heat_transfer_rate_vertical + heat_transfer_rate_horizontal;

    //power_required = total_energy_required/(temperature_rise_time - programElapsedTime) + total_heat_loss;
    //power_required = total_energy_required/(temperature_rise_time - programElapsedTime);
    
    // 2.0 correction factor.
    // hot plate is 1500W max power

    if (fabs(Setpoint - Input) < constrain(7.0*float(Duty2*1.6)/100.0,4.0,7.0))
    {
      // closed looop (PID) temperature rise mode when we get close to setpoint. the fine-tune formula
      // depends also on the open loop calculated Duty2. 7.0, 1.6 4.0 and 7.0 were empirically determined.

      if (!pidmode)
      {
        programSwitchPoint = programElapsedTime;
        pidmode = true;
      }

      total_energy_required = liquid_qty_litres*milk_density*milk_heat_capacity*(Setpoint - Input);
      power_required = boost_factor*(total_energy_required/(temperature_rise_time - programSwitchPoint) + total_heat_loss);
      power_required = constrain(power_required,0.0,hotplate_max_power);
    

      if (current_program_data[1] == -1) 
      { 
        open_rise = true; 
        Duty2 = 100.0; 
        temperature_rise_time = 3600; 
        dbgln(F("OPEN_RISE"),1);
      } //security limit step time for open (max_power) temperature rise
      else 
      {
        temperature_rise_time = float(current_program_data[1]);
        Duty2 = constrain(100.f*power_required/hotplate_max_power,0.f,100.f);
      } // restoring the value to get real program step time (maybe tweak was applied in case delta_t < 2.0)

      //Serial.print("POWER_REQUIRED:");
      //Serial.println(power_required);
      Kp = constrain(100.f*(power_required/5.f)/hotplate_max_power,100.f*total_heat_loss/hotplate_max_power,100.f)*WindowSizeHeat/100.f;
      myPID.SetSampleTime(5000);
      dbg(F("SOL:"),1);
      dbgln(WindowSizeHeat*(1.0 - constrain(Duty2/100.0,0.0,100.0)),1);
      myPID.SetOutputLimits(WindowSizeHeat*(1.0 - constrain(Duty2/100.0,0.0,100.0)),WindowSizeHeat); 
      myPID.SetTunings(Kp,Ki,Kd);
      myPID.SetMode(AUTOMATIC);


      myPID.Compute();
      //Duty = int(100.0*(1.0 - float(Output)/float(WindowSizeHeat)));
    } // end if ... (closed loop)
    else if(Input - Setpoint > 1.0) 
    // override - disable heating completely 
    // as a safety measure in case of temperature overshoot.
    {
      myPID.SetMode(MANUAL);
      power_required = 0.0;
      Duty2 = 0.0;
      Output = WindowSizeHeat;
      pidmode = false;
    } // end else if(Input - Setpoint > 1.0)
    else // open loop temperature rise.
    {
      myPID.SetMode(MANUAL);
      total_energy_required = liquid_qty_litres*milk_density*milk_heat_capacity*(Setpoint - Input);
      power_required = boost_factor*(total_energy_required/(temperature_rise_time - programElapsedTime) + total_heat_loss);
      power_required = constrain(power_required,0.f,hotplate_max_power);
      Duty2 = constrain(100.f*power_required/hotplate_max_power,0.f,100.f);
      Output = int(float(WindowSizeHeat)*float(100.0 - Duty2)/100.0);
      // we nevertheless set Output so that the same EffectiveDuty formula is used whether PID is manual or auto.
      pidmode = false;  
    } // end else open loop temperature rise

    EffectiveDuty = 100.0*float((WindowSizeHeat - Output)/WindowSizeHeat);
    // EffectiveDuty is for printing a human readable duty cycle

    if((stir_duty_pct >= 2 || stir_duty_pct <= 99) & !stir_disabled & !stir_suspend)
    {
      if (millis() - windowStartTimeStir > WindowSizeStir)
      { 
        //time to shift the stir motor Relay Window
        windowStartTimeStir = millis(); //alternate method - does not take into account processing time 
      }
    }
    
    if (millis() - windowStartTimeHeat > WindowSizeHeat)
    { 
      //time to shift the heating SSR Window
      //windowStartTimeHeat += WindowSizeHeat; //
      windowStartTimeHeat = millis(); //alternate method - does not take into account processing time
      
      PrintStatus(EffectiveDuty);
      // print a Value label on the first line of the display
      
      programElapsedTime = (millis() - programStartTime)/1000;
      dbg(F(" PROGRAM_STEP_ELAPSED_TIME:"),1);
      dbgln(programElapsedTime,1);

      if (programElapsedTime > temperature_rise_time)
      {
        // we reached program step end
        // ensure heating is cut and stirring is off
        digitalWrite(HEATER_SSR_PIN,LOW);
        digitalWrite(MOTOR_RELAY_PIN,HIGH);
        program_step++;
        programElapsedTime = 0;
        program_init = false;

        if (program_step >= 4) // we reached program end. use watchdog to reset 
        {
          digitalWrite(HEATER_SSR_PIN, LOW); // make sure heat is turned off
          digitalWrite(MOTOR_RELAY_PIN,HIGH);
          program_ended = true;
          dbgln(F("LAST PROGRAM STEP END"),1);
          myLcdKeypad->clear();
          myLcdKeypad->setCursor(0, 0);   // position the cursor at beginning of the first line
          myLcdKeypad->print(F("PROGRAM END"));
          dbgln(F("BUZZ PROGRAM_END"),1);
          buzz(end_program);
          dbgln(F("PROGRAM_END"),1);
          while(true); // halt processing
        }


        dbg(F("NEXT_STEP:"),1);
        dbgln(program_step,1);
        if (current_program_data[3])
        {
          digitalWrite(HEATER_SSR_PIN, LOW); // make sure heat is turned off
          // program has a pause after step end.
          myLcdAdapter->program_paused = true;
          buzz(end_step);
          myLcdKeypad->clear();
          myLcdKeypad->setCursor(0, 0);   // position the cursor at beginning of the first line
          myLcdKeypad->print(F("PROGRAM PAUSED!"));
          myLcdKeypad->setCursor(0, 1);   // position the cursor at beginning of the first line
          myLcdKeypad->print(F("SELECT TO RESUME"));
  

          while(myLcdAdapter->program_paused)
          {
            scheduleTimers();
          }

        }

      }
      
      //myLcdKeypad->clear();

      //Duty = int(100 *(Output/WindowSizeHeat)); 
      dbg(F("CT:"),1);
      dbg(Input,1);
      dbg(F(" TT:"),1);
      dbg(Setpoint,1);
      dbg(F(" D2:"),1);
      dbg(EffectiveDuty,1);
      dbg(F(" SW DT:"),1);
      dbg(constrain(7.0*float(Duty2)*1.6/100.0,4.0,7.0),1);     
      dbg(F(" POW REQ:"),1);
      dbgln(power_required,1);
      dbg(F(" MODE:"),1);
      dbgln(myPID.GetMode(),1);
      //Serial.print(" O:");
      //Serial.println(Output);
      

    }
    

    //************************************************
    // * turn the output pin on/off based on pid output
    // ************************************************

    if (open_rise) 
    { 
      digitalWrite(HEATER_SSR_PIN, HIGH);
      dbg(F("OPEN_RISE_HEAT\n"),5);
    }
    else
    {

      if (Output < millis() - windowStartTimeHeat) 
      {
        digitalWrite(HEATER_SSR_PIN, HIGH);
        //delay(5000);
        dbg(F("HEAT\n"),5);
        dbg(F("Output Millis windowStartTimeHeat\t"),5);
        dbg(Output,5);
        dbg(F("\t"),5);
        dbg((int32_t) millis(),5);
        dbg(F("\t"),5);
        dbg((int32_t) windowStartTimeHeat,5);
        dbg(F("\n"),5);
        
        

      }
      else 
      {
        digitalWrite(HEATER_SSR_PIN, LOW);
        dbg(F("NO_HEAT\n"),5);
      }

    } // end else open_rise

    //************************************************
    // * turn the stirring on/off based on duty cycle configuration
    // ************************************************
    if(!stir_suspend)
    {
      if (stir_duty_pct*WindowSizeStir/100 < millis() - windowStartTimeStir)
      {
          digitalWrite(MOTOR_RELAY_PIN, LOW); // motor enabled
        
      }
      else
      {
          digitalWrite(MOTOR_RELAY_PIN,HIGH); // motor disabled

      }
    }

  } // end if(!stir_only)
  else
  {
    scheduleTimers();
    dbgln(F("SCHED_TIMERS"),1);
    delay(100);
    PrintStatus(0);  // print a Value label on the first line of the display
  } //end else ...(!stironly)

}