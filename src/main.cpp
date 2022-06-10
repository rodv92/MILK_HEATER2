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
VL6180X range_sensor;


#define MOTOR_RELAY_PIN 3
#define HEATER_SSR_PIN 10

bool self_test = false;
bool debug = true;
bool program_init = false;
bool program_ended = false;
//bool program_paused = false;
bool stirr_only = false;
bool stirr_disabled = false;
uint8_t program_step = 0;

String msg_id[3] = {"SELECT_PROGRAM","PROGRAM_END","ADD_RENNET"};

const PROGMEM int16_t progdata[4][16] = 
{
  // if program is an array of 0 values with 1 for stirring then it will only perform stirring without end.
  {0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0} ,

  //format_of_array {target_temp,rise_time,stir,pause_after,target_temp2,rise_time2,stir2, etc...}
  // if target_temperature = -1 then disable heater
  
  // program fromage suisse type emmental
 
 
 /*
  {32,1200,0,0,
  32,2400,0,1,
  32,2400,1,0,
  49,1800,1,0}
 */

//programme mantien temperature
 {34,7200,0,1,
  0,0,0,0,
  0,0,0,0,
  0,0,0,0}
 ,

 // programme pasteurisation
 {72,1200,0,0,
  72,180,1,1,
  32,2400,1,0,
  32,2400,1,0}
 ,
 /*
  {35,700,0,0,
  35,1200,1,1,
  32,1200,1,0,
  32,1200,1,0}
*/
  
  {53,900,0,0,
  53,3600,0,1,
  40,2400,1,0,
  49,1800,1,0}
 
};
uint16_t current_program_data[4];
//Define Variables we'll be connecting to
double Setpoint, Input, Output = 0.0;


// Units S.I
double liquid_qty_litres;
double heat_transfer_rate;
const double vessel_radius = 0.1375;
const double vessel_height = 0.245;
const double vessel_area = vessel_height*vessel_radius*2*PI;
const double vessel_thickness = 0.001;
const double ambient_temperature = 18.0;
const double vessel_thermal_conductivity = 25.0;

const double liquid_convective_coefficient_vertical = 12.4; //(milk / steel vessel interface from 50° to 35°)
 const double air_convective_coefficient_vertical = 3.8; //(from ambient temperature at 20°C and metal surface at 35°C)
 
 const double air_convective_coefficient_horizontal = 4.8; //(milk / to air interface from 40° to 30°)
 const double liquid_convective_coefficient_horizontal = 13.6; //( steel bottom to milk interface from 50°C to 35°C)
 
 const double milk_thermal_conductivity = 0.622;
 const double milk_heat_capacity = 3970; // J/kg;
 const double milk_density = 1.036; // kg/l

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

double Kp=300, Ki=20, Kd=-50000;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, P_ON_E, REVERSE);

int WindowSize = 1000;
int WindowSizeRise = 20000;
uint8_t EffectiveDuty = 0;
uint8_t Duty2 = 0;

unsigned long windowStartTime;
unsigned long windowStartTimeRise;
unsigned long programStartTime;
unsigned long programElapsedTime;
//bool program_started; // program is running

// Implement specific LcdKeypadAdapter in order to allow receiving key press events
class MyLcdKeypadAdapter : public LcdKeypadAdapter
{
private:
  LcdKeypad* m_lcdKeypad;
  uint8_t m_value; // program number
public:
  bool program_started; // program status
  bool program_paused;
  bool input_qty;
  uint8_t liquid_qty_decilitres;
  uint8_t program_number; // public program number
  MyLcdKeypadAdapter(LcdKeypad* lcdKeypad)
  : m_lcdKeypad(lcdKeypad)
  , m_value(0)
  , program_number(1)
  , program_paused(false)
  , program_started(false)
  , liquid_qty_decilitres(50)
  , input_qty(false)
  { }

  // Specific handleKeyChanged() method implementation - define your actions here
  void handleKeyChanged(LcdKeypad::Key newKey)
  {
    if (0 != m_lcdKeypad)
    {
      if (LcdKeypad::UP_KEY == newKey)
      {
        if (self_test) {Serial.println("UP"); return;}
        if (debug) {Serial.println("UP");}
        if (input_qty) 
        {
          liquid_qty_decilitres++;
          liquid_qty_decilitres = constrain(liquid_qty_decilitres,50,150);
        }
        if ((program_started) && !input_qty) { digitalWrite(MOTOR_RELAY_PIN,LOW); return;}
        else
        {
          m_value++;
          m_value = constrain(m_value,0,3);
          program_number = m_value;
        }
        
      }
      else if (LcdKeypad::DOWN_KEY == newKey)
      {
        if (self_test) {Serial.println("DOWN"); return;}
        if (debug) {Serial.println("DOWN");}
        if (input_qty) 
        {
          liquid_qty_decilitres--;
          liquid_qty_decilitres = constrain(liquid_qty_decilitres,50,150);
        }
        if ((program_started) && !input_qty) { digitalWrite(MOTOR_RELAY_PIN,HIGH); return;}
        else 
        {
          m_value--;
          m_value = constrain(m_value,0,3);
          program_number = m_value;
        }
      }
      else if (LcdKeypad::LEFT_KEY == newKey)
      {
        if (self_test) {Serial.println("LEFT"); return;}
        if (debug) {Serial.println("LEFT");}
        if (program_started) { program_paused = !program_paused;}
  
      }
      else if (LcdKeypad::RIGHT_KEY == newKey)
      {
        if (self_test) {Serial.println("RIGHT"); return;}
        if (debug) {Serial.println("RIGHT");}
        if (program_started) { program_step++;}
        
      }
  
     
      else if (LcdKeypad::SELECT_KEY == newKey)
      {
        if (self_test) {Serial.println("SELECT"); return;}
        if (debug) {Serial.println("SELECT");}
        if(input_qty) {input_qty = false;}
        if(!program_started) {program_started = true;}
        if(program_paused) {program_paused = false;}
      }

      m_lcdKeypad->setCursor(0, 1);            // position the cursor at beginning of the second line
      
      if(input_qty) 
      {
        m_lcdKeypad->print(liquid_qty_decilitres,DEC);             // print the value on the second line of the display
        m_lcdKeypad->print("                ");  // wipe out characters behind the printed value
      }
      if(!program_started)
      {
        m_lcdKeypad->print(m_value,DEC);             // print the value on the second line of the display
        m_lcdKeypad->print("                ");  // wipe out characters behind the printed value
      }
      
      // RGB colored backlight: set according to the current value
      // monochrome backlight: set backlight on or off according to the current value
      m_lcdKeypad->setBacklight(static_cast<LcdKeypad::LcdBacklightColor>(LcdKeypad::LCDBL_WHITE & m_value));
    }
  }
};

MyLcdKeypadAdapter* myLcdAdapter;

void PrintStatus(uint8_t dt)
{
  myLcdKeypad->clear();
  myLcdKeypad->setCursor(0, 0);   // position the cursor at beginning of the first line
  myLcdKeypad->print("CT:");   // print a Value label on the first line of the display
  myLcdKeypad->setCursor(3, 0);   // position the cursor at char 7 of the first line
  myLcdKeypad->print(Input);   // print a Value label on the first line of the display
  myLcdKeypad->setCursor(9, 0);   // position the cursor at char 7 of the first line
  myLcdKeypad->print("TT:");   // print a Value label on the first line of the display
  myLcdKeypad->setCursor(13, 0);   // position the cursor at char 7 of the first line
  myLcdKeypad->print(Setpoint);   // print a Value label on the first line of the display
  myLcdKeypad->setCursor(0, 1);   // position the cursor at beginning of the first line
  myLcdKeypad->print("SP:");   // print a Value label on the first line of the display
  myLcdKeypad->setCursor(3, 1);   // position the cursor at beginning of the first line
  myLcdKeypad->print(program_step);   // print a Value label on the first line of the display
  
  myLcdKeypad->setCursor(5, 1);   // position the cursor at beginning of the first line
  myLcdKeypad->print("ST:");   // print a Value label on the first line of the display
  
  myLcdKeypad->setCursor(8, 1);   // position the cursor at beginning of the first line
  myLcdKeypad->print(current_program_data[2]);

  myLcdKeypad->setCursor(9, 1);   // position the cursor at beginning of the first line
  myLcdKeypad->print("DU:");   // print a Value label on the first line of the display
  
  myLcdKeypad->setCursor(12, 1);   // position the cursor at beginning of the first line
  myLcdKeypad->print(dt);
}
void setup()
{
  pinMode(MOTOR_RELAY_PIN,OUTPUT);
  digitalWrite(MOTOR_RELAY_PIN,HIGH);
  pinMode(HEATER_SSR_PIN, OUTPUT);


  myLcdKeypad = new LcdKeypad();  // instantiate an object of the LcdKeypad class, using default parameters
  myLcdAdapter = new MyLcdKeypadAdapter(myLcdKeypad);
  // Attach the specific LcdKeypadAdapter implementation (dependency injection)
  myLcdKeypad->attachAdapter(myLcdAdapter);
  
  myLcdKeypad->setCursor(0, 0);   // position the cursor at beginning of the first line
  myLcdKeypad->print("INIT");   // print a Value label on the first line of the display
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
  myLcdKeypad->print("RANGE_INIT");   // print a Value label on the first line of the display
  delay(2000);
  Serial.begin(9600);

  

  Serial.println("START");
  
  if(self_test)
  {
    Serial.println("SELF_TEST");
    Serial.print("TEMPERATURE_READ_TEST:");
    sensors.requestTemperatures(); // Send the command to get temperatures
    Input = float(sensors.getTempCByIndex(0));
    Serial.println(Input);
    Serial.print("RANGE_READ_TEST:");
    //range_sensor.setScaling(1);

    
      Serial.print("(Scaling = ");
      Serial.print(range_sensor.getScaling());
      Serial.println("x) ");

    Serial.print("CONV TIME:");
    Serial.println(range_sensor.readReg(0x01C));
    range_sensor.writeReg(0x01C,32);
   
    Serial.print("SYSRANGE__RANGE_CHECK_ENABLES_default_16:");
    Serial.println(range_sensor.readReg(0x02D));
    
    range_sensor.writeReg(0x02D,19);
    Serial.print("SYSRANGE__RANGE_CHECK_ENABLES_default_16:");
    Serial.println(range_sensor.readReg(0x02D));

     Serial.print("SYSRANGE__RANGE_IGNORE_VALID_HEIGHT:");
    Serial.println(range_sensor.readReg(0x025));
    range_sensor.writeReg(0x025,0);
    Serial.print("SYSRANGE__RANGE_IGNORE_VALID_HEIGHT:");
    Serial.println(range_sensor.readReg(0x025));
    

    Serial.print("SYSRANGE__RANGE_IGNORE_THRESHOLD_default_0:");
    Serial.println(range_sensor.readReg16Bit(0x026));
    range_sensor.writeReg16Bit(0x026,0);
    Serial.print("SYSRANGE__RANGE_IGNORE_THRESHOLD_default_0:");
    Serial.println(range_sensor.readReg16Bit(0x026));
    

    
    // crosstalk 5
    Serial.print("CONV TIME:");
    Serial.println(range_sensor.readReg(0x01C));
    
    Serial.print("CROSSTALK COMP:");
    Serial.println(range_sensor.readReg16Bit(0x01E));
    
    //range_sensor.writeReg(0x01E,0x00);
    //range_sensor.writeReg(0x01F,0x00);
    
  
    range_sensor.writeReg16Bit(0x01E,130);
    Serial.print("CROSSTALK COMP:");
    Serial.println(range_sensor.readReg16Bit(0x01E));
    Serial.print("OFFSET RANGE (prev, 13):");
    Serial.println(range_sensor.readReg(0x024));
    Serial.print("OFFSET RANGE (prev, 13):");
    range_sensor.writeReg(0x024,36);
    Serial.print("OFFSET RANGE (prev, 13):");
    Serial.println(range_sensor.readReg(0x024));
    
    
    
    //Serial.println(range_sensor.readReg(0x01E));
    //Serial.println(range_sensor.readReg(0x01F));
    
    
    
    for(i=0;i<20;i++)
    {
      delay(200);
      //range = range_sensor.readReg(0x064);
      range = range_sensor.readRangeSingleMillimeters();
      
      Serial.print("RESULT_RANGE_STATUS:");
      Serial.println(range_sensor.readReg(0x04D));
      avgdist += range;
      avgret += range_sensor.readReg16Bit(0x066);
      Serial.print("RANGE:");
      Serial.println(range);
      Serial.print("RETURN RATE:");
      Serial.println(range_sensor.readReg16Bit(0x066));

      if (range_sensor.timeoutOccurred()) { Serial.println(" TIMEOUT"); }
    }
    
    avgdist = avgdist/20.0;
    Serial.println("AVGDIST:");
    Serial.println(avgdist);
   
    avgret = avgret/(20.0*128.0);
    Serial.println("AVGRET:");
    Serial.println(avgret);
    
    crosstalk = avgret*(1.0 - avgdist/100.0);
    Serial.print("crosstalk:");
    Serial.println(crosstalk);

    Serial.print("THR LOW:");
    Serial.println(range_sensor.readReg(0x01A));
    

    Serial.print("THR HIGH:");
    Serial.println(range_sensor.readReg(0x019));
    

    Serial.println("MOTOR_RELAY_TEST");
    digitalWrite(MOTOR_RELAY_PIN,LOW);
    delay(5000);
    digitalWrite(MOTOR_RELAY_PIN,HIGH);
    Serial.println("HEATER_SSR_TEST");
    digitalWrite(HEATER_SSR_PIN,HIGH);
    delay(5000);
    digitalWrite(HEATER_SSR_PIN,LOW);
    Serial.println("LCD_TEST");

    // does not work : myLcdKeypad->setBackLightOn(false);
    
    myLcdKeypad->setCursor(0, 0);   // position the cursor at beginning of the first line
    myLcdKeypad->print("ABCDEFGHIJKLMNOP");   // print a Value label on the first line of the display

    myLcdKeypad->setCursor(0, 1);   // position the cursor at beginning of the first line
    myLcdKeypad->print("ABCDEFGHIJKLMNOP");   // print a Value label on the first line of the display
    //myLcdKeypad->noDisplay();
    Serial.println("SELF_TEST_END");
  }


  sensors.requestTemperatures(); // Send the command to get temperatures
  delay(1000);
  
  Input = float(sensors.getTempCByIndex(0));

  myLcdKeypad->clear();
  myLcdKeypad->setCursor(0, 0);   // position the cursor at beginning of the first line
  myLcdKeypad->print("TEMP:");   // print a Value label on the first line of the display
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
    myLcdKeypad->clear();
    myLcdKeypad->setCursor(0, 0);   // position the cursor at beginning of the first line
    myLcdKeypad->print("SELECT PROGRAM:");   // print a Value label on the first line of the display
    
  
  
  windowStartTime = millis();
  windowStartTimeRise = millis();

 

  Serial.println("END SETUP");
}


void loop() {
  // put your main code here, to run repeatedly:
  scheduleTimers();  // Get the timer(s) ticked, in particular the LcdKeypad dirver's keyPollTimer
  pinMode(HEATER_SSR_PIN, OUTPUT);
  pinMode(MOTOR_RELAY_PIN,OUTPUT);
  
  if (self_test) {return;}
  if (program_ended) 
  {
    Serial.println("PROGRAM_END");
    delay(10000);
    return;
  }

  while(!myLcdAdapter->program_started)
  {
    scheduleTimers();  // Get the timer(s) ticked, in particular the LcdKeypad dirver's keyPollTimer
  }
  // program_started = true means the user has selected a program.
  // program_init = false means the selected program needs to be initialized
    

  /*
  while(program_paused)
  {
    scheduleTimers();
    
  }
  */

  if(!program_init) 
  {

    Serial.println("SELECTED PROGRAM:");
    Serial.println(myLcdAdapter->program_number);
    Serial.println("PROGRAM STEP:");
    Serial.println(program_step);

    for(k=0;k<4;k++)
    {
      current_program_data[k] = pgm_read_word(&(progdata[myLcdAdapter->program_number][k+program_step*4]));
      Serial.println("READ PROGRAM DATA");
      Serial.println(current_program_data[k]);
    }



    /*
    for(i=0;i<20;i++)
      {
        //range = range_sensor.readReg(0x064);
        range = range_sensor.readRangeSingleMillimeters();
        avgdist += range;
        avgret += range_sensor.readReg16Bit(0x066);
        Serial.println(range);
        //Serial.print("RETURN RATE:");
        //Serial.println();

        if (range_sensor.timeoutOccurred()) { Serial.println(" TIMEOUT"); }
      }
      
      avgdist = avgdist/20.0;
      Serial.println("AVG_DIST:");
      Serial.println(avgdist);
      liquid_qty_litres = constrain(3.1416*1.375*1.375*(250 - avgdist)/100.0,0,15);
    
    
    */

    if (current_program_data[0] == 0) { stirr_only = true;}
    
    if ((program_step == 0) && !stirr_only)
    {

      myLcdAdapter->input_qty = true;
      myLcdKeypad->clear();
      myLcdKeypad->setCursor(0, 0);   // position the cursor at beginning of the first line
      myLcdKeypad->print("INPUT QTY dL:");   // print a Value label on the first line of the display

      while(myLcdAdapter->input_qty)
      {
        scheduleTimers();
      }

      liquid_qty_litres = (myLcdAdapter->liquid_qty_decilitres)/10.0;

    }

    Serial.println("LIQUID_QTY_LITRES:");
    Serial.println(liquid_qty_litres);
    
    
    //Kp = constrain(52*liquid_qty_litres,10,780);
    //Kp = current_program_data[2];

    Serial.println("kp=");
    Serial.println(Kp);

    
    if(!stirr_only)
    {
      Setpoint = float(current_program_data[0]); // Degrees Celsius
      
      if ((Setpoint - Input) < 2.0) { temperature_rise_time = 600; } // We don't bother having a controlled rise if it is already close to target, so we can jump to closed loop already}
      else { temperature_rise_time = float(current_program_data[1]);}
      stirr_disabled = !bool(current_program_data[2]);
      
      Serial.println("SETPOINT");
      Serial.println(Setpoint);
      Serial.println("TEMP_RISE_TIME");
      Serial.println(temperature_rise_time);


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
      
      // added fill factor and its influence on exposed internal vertical steel surface to air.
      heat_transfer_rate_vertical =  (vertical_surface_U_value*vessel_area)*(1 + (15.0 - liquid_qty_litres)/15.0)*(Input - ambient_temperature);
      
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
      power_required = 2.0*((total_energy_required/temperature_rise_time) + total_heat_loss);
      // 1.8 correction factor.
      // hot plate is 1500W max power
      power_required = constrain(power_required,0.0,1500.0);
      temperature_rise_time = float(current_program_data[1]); // restoring the value to get real program step time (maybe tweak was applied in case delta_t < 2.0)
      Serial.print("POWER_REQUIRED:");
      Serial.println(power_required);
      Kp = constrain(fabs(power_required)/15.0,0.0,100.0)*WindowSize/100.0;
      Duty2 = int(constrain(power_required/15.0,0.0,100.0));
      Serial.print("Duty2:");
      Serial.println(Duty2);

      //Duty2 = 100 - Duty;
      Serial.print("Kp=");
      Serial.println(Kp);
      myPID.SetTunings(Kp,Ki,Kd);
      //initialize the variables we're linked to
  
      //tell the PID to range between 0 and the full window size
      //myPID.SetOutputLimits(int(WindowSize*0.72), WindowSize);
      myPID.SetOutputLimits(0, WindowSize);
      
      myPID.SetSampleTime(5000);
      //turn the PID on
      myPID.SetMode(MANUAL);
    }
    else 
    {
      PrintStatus(0);  // print a Value label on the first line of the display
    }
  // start stirring;
  
    if (stirr_disabled)
    {
      Serial.println("STIRR_DISABLED");
      digitalWrite(MOTOR_RELAY_PIN,HIGH);
    }
    else
    {
      Serial.println("STIRR_ENABLED");
      digitalWrite(MOTOR_RELAY_PIN,LOW);
    }
    
    
    Serial.println("STEP_INIT_END");
    program_init = true;
    programStartTime = millis();
  }

  
  sensors.requestTemperatures(); // Send the command to get temperatures

  Input = float(sensors.getTempCByIndex(0));

  if (!stirr_only) 
  {

    if (fabs(Setpoint - Input) < 3.0) 
    {
      heat_transfer_rate_vertical =  (vertical_surface_U_value*vessel_area)*(1 + (15.0 - liquid_qty_litres)/15.0)*(Input - ambient_temperature);
      heat_transfer_rate_horizontal = horizontal_surface_U_value*(PI*vessel_radius*vessel_radius)*(Input - ambient_temperature);
      total_heat_loss = heat_transfer_rate_vertical + heat_transfer_rate_horizontal;

      total_energy_required = liquid_qty_litres*milk_density*milk_heat_capacity*(Setpoint - Input);
      power_required = 1.85*(total_energy_required/(temperature_rise_time - programElapsedTime) + total_heat_loss);
      // 1.8 correction factor.
      // hot plate is 1500W max power
      power_required = constrain(power_required,0.0,1500.0);
      
      temperature_rise_time = float(current_program_data[1]); // restoring the value to get real program step time (maybe tweak was applied in case delta_t < 2.0)
      //Serial.print("POWER_REQUIRED:");
      //Serial.println(power_required);
      Kp = constrain(fabs(power_required)/15.0,0.0,100.0)*WindowSize/100.0;
      Duty2 = int(constrain(power_required/15.0,0.0,100.0));

      myPID.SetOutputLimits(WindowSize*(1.0 - 2.2*Duty2/100.0),WindowSize); // 1.6 boost factor
      myPID.SetTunings(Kp,Ki,Kd);
      myPID.SetMode(AUTOMATIC);


      myPID.Compute();
      //Duty = int(100.0*(1.0 - float(Output)/float(WindowSize)));
    }
    else if(Input - Setpoint > 3.0) // disable heating completely
    {
      myPID.SetMode(MANUAL);
      Output = WindowSize;
    }
    else // controlled rise time mode (open loop)
    {
      myPID.SetMode(MANUAL);
      Output = int(float(WindowSize)*float(100.0 - Duty2)/100.0);  
    }

    EffectiveDuty = int(100.0*float((WindowSize - Output)/WindowSize));
    
    if (millis() - windowStartTime > WindowSizeRise)
    { //time to get temperature rise rate
    //RiseRate = Input - PreviousInput;
    //PreviousInput = Input;
    }
    /************************************************
     * turn the output pin on/off based on pid output
     ************************************************/
    if (millis() - windowStartTime > WindowSize)
    { //time to shift the Relay Window

      PrintStatus(EffectiveDuty);
      // print a Value label on the first line of the display
      
      programElapsedTime = (millis() - programStartTime)/1000;
      Serial.print(" PROGRAM_STEP_ELAPSED_TIME:");
      Serial.println(programElapsedTime);

      if (programElapsedTime > temperature_rise_time)
      {
        digitalWrite(HEATER_SSR_PIN,LOW);
        digitalWrite(MOTOR_RELAY_PIN,HIGH);
        program_step++;
        programElapsedTime = 0;
        program_init = false;
        Serial.print("NEXT_STEP:");
        Serial.println(program_step);
        if (current_program_data[3])
        {
          myLcdAdapter->program_paused = true;
          myLcdKeypad->clear();
          myLcdKeypad->setCursor(0, 0);   // position the cursor at beginning of the first line
          myLcdKeypad->print("PROGRAM PAUSED!");
          myLcdKeypad->setCursor(0, 1);   // position the cursor at beginning of the first line
          myLcdKeypad->print("SELECT TO RESUME");
  

          while(myLcdAdapter->program_paused)
          {
            scheduleTimers();
          }

        }

        if (program_step >= 4) {program_ended = true; return;}
      }
      
      windowStartTime += WindowSize;
      //myLcdKeypad->clear();

      //Duty = int(100 *(Output/WindowSize)); 
      Serial.print("CT:");
      Serial.print(Input);
      Serial.print(" TT:");
      Serial.print(Setpoint);
      Serial.print(" D2:");
      Serial.print(EffectiveDuty);
      Serial.print(" MODE:");
      Serial.println(myPID.GetMode());
      
      //Serial.print(" O:");
      //Serial.println(Output);
      

    }
    
    //Output = max(0,Output);
    //if (Output < millis() - windowStartTime) AnalogWrite(RELAY_PIN, HIGH);
    
    
    if (Output < millis() - windowStartTime) 
    {
      digitalWrite(HEATER_SSR_PIN, HIGH);
      //delay(5000);
      //Serial.println("HEAT");
    }
    else 
    {
      digitalWrite(HEATER_SSR_PIN, LOW);
      //Serial.println("NO_HEAT");
    }
  } // end if(!stirr_only)
  else
  {
    delay(1000);
    PrintStatus(0);  // print a Value label on the first line of the display
  } //end else ...(!stirronly)

}