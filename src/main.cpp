#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#include <PID_v1.h>

// TEMPERATURE SENSOR
#define ONE_WIRE_BUS 2

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

#define RELAY_PIN 3

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

void setup()
{

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