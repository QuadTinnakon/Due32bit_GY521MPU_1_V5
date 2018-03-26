/*
project_Quad 32 bit Arduino Due Ultrasonic04.h
//GPS
1. stabilized quadrotor 
by: tinnakon kheowree 
0860540582
tinnakon_za@hotmail.com
tinnakonza@gmail.com
http://quad3d-tin.lnwshop.com/
https://www.facebook.com/tinnakonza

#include "Ultrasonic.h"
 Ultrasonic LV-EZ4  AnalogRead
 every 50mS, (20-Hz rate)
 from 6-inches out to 254-inches
 from 0.15-m out to 6.45 m
 5V yields ~9.8mV/in. , 1 in = 0.0254 m =~9.8mV/0.0254 m = 0.38582 v/m
 
 //Ultrasonic LV-EZ4
  //int sensorValue = analogRead(A0);
  //hz_Ultra = (sensorValue*5/1024.0)/0.38528;//5V yields ~9.8mV/in. , 1 in = 0.0254 m =~9.8mV/0.0254 m = 0.38582 v/m
  //hz_Ultra = constrain(hz_Ultra, 0, 6.45);//m
  
  //Ultrasonic_HC-SR04 // create a trigger pulse for 10 us
  //digitalWrite(HCSR04_TriggerPin, LOW);
  //delayMicroseconds(2);
  
 Ultrasonic_HC-SR04
  from 0.02 -m out to 1.4 m
 //Just connect VCC to (+) on D9, trig to D9, echo to D10, Gnd to (-)
 */
#define HCSR04_TriggerPin 27 // should be modified to 9  12 in next version
#define HCSR04_EchoPin 26     // should be modified to 10  11 in next version

float hz_Ultra = 0.0f;
float Altitude_sona = 0.0f;
float Altitude_sona2 = 0.0f;
float Altitude_sonaf = 0.0f;
float Altitude_sonaold = 0.0f;
float vz_sona = 0.0f;
float vz_sona2 = 0.0f;
float vz_sonaf = 0.0f;

float Altitude_II = 0.0f;
float Altitude_Baro_ult = 0.0f;
float Vz_Baro_ult = 0.0f;

unsigned long HCSR04_startTime = 0;
unsigned long HCSR04_echoTime = 0;
int  tempSonarAlt=0;
// EchoPin will change to high signalize beginning
// and back to low after 58*cm us
// First interrupt is needed to start measurement, second interrupt to calculate the distance


void UltaHandler() {
  // Here is a routine missing, to check, if the interrupt was raised for echo pin - not needed at the moment, because we don't have any interrupts
  // for this interrupt group, but maybe later
  uint8_t sensorVal = digitalRead(HCSR04_EchoPin);
  if (sensorVal == 1) { //indicates if the EchoPin is at a high state
    HCSR04_startTime = micros();
  }
  else {
    HCSR04_echoTime = micros() - HCSR04_startTime;
    if (HCSR04_echoTime <= 25000)      // maximum = 4,31 meter - 30000 us means out of range
      tempSonarAlt = HCSR04_echoTime / 5.8f;//to mm
    else
      tempSonarAlt = 4300;
  }
}
void UltrasonicRead()
{
  digitalWrite(HCSR04_TriggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(HCSR04_TriggerPin, LOW);
  Altitude_sona = tempSonarAlt/1000.0f;//m
  //Altitude_sonaf = (Altitude_sona + Altitude_sona2)/2.0;//filter
  Altitude_sonaf = Altitude_sonaf + (((Altitude_sona + Altitude_sona2)/2.0f) - Altitude_sonaf)*0.622012f;//*0.687 filter 42 Hz
  Altitude_sona2 = Altitude_sona;
  vz_sona = (Altitude_sonaf - Altitude_sonaold)/0.0500050005f;//diff 20 Hz = 0.05 s
  Altitude_sonaold = Altitude_sonaf;
  //vz_sonaf = (vz_sona + vz_sona2)/2.0;//filter
  //vz_sonaf = vz_sonaf + (((vz_sona + vz_sona2)/2.0) - vz_sonaf)*0.62;//filter 42 Hz
  //vz_sona2 = vz_sona;
  vz_sonaf = constrain(vz_sona, -3.0f, 3.0f);
  //Altitude_sonano = constrain(Altitude_sonano, 0, 3.0);//m
//Ultrasonic max 0.6 m change to Baro//////////////////////////////
if(z1_hat > 0.4f){//Altitude_sonaf
  Altitude_Baro_ult = Altitude_barof;
  Vz_Baro_ult = baro_vz;
  if(Mode == 2 || Mode == 3 && GPS_FIX  == 1){
    Vz_Baro_ult = (float)vel_down*0.01f;//m/s
  }
}
else{
  float error_Altitude = Altitude_sonaf - Altitude_barof;
  Altitude_II = Altitude_II + (error_Altitude*0.019501f);//0.0185 0.005 ,,20 Hz = 0.05
  Altitude_II = constrain(Altitude_II, -100.0f, 100.0f);
  Altitude_Baro_ult = Altitude_sonaf;
  Vz_Baro_ult = vz_sonaf;
}
}
void UltrasonicInt()
{
  Serial.print("Ultrasonic-Int");Serial.print("\t");
  pinMode(HCSR04_EchoPin,INPUT);
  pinMode(HCSR04_TriggerPin,OUTPUT);
  attachInterrupt(HCSR04_EchoPin, UltaHandler, CHANGE); //RISING  FALLING CHANGE
  //PCICR |= (1<<PCIE0);// enable PCINT0 // PCINT 0-7 belong to PCIE0 //HCSR04_EchoPin_PCICR
  //PCMSK0 = (1<<PCINT4);//PB4 ( OC2A/PCINT4 )= Digital pin 10 (PWM) 
  // Mask Pin PCINT5 - all other PIns PCINT0-7 are not allowed to create interrupts!
  UltrasonicRead();
  delay(100);
  UltrasonicRead();
  Serial.print(Altitude_sona);Serial.print("\n");
}
///////////////////////////////////////////////////////////////////////////////////////////////
float getAltitude(float pressure2, float temperature2)
{
  //return (1.0f - pow(pressure2/sea_press, 0.190295f)) * 44330.0f;
  //return log(sea_press/pressure2) * (temperature2+273.15f) * 29.271267f; // in meter   1007.23 1008.83
  return ((pow((sea_press/pressure2),1/5.257f)-1.0f)*(temperature2+273.15f))/0.0065f;
}
void pushAvg(float val, float val2)
{
  movavg_buff[movavg_i] = val;
  movavg_buffT[movavg_i] = val2;
  movavg_i = (movavg_i + 1) % MOVAVG_SIZE;
}
float getAvg(float * buff, int size)
{
  float sum=0.0f;
  for(int i=0;i<size;i++)
  {
    sum += buff[i];
  }
  return sum/size;
}
