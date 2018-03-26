/*
project_Quad 32 bit Arduino Due motorX4sam3x8e.h
1. stabilized quadrotor 
by: tinnakon kheowree 
0860540582
tinnakon_za@hotmail.com
tinnakonza@gmail.com
http://quad3d-tin.lnwshop.com/
https://www.facebook.com/tinnakonza
*/
int MOTOR_FrontL_PIN = 4;//PWML7 = pin 6
int MOTOR_FrontR_PIN = 5;//PWML6 = pin 7
int MOTOR_BackL_PIN = 7;// PWML5 = pin 8
int MOTOR_BackR_PIN = 6;// PWML4 = pin 9
#define PWM_DUE_FREQUENCY 400   //400 in Hz
#define PWM_DUE_PERIOD 2500   // in us
//#define PWM_PRESCALER 8
//#define PWM_COUNTER_PERIOD (F_CPU/PWM_PRESCALER/PWM_FREQUENCY)
float motor_FrontL = 1000.0f;
float motor_FrontR = 1000.0f;
float motor_BackL = 1000.0f;
float motor_BackR = 1000.0f;
//lag filter motor/
float motor_FrontLf = 1000.0f;
float motor_FrontLold = 1000.0f;
float motor_FrontLold2 = 1000.0f;
float motor_FrontRf = 1000.0f;
float motor_FrontRold = 1000.0f;
float motor_FrontRold2 = 1000.0f;
float motor_BackLf = 1000.0f;
float motor_BackLold = 1000.0f;
float motor_BackLold2 = 1000.0f;
float motor_BackRf = 1000.0f;
float motor_BackRold = 1000.0f;
float motor_BackRold2 = 1000.0f;
///////////////////////////////////////////////////////////////////////////////
static void setPWMpin(uint32_t pin) {
  PIO_Configure(g_APinDescription[pin].pPort,
                PIO_PERIPH_B, //hack Arduino does not allow high PWM by default
                g_APinDescription[pin].ulPin,
                g_APinDescription[pin].ulPinConfiguration);
}
static void configOneMotor(uint8_t ch, uint32_t period) {
  PWMC_ConfigureChannel(PWM, ch, PWM_CMR_CPRE_CLKA, 0, 0);
  PWMC_SetPeriod(PWM, ch, period);
  PWMC_SetDutyCycle(PWM, ch, MINCOMMAND);
  PWMC_EnableChannel(PWM, ch);
}
void motor_command_all() 
{ 
  for (int j = 0 ; j <= 50 ; j++)
  {
    motor_FrontL = 1000.0f;
    motor_FrontR = 1000.0f;
    motor_BackL = 1000.0f;
    motor_BackR = 1000.0f;
  PWMC_SetDutyCycle(PWM, 4, 1000);
  PWMC_SetDutyCycle(PWM, 5, 1000);
  PWMC_SetDutyCycle(PWM, 6, 1000);
  PWMC_SetDutyCycle(PWM, 7, 1000);
  delay(20);
}
}
void motor_initialize() 
{
  Serial.print("motor_initialize");Serial.print("\n");
  pinMode(MOTOR_FrontL_PIN,OUTPUT);  
  pinMode(MOTOR_FrontR_PIN,OUTPUT); 
  pinMode(MOTOR_BackL_PIN,OUTPUT); 
  pinMode(MOTOR_BackR_PIN,OUTPUT); 
  //digitalWrite(MOTOR_FrontL_PIN, LOW);
  //digitalWrite(MOTOR_FrontR_PIN, LOW);
  //digitalWrite(MOTOR_BackL_PIN, LOW);
  //digitalWrite(MOTOR_BackR_PIN, LOW);
  delay(1000);
   setPWMpin(6);
   PWMC_DisableChannel(PWM, 4);
   setPWMpin(7);
   PWMC_DisableChannel(PWM, 5);
   setPWMpin(8);
   PWMC_DisableChannel(PWM, 6);
   setPWMpin(9);
   PWMC_DisableChannel(PWM, 7);
   pmc_enable_periph_clk(ID_PWM);
  PWMC_ConfigureClocks(1000000,0,VARIANT_MCK);// set PWM clock A to 1MHz
  configOneMotor(4, PWM_DUE_PERIOD);
  configOneMotor(5, PWM_DUE_PERIOD);
  configOneMotor(6, PWM_DUE_PERIOD);
  configOneMotor(7, PWM_DUE_PERIOD);
  motor_command_all();
}
void motor_command() 
{
  PWMC_SetDutyCycle(PWM, MOTOR_FrontL_PIN, motor_FrontLf);//6 = PWML7
  PWMC_SetDutyCycle(PWM, MOTOR_BackR_PIN, motor_BackRf);//7 = PWML6
  PWMC_SetDutyCycle(PWM, MOTOR_FrontR_PIN, motor_FrontRf);//8 = PWML5
  PWMC_SetDutyCycle(PWM, MOTOR_BackL_PIN, motor_BackLf);//9 = PWML4
}
  /********************************************************************/
  /****           ESCs calibration  By tinnakon                    ****/
  /********************************************************************/
void ESC_calibration() {
  Serial.print("ESC_calibration");Serial.print("\n");
   for (int i = 0; i < 5; i++)
  {
    computeRC();
    if(CH_THR > MAXCHECK)
    {
     ESC_calibra = 1; 
    }
    else
    {
     ESC_calibra = 0;
    }
   delay(20);
  }
  int jprint = 0;
  while(ESC_calibra == 1){
   computeRC();
   motor_FrontL = (CH_THR - 500.0f)*1.5f;
   motor_FrontR = (CH_THR - 500.0f)*1.5f;
   motor_BackL = (CH_THR - 500.0f)*1.5f;
   motor_BackR = (CH_THR - 500.0f)*1.5f;
   
   motor_FrontL = constrain(motor_FrontL, MINCOMMAND, MAXCOMMAND);
   motor_FrontR = constrain(motor_FrontR, MINCOMMAND, MAXCOMMAND);
   motor_BackL = constrain(motor_BackL, MINCOMMAND, MAXCOMMAND);
   motor_BackR = constrain(motor_BackR, MINCOMMAND, MAXCOMMAND);
  
  PWMC_SetDutyCycle(PWM, 7, motor_FrontL);
  PWMC_SetDutyCycle(PWM, 6, motor_BackR);
  PWMC_SetDutyCycle(PWM, 5, motor_FrontR);
  PWMC_SetDutyCycle(PWM, 4, motor_BackL);
  
   jprint++;
   if(jprint > 10)
   {
     jprint = 0;
   Serial.print(motor_FrontL);Serial.print("\t");
   Serial.print(motor_FrontR);Serial.print("\t");    
   Serial.print(motor_BackL);Serial.print("\t");     
   Serial.print(motor_BackR);Serial.println("\t");    
   //Serial1.println(motor_Back);
     if(Status_LED == LOW)
     Status_LED = HIGH;
     else
     Status_LED = LOW;
     digitalWrite(13, Status_LED);
     digitalWrite(Pin_LED_R, Status_LED);
     digitalWrite(Pin_LED_G, Status_LED);
     digitalWrite(Pin_LED_B, Status_LED);
   }
   delay(20);
  }
}
/*
void motor_Lag(){
  ////////lag lead compensator filter motor////////////////////////////////////
     float k_Lag = 2.45;//2.65 3.95 5.85 1.85 0.45 1.078 1.0 1.25
     float k_Lead = 25.5;//35.25
     float diff_motor_FrontL = (motor_FrontL - motor_FrontLold2)/0.02;
     float temp_motor_FrontL = diff_motor_FrontL + (motor_FrontL - motor_FrontLf)*k_Lead;//35.5
     motor_FrontLf = motor_FrontLf + (temp_motor_FrontL*G_Dt*k_Lag);
     
     float diff_motor_FrontR = (motor_FrontR - motor_FrontRold2)/0.02;
     float temp_motor_FrontR = diff_motor_FrontR + (motor_FrontR - motor_FrontRf)*k_Lead;//35.5
     motor_FrontRf = motor_FrontRf + (temp_motor_FrontR*G_Dt*k_Lag);
                        
     float diff_motor_BackL = (motor_BackL - motor_BackLold2)/0.02;
     float temp_motor_BackL = diff_motor_BackL + (motor_BackL - motor_BackLf)*k_Lead;//35.5
     motor_BackLf = motor_BackLf + (temp_motor_BackL*G_Dt*k_Lag);
     
     float diff_motor_BackR = (motor_BackR - motor_BackRold2)/0.02;
     float temp_motor_BackR = diff_motor_BackR + (motor_BackR - motor_BackRf)*k_Lead;//35.5
     motor_BackRf = motor_BackRf + (temp_motor_BackR*G_Dt*k_Lag);
     
     motor_FrontLold2 = motor_FrontLold;
     motor_FrontLold = motor_FrontL;//store PWM for next diff
     motor_FrontRold2 = motor_FrontRold;
     motor_FrontRold = motor_FrontR;
     motor_BackLold2 = motor_BackLold;
     motor_BackLold = motor_BackL;
     motor_BackRold2 = motor_BackRold;
     motor_BackRold = motor_BackR;
}
*/
void motor_Mix(){
      motor_FrontL = uAltitude*0.97f + u_pitch*0.7071f + u_roll*0.7071f - u_yaw*0.5f;//Front L , cos45 = 0.7071
      motor_FrontR = uAltitude*0.97f + u_pitch*0.7071f - u_roll*0.7071f + u_yaw*0.5f;//Front R
      motor_BackL = uAltitude*0.97f - u_pitch*0.7071f + u_roll*0.7071f + u_yaw*0.5f;//Back L
      motor_BackR = uAltitude*0.97f - u_pitch*0.7071f - u_roll*0.7071f - u_yaw*0.5f;////Back R
/*
#ifdef Quad_P
      motor_Front = uAltitude + u_pitch - u_yaw;
      motor_Right = uAltitude - u_roll + u_yaw;
      motor_Left = uAltitude + u_roll + u_yaw;
      motor_Back = uAltitude - u_pitch - u_yaw;
#endif
      motor_FrontL = constrain(motor_FrontL, MINCOMMAND, MAXCOMMAND);
      motor_FrontR = constrain(motor_FrontR, MINCOMMAND, MAXCOMMAND);
      motor_BackL = constrain(motor_BackL, MINCOMMAND, MAXCOMMAND);
      motor_BackR = constrain(motor_BackR, MINCOMMAND, MAXCOMMAND);
*/
       if (CH_THR < MINTHROTTLE)
        {
          roll_I_rate = 0.0f;
          pitch_I_rate = 0.0f;
          yaw_I_rate = 0.0f;
          motor_FrontL = 1000.0f;
          motor_FrontR = 1000.0f;
          motor_BackL = 1000.0f;
          motor_BackR = 1000.0f;
        }
        if(armed == 1)
        {
         //motor_Lag();
         motor_FrontLf = constrain(motor_FrontL, MINTHROTTLE, MAXCOMMAND);//set PWM data (1000 - 2000)*2 to (2000 - 4000)
         motor_FrontRf = constrain(motor_FrontR, MINTHROTTLE, MAXCOMMAND);
         motor_BackLf = constrain(motor_BackL, MINTHROTTLE, MAXCOMMAND);
         motor_BackRf = constrain(motor_BackR, MINTHROTTLE, MAXCOMMAND);
        }
        else
        {
          motor_FrontLf = 1000.0f;//set PWM data (1000 - 2000)*2 to (2000 - 4000)
          motor_FrontRf = 1000.0f;
          motor_BackLf = 1000.0f;
          motor_BackRf = 1000.0f;
        }
}
