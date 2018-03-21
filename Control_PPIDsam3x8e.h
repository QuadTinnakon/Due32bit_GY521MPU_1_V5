/*
project_Quad 32 bit Arduino Due Control_PPIDsam3x8e.h
1. stabilized quadrotor 
by: tinnakon kheowree 
0860540582
tinnakon_za@hotmail.com
tinnakonza@gmail.com
http://quad3d-tin.lnwshop.com/
https://www.facebook.com/tinnakonza
*/
float u_roll = 0.0f;
float u_pitch = 0.0f;
float u_yaw = 0.0f;

float roll_I_rate = 0.0f;
float roll_D_rate = 0.0f;
float setpoint_rollold = 0.0f;
float setpoint_rate_roll = 0.0f;
float error_rollold = 0.0f;
float error_rate_rollold = 0.0f;
float pitch_I_rate = 0.0f;
float pitch_D_rate = 0.0f;
float setpoint_pitchold = 0.0f;
float setpoint_rate_pitch = 0.0f;
float error_pitchold = 0.0f;
float error_rate_pitchold = 0.0f;
float yaw_I_rate = 0.0f;
float yaw_D_rate = 0.0f;
float error_rate_yawold = 0.0f;

//Automatic take-off and landing
float err_hz = 0.0f;
int time_auto = 0;
float h_counter = 0.1f;//0.08
float h_counter_old = 0.1f;
float Vz_Hold = 0.0f;
float hz_I = 0.0f;
float hz_D_rate = 0.0f;
float error_Vz_old = 0.0f;
uint8_t takeoff = 0;
uint8_t endAuto = 0;

void Control_PPIDRate(){
// ROLL CONTROL P-PID  control  By tinnakon///////////
  float setpoint_roll = ((CH_AILf-CH_AIL_Cal)*0.085);//0.12 max +-45 deg  ////+-18  + Control_YBf
  applyDeadband(setpoint_roll, 2.5);//2.5 ,1.2
  if(Mode == 2 || Mode == 3 && GPS_FIX  == 1){
    setpoint_roll = Control_YBf;
  }
  setpoint_rate_roll = (0.095*setpoint_rate_roll/(0.095+G_Dt)) + ((setpoint_roll-setpoint_rollold)/(0.095+G_Dt));//0.065 ,Diff remote
  setpoint_rollold = setpoint_roll;
  setpoint_rate_roll = constrain(setpoint_rate_roll, -80, 80);//+-80 deg/s
  float error_roll = setpoint_roll - (ahrs_r*RAD_TO_DEG);//ahrs_r*ToDeg
  float error_rate_roll = setpoint_rate_roll + error_roll*Kp_levelRoll  - GyroXf*RAD_TO_DEG;
  roll_I_rate += error_rate_roll*Ki_rateRoll*G_Dt;
  roll_I_rate = constrain(roll_I_rate, -5, 5);//5 ,+-150
  roll_D_rate = (tar*roll_D_rate/(tar+G_Dt)) + ((error_rate_roll-error_rate_rollold)/(tar+G_Dt));
  error_rate_rollold = error_rate_roll;
  u_roll = Kp_rateRoll*error_rate_roll + roll_I_rate + Kd_rateRoll*roll_D_rate;
  u_roll = constrain(u_roll, -280, 280);//+-250 +-300 120
// PITCH CONTROL P-PID  control  By tinnakon///////////
  float setpoint_pitch = ((CH_ELEf-CH_ELE_Cal)*-0.085);//max +-45 deg  ////+-18 - Control_XBf
  applyDeadband(setpoint_pitch, 2.5);//1.2
    if(Mode == 2 || Mode == 3 && GPS_FIX  == 1){
    setpoint_pitch = Control_XBf;
  }
  setpoint_rate_pitch = (0.095*setpoint_rate_pitch/(0.095+G_Dt)) + ((setpoint_pitch-setpoint_pitchold)/(0.095+G_Dt));//0.065 Diff remote
  setpoint_pitchold = setpoint_pitch;
  setpoint_rate_pitch = constrain(setpoint_rate_pitch, -80, 80);//+-80
  float error_pitch = setpoint_pitch - (ahrs_p*RAD_TO_DEG);//ahrs_p*RAD_TO_DEG
  float error_rate_pitch = setpoint_rate_pitch + error_pitch*Kp_levelPitch - GyroYf*RAD_TO_DEG;
  pitch_I_rate += error_rate_pitch*Ki_ratePitch*G_Dt;
  pitch_I_rate = constrain(pitch_I_rate, -5, 5);//+-150
  pitch_D_rate = (tar*pitch_D_rate/(tar+G_Dt)) + ((error_rate_pitch-error_rate_pitchold)/(tar+G_Dt));
  error_rate_pitchold = error_rate_pitch;
  u_pitch = Kp_ratePitch*error_rate_pitch + pitch_I_rate + Kd_ratePitch*pitch_D_rate;
  u_pitch = constrain(u_pitch, -280, 280);//+-250 +-300 120
// YAW  CONTROL P-PID  control  By tinnakon///////////
float setpoint_rate_yaw = (CH_RUDf-CH_RUD_Cal)*0.55;//0.4 0.35
  applyDeadband(setpoint_rate_yaw, 8.5);//6.5
  if(abs(setpoint_rate_yaw) > 0.1){
   setHeading = ahrs_y*RAD_TO_DEG;// 0 degree ,ahrs_tin.h
  }
  float error_yaw = 0.0 - Heading;
  float error_rate_yaw = setpoint_rate_yaw + error_yaw*Kp_levelyaw - GyroZf*RAD_TO_DEG;
  yaw_I_rate += error_rate_yaw*Ki_rateYaw*G_Dt;
  yaw_I_rate = constrain(yaw_I_rate, -120, 120);//+-120
  yaw_D_rate = (tar*yaw_D_rate/(tar+G_Dt)) + ((error_rate_yaw-error_rate_yawold)/(tar+G_Dt));//0.095
  error_rate_yawold = error_rate_yaw;
  u_yaw = Kp_rateYaw*error_rate_yaw + yaw_I_rate + Kd_rateYaw*yaw_D_rate;
  u_yaw = constrain(u_yaw, -340, 340);//+-340 +-170 +-150
 ////Altitude//////////////////////////////////////////////////////////////////////////////////////////////////////  
 Velocity_THR = (CH_THRf - 1500)*0.00469;//0.00469 +- 2.1 m/s
 applyDeadband(Velocity_THR, 0.23);//0.23 0.17
  if(Mode == 1 || Mode == 2)//Altitude Hold, 
  {
    Altitude_Hold = Altitude_Hold + (Velocity_THR*G_Dt);//Integral Velocity_THR
    Altitude_Hold = constrain(Altitude_Hold, 0.0, Altitude_Max);//0 - 10 m
    err_hz = Altitude_Hold - z1_hat;
    err_hz = constrain(err_hz, -1.0, 1.0);//+-2 m
    float error_Vz = Velocity_THR*0.85 - z2_hat;
    hz_D_rate = (tar*hz_D_rate/(tar+G_Dt)) + ((error_Vz - error_Vz_old)/(tar+G_Dt));// D Controller
    error_Vz_old = error_Vz;
   // float error_hII = err_hz*1.25 + error_Vz;//0.55
    hz_I = hz_I + (Ki_altitude*err_hz*G_Dt);
    hz_I = constrain(hz_I, -100, 600);//+-600
    float uthrottle2 = (err_hz*Kp_altitude) + hz_I + (error_Vz*Kd_altitude) + (hz_D_rate*Kd2_altitude) - (z3_hat*Ka_altitude);//state feedback
    //uthrottle = uthrottle2*m_quad/(cos_roll*cos_pitch);// = u*m*d/b*(cosr*cosp)     ,m*d/b = 122.33  (cos_roll*cos_pitch)
    uthrottle = uthrottle2;//  /cos_rollcos_pitch
  }
  else if(Mode == 3)//Automatic  Takeoff, Landing
  {
    err_hz = h_counter - z1_hat;
    err_hz = constrain(err_hz, -1.0, 1.0);//+-2 m
    float error_Vz = Vz_Hold - z2_hat;//Vz_Hold 
    hz_D_rate = (tar*hz_D_rate/(tar+G_Dt)) + ((error_Vz - error_Vz_old)/(tar+G_Dt));// D Controller
    error_Vz_old = error_Vz;
    //float error_hII = err_hz*1.25 + error_Vz;//0.55
    hz_I = hz_I + (Ki_altitude*err_hz*G_Dt);//1.9 - 2.9  18.5  22.5
    hz_I = constrain(hz_I, -100, 600);//+-200
    float uthrottle2 = (err_hz*Kp_altitude) + hz_I + (error_Vz*Kd_altitude) + (hz_D_rate*Kd2_altitude) - (z3_hat*Ka_altitude);//state feedback
    //uthrottle = uthrottle2*m_quad/(cos_roll*cos_pitch);// = u*m*d/b*(cosr*cosp)     ,m*d/b = 122.33  (cos_roll*cos_pitch)
    uthrottle = uthrottle2;//  /cos_rollcos_pitch
  }
  else
  {
    hz_I = 0.0;
    uthrottle = 0.0;
    Altitude_Hold = z1_hat;
    I_throttle = CH_THRf;
  }//end Altitude Hold
//uthrottle = constrain(uthrottle, -200, 200);//+-120 +-150
uAltitude = I_throttle + uthrottle;//m*g = 10.8 N = 
uAltitude = constrain(uAltitude, MINCOMMAND, MAXTHROTTLE);
}
/////////////////////////////
void LED_Auto(){
  Counter_LED_Auto++;
  if(Counter_LED_Auto > 5){
  digitalWrite(Pin_LED_R, HIGH);
  }
  if(Counter_LED_Auto > 7){
  digitalWrite(Pin_LED_R, LOW);
  Counter_LED_Auto = 0;
  }
}
///////////////////////
void Chack_Command(){
   if(AUX_1 <= (AltHold-10))//Stabilize 
  {
    Mode = 0;
  }
   if(AUX_1 > (AltHold-10) && AUX_1 <= (AltHold+10))//Altitude Hold, 
  {
    Mode = 1;
  }
   if(AUX_1 > (PositionHold-10) && AUX_1 <= (PositionHold+10))//Position Hold
  {
    Mode = 2;
  }  
  if(AUX_1 > (Auto-10))//Automatic  Takeoff
  {
    Mode = 3;
    LED_Auto();
  }
  if(AUX_1 > (RTH-10) && AUX_1 <= (RTH+10))//RTH
  {
   Mode = 2;
   target_LAT = GPS_LAT_HOME;//GPS_LAT_Hold
   target_LON = GPS_LON_HOME;//GPS_LON_Hold
  }
  //////////////////
   if(AUX_2 <= 1300)//Set Home 
  {
    GPS_LAT_HOME = x1_hat;
    GPS_LON_HOME = y1_hat;
    digitalWrite(Pin_LED_G, HIGH);
    digitalWrite(Pin_LED_R, LOW);
  }
   if(AUX_2 >= 1700)//Set waypoint1  
  {
    waypoint1_LAT = x1_hat;
    waypoint1_LON = y1_hat;
    digitalWrite(Pin_LED_R, HIGH);
    digitalWrite(Pin_LED_G, LOW);
  }
}//end Chack_Command()
///////////////////////////////////////////////////////////////////////////////////
void Automatictakeland(){
 //Altitude control and 1 waypoint navigation
  if(Mode == 3 && CH_THRf > MINCHECK && armed == 1)
  {
    if(time_auto < 2){//Check time < 5
      takeoff = 1;
    }
     if(z1_hat >= h_control && endAuto == 1)//waypoint1
    {
      target_LAT = waypoint1_LAT;
      target_LON = waypoint1_LON;
      Status_waypoint = 1;
    }
     if(time_auto > 8 && abs(error_LAT) <= 70 && abs(error_LON) <= 70 && endAuto == 1 && Status_waypoint == 1)//50 10 Landing and position hold mode
    {
      timeLanding++;
      if(timeLanding >= 30)//relay 3 s Landing
      {
       takeoff = 0;
      }
    }
  }
  else//(Mode == 3 && CH_THR > MINCHECK && armed == 1)
  {
    takeoff = 0;
    timeLanding = 0;
    timeOff = 0;
    time_auto = 0;
    h_counter = 0.1;//0.0
    h_counter_old = 0.1;
    Vz_Hold = 0.0;
    Status_waypoint = 0;
  } 
////////////////////////////////////////////////////////////////// 
       if(h_counter < h_control && takeoff == 1)//take-off
      {
        endAuto = 1;
        h_counter = h_counter + 0.042;//0.023 0.01- 0.03  0.1 = 10 s  //ramp input hz take-off
        Vz_Hold = (h_counter - h_counter_old)/0.1;//(m/s) roop 10 Hz
        h_counter_old = h_counter;
      }
       if(takeoff == 0 && endAuto == 1)//landing
      {
        h_counter = h_counter - 0.029;//0.023 ramp input hz  landing
        Vz_Hold = (h_counter - h_counter_old)/0.1;//(m/s) roop 10 Hz
        h_counter_old = h_counter;
        if(z1_hat <= 0.18)
        {
         endAuto = 0;
        }
      }
////////////////////////////////////
      if(time_auto > 25 && endAuto == 0) //15 s End waypoint quadrotor
        {
          timeOff++;
          if(timeOff > 60)//relay 6 s time-Off
          {
           armed = 0;
          } 
        }  
///////////////////////////////////////////////////////////
}
///////////////////////////////////////////////////////////////////////////////
void Control_PositionHold(){
  if(Mode == 2 || Mode == 3 && GPS_FIX  == 1){
          float targetRB_speedLAT = ((CH_ELEf-CH_ELE_Cal)*0.444);//vX Body Frame ,,= 200 cm/s
          float targetRB_speedLON = ((CH_AILf-CH_AIL_Cal)*0.444);//vY Body Frame
          applyDeadband(targetRB_speedLAT, 9.20f);//22.22 ,, +-50
          applyDeadband(targetRB_speedLON, 9.20f);//22.22
          targetRE_speedLAT = (targetRB_speedLAT*DCM00 + targetRB_speedLON*DCM01);//vX Earth Frame
          targetRE_speedLON = (targetRB_speedLAT*DCM10 + targetRB_speedLON*DCM11);//vY Earth Frame
/////////////////////////////////////////////////////////////////////////////////////////////////////
          target_LAT = target_LAT + ((double)targetRE_speedLAT*0.0498753d/cm_per_deg_lat);//roop 20 Hz = 0.0498753 s
          target_LON = target_LON + ((double)targetRE_speedLON*0.0498753d/cm_per_deg_lon);//roop 20 Hz = 0.0498753 s
    ////////////////////////////////////////////////////////////////////////////////////////////
          error_LAT = (double)(target_LAT - x1_hat)*cm_per_deg_lat; //*10939761.4 X Error cm
          error_LON = (double)(target_LON - y1_hat)*cm_per_deg_lon;//*6371000.0 Y Error   cm
          error_LAT = constrain(error_LAT,-500.0f,500.0f);//200 = +-2 m
          error_LON = constrain(error_LON,-500.0f,500.0f);
          float target_speedLAT = error_LAT*Kp_speed;//P Control Velocity GPS //+-200 cm/s = 2m/s
          float target_speedLON = error_LON*Kp_speed;//P Control Velocity GPS
          //target_speedLAT = constrain(target_speedLAT,-200,200);//+-200 cm/s = 2m/s
          //target_speedLON = constrain(target_speedLON,-200,200);

          float error_rate_LAT = target_speedLAT - vx2_hat;//m/s to cm/s
          float error_rate_LON = target_speedLON - vy2_hat;

          error_rate_LAT = constrain(error_rate_LAT,-300.0f,300.0f);//+-300 cm/s ,200
          error_rate_LON = constrain(error_rate_LON,-300.0f,300.0f);
          GPSI_LAT = GPSI_LAT + (error_rate_LAT*Ki_gps*0.05);//5Hz=0.2 ,, 20 Hz = 0.05
          GPSI_LON = GPSI_LON + (error_rate_LON*Ki_gps*0.05);  
          GPSI_LAT = constrain(GPSI_LAT,-200.0f,200.0f);//win speed +-200 cm/s
          GPSI_LON = constrain(GPSI_LON,-200.0f,200.0f);//250
          //Control_XEf = error_rate_LAT*Kd_gps;//P Control speed 
          //Control_YEf = error_rate_LON*Kd_gps;
          Control_XEf = error_LAT*Kp_gps + error_rate_LAT*Kd_gps + GPSI_LAT;//PID Control speed 
          Control_YEf = error_LON*Kp_gps + error_rate_LON*Kd_gps + GPSI_LON;
          Control_XEf = constrain(Control_XEf,-800.0f,800.0f);//PWM 1000 - 1900
          Control_YEf = constrain(Control_YEf,-800.0f,800.0f);
          //The desired roll and pitch angles by tinnakon 
          float urolldesir = (Control_YEf*m_quad)/uAltitude;//uAltitude/2 = 1000 - 1900
          float upitchdesir = (Control_XEf*m_quad*-1.0f)/uAltitude;//*-1
          urolldesir = constrain(urolldesir,-0.7f,0.7f);//+-0.7 = +-44.427
          upitchdesir = constrain(upitchdesir,-0.7f,0.7f);
          float temp_YBf = asin(urolldesir)*RAD_TO_DEG;//Control Earth Frame
          float temp_XBf = asin(upitchdesir)*RAD_TO_DEG;//Control Earth Frame
          Control_XBf = (DCM00*temp_XBf) + (DCM01*temp_YBf);//Control Body Frame use Rotation matrix
          Control_YBf = (DCM10*temp_XBf) + (DCM11*temp_YBf);//Control Body Frame use Rotation matrix
          Control_XBf = constrain(Control_XBf, -20.0f, 20.0f);//+-20 deg
          Control_YBf = constrain(Control_YBf, -20.0f, 20.0f);//+-20 deg
          
          //The desired roll and pitch angles by paper Modeling and Backstepping-based Nonlinear Control 
          //Ashfaq Ahman Mian 2008 (eq.25 and eq.26)
          //float urolldesir = ((Control_XEf*m_quad*sin(ahrs_y))/uAltitude) - ((Control_YEf*m_quad*cos_yaw)/uAltitude);
          //float upitchdesir = ((Control_XEf*m_quad)/(uAltitude*cos_roll*cos_yaw)) - ((sin(ahrs_r)*sin(ahrs_y))/(cos_roll*cos_yaw));
          //urolldesir = constrain(urolldesir,-0.7,0.7);//+-0.7 = +-44.427
          //upitchdesir = constrain(upitchdesir,-0.7,0.7);
          //Control_YBf = asin(urolldesir)*RAD_TO_DEG;//Control roll eq.25 
          //Control_XBf = asin(upitchdesir)*RAD_TO_DEG*-1.0;//Control roll eq.26
          //Control_XBf = constrain(Control_XBf, -20, 20);//+-20 +- 44
          //Control_YBf = constrain(Control_YBf, -20, 20);
  }
  else{
      Control_XBf = 0.0f;
      Control_YBf = 0.0f;
      GPSI_LAT = 0.0f;
      GPSI_LON = 0.0f;
      target_LAT = x1_hat;//GPS_LAT_Hold
      target_LON = y1_hat;//GPS_LON_Hold
  }
}//end  Control_PositionHold()
/////////////////////////////////////////////////////////////////////////
void Cal_GPS(){
//GPS_LAT1 = GPS_coordLAT*1e-7;///10000000.0 1e-7 degrees / position as degrees (*10E7)
//GPS_LON1 = GPS_coordLON*1e-7;
velocity_northff = velocity_north + (gyroY_Earth*arm_GPS);//+ (gyroY_Earth*arm_GPS) velocity GPS Rotated Frame of arm gps
velocity_eastff = velocity_east - (gyroX_Earth*arm_GPS);//- (gyroX_Earth*arm_GPS)
//Expressing latitude and longitude as linear units Earth's average meridional radius is 6367449 m
////https://en.wikipedia.org/wiki/Geographic_coordinate_system ///mn
float latMid = DEG_TO_RAD*GPS_coordLAT*1e-7;
cm_per_deg_lat = (111132.92 - 559.82*cos(2*latMid) + 1.175*cos(4*latMid))*100.0d;
cm_per_deg_lon = (111412.84*cos(latMid) - 93.5*cos(3*latMid))*100.0d;
  /*
  if(GPS_FIX  == 1){
 //Apply moving average filter to GPS data
      GPS_filter_index = (GPS_filter_index+1) % 4;// 4 moving average
      GPS_SUM_LAT[GPS_filter_index] = GPS_LAT1;
      GPS_SUM_LON[GPS_filter_index] = GPS_LON1;
      float sum1=0.0;
      float sum2=0.0;
      for(int i=0;i<4;i++)
      {
       sum1 += GPS_SUM_LAT[i];
       sum2 += GPS_SUM_LON[i];
      }
     GPS_LAT1f = sum1/4.0;
     GPS_LON1f = sum2/4.0;
  }
  else{
    GPS_numSat = 0;
  }
///////////////////////////////////////
     //Diff speed
     actual_speedX = (GPS_LAT1f - GPS_LAT1_old)*6371000.0/0.3;//5 Hz = 0.2 ,cm/s  10000000.0 ,R = 6371000.0
     actual_speedY = (GPS_LON1f - GPS_LON1_old)*637100.0/0.3;//cm/s
     //actual_speedX = constrain(actual_speedX, -400, 400);//+-400 cm/s
     //actual_speedY = constrain(actual_speedY, -400, 400);//+-400 cm/s  
     GPS_LAT1_old = GPS_LAT1f;
     GPS_LON1_old = GPS_LON1f;
     actual_speedXf = (actual_speedX + actual_speedXold)/2.0;//Moving Average Filters/
     actual_speedYf = (actual_speedY + actual_speedYold)/2.0;//Moving Average Filters/
     actual_speedXold = actual_speedX;
     actual_speedYold = actual_speedY;
  
/////////////LeadFilter GPS/////////////////////////////////
    float lag_in_seconds = 0.85;//1.0 0.5
    float accel_contribution = (actual_speedXf - _last_velocityX) * lag_in_seconds * lag_in_seconds;
    float vel_contribution = actual_speedXf * lag_in_seconds;
    _last_velocityX = actual_speedXf;    // store velocity for next iteration
    GPS_LAT1lead = GPS_LAT1f  + vel_contribution + accel_contribution;
    float accel_contributio = (actual_speedYf - _last_velocityY) * lag_in_seconds * lag_in_seconds;
    float vel_contributio = actual_speedYf * lag_in_seconds;
    _last_velocityY = actual_speedYf;    // store velocity for next iteration
    GPS_LON1lead = GPS_LON1f  + vel_contributio + accel_contributio;
    */
}
