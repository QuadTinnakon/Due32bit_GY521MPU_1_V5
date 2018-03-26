/*
project_Quad 32 bit Arduino Due  Aconfigsam3x8e.h
1. stabilized quadrotor 
by: tinnakon kheowree 
0860540582
tinnakon_za@hotmail.com
tinnakonza@gmail.com
http://quad3d-tin.lnwshop.com/
https://www.facebook.com/tinnakonza
*/

//The type of multicopter  
//#define HEX_X
#define Quad_X
//#define Quad_P
#define CPPM
///////////////Mode///////////////////////////
#define Mannual 1028
#define AltHold 1252
#define RTH 1707
#define PositionHold 1484  //Loiter mode
#define Auto 2020
#define FailSafe 980
////////////////////////////////////////////////
#define MINTHROTTLE 1064 //1090
#define MAXTHROTTLE 1750 //1850
#define MINCOMMAND 1000
#define MAXCOMMAND 1850
#define MIDRC 1500
#define MINCHECK 1100
#define MAXCHECK 1900
int Mode = 0;
//Mode 0 = Stabilize
//Mode 1 = Altitude Hold
//Mode 2 = Position Hold ,Loiter
//Mode 3 = Automatic  Takeoff ,Landing
//Accelerometer calibration constants; use the Calibrate example from print(accelRaw[XAXIS]);
int A_X_MIN = -4126;    //
int A_X_MAX = 4066;     //
int A_Y_MIN = -4096;    //
int A_Y_MAX = 4092;     //
int A_Z_MIN = -4165;    //
int A_Z_MAX = 4150;     //4007
//magnetometer calibration constants; use the Calibrate example from print(MagXf);
int M_X_MIN = -190;    //-490 -654  -693   -688
int M_X_MAX = 554;     //310 185   209    170
int M_Y_MIN = -230;    //-369 -319  -311   -310
int M_Y_MAX = 510;     //397 513   563    546
int M_Z_MIN = -420;    //-392 -363  -374   -377
int M_Z_MAX = 360;     //346 386   429    502
//GPS //สตาร์ท//////////////////////////////////////
double GPS_LAT_HOME = 13.8681811d;//13.8681811d ,13.868180,100.496297 13868180
double GPS_LON_HOME = 100.4962971d;//100.4962971d
//ลงจอด
double waypoint1_LAT = 13.8681811d;//13.868544, 100.496433
double waypoint1_LON = 100.4962971d;//B,
/////////////////////////////
double target_LAT = GPS_LAT_HOME;//13.8681811d ,13.868180 100.496297
double target_LON = GPS_LON_HOME;//100.4962971d
// Automatic take-off and landing 
float h_control = 3.5f;  //2.7 0.6 0.9 meter
float Altitude_Max = 20.0f;//max 20 meter
//Parameter system Quadrotor/////////////////////////////////////////
float m_quad = 1.18f;     //kg
float L_quad = 0.235f;   //m quad+=0.25   I = (1/3)mL2 = 0.02291
float c_quad = 0.00364f; //N.s/m
float c_XYquad = 0.0000364f; //N.s/cm
//float k_XYquad = 4.364;//N/cm ,,8.364 10.364
float arm_GPS = 18.3f; //17 cm
float Fight_time = 480.1f; //s = 8 min
/////////////////////////////////////////////////////////////////////
//P-PID-------------Rate
float Kp_rateRoll = 1.92f;//2.42 2.78 1.18 5.28
float Ki_rateRoll = 0.12f;//1.12 2.75
float Kd_rateRoll = 0.045f;//0.035 0.075 0.15 0.085 0.025 - 0.045

float Kp_ratePitch = 1.92f;//2.42 1.18 5.28
float Ki_ratePitch = 0.12f;//1.12 2.75 0.5 - 2.8
float Kd_ratePitch = 0.045f;//0.075 0.15 0.078 0.025 - 0.045

float Kp_rateYaw = 2.75f;//2.75 3.75 5.75 1.75 - 3.450  350.0
float Ki_rateYaw = 0.95f;//1.85 3.65  2.95
float Kd_rateYaw = 0.085f;//0.095 0.035 0.065

//PID--------------Stable
float Kp_levelRoll= 4.95f;//4.2 6.2 7.8 9.2 

float Kp_levelPitch= 4.95f;//4.2 6.2 9.2 

float Kp_levelyaw= 6.15f;//4.2 4.5

//stat feedback--------------Altitude
float Kp_altitude = 180.2f;//155.2 225.2 265.2  175.0  165.0
float Ki_altitude = 11.45f;//2.1 12.25 52.13 2.13 0.018 2.5,0.0
float Kd_altitude = 235.2f;//210.2 185.2 195.2 185.2 250 280.5 315.5 120
float Kd2_altitude = 33.4f;//23.4 35.25 18.25 22.25 42.5 12.2 1.25
float Ka_altitude = 0.11f;//8.5 18.5 26 32.5 38.5 41.5 35 25 - 45
//////////////////RC//////////////////////////////////
//#define tarremote 0.025 // fast
//#define tarremote 0.062  //0.092 slow 0.12 0.02 0.08 remote 
float tar = 0.011f; //0.012 0.015
float tarremote = 0.065f;//0.095 0.065

//PID GPS////////////////////////////////////////////
float Kp_gps = 0.135f;//0.1405 0.245 0.15 0.101 2.101 5.101
float Ki_gps = 0.122f;//0.122 0.68 2.68 0.25 0.085 0.15
float Kd_gps = 1.531f;//1.850 2.85 3.35 4.35 1.05 1.9 4.3 0.35 1.35 3.35
float Kp_speed = 0.437f;//0.247 0.43 0.37 0.27 0.15 0.35 0.095 min 0.15

#define Pin_Laser 2
#define Pin_LED_B 4
#define Pin_LED_G 5
#define Pin_LED_R 3
//Observer hz////////////////////////////////////////
float Velocity_THR = 0.0f;
float Altitude_Hold = 0.0f;
float uthrottle=0.0f;
float I_throttle = 1050.0f;
float uAltitude = 1000.0f;
float accrX_Earth = 0.0f;
float accrY_Earth = 0.0f;
float accrZ_Earth = 0.0f;
float accrX_Earthf = 0.0f;
float accrY_Earthf = 0.0f;
float accrZ_Earthf = 0.0f;
float gyroX_Earth = 0.0f;
float gyroY_Earth = 0.0f;
float gyroZ_Earth = 0.0f;
float velocity_northff = 0.0f;
float velocity_eastff = 0.0f;
//kalman//////////////////////////////////////
double x1_xt = 0.0d;
float x2_vt = 0.0f;
float x3_at = 0.0f;
double y1_xt = 0.0d;
float y2_vt = 0.0f;
float y3_at = 0.0f;
double vx1_hat=0.0d;
float vx2_hat=0.0f;
float vx3_hat=0.0f;
double vy1_hat=0.0d;
float vy2_hat=0.0f;
float vy3_hat=0.0f;
double x1_hat = GPS_LAT_HOME;
double y1_hat = GPS_LON_HOME;

float z1_hat = 0.0f;
float z2_hat = 0.0f;
float z3_hat = 0.0f;
float z3_acc = 0.0f;
float z1_hat2 = 0.0f;
float z2_hat2 = 0.0f;
float u_z = 0.0f;
float u_zold = 0.0f;
float u_zdot = 0.0f;
float u_Ydot = 0.0f;
float u_Xdot = 0.0f;

float baro_vz = 0.0f;
float baro_vz_old = 0.0f;

//GPS
//double GPS_LAT1 = 13.8681811d;
//double GPS_LON1 = 100.4962971d;
//float GPS_LAT1Lf = 0.0;
//float GPS_LON1Lf = 0.0;
//float GPS_LAT1lead = 0.0;
//float GPS_LON1lead = 0.0;
//float GPS_speed = 0.0;
double cm_per_deg_lat = 11113295.451d;
double cm_per_deg_lon = 11113195.451d;
//float actual_speedY = 0.0;
//float actual_speedXf = 0.0;
//float actual_speedYf = 0.0;
//float actual_speedXold = 0.0;
//float actual_speedYold = 0.0;
//float _last_velocityX = 0.0;
//float _last_velocityY = 0.0;
//float GPS_LAT1_old = GPS_LAT_HOME;
//float GPS_LON1_old = GPS_LON_HOME;
float Control_XEf = 0.0f;
float Control_YEf = 0.0f;
float Control_XBf = 0.0f;
float Control_YBf = 0.0f;
float targetRE_speedLAT = 0.0f;//vX Earth Frame
float targetRE_speedLON = 0.0f;//vY Earth Frame
//byte Read_command = 0;
//float GPS_hz = 0.0;
//float GPS_vz = 0.0;
float yaw_bearing = 0.0f;
//float GPS_Distance = 0.0;
float error_LAT = 0.0f;
float error_LON = 0.0f;  
float GPSI_LAT = 0.0f;
float GPSI_LON = 0.0f;  
//uint8_t GPS_filter_index = 0;
//float GPS_SUM_LAT[5];
//float GPS_SUM_LON[5];
bool Status_LED_GPS = LOW;
uint8_t Status_waypoint = 0;
uint8_t Counter_LED_GPS = 0;
//int Voltage = 0;
//int Ampere = 0;
////////time roop////////////////////////////////////////////
#define TASK_200HZ 5
#define TASK_100HZ 10
#define TASK_50HZ 20
#define TASK_20HZ 50
#define TASK_10HZ 100
#define TASK_5HZ 200
#define TASK_2HZ 500
#define TASK_1HZ 1000
#define TASK_NoHZ 1200
#define RAD_TO_DEG 57.295779513082320876798154814105
#define DEG_TO_RAD 0.017453292519943295769236907684886

//direction cosine matrix (DCM)   Rotated Frame to Stationary Frame ZYX
float DCM00 = 1.0f;
float DCM01 = 0.0f;
float DCM02 = 0.0f;
float DCM10 = 0.0f;
float DCM11 = 1.0f;
float DCM12 = 0.0f;
float DCM20 = 0.0f;
float DCM21 = 0.0f;
float DCM22 = 1.0f;
//float DCM23 = 1.0;
float cos_rollcos_pitch = 1.0f;

// Main loop variables
unsigned long currentTime = 0;
unsigned long previousTime = 0;
uint16_t frameCounter = 0;
uint8_t timeLanding = 0;
uint8_t timeOff = 0;
byte armed = 0;
float G_Dt = 0.004994121f; 
long Dt_roop = 100;
bool Status_LED = LOW;
uint8_t ESC_calibra = 0;
uint8_t Counter_LED_Auto = 0;
double GPS_LAT_HOMEe = GPS_LAT_HOME;
double GPS_LON_HOMEe = GPS_LON_HOME;
//Baro//////////////////////////////////////////////
//MS561101BA32bit baro = MS561101BA32bit();
AP_Baro_MS5611 baro;
#define  MOVAVG_SIZE 10 //100 80 20
float movavg_buff[MOVAVG_SIZE];
float movavg_buffT[MOVAVG_SIZE];
int movavg_i=0;
float sea_press=1013.25f;
float temperaturetr = 32.5f;
float temperaturetrf = 32.5f;
float presser=1013.25f;
float presserf=1013.25f;
float presserfF=1013.25f;
float Altitude_baro = 0.0f;
float Altitude_barof=0.0f;
float Altitude_Ground = 0.0f;
