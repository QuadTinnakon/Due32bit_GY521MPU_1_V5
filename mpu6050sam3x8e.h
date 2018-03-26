/*
project_Quad 32 bit Arduino Due mpu6050sam3x8e.h
1. stabilized quadrotor 
by: tinnakon kheowree 
0860540582
tinnakon_za@hotmail.com
tinnakonza@gmail.com
http://quad3d-tin.lnwshop.com/
https://www.facebook.com/tinnakonza
//#include "mpu6050.h" MPU 6050 I2C Gyroscope and Accelerometer MPU6050 HMC5883L Magnetometer
*/
#define XAXIS 0
#define YAXIS 1
#define ZAXIS 2
float gyro[3];
float accel[3];
float gyroScaleFactor = radians(2000.0f/32768.0f);//32768.0 32730 , +-32756  32768.0
float accelScaleFactor = 9.80665f;//9.80665/4100.62 9.81/8192.09 9.81 / 8205  
uint8_t gyroSamples = 0;
uint8_t gyroSamples2 = 0;
uint8_t accSamples = 0;
int16_t gyroRaw[3];
float gyroSum[3];
int16_t accelRaw[3];
float accelRawffx,accelRawffy,accelRawffz;
float accelSum[3];
float AccXm,AccYm,AccZm;
float AccX,AccY,AccZ;
float AccXf,AccYf,AccZf;
float AccX2,AccY2,AccZ2;
float GyroXf,GyroYf,GyroZf;
float gyro_offsetX,gyro_offsetY,gyro_offsetZ,acc_offsetX,acc_offsetY,acc_offsetZ;
float acc_offsetZ2 = 9.80665f;
float GyroX,GyroY,GyroZ,GyroTemp;
float GyroX2,GyroY2,GyroZ2;
float Accel[3] = {0.0f,0.0f,0.0f};
float filteredAccel[3] = {0.0f,0.0f,0.0f};

//sensor MPU6050 -------------------------------------
// MPU 6050 Registers
#define MPU6050_ADDRESS         0x68
//#define MPUREG_WHOAMI           0x75
#define MPUREG_SMPLRT_DIV       0x19
#define MPUREG_CONFIG           0x1A
#define MPUREG_GYRO_CONFIG      0x1B
#define MPUREG_ACCEL_CONFIG     0x1C
//#define MPUREG_FIFO_EN          0x23
#define MPUREG_INT_PIN_CFG      0x37
//#define MPUREG_INT_ENABLE       0x38
//#define MPUREG_INT_STATUS       0x3A
#define MPUREG_ACCEL_XOUT_H     0x3B
//#define MPUREG_ACCEL_XOUT_L     0x3C
//#define MPUREG_ACCEL_YOUT_H     0x3D
//#define MPUREG_ACCEL_YOUT_L     0x3E
//#define MPUREG_ACCEL_ZOUT_H     0x3F
//#define MPUREG_ACCEL_ZOUT_L     0x40
//#define MPUREG_TEMP_OUT_H       0x41
//#define MPUREG_TEMP_OUT_L       0x42
#define MPUREG_GYRO_XOUT_H      0x43
//#define MPUREG_GYRO_XOUT_L      0x44
//#define MPUREG_GYRO_YOUT_H      0x45
//#define MPUREG_GYRO_YOUT_L      0x46
//#define MPUREG_GYRO_ZOUT_H      0x47
//#define MPUREG_GYRO_ZOUT_L      0x48
//#define MPUREG_USER_CTRL        0x6A
#define MPUREG_PWR_MGMT_1       0x6B
//#define MPUREG_PWR_MGMT_2       0x6C
//#define MPUREG_FIFO_COUNTH      0x72
//#define MPUREG_FIFO_COUNTL      0x73
//#define MPUREG_FIFO_R_W         0x74
// Configuration bits
//#define BIT_SLEEP               0x40
#define BIT_H_RESET             0x80
//#define BITS_CLKSEL             0x07
//#define MPU_CLK_SEL_PLLGYROX    0x01
#define MPU_CLK_SEL_PLLGYROZ    0x03
//#define MPU_EXT_SYNC_GYROX      0x02
//#define BITS_FS_250DPS          0x00
//#define BITS_FS_500DPS          0x08
//#define BITS_FS_1000DPS         0x10
#define BITS_FS_2000DPS         0x18
//#define BITS_FS_MASK            0x18
//#define BITS_DLPF_CFG_256HZ  0x00// //Default settings LPF 256Hz/8000Hz sample
//#define BITS_DLPF_CFG_188HZ         0x01
//#define BITS_DLPF_CFG_98HZ          0x02
#define BITS_DLPF_CFG_42HZ          0x03
//#define BITS_DLPF_CFG_20HZ          0x04
//#define BITS_DLPF_CFG_10HZ          0x05
//#define BITS_DLPF_CFG_5HZ           0x06
//#define BITS_DLPF_CFG_2100HZ_NOLPF  0x07
//#define BITS_DLPF_CFG_MASK          0x07
//#define BIT_INT_ANYRD_2CLEAR    0x10
//#define BIT_RAW_RDY_EN          0x01
//#define BIT_I2C_IF_DIS          0x10
//#define BIT_INT_STATUS_DATA     0x01
//sensor ---------------------
#define applyDeadband(value,deadband)   \
  if(fabs(value) < deadband) {          \
    value = 0.0f;                        \
  } else if(value > 0.0f){               \
    value -= deadband;                  \
  } else if(value < 0.0f){               \
    value += deadband;                  \
  }
void mpu6050_initialize()
{
    Serial.print("mpu6050_initialize");Serial.print("\n");
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(MPUREG_PWR_MGMT_1);    // Chip reset DEVICE_RESET 1
    Wire.write(BIT_H_RESET);//DEVICE_RESET
    Wire.endTransmission();  
    delay(30);// Startup delay      
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(MPUREG_PWR_MGMT_1);
    Wire.write(MPU_CLK_SEL_PLLGYROZ);//CLKSEL 3 (PLL with Z Gyro reference)
    Wire.endTransmission();    
    delay(5);
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(MPUREG_SMPLRT_DIV);// SAMPLE RATE
    Wire.write(0x00);//// Sample rate = 1kHz
    Wire.endTransmission();  
    delay(5);
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(MPUREG_CONFIG);
    Wire.write(BITS_DLPF_CFG_42HZ);//98 BITS_DLPF_CFG_188HZ, BITS_DLPF_CFG_42HZ, default DLPF_CFG = 0 => ACC bandwidth = 260Hz  GYRO bandwidth = 256Hz)
    Wire.endTransmission();  
    delay(5);   
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(MPUREG_GYRO_CONFIG);
    Wire.write(BITS_FS_2000DPS);//BITS_FS_1000DPS FS_SEL = 3: Full scale set to 2000 deg/sec,  BITS_FS_2000DPS
    Wire.endTransmission(); 
    delay(5);  
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(MPUREG_ACCEL_CONFIG);
    Wire.write(0x10);//0x10 = 1G=4096, AFS_SEL=2 (0x00 = +/-2G)  1G = 16,384 //0x10 = 1G = 4096 ,//0x08 = +-4g
    Wire.endTransmission();
    delay(5);
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(MPUREG_INT_PIN_CFG);// enable I2C bypass for AUX I2C
    Wire.write(0x00);//0x00=off ,, 0x02 , I2C_BYPASS_EN=1
    Wire.endTransmission();     
}
void mpu6050_Gyro_Values()
{
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(MPUREG_GYRO_XOUT_H);
    Wire.endTransmission();
    Wire.requestFrom(MPU6050_ADDRESS, 6);
     int i = 0;
     byte result[6];
  while(Wire.available())    
  { 
    result[i] = Wire.read(); 
    i++;
  }
  Wire.endTransmission();
    gyroRaw[XAXIS] = ((result[0] << 8) | result[1]);//-12     -3
    gyroRaw[YAXIS] = ((result[2] << 8) | result[3])*-1;//37      15
    gyroRaw[ZAXIS] = ((result[4] << 8) | result[5])*-1;//11    5
}	
void mpu6050_Accel_Values()
{
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(MPUREG_ACCEL_XOUT_H);
    Wire.endTransmission();
    Wire.requestFrom(MPU6050_ADDRESS, 6);
    int i = 0;
    byte result[6];
  while(Wire.available())    
  { 
    result[i] = Wire.read(); 
    i++;
  }
  Wire.endTransmission();
    accelRaw[XAXIS] = ((result[0] << 8) | result[1])*-1 + 369;//+ 105 + 100  max=4054.46, min=-4149.48
    accelRaw[YAXIS] = ((result[2] << 8) | result[3]) + 40;// - 45 - 35      max=4129.57, min=-4070.64
    accelRaw[ZAXIS] = ((result[4] << 8) | result[5]) - 237;// + 150 + 170    max=4014.73 , min=-4165.13
     // adjust for  acc axis offsets/sensitivity differences by scaling to +/-1 g range
  AccXm = ((float)(accelRaw[XAXIS] - A_X_MIN) / (A_X_MAX - A_X_MIN))*2.0f - 1.0f;
  AccYm = ((float)(accelRaw[YAXIS] - A_Y_MIN) / (A_Y_MAX - A_Y_MIN))*2.0f - 1.0f;
  AccZm = ((float)(accelRaw[ZAXIS] - A_Z_MIN) / (A_Z_MAX - A_Z_MIN))*2.0f - 1.0f;
}
void mpu6050_readGyroSum() {
    mpu6050_Gyro_Values();
    gyroSum[XAXIS] += gyroRaw[XAXIS];
    gyroSum[YAXIS] += gyroRaw[YAXIS];
    gyroSum[ZAXIS] += gyroRaw[ZAXIS];
    gyroSamples++;
}
void mpu6050_Get_gyro()
{       
    if(gyroSamples == 0){
      gyroSamples = 1;
    }
    GyroX = (gyroSum[XAXIS] / gyroSamples)*gyroScaleFactor - gyro_offsetX;// Calculate average
    GyroY = (gyroSum[YAXIS] / gyroSamples)*gyroScaleFactor - gyro_offsetY;
    GyroZ = (gyroSum[ZAXIS] / gyroSamples)*gyroScaleFactor - gyro_offsetZ;            
    gyroSum[XAXIS] = 0.0f;// Reset SUM variables
    gyroSum[YAXIS] = 0.0f;
    gyroSum[ZAXIS] = 0.0f;
    gyroSamples2 = gyroSamples;
    gyroSamples = 0;            
}
void mpu6050_readAccelSum() {
    mpu6050_Accel_Values();
    accelSum[XAXIS] += AccXm;
    accelSum[YAXIS] += AccYm;
    accelSum[ZAXIS] += AccZm;  
    accSamples++;
}
void mpu6050_Get_accel()
{
    if(accSamples == 0){
      accSamples = 1;
    }
    AccX = (accelSum[XAXIS] / accSamples)*accelScaleFactor - acc_offsetX;// Calculate average
    AccY = (accelSum[YAXIS] / accSamples)*accelScaleFactor - acc_offsetY;
    AccZ = (accelSum[ZAXIS] / accSamples)*accelScaleFactor;// Apply correct scaling (at this point accel reprensents +- 1g = 9.81 m/s^2)
    accelSum[XAXIS] = 0.0f;    // Reset SUM variables
    accelSum[YAXIS] = 0.0f;
    accelSum[ZAXIS] = 0.0f; 
    accSamples = 0;   
}
void sensor_Calibrate()
{
    Serial.print("Sensor_Calibrate");Serial.println("\t");
    gyroSum[XAXIS] = 0.0f;// Reset SUM variables
    gyroSum[YAXIS] = 0.0f;
    gyroSum[ZAXIS] = 0.0f;
    gyroSamples = 0;  
    accelSum[XAXIS] = 0.0f;    // Reset SUM variables
    accelSum[YAXIS] = 0.0f;
    accelSum[ZAXIS] = 0.0f; 
    for (uint8_t i=0; i<45; i++) //Collect 60, 100 samples
    {
        Serial.print("- ");
        mpu6050_readGyroSum();
        mpu6050_readAccelSum();
        Mag5883Read();
        digitalWrite(13, HIGH);
        digitalWrite(Pin_LED_R, HIGH);
        delay(20);
        digitalWrite(13, LOW);
        digitalWrite(Pin_LED_R, LOW);
        delay(20);
    }
    Serial.println("- ");
    gyro_offsetX = (gyroSum[XAXIS]/gyroSamples)*gyroScaleFactor;
    gyro_offsetY = (gyroSum[YAXIS]/gyroSamples)*gyroScaleFactor;
    gyro_offsetZ = (gyroSum[ZAXIS]/gyroSamples)*gyroScaleFactor;
    acc_offsetX = (accelSum[XAXIS]/gyroSamples)*accelScaleFactor;
    acc_offsetY = (accelSum[YAXIS]/gyroSamples)*accelScaleFactor;
    acc_offsetZ = (accelSum[ZAXIS]/gyroSamples)*accelScaleFactor;
    acc_offsetZ2 = sqrt(acc_offsetX*acc_offsetX + acc_offsetY*acc_offsetY + acc_offsetZ*acc_offsetZ);
    AccZf = acc_offsetZ;//15.4
    MagXf = MagX1;
    MagYf = MagY1;
    MagZf = MagZ1;
    gyroSamples = 0;
    accSamples = 0;
    Serial.print("GYRO_Calibrate");Serial.print("\t");
    Serial.print(gyro_offsetX);Serial.print("\t");//-0.13
    Serial.print(gyro_offsetY);Serial.print("\t");//-0.10
    Serial.print(gyro_offsetZ);Serial.println("\t");//0.03 
    Serial.print("ACC_Calibrate");Serial.print("\t");
    Serial.print(acc_offsetX);Serial.print("\t");
    Serial.print(acc_offsetY);Serial.print("\t");
    Serial.print(acc_offsetZ);Serial.print("\t");
    Serial.print(acc_offsetZ2);Serial.println("\t"); 
    Serial.print("MAG_Calibrate");Serial.print("\t");
    Serial.print(MagXf);Serial.print("\t");
    Serial.print(MagYf);Serial.print("\t");
    Serial.print(MagZf);Serial.println("\t");
acc_offsetX = 0.02f;//-0.18 0.11 -0.36  Trim PITCH CONTROL   -10.07	-10.55	-9.82
acc_offsetY = 0.09f;//0.16 -0.14 0.18 Trim ROLL CONTROL     10.39	9.74	11
//acc_offsetZ = 0.0;//0.245 0.235 10.2
}
/* ******************************************************************
  ***      Accelerometers trim     Remote Trim By tinnakon   ***
//With the help of your roll and pitch stick you could now trim the ACC mode.
//You must first put the throttle stick in maximal position. (obviously with motors disarmed)
//full PITCH forward/backward and full ROLL left/right (2 axis possibilities) will trim the level 
//mode according to the neutral angle you want to change.
//The status LED will blink to confirm each ticks.
*/
  void Remote_TrimACC() {
     if(CH_THR > MAXCHECK && armed == 0)
    {
 ////Trim ROLL CONTROL/////////////
      if(CH_AIL > MAXCHECK)
      {
        acc_offsetY = acc_offsetY + 0.04f;
           for (int i = 0; i < 5; i++)
           {
            digitalWrite(13, HIGH);
            delay(20);
            digitalWrite(13, LOW);
            delay(20);
           }
           Serial.println(acc_offsetY);
      }
      if(CH_AIL < MINCHECK)
      {
        acc_offsetY = acc_offsetY - 0.04f;
           for (int i = 0; i < 5; i++)
           {
            digitalWrite(13, HIGH);
            delay(20);
            digitalWrite(13, LOW);
            delay(20);
           }
           Serial.println(acc_offsetY);
      }
 ///////Trim PITCH CONTROL//////////////////////
         if(CH_ELE > MAXCHECK)
      {
        acc_offsetX = acc_offsetX + 0.04f;
           for(int i = 0; i < 5; i++)
           {
            digitalWrite(13, HIGH);
            delay(20);
            digitalWrite(13, LOW);
            delay(20);
           }
           Serial.println(acc_offsetX);
      }
      if(CH_ELE < MINCHECK)
      {
        acc_offsetX = acc_offsetX - 0.04f;
           for(int i = 0; i < 5; i++)
           {
            digitalWrite(13, HIGH);
            delay(20);
            digitalWrite(13, LOW);
            delay(20);
           }
           Serial.println(acc_offsetX);
      }
    }//end CH_THR > MAXCHECK && armed == 0
  }///////////////////////////////////////////////////////
