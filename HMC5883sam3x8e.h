/*
project_Quad 32 bit Arduino Due  HMC5883sam3x8e.h
1. stabilized quadrotor 
by: tinnakon kheowree 
0860540582
tinnakon_za@hotmail.com
tinnakonza@gmail.com
http://quad3d-tin.lnwshop.com/
https://www.facebook.com/tinnakonza
//#include "HMC5883L.h" I2C  HMC5883L Magnetometer
*/

#define HMC5883_Address (0x3C >> 1) // I2C adress: 0x3C (8bit)   0x1E (7bit)
#define HMC5883_REGISTER_MAG_MR_REG_M 0x02
#define HMC5883_REG_DATA_OUTPUT_X_MSB 0x03
int16_t MagX1,MagY1,MagZ1;
float MagXf,MagYf,MagZf;
float c_magnetom_x;
float c_magnetom_y;
float c_magnetom_z;

void MagHMC5883Int()
{
  Wire1.beginTransmission(HMC5883_Address); //open communication with HMC5883
  Wire1.write(0x00); //Configuration Register A
  Wire1.write(0x18);// measurement output.1 ,, 75 Hz , Normal measurement
  //Wire1.write(0x78); //num samples: 8 ; output rate: 15Hz ; normal measurement mode
  Wire1.endTransmission();
  delay(10);
  Wire1.beginTransmission(HMC5883_Address); //open communication with HMC5883
  Wire1.write(0x01); //Configuration Register B
  Wire1.write(0x20); //configuration gain 1.3Ga
  Wire1.endTransmission();
  delay(10);
  //Put the HMC5883 IC into the correct operating mode
  Wire1.beginTransmission(HMC5883_Address); //open communication with HMC5883
  Wire1.write(0x02); //select mode register
  Wire1.write(0x00); //continuous measurement mode
  Wire1.endTransmission();
  delay(10);
}

void Mag5883Read()
{
  int i = 0;
  byte result[6];
  Wire1.beginTransmission(HMC5883_Address);
  Wire1.write(HMC5883_REG_DATA_OUTPUT_X_MSB);
  Wire1.endTransmission();
  Wire1.requestFrom(HMC5883_Address, 6); //ปัญหา ถ้าสายไฟหลุด โปรแกรมจะค้างตรงนี้ รออ่านค่า เข็มทิศ
    while(Wire1.available())    
  { 
    result[i] = Wire1.read(); 
    i++;
  }
    //result[0] = Wire1.read(); 
    //result[1] = Wire1.read();
    //result[2] = Wire1.read();
    //result[3] = Wire1.read();
    //result[4] = Wire1.read();
    //result[5] = Wire1.read();  
  Wire1.endTransmission();
  MagY1 = (result[0] << 8) | result[1];//X 
  MagZ1 = (result[2] << 8) | result[3];
  MagX1 = ((result[4] << 8) | result[5])*-1;//Y
  if(abs(MagY1) > 1000 || abs(MagZ1) > 1000 || abs(MagX1) > 1000){
    MagX1 = 1;
    MagY1 = 1;
    MagZ1 = 1;
  }
  MagXf = MagXf + (MagX1 - MagXf)*0.45;//0.45
  MagYf = MagYf + (MagY1 - MagYf)*0.45;
  MagZf = MagZf + (MagZ1 - MagZf)*0.45;
 // adjust for  compass axis offsets/sensitivity differences by scaling to +/-5 range
  c_magnetom_x = ((float)(MagXf - M_X_MIN) / (M_X_MAX - M_X_MIN))*10.0 - 5.0;
  c_magnetom_y = ((float)(MagYf - M_Y_MIN) / (M_Y_MAX - M_Y_MIN))*10.0 - 5.0;
  c_magnetom_z = ((float)(MagZf - M_Z_MIN) / (M_Z_MAX - M_Z_MIN))*10.0 - 5.0;
}
void Mag_Calibrate()//Calibration_sensor Magnetometer
{
    // Output MIN/MAX values
    M_X_MIN = 0;
    M_Y_MIN = 0;
    M_Z_MIN = 0;
    M_X_MAX = 0;
    M_Y_MAX = 0;
    M_Z_MAX = 0; 
    Serial.print("magn x,y,z (min/max) = ");
    for (int i = 0; i < 600; i++) {//Calibration 30 s
      digitalWrite(13, HIGH);//30
      Mag5883Read();
      if (MagX1 < M_X_MIN) M_X_MIN = MagX1;
      if (MagX1 > M_X_MAX) M_X_MAX = MagX1;
      if (MagY1 < M_Y_MIN) M_Y_MIN = MagY1;
      if (MagY1 > M_Y_MAX) M_Y_MAX = MagY1;
      if (MagZ1 < M_Z_MIN) M_Z_MIN = MagZ1;
      if (MagZ1 > M_Z_MAX) M_Z_MAX = MagZ1;
      delay(25);
      digitalWrite(13, LOW);//30
      delay(25);
    }
      Serial.print(M_X_MIN);Serial.print("/");
      Serial.print(M_X_MAX);Serial.print("\t");
      Serial.print(M_Y_MIN);Serial.print("/");
      Serial.print(M_Y_MAX);Serial.print("\t");
      Serial.print(M_Z_MIN);Serial.print("/");
      Serial.print(M_Z_MAX);
      Serial.print("\n");
}
