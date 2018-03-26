# Due32bit_GY521MPU_1_V5

/*
 Test By tinnakon kheowree  project 
 
 [Videos](https://www.youtube.com/watch?v=N_tQC7pwRic)
 วีดีโอ !(https://youtu.be/N_tQC7pwRic)
 
  ![1](https://user-images.githubusercontent.com/9403558/37556558-9306c558-2a2a-11e8-84a0-af680209cdbf.jpg)
 
 ![48](https://user-images.githubusercontent.com/9403558/37556570-ce2ba02c-2a2a-11e8-8c49-65559fa065e3.jpg)
 
 ![49](https://user-images.githubusercontent.com/9403558/37556585-f373d02a-2a2a-11e8-9cd1-e0e0c643fca0.jpg)
 
![tk32bit_diagram_v2](https://user-images.githubusercontent.com/9403558/37701027-1cc0fcda-2d20-11e8-8fea-e1986bf25d64.png)

![arduinodue_front3](https://user-images.githubusercontent.com/9403558/37701060-3922921c-2d20-11e8-9a92-98f3d64b9404.png)

![controlx](https://user-images.githubusercontent.com/9403558/37701104-64c4d380-2d20-11e8-99a1-7624189fc246.png)

![mat_roll](https://user-images.githubusercontent.com/9403558/37701246-d2e48054-2d20-11e8-88aa-2b021274caf0.png)

สมการการเคลื่อนที่ โดรน เครื่องบิน 4 ใบพัด

![matmodel](https://user-images.githubusercontent.com/9403558/37702954-bbacbf90-2d26-11e8-9e8c-ea302304ef3c.png)
 
 tinnakon_za@hotmail.com
 
 tinnakonza@gmail.com
 
 http://quad3d-tin.lnwshop.com/
 
 https://www.facebook.com/tinnakonza

12/03/2561     write Due32bit_GY521MPU_1_V3  ,,2,3,4,5,6,7,8,9,10,11,

                                               // Roop time 1000Hz,,// Timer3 Interrupt handler
                                              ,velocity GPS Rotated Frame of arm gps
                                              ,Altitude control
                                              
 16/03/2561     write Due32bit_GY521MPU_1_V4  ,Observer_kalman_filter Z ,, Velocity_THR
 
 26/03/2561     write Due32bit_GY521MPU_1_V5_2  
 
                              ,Observer_kalman_filter X,Y
                              ,RECIEVER CH_AILf, CH_ELEf ,targetRB_speedLAT ,targetRB_speedLON ,,cm/s
                              ,double ,,float Lat,Lon error 9999999.9963922281796504452323604   
                              ,,9999999.9995023947491795059296384
                              ,gain kp GPS ,,yaw_bearing,,k1 kalman Altitude,, uthrottle/cos_rollcos_pitch
 
support : Arduino 1.5.8   Arduino Due 32 bit  , MPU6050  MS5611

• Atmel SAM3X8E ARM Cortex-M3 CPU 32-bit a 84 MHz clock, ARM core microcontroller

• MPU6050 Gyro Accelerometer //I2C 400kHz nois gyro +-0.05 deg/s , acc +-0.04 m/s^2

• MS5611 Barometer//SPI

• HMC5883L Magnetometer //I2C_BYPASS ,I2C 400kHz

• GPS NEO-M8N //

Timer

timer 7 TC2-1  use read ppm

Quad-X

pin 9 FRONTL  M1 CW        M2 CCW  FRONTR pin 8

          pin 9          pin 8
              \         / 
                \ --- /
                 |   |
                / --- \
              /         \ 
         pin 6            pin 7
              
pin 6 motor_BackL  M4 CCW      M3 CW  motor_BackR  pin 7

----------rx-----------  

A8 = PPM 8 CH

 */
