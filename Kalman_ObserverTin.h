/*
project_Quad 32 bit Arduino Due 
//GPS
1. stabilized quadrotor 
by: tinnakon kheowree 
0860540582
tinnakon_za@hotmail.com
tinnakonza@gmail.com
http://quad3d-tin.lnwshop.com/
https://www.facebook.com/tinnakonza

//A=[1 0.01;0 1]; //B=[0;0.01];  //C = [1 0];
//Plant = ss(A,[B B],C,0,-1,'inputname',{'u' 'w'},'outputname','y');
//Q = 0.1; % A number greater than zero
//R = 0.135; % A number greater than zero
//[kalmf,L,P,M,Z] = kalman(Plant,Q,R); //%kalmf = kalmf(1,:); //M,   % innovation gain
*/
void Observer_kalman_filter()
{
 u_z = ((motor_FrontLf + motor_FrontRf + motor_BackLf + motor_BackRf) - 4000.0f)*0.00630057f - 8.81f;//1557us = 1.16kg  uz - g ,,unit N *0.001691) - 9.81
 u_zdot = (u_z - u_zold)/(G_Dt*DCM22);
 u_zold = u_z;
 u_Xdot = u_zdot*DCM02*-100.0f;
 u_Ydot = u_zdot*DCM12*-100.0f;
///Complimentary Filter, Observer velocity vx vy vz kalman//accelerometer and GPS/////////////////////////////////////////////////////
     //Predicted (a priori) state estimate ,,m*x_dotdot + c*x_dot = m*g + u_x*(cos r*sin p*cos y + sin r*sin y)
      x3_at = vx3_hat + (u_Xdot - vx3_hat*c_XYquad/m_quad)*G_Dt;//cm/s^2
      x2_vt = vx2_hat + x3_at*G_Dt;//cm/s
      x1_xt = vx1_hat + (double)x2_vt*G_Dt;//cm
///Y/////////////////////////////////////////////////////////////////////////////////
      y3_at = vy3_hat + (u_Ydot - vy3_hat*c_XYquad/m_quad)*G_Dt;
      y2_vt = vy2_hat + y3_at*G_Dt;//cm/s
      y1_xt = vy1_hat + (double)y2_vt*G_Dt;
      //Updated (a posteriori) state estimate //Update estimate with measurement zk
      double er_xt = (GPS_coordLAT*1e-7 - 13.86818012d)*cm_per_deg_lat - x1_xt;//R = 6,367,449 m; // Radius of earth in KM = 109397.6142 xx
      float er_vt = velocity_north - x2_vt;//cm/s
      float er_at = accrX_Earth*100.0f - x3_at;//cm/s^2
      double ey_xt = (GPS_coordLON*1e-7 - 100.49629712d)*cm_per_deg_lon - y1_xt;//R = 6,367,449 m; // Radius of earth in KM = 109397.6142 xx
      float ey_vt = velocity_east - y2_vt;//cm/s
      float ey_at = accrY_Earth*100.0f - y3_at;//cm/s^2
      vx3_hat = x3_at + er_at*0.058824f;//0.078824 0.088824
      vx2_hat = x2_vt + er_vt*0.011002f;//0.015521 0.040521 0.080521 0.012821 kalman gain ,0.009821, 0.01321 ,0.0221, nois = +-0.002240243
      vx1_hat = x1_xt + er_xt*0.00411881d;//0.0011882 0.051882 0.191882 0.091882 0.0488824 0.188824
      x1_hat = 13.86818012d + (vx1_hat/cm_per_deg_lat);
//Y/////////////////////////////////////////////////////////////////////////////
      vy3_hat = y3_at + ey_at*0.058824f;//0.078824 0.088824
      vy2_hat = y2_vt + ey_vt*0.011002f;//0.015521 0.040521 0.080521 0.012821 kalman gain ,0.009821, 0.01321 ,0.0221, nois = +-0.001240243
      vy1_hat = y1_xt + ey_xt*0.00411881d;//0.0011882 0.051882 0.191882 0.091882 0.0488824 0.188824
      y1_hat = 100.49629712d + (vy1_hat/cm_per_deg_lon);
//Predicted (a priori) state estimate  Altitude
z3_acc = z3_hat + (u_zdot - c_quad*z3_hat)*G_Dt;//1.52, 1.23 ,z3_hat = Acceleration,,m/s^2
z2_hat2 = z2_hat + z3_acc*G_Dt;//z2_hat = velocity ,, m/s
z1_hat2 = z1_hat + z2_hat2*G_Dt;//z1_hat = Altitude ,, m
//z1_hat = constrain(z1_hat, 0, 100);//0 - 100 m
///////////////////////////////////////////////////////////////////
//Updated (a posteriori) state estimate //Update estimate with measurement zk
float ee1 = (Altitude_Baro_ult - z1_hat2);
float ee2 = (Vz_Baro_ult - z2_hat2);
float ee3 = (accrZ_Earth - z3_acc);
//ui_z1_hat = ui_z1_hat + ee3*G_Dt;//I,0.00086056 0.001056 ,0.004056 ,0.008056 ,0.02156 ,1.0156 ,0.156
z3_hat = z3_acc + 0.088824f*ee3;//0.018824 ,0.18824 m/s^2 ,,0.12824 0.21845
//K3 = 0.00124 0.12502 ,, k3 = 0.01641023 ,, k-3 = 0.12367 0.72367
z2_hat = z2_hat2 + 0.000981202f*ee2;//m/s 0.00151202 0.00751202 0.086056
//K2 = 0.0091202 0.012502 0.0002141023, 0.0001641, 0.00145, 0.00045,,,k2 = 0.0001641023 0.097502,  0.065502, 0.009
z1_hat = z1_hat2 + 0.0061102f*ee1;
//K1 =0.0061102  0.0081102 0.011102 0.015102 0.035102, 0.015102 0.065 0.2887 ,, k1 = 0.0001741023 0.09187 0.01187 0.0140
}
