#include <Wire.h>
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter
#include <Encoder.h>

#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"
//**********define motors****************//
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Adafruit_DCMotor *M_L = AFMS.getMotor(1);
Adafruit_DCMotor *M_R = AFMS.getMotor(2);

Encoder knobLeft(2, 3);
Encoder knobRight(18, 19);

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

//**************  1 ------ Left     2----------Right   ******************



#define CURRENT_LIMIT   1000                     // high current warning
#define LOW_BAT         10000                   // low bat warning
#define LOOPTIME        50                     // PID loop time  ******do not change
#define NUMREADINGS     10                      // samples for Amp average

float Ts = 0.05;

 
int readings[NUMREADINGS];
unsigned long lastMilli = 0;                    // loop timing 
unsigned long lastMilliPrint = 0;               // loop timing
float speed_act_1 = 0;                              // speed (actual value)
float speed_act_2 = 0;                              // speed (actual value)
int PWM_val_1 = 0;                                // (25% = 64; 50% = 127; 75% = 191; 100% = 255)
int PWM_val_2 = 0;
int voltage = 0;                                // in mV
int current = 0;                                // in mA
int long count_1 = 0;                        // rev counter of Left Motor
int long count_2 = 0;                        // rev counter of Right Motor
int long countAnt_1 = 0;
int long countAnt_2 = 0;

//************************filter gain design*****************
float w_est_L = 0;
float w_est_R = 0;
float w_est_old_L;
float w_est_old_R;
float Km = 0.09516;
float Ke = 0.9048;

//********************gain design ************************************
float K1_1 = -0.002797284499121;
float K1_2 = -0.002782939450407;
float K1_3 = -0.050940458110238;
float K1_4 = -9.968014394097996e-04;
float K2_1 = 0.007235403639657;
float K2_2 = 9.616641614941482e-04;
float Kv_a = K1_2+K1_1*Ts/2;
float Kv_b = -K1_2+K1_1*Ts/2;
float Ka_a = K1_4+K1_3*Ts/2;
float Ka_b = -K1_4+K1_3*Ts/2;

// ********motor control*******
//***************motor gain*******
float Kp_m = 5;
float Ki_m= 3;
float K_a = (Kp_m+Ki_m*LOOPTIME/2000); // discrete pi gains
float K_b = (-Kp_m+Ki_m*LOOPTIME/2000);
//*********motor input***********;
float vol_in_L;
float vol_in_R;
float w_dot_req_L;
float w_dot_req_R;
float w_dot_act_L;
float w_dot_act_R;

//****************CONTROL SIGNAL***********************************
float X_in = 0;
float X_dot_in = 0;
float yaw_in = 0;
float yaw_dot_in = 0;

//*****************Parameters*************
float wheel_r = 0.045085; //wheel radius meter
float pi = 3.1415926;
float D = 0.24384; //m lateral distance between the contact patches of the wheel
float deg2rad = 2*pi/(360);
float rad2deg = 360/(2*pi);
float J = 0.0079; // moment of inertia about rotation axis

//************States**********
float X_act;
float X_act_dot;
float yaw_act;
float yaw_act_dot;

//***************system control input********
float torque_sum=0 ;
float torque_min=0;
float torque_L;
float torque_R;

//****************debug*************print
  float old_error_w_dot_L=0;
 float old_error_w_dot_R=0;
 float error_w_dot_L;
 float error_w_dot_R;
float error_w_dot_L_old = 0;
float error_w_dot_R_old = 0;
  float speed_m_1;
  float speed_m_2;
  float speed_act_1_old = 0;
 float speed_act_2_old = 0;
 float speed_act_1_oold = 0;
 float speed_act_2_oold = 0;


/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
//int16_t tempRaw;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter
double gyroYrate;
double gyroXrate;


uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data
// TODO: Make calibration routine
int pin_gyro;

void setup() {
  AFMS.begin();
  Serial.begin(9600);
  Wire.begin();
#if ARDUINO >= 157
  Wire.setClock(100000UL); // Set I2C frequency to 400kHz
#else
  TWBR = ((F_CPU / 100000UL) - 16) / 2; // Set I2C frequency to 400kHz
#endif

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }

  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (i2cData[0] << 8) | i2cData[1];
  accY = (i2cData[2] << 8) | i2cData[3];
  accZ = (i2cData[4] << 8) | i2cData[5];


  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
//  gyroXangle = roll;
//  gyroYangle = pitch;
//  compAngleX = roll;
//  compAngleY = pitch;

  timer = micros();

  analogReference(EXTERNAL);                            // Current external ref is 3.3V
  
  for(int i=0; i<NUMREADINGS; i++)   readings[i] = 0;  // initialize readings to 0

  //attachInterrupt(pin_gyro,readgyro,FALLING);
  
}

void loop() {
  
    readgyro();
    rencoder();
   double dt = (double)(millis() - timer) / 1000; // Calculate delta time
   timer = millis();

#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

   gyroXrate = gyroX / 131.0; // Convert to deg/s
   gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;
    
    if((millis()-lastMilli) >= LOOPTIME){
    static float old_error_v=0;
    static float old_error_a = 0;
    lastMilli=millis();
    getMotorData();
    X_act = count_1*2*pi/(64*29)*wheel_r;
    X_act_dot = speed_act_1*wheel_r;
    yaw_act = (count_1-count_2)*2*pi*wheel_r/(64*29*D);
    yaw_act_dot = (speed_act_1-speed_act_2)*wheel_r/D;
    float error_v = X_act_dot-X_dot_in;
    float error_a = gyroYrate*deg2rad;
    torque_sum = torque_sum-(Kv_a*error_v+Kv_b*old_error_v+Ka_a*error_a+Ka_b*old_error_a);
    old_error_v = error_v;
    old_error_a = error_a;
    torque_min = 0; // -(K2_1*(yaw_act-yaw_in)-K2_2*(yaw_act_dot-yaw_dot_in));
    torque_L = (torque_sum+torque_min)/2;
    torque_R = torque_sum-torque_L;
    w_dot_req_L = torque_L/J;
    w_dot_req_R = torque_R/J;
    old_error_w_dot_L = error_w_dot_L;
    old_error_w_dot_R = error_w_dot_R;
    error_w_dot_L = w_dot_req_L-w_dot_act_L;
    error_w_dot_R = w_dot_req_R-w_dot_act_R;  
    vol_in_L = vol_in_L+K_a*error_w_dot_L+K_b*old_error_w_dot_L;
    vol_in_R = vol_in_R+K_a*error_w_dot_R+K_b*old_error_w_dot_R;
      if(vol_in_L>0) {
      M_L->run(FORWARD);
      }
      if(vol_in_L<0) {
      M_L->run(BACKWARD);
      }
      if(vol_in_R>0) {
      M_R->run(FORWARD);
      }
      if(vol_in_R<0) {
      M_R->run(BACKWARD);
      } 
  
  PWM_val_1 = volt2pwm(abs(vol_in_L));
  PWM_val_2 = volt2pwm(abs(vol_in_R));
  M_L->setSpeed(PWM_val_1);
  M_R->setSpeed(PWM_val_2);
    printdata();
    }
}

void printdata(){
  //Serial.print("KalAngleY = ");
  Serial.print(kalAngleY); Serial.print(" ");
  Serial.print(vol_in_L );Serial.println(" ");
  //Serial.println(PWM_val_2);Serial.print(" ");
  //Serial.print("\t");
}

void readgyro(){
   while (i2cRead(0x3B, i2cData, 14));
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  gyroX = (i2cData[8] << 8) | i2cData[9];
  gyroY = (i2cData[10] << 8) | i2cData[11];
  gyroZ = (i2cData[12] << 8) | i2cData[13];
}



void rencoder()  {
  countAnt_1 = count_1;
  countAnt_1 = count_1;                                   
  count_1 = knobLeft.read();
  count_2 = knobRight.read();
}

void getMotorData()  {
  speed_m_1 = (count_1 - countAnt_1)*2*pi*1000/(64*29*LOOPTIME);//   radia/s
  speed_m_2 = (count_2 - countAnt_2)*2*pi*1000/(64*29*LOOPTIME);
  w_est_old_L = w_est_L;
  w_est_old_R = w_est_R;
  w_est_L = Ke*w_est_old_L+Km*speed_m_1;
  w_est_R = Ke*w_est_old_R+Km*speed_m_2; 
  speed_act_1 = w_est_L;
  speed_act_2 = w_est_R; 
  w_dot_act_L = (3*speed_act_1-4*speed_act_1_old+speed_act_1_oold)*1000/(LOOPTIME);
  w_dot_act_R = (3*speed_act_2-4*speed_act_2_old+speed_act_2_oold)*1000/(LOOPTIME);
  speed_act_1_old = speed_act_1;
  speed_act_1_oold = speed_act_1_old;
  speed_act_2_old = speed_act_2;
  speed_act_2_oold = speed_act_2_old;
}


//void MotorVoltageUpdate ()  {
//
//
//  
//  error_w_dot_L = w_dot_req_L-w_dot_act_L;
//  error_w_dot_R = w_dot_req_R-w_dot_act_R;
//  int_error_w_dot_L = int_error_w_dot_L+error_w_dot_L*(LOOPTIME/1000);
//  int_error_w_dot_R = int_error_w_dot_R+error_w_dot_R*(LOOPTIME/1000);
//  vol_in_L = error_w_dot_L*Kp_m+int_error_w_dot_L*Ki_m;
//  vol_in_R = error_w_dot_R*Kp_m+int_error_w_dot_R*Ki_m;
//
//    if(vol_in_L>0) {
//      digitalWrite(InA1, LOW);
//      digitalWrite(InB1, HIGH);
//      }
//      if(vol_in_L<0) {
//        digitalWrite(InA1,HIGH);
//        digitalWrite(InB1, LOW);
//      }
//      if(vol_in_R>0) {
//      digitalWrite(InA2, LOW);
//      digitalWrite(InB2, HIGH);
//      }
//      if(vol_in_R<0) {
//        digitalWrite(InA2,HIGH);
//        digitalWrite(InB2, LOW);
//      } 
//  
//  PWM_val_1 = volt2pwm(abs(vol_in_L));
//  PWM_val_2 = volt2pwm(abs(vol_in_R));
//  
//}

int volt2pwm(float vol){
  int pwm;
  pwm = 0.6272*vol*vol*vol-6.475*vol*vol+27.72*vol+11.53;
  return constrain(pwm,0,255);
}
