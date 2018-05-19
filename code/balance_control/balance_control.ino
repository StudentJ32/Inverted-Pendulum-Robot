/* Copyright (C) exclusively belongs to Jason Z.  All rights reserved.

 This software may be distributed and modified under the terms of the GNU
 General Public License version 2 (GPL2) as published by the Free Software
 Foundation and appearing in the file GPL2.TXT included in the packaging of
 this file. Please note that GPL2 Section 2[b] requires that all works based
 on this software must also be made publicly available under the terms of
 the GPL2 ("Copyleft").

 Contact information
 -------------------

 Kristian Lauszus, TKJ Electronics
 Web      :  http://www.tkjelectronics.com
 e-mail   :  kristianl@tkjelectronics.com
 */

#include <Wire.h>
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

//**************  1 ------ Left     2----------Right   ******************

//*********Motor Left defines*****************
#define InA1             6                      // INA motor pin
#define InB1            7                      // INB motor pin 
#define PWM1            5                       // PWM motor pin
#define encodPinA1      4                       // encoder A pin
#define encodPinB1      3                       // encoder B pin

//*********Motor Right defines*****************
#define InA2            9                      // INA motor pin
#define InB2            8                      // INB motor pin 
#define PWM2            10                       // PWM motor pin
#define encodPinA2      11                       // encoder A pin
#define encodPinB2      12                       // encoder B pin




#define CURRENT_LIMIT   1000                     // high current warning
#define LOW_BAT         10000                   // low bat warning
#define LOOPTIME        50                     // PID loop time
#define NUMREADINGS     10                      // samples for Amp average




 int encoder0PinALast_1 = LOW;
 int n_1 = LOW;
 int encoder0PinALast_2 = LOW;
 int n_2 = LOW;

 
int readings[NUMREADINGS];
unsigned long lastMilli = 0;                    // loop timing 
unsigned long lastMilliPrint = 0;               // loop timing
int speed_req_1;                            // speed (Set Point) of Left 
float speed_act_1 = 0;                              // speed (actual value)
int speed_req_2;                            // speed (Set Point) of Right 
float speed_act_2 = 0;                              // speed (actual value)
int PWM_val_1 = 0;                                // (25% = 64; 50% = 127; 75% = 191; 100% = 255)
int PWM_val_2 = 0;
int voltage = 0;                                // in mV
int current = 0;                                // in mA
volatile long count_1 = 0;                        // rev counter of Left Motor
volatile long count_2 = 0;                        // rev counter of Right Motor
float Kp =   1.5;                                // PID proportional control Gain
float Kd =    1;  // PID Derivitave control gain

//********************gain design ************************************
float K1_1 = -0.002993949177662;
float K1_2 = -0.003768432089662;
float K1_3 = -0.060673150916429;
float K1_4 = -0.001277797979280;
float K2_1 = 0.007315078724652;
float K2_2 = 7.908193215839999e-04;
// ********motor control*******
//***************motor gain*******
float Kp_m = 2;
float Ki_m= 150;
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
float J = 0.002; // moment of inertia about rotation axis

//************States**********
float X_act;
float X_act_dot;
float yaw_act;
float yaw_act_dot;

//***************system control input********
float torque_sum;
float torque_min;
float torque_L;
float torque_R;


/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
//int16_t tempRaw;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

// TODO: Make calibration routine

void setup() {
  Serial.begin(115200);
  Wire.begin();
#if ARDUINO >= 157
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
#else
  TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
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
  pinMode(InA1, OUTPUT);
  pinMode(InB1, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(encodPinA1, INPUT); 
  pinMode(encodPinB1, INPUT); 
  digitalWrite(encodPinA1, HIGH);                      // turn on pullup resistor
  digitalWrite(encodPinB1, HIGH);

  pinMode(InA2, OUTPUT);
  pinMode(InB2, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(encodPinA2, INPUT); 
  pinMode(encodPinB2, INPUT); 
  digitalWrite(encodPinA2, HIGH);                      // turn on pullup resistor
  digitalWrite(encodPinB2, HIGH);
  
  attachInterrupt(1, rencoder, FALLING);
  for(int i=0; i<NUMREADINGS; i++)   readings[i] = 0;  // initialize readings to 0

  analogWrite(PWM1, PWM_val_1);
  digitalWrite(InA1, LOW);
  digitalWrite(InB1, HIGH);
  analogWrite(PWM2, PWM_val_2);
  digitalWrite(InA2, LOW);
  digitalWrite(InB2, HIGH);
  
}

void loop() {
  
  /* Update all the values */
  while (i2cRead(0x3B, i2cData, 14));
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  gyroX = (i2cData[8] << 8) | i2cData[9];
  gyroY = (i2cData[10] << 8) | i2cData[11];
  gyroZ = (i2cData[12] << 8) | i2cData[13];

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

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
    rencoder();
   if((millis()-lastMilli) >= LOOPTIME)   {                                    // enter tmed loop
    lastMilli = millis();
    getMotorData();
    X_act = count_1*2*pi/1600*wheel_r;
    X_act_dot = speed_act_1*2*pi*wheel_r/60;
    yaw_act = (count_1-count_2)*2*pi*wheel_r/(1600*D);
    yaw_act_dot = (speed_act_1-speed_act_2)*2*pi*wheel_r/(1600*D);
    torque_sum = -(K1_1*(X_act-X_in)-K1_2*(X_act_dot-X_dot_in)-K1_3*(kalAngleY*deg2rad)-K1_4*gyroYrate*deg2rad);
    torque_min = -(K2_1*(yaw_act-yaw_in)-K2_2*(yaw_act_dot-yaw_dot_in));
    torque_L = (torque_sum+torque_min)/2;
    torque_R = torque_sum-torque_L;
    w_dot_req_L = torque_L/J;
    w_dot_req_R = torque_R/J;
    MotorVoltageUpdate;
    analogWrite(PWM1, PWM_val_1);
    analogWrite(PWM2, PWM_val_2);
    //printdata();
  }

 

}



void getMotorData()  {                                                    
static long countAnt_1 = 0;
static long countAnt_2 = 0;
static float speed_act_1_old = 0;
static float speed_act_2_old = 0;
static float speed_act_1_oold = 0;
static float speed_act_2_oold = 0;
  speed_act_1 = (count_1 - countAnt_1)*2*pi/1600;//   radia/s
  countAnt_1 = count_1;
  speed_act_2 = (count_2 - countAnt_2)*2*pi/1600;
  countAnt_2 = count_2;
  w_dot_act_L = (3*speed_act_1-4*speed_act_1_old+speed_act_1_oold)/LOOPTIME;
  w_dot_act_R = (3*speed_act_2-4*speed_act_2_old+speed_act_2_oold)/LOOPTIME;
  speed_act_1_old = speed_act_1;
  speed_act_1_oold = speed_act_1_old;
  speed_act_2_old = speed_act_2;
  speed_act_2_oold = speed_act_2_old;  
}

void MotorVoltageUpdate ()  {
  static float int_error_w_dot_L = 0;
  static float error_w_dot_L_old = 0;
  static float int_error_w_dot_R = 0;
  static float error_w_dot_R_old = 0;
  float error_w_dot_L;
  float error_w_dot_R;
  
  error_w_dot_L = w_dot_req_L-w_dot_act_L;
  error_w_dot_R = w_dot_req_R-w_dot_act_R;
  int_error_w_dot_L = int_error_w_dot_L+error_w_dot_L*LOOPTIME;
  int_error_w_dot_R = int_error_w_dot_R+error_w_dot_R*LOOPTIME;
  vol_in_L = error_w_dot_L*Kp_m+int_error_w_dot_L*Ki_m;
  vol_in_R = error_w_dot_R*Kp_m+int_error_w_dot_R*Ki_m;

    if(vol_in_L>0) {
      digitalWrite(InA1, LOW);
      digitalWrite(InB1, HIGH);
      }
      if(vol_in_L<0) {
        digitalWrite(InA1,HIGH);
        digitalWrite(InB1, LOW);
      }
      if(vol_in_R>0) {
      digitalWrite(InA2, LOW);
      digitalWrite(InB2, HIGH);
      }
      if(vol_in_R<0) {
        digitalWrite(InA2,HIGH);
        digitalWrite(InB2, LOW);
      } 
  
  PWM_val_1 = volt2pwm(abs(vol_in_L));
  PWM_val_2 = volt2pwm(abs(vol_in_R));
  
}

int volt2pwm(float vol){
  int pwm;
  pwm = 0.6272*vol*vol*vol-6.475*vol*vol+27.72*vol+11.53;
  return constrain(pwm,0,255);
}

//int updatePid(int command, int targetValue, int currentValue)   {             // compute PWM value
//float pidTerm = 0;                                                            // PID correction
//int error=0;                                  
//static int last_error=0;                             
// error = abs(targetValue) - abs(currentValue); 
// pidTerm = (Kp * error) + (Kd * (error - last_error));                            
// last_error = error;
// return constrain(command + int(pidTerm), 0, 255);
//}

void rencoder()  {                                   
n_1 = digitalRead(encodPinA1);
if ((encoder0PinALast_1 == LOW) && (n_1 == HIGH)){
  if (digitalRead(encodPinB1) == LOW) {
  count_1--;
  } else{
    count_1++;
    }
}
encoder0PinALast_1 = n_1;

n_2 = digitalRead(encodPinA2);
if ((encoder0PinALast_2 == LOW) && (n_2 == HIGH)){
  if (digitalRead(encodPinB2) == LOW) {
  count_2--;
  } else{
    count_2++;
    }
}
encoder0PinALast_2 = n_2;
}

void printdata() {

  Serial.print("rpm_1 = ");Serial.println(speed_act_1);  Serial.print("\t");
  Serial.print("rpm_2 = ");Serial.println(speed_act_2);  Serial.print("\t");
  Serial.print("kalAngleX = ");Serial.print(kalAngleX); Serial.print("\t");
  Serial.print("kalAngleY = ");Serial.print(kalAngleY); Serial.println("\t");
  
}


