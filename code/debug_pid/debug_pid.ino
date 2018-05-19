#include <Wire.h>
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter
#include <Encoder.h>

#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"
//**********define motors****************//
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Adafruit_DCMotor *M_L = AFMS.getMotor(1);
Adafruit_DCMotor *M_R = AFMS.getMotor(2);

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

//**************  1 ------ Left     2----------Right   ******************



#define CURRENT_LIMIT   1000                     // high current warning
#define LOW_BAT         10000                   // low bat warning
#define LOOPTIME        10                     // PID loop time  ******do not change
#define NUMREADINGS     10                      // samples for Amp average

 

unsigned long lastMilli = 0;                    // loop timing 
unsigned long lastMilliPrint = 0;               // loop timing
float speed_act_1 = 0;                              // speed (actual value)
float speed_act_2 = 0;                              // speed (actual value)
int PWM_val_1 = 0;                                // (25% = 64; 50% = 127; 75% = 191; 100% = 255)
int PWM_val_2 = 0;



//********************gain design ************************************
float Kp = 25;
float Ki = 3;
float Kd = 0;
//*********motor input***********;
float vol_in_L;
float vol_in_R;


/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
//int16_t tempRaw;


double  kalAngleY; // Calculated angle using a Kalman filter
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

  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
  kalmanY.setAngle(pitch);

  timer = micros();

  analogReference(EXTERNAL);                            // Current external ref is 3.3V

  
}

void loop() {
  
    
    //rencoder();
    
    if((millis()-lastMilli) >= LOOPTIME){
     readgyro();
    double error;
    static double error_old;
    lastMilli=millis();
    //getMotorData();
    error = kalAngleY+1.5;
    PWM_val_1 = error*Kp + Ki*(error+error_old)*LOOPTIME*2/1000;
    PWM_val_1=constrain(PWM_val_1,-255,255);
    error_old = error;
    PWM_val_2 = PWM_val_1;
      if(PWM_val_1<0) {
      M_L->run(FORWARD);
      }
      if(PWM_val_1>0) {
      M_L->run(BACKWARD);
      }
      if(PWM_val_1<0) {
      M_R->run(FORWARD);
      }
      if(PWM_val_1>0) {
      M_R->run(BACKWARD);
      } 
 
  M_L->setSpeed(abs(PWM_val_1));
  M_R->setSpeed(abs(PWM_val_2));
    printdata();
    }
}

void printdata(){
  //Serial.print("KalAngleY = ");
  Serial.println(kalAngleY); Serial.print(" ");
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

  gyroYangle += gyroYrate * dt;

  // Reset the gyro angle when it has drifted too much
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;
}



int volt2pwm(float vol){
  int pwm;
  pwm = 0.6272*vol*vol*vol-6.475*vol*vol+27.72*vol+11.53;
  return constrain(pwm,0,255);
}
