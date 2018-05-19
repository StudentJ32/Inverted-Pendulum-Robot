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
Encoder knobRight(18,19);

//*********Motor Right defines*****************
//#define Vpin            0                       // battery monitoring analog pin
//#define Apin            1                       // motor current monitoring analog pin

#define CURRENT_LIMIT   1000                     // high current warning
#define LOW_BAT         10000                   // low bat warning
#define LOOPTIME        10                     // PID loop time
#define NUMREADINGS     10                      // samples for Amp average

//************************filter gain design*****************
float speed_m_1;
float speed_m_2;

 float speed_act_1_old = 0;
 float speed_act_2_old = 0;
 float speed_act_1_oold = 0;
 float speed_act_2_oold = 0;


int readings[NUMREADINGS];
unsigned long lastMilli = 0;                    // loop timing 
unsigned long lastMilliPrint = 0;// loop timing
int long unsigned lastMilliPulse = 0;
int PWM_val = 0;                                // (25% = 64; 50% = 127; 75% = 191; 100% = 255)
int voltage = 0;                                // in mV
int current = 0;                                // in mA
float Kp =   .4;                                // PID proportional control Gain
float Kd =    1;  // PID Derivitave control gain
float pi = 3.1415926;
int time_pulse = 4000;


float speed_act_1 = 0;                              // speed (actual value)
float speed_act_2 = 0;                              // speed (actual value)              
long count_1 = 0;                  // rev counter of Left Motor
long count_2 = 0;                        // rev counter of Right Motor
long countAnt_1  = 0;
 long countAnt_2 = 0;

float w_dot_act_L;
float w_dot_act_R;


void setup() {
    AFMS.begin();
  analogReference(EXTERNAL);                            // Current external ref is 3.3V
  Serial.begin(9600);
  for(int i=0; i<NUMREADINGS; i++)   readings[i] = 0;  // initialize readings to 0
}

void loop() {
      M_L->run(FORWARD);
      M_R->run(FORWARD);;
      rencoder();
//            if (count_1 < 64*29/2){
//      M_L->setSpeed(100);
//      }
//      if (count_1 > 16*29/4){
//      M_L->setSpeed(0);
//      }
      if (millis()%8000<=time_pulse && millis()%4000>=0){
       M_L->setSpeed(0);
       M_R->setSpeed(0);
      }
      if(millis()%8000>time_pulse){
        lastMilliPulse = millis();
       M_L->setSpeed(255);
       M_R->setSpeed(255);  
      }

 rencoder();
  if((millis()-lastMilli) >= LOOPTIME)   {                                    // enter tmed loop
    lastMilli = millis();
     
    getMotorData();
  }
     printMotorInfo();   
}



void rencoder()  {                             
  count_1 = knobLeft.read();
  count_2 = knobRight.read();

  
}

void getMotorData(){

  float w_dot_m_L;
  float w_dot_m_R;
  static float timer=0;
  
  double dt = (double)(millis()-timer)/1000;
  
  speed_m_1 = (float) (count_1 - countAnt_1)*2*pi/(6400*dt);
  speed_act_1 = speed_m_1;//SpeedFilter_1(speed_m_1);
  speed_m_2 = (float) (count_2 - countAnt_2)*2*pi/(6400*dt);
  speed_act_2 = speed_m_2;//SpeedFilter_2(speed_m_2);
  w_dot_m_L = (float) -(-3*speed_act_1+4*speed_act_1_old-speed_act_1_oold)/(2*dt);
  w_dot_m_R = (float) -(-3*speed_act_2+4*speed_act_2_old-speed_act_2_oold)/(2*dt);
  speed_act_1_old = speed_act_1;
  speed_act_1_oold = speed_act_1_old;
  speed_act_2_old = speed_act_2;
  speed_act_2_oold = speed_act_2_old;
  w_dot_act_L = w_dot_m_L;//WdotFilter_1(w_dot_m_L);
  w_dot_act_R = w_dot_m_R;//WdotFilter_1(w_dot_m_R);
  countAnt_1 = count_1;
  countAnt_2 = count_2;
  timer = millis();
}


float SpeedFilter_1 (float speed_m){
 float km = 0.09516;
 float ke = 0.9048;
 static float speed_e1 = 0;

 speed_e1 = speed_e1*ke+speed_m*km;
 return speed_e1;
}

float SpeedFilter_2 (float speed_m){
 float km = 0.09516;
 float ke = 0.9048;
 static float speed_e2 = 0;

 speed_e2 = speed_e2*ke+speed_m*km;
 return speed_e2;
}

float WdotFilter_1 (float w_dot) {
 float km = 0.09516;
 float ke = 0.9048;
 static float w_dot_e1 = 0;

 w_dot_e1 = w_dot_e1*ke+w_dot*km;
 return w_dot_e1;
}

float WdotFilter_2 (float w_dot) {
 float km = 0.09516;
 float ke = 0.9048;
 static float w_dot_e2 = 0;

 w_dot_e2 = w_dot_e2*ke+w_dot*km;
 return w_dot_e2;
}

int updatePid(int command, int targetValue, int currentValue)   {             // compute PWM value
float pidTerm = 0;                                                            // PID correction
int error=0;                                  
static int last_error=0;                             
 error = abs(targetValue) - abs(currentValue); 
 pidTerm = (Kp * error) + (Kd * (error - last_error));                            
 last_error = error;
 return constrain(command + int(pidTerm), 0, 255);
}




void printMotorInfo()  {                                                      // display data
 if((millis()-lastMilliPrint) >=10 )   {                     
   lastMilliPrint = millis();
    Serial.print(speed_act_1);Serial.print(" ") ;
//    Serial.print(speed_act_2);Serial.print(" ") ;
//    Serial.print(w_dot_act_L);Serial.print(" ") ;
//    Serial.print(w_dot_act_R);Serial.print(" ") ;
    Serial.print(millis()); Serial.println(" "); 
   //Serial.print(w_dot_act_L);Serial.println(" ");
   //Serial.print("SP:");             Serial.print(speed_req);  
   //Serial.print("  RPM:");          Serial.print(speed_act);
   //Serial.print("  PWM:");          Serial.println(PWM_val);
 }}

