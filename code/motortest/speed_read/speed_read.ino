#define InA1            8                      // INA motor pin
#define InB1            9                      // INB motor pin 
#define PWM1            10                       // PWM motor pin
#define encodPinA1      12                       // encoder A pin
#define encodPinB1      11                       // encoder B pin

//*********Motor Right defines*****************
#define InA2            7                      // INA motor pin
#define InB2            6                      // INB motor pin 
#define PWM2            5                       // PWM motor pin
#define encodPinA2      3                       // encoder A pin
#define encodPinB2      4                       // encoder B pin
//#define Vpin            0                       // battery monitoring analog pin
//#define Apin            1                       // motor current monitoring analog pin

#define CURRENT_LIMIT   1000                     // high current warning
#define LOW_BAT         10000                   // low bat warning
#define LOOPTIME        50                     // PID loop time
#define NUMREADINGS     10                      // samples for Amp average



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
float ang_pos;
float ang_vel;
float ang_pos_old=0;
float ang_vel_old=0;

 int encoder0PinALast_1 = LOW;
 int n_1 = LOW;
 int encoder0PinALast_2 = LOW;
 int n_2 = LOW;

float speed_act_1 = 0;                              // speed (actual value)
float speed_act_2 = 0;                              // speed (actual value)
int PWM_val_1 = 0;                                // (25% = 64; 50% = 127; 75% = 191; 100% = 255)
int PWM_val_2 = 0;                         
volatile long count_1 = 0;                        // rev counter of Left Motor
volatile long count_2 = 0;                        // rev counter of Right Motor
 

void setup() {
  analogReference(EXTERNAL);                            // Current external ref is 3.3V
  Serial.begin(9600);
  pinMode(InA1, OUTPUT);
  pinMode(InB1, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(encodPinA1, INPUT); 
  pinMode(encodPinB1, INPUT); 
  digitalWrite(encodPinA1, HIGH);                      // turn on pullup resistor
  digitalWrite(encodPinB1, HIGH);
  attachInterrupt(1, rencoder, FALLING);
  for(int i=0; i<NUMREADINGS; i++)   readings[i] = 0;  // initialize readings to 0

  analogWrite(PWM1, PWM_val);
  digitalWrite(InA1, HIGH);
  digitalWrite(InB1, LOW);
}

void loop() {
      digitalWrite(InA1, LOW);
      digitalWrite(InB1, HIGH);
      digitalWrite(InA2, LOW);
      digitalWrite(InB2, HIGH);
      analogWrite(PWM1, 200);
       analogWrite(PWM2, 200);
      rencoder();
//      if (millis()%8000<=time_pulse && millis()%4000>=0){
//        analogWrite(PWM1, 0);
//        analogWrite(PWM2, 0);
//      }
//      if(millis()%8000>time_pulse){
//        lastMilliPulse = millis();
//        analogWrite(PWM1, 200);
//        analogWrite(PWM2, 200);
//      }

      
  if((millis()-lastMilli) >= LOOPTIME)   {                                    // enter tmed loop
    lastMilli = millis();
    getMotorData(); 
  }
     printMotorInfo();   
}



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


void getMotorData()  {                                                    
static long countAnt_1 = 0;
static long countAnt_2 = 0;
static float speed_act_1_old = 0;
static float speed_act_2_old = 0;
static float speed_act_1_oold = 0;
static float speed_act_2_oold = 0;
  speed_act_1 = (count_1 - countAnt_1)*2*pi/(16*30);//   radia/s
  countAnt_1 = count_1;
  speed_act_2 = (count_2 - countAnt_2)*2*pi/(16*30);
  countAnt_2 = count_2;
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
 if((millis()-lastMilliPrint) >=60 )   {                     
   lastMilliPrint = millis();
   Serial.print(speed_act_1);Serial.print(" ") ;
   Serial.print(speed_act_2);Serial.print(" "); 
   Serial.print(millis());Serial.println(" ");
   //Serial.print("SP:");             Serial.print(speed_req);  
   //Serial.print("  RPM:");          Serial.print(speed_act);
   //Serial.print("  PWM:");          Serial.println(PWM_val); 
 }}

