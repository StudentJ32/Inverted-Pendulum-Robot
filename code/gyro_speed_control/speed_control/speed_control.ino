#define InA1            10                      // INA motor pin
#define InB1            11                      // INB motor pin 
#define PWM1            9                       // PWM motor pin
#define encodPinA1      5                       // encoder A pin
#define encodPinB1      6                       // encoder B pin
//#define Vpin            0                       // battery monitoring analog pin
//#define Apin            1                       // motor current monitoring analog pin

#define CURRENT_LIMIT   1000                     // high current warning
#define LOW_BAT         10000                   // low bat warning
#define LOOPTIME        100                     // PID loop time
#define NUMREADINGS     10                      // samples for Amp average



int readings[NUMREADINGS];
unsigned long lastMilli = 0;                    // loop timing 
unsigned long lastMilliPrint = 0;               // loop timing
int speed_req = 24;                            // speed (Set Point)
float speed_act = 0;                              // speed (actual value)
int PWM_val = 0;                                // (25% = 64; 50% = 127; 75% = 191; 100% = 255)
int voltage = 0;                                // in mV
int current = 0;                                // in mA
volatile long count = 0;                        // rev counter
float Kp =   2;                                // PID proportional control Gain
float Kd =    1;  // PID Derivitave control gain


 int encoder0PinALast = LOW;
 int n = LOW;

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
  digitalWrite(InA1, LOW);
  digitalWrite(InB1, HIGH);
}

void loop() {
      digitalWrite(InA1, LOW);
      digitalWrite(InB1, HIGH);
      rencoder();
  if((millis()-lastMilli) >= LOOPTIME)   {                                    // enter tmed loop
    lastMilli = millis();
    getMotorData(); 
    PWM_val= updatePid(PWM_val, speed_req, speed_act);
    analogWrite(PWM1, PWM_val);
  }
     printMotorInfo();   
}


void getMotorData()  {                                                        // calculate speed, volts and Amps                                                   // last count
static long countAnt = 0;
  speed_act = ((count - countAnt)*(60*(1000/LOOPTIME)))/(16*30);          // 16 pulses X 29 gear ratio = 464 counts per output shaft rev
  countAnt = count; 
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


void rencoder()  {                                    // pulse and direction, direct port reading to save cycles
n = digitalRead(encodPinA1);
if ((encoder0PinALast == LOW) && (n == HIGH)){
  if (digitalRead(encodPinB1) == LOW) {
  count--;
  } else{
    count++;
    }
}
encoder0PinALast = n;
}


void printMotorInfo()  {                                                      // display data
 if((millis()-lastMilliPrint) >= 500)   {                     
   lastMilliPrint = millis();
   Serial.print("SP:");             Serial.print(speed_req);  
   Serial.print("  RPM:");          Serial.print(speed_act);
   Serial.print("  PWM:");          Serial.println(PWM_val); 
 }}


