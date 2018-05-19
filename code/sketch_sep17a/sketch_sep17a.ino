  // MPU-6050 Short Example Sketch
// By Arduino User JohnChi
// August 17, 2014
// Public Domain
#include<Wire.h>
const int MPU=0x68;  // I2C address of the MPU-6050
int16_t GyX,GyY,GyZ,X_raw;
int off_x = 632; int off_y = -399; int off_z = 14745;
int Filter_Value;
void setup(){
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(9600);
}
void loop(){
    Filter_Value = Filter();       // 获得滤波器输出值
  Serial.print(Filter_Value);Serial.print("\t"); // 串口输出
  delay(5);
}

int Get_AD() {

  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,14,true);  // request a total of 14 registers
  X_raw=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
   delay(1);
  return X_raw;
}
 
// 中位值平均滤波法（又称防脉冲干扰平均滤波法）（算法1）

#define FILTER_N 10
int Filter() {
  int i;
  int filter_sum = 0;
  for(i = 0; i < FILTER_N; i++) {
    filter_sum += Get_AD();
  }
  return (int)(filter_sum / FILTER_N);
}

