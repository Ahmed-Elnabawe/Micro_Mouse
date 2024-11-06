#include "cells.h"
#include "fifo.h"
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h> 
/*------------------- IR pins -------------------*/
const int frontIRPin = 4;  // Front IR sensor
const int rightIRPin = 15;  // Right IR sensor
const int leftIRPin = 13;   // Left IR sensor
/*------------------- motors pins -------------------*/
const int left1 = 26;
const int left2 = 14;
const int right1 = 19;
const int right2 = 32;
/*------------------- encoder pin -------------------*/
const int EncoderrightIRPin = 16; // Right encoder على GPIO 16
/*------------------- gyro variables -------------------*/
Adafruit_MPU6050 mpu;
float gyro_ratio = 90*(90/105.0588235294118)*(90/88.2);  // gyro ratio 
int LastTime = 0;
float x = 0, y = 0, z = 0;
float XError = 0, YError = 0, ZError = 0;
/*------------------- encoder variables -------------------*/
volatile int leftPulseCount = 0;
volatile int rightPulseCount = 0;
const float distancePerPulse = 0.6342857142857143;
const float targetDistance = 19.5; 
int targetPulses; 
/*------------------- PID variables -------------------*/
int maxSpeed=200;
float mkp =0.046, mki=0.002081 ,mkd=5.2;
//float mkp =0.046, mki=0.000881 ,mkd=4.7;
//float mkp =0.044, mki=0.00055 ,mkd=3.71; // PID values for moving 
int turnSpeed=80;
int leftSpeed = maxSpeed, rightSpeed = maxSpeed;
float target = 0;
float last_target = 0;
float value;
float error;
float integral = 0;
float propotional = 0;
float derivative = 0;
float lastError = 0;
/*------------------- algorithm and maze variables -------------------*/
FIFO_buffer_t queue ;
robot_position_t head_pos = head_to_right ;
robot_position_t Last_head_pos = last_head_start ;
element_type queue_buffer[fifo_size];
cell_t g_maze[MAZE_SIZE][MAZE_SIZE];
cell_t *stp = &g_maze[0][0]; // start pointer 
/*------------------- encoder Interuupt-------------------*/
void IRAM_ATTR countRightPulses() {
  rightPulseCount++;
}
/*------------------- IR reading function -------------------*/
bool readFrontIR() {
  return !digitalRead(frontIRPin); // Read front IR sensor value
}

bool readRightIR() {
  return !digitalRead(rightIRPin); // Read right IR sensor value
}

bool readLeftIR() {
  return !digitalRead(leftIRPin); // Read left IR sensor value
}
/*------------------- setup function -------------------*/
void setup() {
  // init mpu
  mpu.begin();
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  // set the motor pins to output 
  pinMode(left1, OUTPUT);
  pinMode(left2, OUTPUT);
  pinMode(right1, OUTPUT);
  pinMode(right2, OUTPUT);
  // set the IR pins to input 
  pinMode(frontIRPin, INPUT);
  pinMode(rightIRPin, INPUT);
  pinMode(leftIRPin, INPUT);
  // define interrupt for encoder 
  attachInterrupt(digitalPinToInterrupt(EncoderrightIRPin), countRightPulses, RISING);
  // calculate targetPulses
  targetPulses = targetDistance / distancePerPulse;
  // Calibration mpu
  delay(1000);
  for (int i = 0; i < 200; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    XError += g.gyro.x;
    YError += g.gyro.y;
    ZError += g.gyro.z;
  }
  XError /= 200.0;
  YError /= 200.0;
  ZError /= 200.0;
  delay(500);
  LastTime = millis();
  // init queue 
  FIFO_init(&queue, queue_buffer, fifo_size);
  // init maze cells 
  init_all_cells(g_maze, (uint16_t)MAZE_SIZE);
  delay(1000);
}

void loop() {
      goto_smallest_val(g_maze, stp);
}
/*----------------------- function to read the angle form gyro -----------------------*/
void readGyro()
{
 /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  int Time = millis();
  int dt = Time - LastTime;
  float Sum;
  Sum = (g.gyro.x - XError);
  if(fabs(Sum) > 0.04) x += Sum * dt/1000.0 * 180/M_PI;
  LastTime = Time;
  Serial.println("");
  delay(30);
}
/*----------------------- function to move forward with PID  -----------------------*/
void moveForward() {
  delay(500);
  readGyro();
  leftSpeed = maxSpeed; rightSpeed = maxSpeed;
  leftPulseCount = 0;
  rightPulseCount=0;
  if(Last_head_pos==head_pos) // check if the head position changed
  {
    target=last_target;
  }
  else
  {
    target=x;
  }
  lastError=0;
  propotional=0;
  integral=0;
  derivative=0;
  while (rightPulseCount*distancePerPulse < targetDistance) {
    readGyro();
    // Proportional calculation
    error = target - x; 
    propotional = error * mkp;
    // Integral calculation
    integral += error; 
    integral *= mki;
    // Derivative calculation
    derivative = error - lastError;
    derivative *= mkd;
    // Output calculation
    float output = propotional + integral + derivative;
    output = constrain(output, -200, 200);
    leftSpeed -= output;
    rightSpeed += output;
    rightSpeed = constrain(rightSpeed, 0, 200);
    leftSpeed = constrain(leftSpeed, 0, 200);    
    analogWrite(left1, leftSpeed);
    analogWrite(left2, 0);
    analogWrite(right1, rightSpeed);
    analogWrite(right2, 0);
    lastError = error;
  }
  last_target=target;
  Last_head_pos=head_pos;
  stopMotors();
}
/*----------------------- function to turn right with  gyro -----------------------*/
void turnRight()
{
  delay(500);    
  readGyro();
 float  d_angle =x-gyro_ratio;
  Serial.print("d_angle .");Serial.println(d_angle);
  while(x>d_angle)
  {
  analogWrite(left1, turnSpeed);
  analogWrite(left2, 0);
  analogWrite(right1, 0);
  analogWrite(right2, turnSpeed);
  Serial.print("d_angle: "); Serial.println(d_angle);
  Serial.print("x: "); Serial.println(x);
  readGyro();
  }
  stopMotors();
}
/*----------------------- function to turn left with  gyro -----------------------*/
void turnLeft()
{
  delay(500);
  readGyro();
 float  d_angle =x+gyro_ratio;
  while(x<d_angle)
  {
  analogWrite(left1, 0);
  analogWrite(left2, turnSpeed);
  analogWrite(right1, turnSpeed);
  analogWrite(right2, 0);
  Serial.print("d_angle: "); Serial.println(d_angle);
  Serial.print("x: "); Serial.println(x);
  readGyro();
  }
  stopMotors();
}
/*----------------------- function to stop motors-----------------------*/
void stopMotors() {
  analogWrite(left1, 0);
  analogWrite(left2, 0);
  analogWrite(right1, 0);
  analogWrite(right2, 0);
  delay(500);  
}







