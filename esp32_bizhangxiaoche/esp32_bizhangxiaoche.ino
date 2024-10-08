// 引入所需库
#include <Arduino.h>

// 定义超声波传感器引脚
const int trigPin = 5;  // Trig引脚连接到GPIO5
const int echoPin = 18; // Echo引脚连接到GPIO18

// 定义电机控制引脚
const int motor1Pin1 = 16; // 电机1方向控制引脚1
const int motor1Pin2 = 17; // 电机1方向控制引脚2
const int motor2Pin1 = 19; // 电机2方向控制引脚1
const int motor2Pin2 = 21; // 电机2方向控制引脚2

// 定义电机速度控制引脚（PWM）
const int enableA = 4; // 电机1使能引脚（PWM）
const int enableB = 2; // 电机2使能引脚（PWM）

// 定义避障距离阈值（单位：厘米）
const int obstacleDistance = 20; // 当前方障碍物距离小于20厘米时避障

void setup() {
  // 初始化串口通信，便于调试
  Serial.begin(115200);
  
  // 设置超声波传感器引脚模式
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  // 设置电机控制引脚模式
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  
  // 设置电机使能引脚为输出
  pinMode(enableA, OUTPUT);
  pinMode(enableB, OUTPUT);
  
  // 初始化电机为停止状态
  stopMotors();
}

void loop() {
  // 获取前方距离
  long distance = getDistance();
  
  Serial.print("距离：");
  Serial.print(distance);
  Serial.println(" cm");
  
  if (distance > obstacleDistance) {
    // 如果前方没有障碍物，前进
    moveForward();
  } else {
    // 如果检测到障碍物，执行避障动作
    stopMotors();
    delay(500); // 停止一段时间
    moveBackward();
    delay(500); // 后退一段时间
    turnRight();
    delay(500); // 向右转一段时间
  }
  
  delay(100); // 主循环延时
}

// 获取超声波距离函数
long getDistance() {
  // 发送10us的高电平脉冲到Trig引脚
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // 读取Echo引脚的高电平持续时间
  long duration = pulseIn(echoPin, HIGH, 30000); // 超时设为30ms
  
  // 计算距离，声速340m/s，往返时间
  long distanceCm = duration * 0.034 / 2;
  
  return distanceCm;
}

// 前进函数
void moveForward() {
  // 设置电机1向前
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  
  // 设置电机2向前
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);
  
  // 设置电机速度（0-255）
  analogWrite(enableA, 200); // 电机1速度
  analogWrite(enableB, 200); // 电机2速度
}

// 后退函数
void moveBackward() {
  // 设置电机1向后
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);
  
  // 设置电机2向后
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH);
  
  // 设置电机速度
  analogWrite(enableA, 200);
  analogWrite(enableB, 200);
}

// 向左转函数
void turnLeft() {
  // 设置电机1向后
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);
  
  // 设置电机2向前
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);
  
  // 设置电机速度
  analogWrite(enableA, 150);
  analogWrite(enableB, 150);
}

// 向右转函数
void turnRight() {
  // 设置电机1向前
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  
  // 设置电机2向后
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH);
  
  // 设置电机速度
  analogWrite(enableA, 150);
  analogWrite(enableB, 150);
}

// 停止电机函数
void stopMotors() {
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW);
  
  analogWrite(enableA, 0);
  analogWrite(enableB, 0);
}