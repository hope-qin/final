/*******************************************************************
 * 遥控车最终实现 / RC Car Final Implementation
 * 功能包括：指南针导航、双模式控制、距离追踪、错误处理
 * Features: Compass navigation, dual-mode control, distance tracking, error handling
 *******************************************************************/

#include <LiquidCrystal.h>  // LCD显示库 / LCD display library
#include <Wire.h>           // I2C通信库 / I2C communication library
#include <EEPROM.h>         // EEPROM存储库 / EEPROM storage library

// 初始化LCD屏幕 / Initialize LCD screen
LiquidCrystal lcd(38, 36, 35, 34, 33, 30);

// CMPS14指南针设置 / CMPS14 compass settings
#define CMPS14_ADDRESS 0x60    // 指南针I2C地址 / Compass I2C address
#define BEARING_Register 0x01  // 方位角寄存器 / Bearing angle register

// 电机控制引脚定义 / Motor control pin definitions
#define FORWARD_DIRECTION 1     // 前进方向 / Forward direction
#define REVERSE_DIRECTION 0     // 后退方向 / Reverse direction
#define LEFT_MOTOR_DIR_PIN 7    // 左电机方向引脚 / Left motor direction pin
#define RIGHT_MOTOR_DIR_PIN 8   // 右电机方向引脚 / Right motor direction pin
#define LEFT_MOTOR_PWM_PIN 9    // 左电机PWM引脚 / Left motor PWM pin
#define RIGHT_MOTOR_PWM_PIN 10  // 右电机PWM引脚 / Right motor PWM pin

// 摇杆引脚 / Joystick pins
#define JOYSTICK_X A0   // 摇杆X轴 / Joystick X-axis
#define JOYSTICK_Y A1   // 摇杆Y轴 / Joystick Y-axis
#define JOYSTICK_BTN 4  // 摇杆按钮 / Joystick button

// 编码器引脚设置 / Encoder pin setup
const int LEFT_ENCODER_PIN = 2;   // 左编码器引脚 / Left encoder pin
const int RIGHT_ENCODER_PIN = 3;  // 右编码器引脚 / Right encoder pin

// 电机控制和导航常量 / Constants for motor control and navigation
const int MIN_TURN_SPEED = 120;                // 最小转向速度 / Minimum turning speed
const int MAX_TURN_SPEED = 180;                // 最大转向速度 / Maximum turning speed
const int HEADING_TOLERANCE = 1;               // 方向容差 / Heading tolerance
const float PULSES_PER_CM = 13.31;             // 每厘米脉冲数 / Pulses per centimeter
const unsigned long HEARTBEAT_TIMEOUT = 5000;  // 心跳超时时间(毫秒) / Heartbeat timeout (ms)

// 控制变量 / Control variables
bool isESPMode = false;           // ESP模式标志 / ESP mode flag
int compassOffset = 0;            // 指南针偏移量 / Compass offset
unsigned long lastHeartbeat = 0;  // 最后心跳时间 / Last heartbeat time
int targetHeading = 0;            // 目标方向 / Target heading
int targetDistance = 0;           // 目标距离 / Target distance
bool isMoving = false;            // 运动标志 / Movement flag
String lastCommand = "";          // 最后命令 / Last command

// 编码器变量 / Encoder variables
volatile long leftPulses = 0;   // 左编码器脉冲计数 / Left encoder pulse count
volatile long rightPulses = 0;  // 右编码器脉冲计数 / Right encoder pulse count
float leftDistance = 0;         // 左轮行驶距离 / Left wheel travel distance
float rightDistance = 0;        // 右轮行驶距离 / Right wheel travel distance

// 函数原型 / Function prototypes
void handleSerialCommand();                     // 处理串口命令 / Handle serial commands
void processJoystick();                         // 处理摇杆输入 / Process joystick input
void updateLCD();                               // 更新LCD显示 / Update LCD display
int getCompassHeading();                        // 获取指南针方向 / Get compass heading
void drive(int distance, int speed);            // 控制行驶 / Control driving
void turn(int targetDegree, int speed);         // 控制转向 / Control turning
void setMotors(int leftSpeed, int rightSpeed);  // 设置电机速度 / Set motor speeds
void calibrateCompass();                        // 校准指南针 / Calibrate compass
void checkHeartbeat();                          // 检查心跳 / Check heartbeat
String getCardinalDirection(int heading);       // 获取方向描述 / Get cardinal direction

// 编码器中断服务程序 / Encoder Interrupt Service Routines
void leftEncoderISR() {
  leftPulses++;  // 增加左编码器计数 / Increment left encoder count
}

void rightEncoderISR() {
  rightPulses++;  // 增加右编码器计数 / Increment right encoder count
}

// 初始化设置 / Setup initialization
void setup() {
  // 初始化串口通信 / Initialize serial communication
  Serial.begin(9600);

  // 初始化LCD / Initialize LCD
  lcd.begin(20, 4);
  lcd.print("Initializing...");

  // 初始化I2C通信 / Initialize I2C communication
  Wire.begin();

  // 设置电机引脚 / Setup motor pins
  pinMode(LEFT_MOTOR_DIR_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_DIR_PIN, OUTPUT);
  pinMode(LEFT_MOTOR_PWM_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM_PIN, OUTPUT);

  // 设置编码器引脚和中断 / Setup encoder pins and interrupts
  pinMode(LEFT_ENCODER_PIN, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN), rightEncoderISR, RISING);

  // 设置摇杆引脚 / Setup joystick pin
  pinMode(JOYSTICK_BTN, INPUT_PULLUP);

  // 校准指南针 / Calibrate compass
  calibrateCompass();

  // 第一步：设置引脚模式
  pinMode(LEFT_MOTOR_PWM_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM_PIN, OUTPUT);
  pinMode(LEFT_MOTOR_DIR_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_DIR_PIN, OUTPUT);

  // 第二步：立即停止电机
  analogWrite(LEFT_MOTOR_PWM_PIN, 0);
  analogWrite(RIGHT_MOTOR_PWM_PIN, 0);
  
  // 第三步：设置初始方向
  digitalWrite(LEFT_MOTOR_DIR_PIN, FORWARD_DIRECTION);
  digitalWrite(RIGHT_MOTOR_DIR_PIN, FORWARD_DIRECTION);
  
  delay(100);  // 给系统一些稳定时间

  lcd.clear();
}

// 主循环 / Main loop
void loop() {
// 检查模式切换
  if (digitalRead(JOYSTICK_BTN) == LOW) {
    delay(50);  // 消抖
    if (digitalRead(JOYSTICK_BTN) == LOW) {
      // 在切换模式前停止电机
      setMotors(0, 0);
      
      isESPMode = !isESPMode;
      
      // 等待按钮释放，并确保摇杆回中
      while (digitalRead(JOYSTICK_BTN) == LOW) {
        setMotors(0, 0);  // 确保在等待期间电机保持停止
      }
      
      // 增加一个短暂延时，让摇杆值稳定
      delay(100);
    }
  }

  // 基于模式处理控制 / Handle control based on mode
  if (isESPMode) {
    checkHeartbeat();
    if (Serial.available()) {
      handleSerialCommand();
    }
  } else {
    processJoystick();
  }

  // 更新距离 / Update distances
  leftDistance = leftPulses / PULSES_PER_CM;
  rightDistance = rightPulses / PULSES_PER_CM;

  // 更新LCD显示 / Update LCD display
  updateLCD();

  delay(50);  // 小延迟防止过载 / Small delay to prevent overwhelming
}

// 校准指南针 / Calibrate compass
void calibrateCompass() {
  int initialReading = getCompassHeading();
  compassOffset = (360 - initialReading) % 360;
  lcd.clear();
  lcd.print("Compass calibrated");
  lcd.setCursor(0, 1);
  lcd.print("Offset: ");
  lcd.print(compassOffset);
  delay(2000);
}

// 获取指南针读数 / Get compass reading
int getCompassHeading() {
  Wire.beginTransmission(CMPS14_ADDRESS);
  Wire.write(BEARING_Register);
  Wire.endTransmission();

  Wire.requestFrom(CMPS14_ADDRESS, 1);
  if (Wire.available()) {
    int raw = Wire.read();
    int adjusted = (raw + compassOffset) % 360;
    return adjusted;
  }
  return -1;  // 错误状态 / Error condition
}

// 处理串口命令 / Handle serial commands
void handleSerialCommand() {
  String command = Serial.readStringUntil('\n');
  lastCommand = command;

  // 解析命令 / Parse command
  if (command.startsWith("Move:")) {
    int distance = command.substring(5).toInt();
    drive(distance, MAX_TURN_SPEED);
  } else if (command.startsWith("Turn:")) {
    if (command.substring(5) == "FindNorth") {
      turn(0, MIN_TURN_SPEED);  // 寻找北方 / Find North
    } else {
      int degree = command.substring(5).toInt();
      turn(degree, MIN_TURN_SPEED);
    }
  } else if (command == "heartbeat") {
    lastHeartbeat = millis();
    Serial.println("alive");  // 回应心跳信号 / Respond to heartbeat
  }
}

// 处理摇杆输入 / Process joystick input
void processJoystick() {
 
  const int DEAD_ZONE = 100;  // 增加死区范围
  
  // 读取摇杆值并中心化 / Read joystick values and center them
  int xValue = analogRead(JOYSTICK_X) - 512;
  int yValue = analogRead(JOYSTICK_Y) - 512; 

  // 更大的死区处理
  if (abs(xValue) < DEAD_ZONE) xValue = 0;
  if (abs(yValue) < DEAD_ZONE) yValue = 0;

  // 转换为电机速度 / Convert to motor speeds
  int leftSpeed = yValue + xValue;
  int rightSpeed = yValue - xValue;

  // 映射到电机范围 / Map to motor range
  leftSpeed = map(leftSpeed, -512, 512, -255, 255);
  rightSpeed = map(rightSpeed, -512, 512, -255, 255);

  setMotors(leftSpeed, rightSpeed);
}

// 设置电机速度 / Set motor speeds
void setMotors(int leftSpeed, int rightSpeed) {
  // 设置方向 / Set directions
  digitalWrite(LEFT_MOTOR_DIR_PIN, leftSpeed >= 0 ? FORWARD_DIRECTION : REVERSE_DIRECTION);
  digitalWrite(RIGHT_MOTOR_DIR_PIN, rightSpeed >= 0 ? FORWARD_DIRECTION : REVERSE_DIRECTION);

  // 设置速度 / Set speeds
  analogWrite(LEFT_MOTOR_PWM_PIN, abs(leftSpeed));
  analogWrite(RIGHT_MOTOR_PWM_PIN, abs(rightSpeed));
}

// 控制行驶距离 / Control driving distance
void drive(int distance, int speed) {
  // 重置编码器 / Reset encoders
  leftPulses = 0;
  rightPulses = 0;

  // 计算目标脉冲数 / Calculate target pulses
  long targetPulses = abs(distance) * PULSES_PER_CM;

  // 设置方向并开始移动 / Set direction and start moving
  int direction = distance >= 0 ? FORWARD_DIRECTION : REVERSE_DIRECTION;
  digitalWrite(LEFT_MOTOR_DIR_PIN, direction);
  digitalWrite(RIGHT_MOTOR_DIR_PIN, direction);

  // 移动直到达到目标距离 / Move until target distance is reached
  while ((leftPulses + rightPulses) / 2 < targetPulses) {
    analogWrite(LEFT_MOTOR_PWM_PIN, speed);
    analogWrite(RIGHT_MOTOR_PWM_PIN, speed);

    // 卡住检测 / Stuck detection
    static unsigned long lastPulseTime = millis();
    if (millis() - lastPulseTime > 1000 && isMoving) {
      lcd.setCursor(0, 3);
      lcd.print("Error: Car stuck!");  // 显示卡住错误 / Display stuck error
      break;
    }
    if (leftPulses != 0 || rightPulses != 0) {
      lastPulseTime = millis();
    }
  }

  // 停止电机 / Stop motors
  setMotors(0, 0);
}

// 转向控制 / Turn control
void turn(int targetDegree, int speed) {
  int currentHeading = getCompassHeading();
  if (currentHeading < 0) {
    lcd.setCursor(0, 3);
    lcd.print("Compass Error!");  // 指南针错误 / Compass error
    return;
  }

  // 计算最短转向方向 / Calculate shortest turn direction
  int diff = (targetDegree - currentHeading + 360) % 360;
  if (diff > 180) diff -= 360;

  // 转向直到达到目标角度 / Turn until target angle is reached
  while (abs(diff) > HEADING_TOLERANCE) {
    if (diff > 0) {
      setMotors(speed, -speed);  // 右转 / Turn right
    } else {
      setMotors(-speed, speed);  // 左转 / Turn left
    }

    currentHeading = getCompassHeading();
    diff = (targetDegree - currentHeading + 360) % 360;
    if (diff > 180) diff -= 360;
  }

  setMotors(0, 0);  // 停止 / Stop
}

// 检查ESP心跳 / Check ESP heartbeat
void checkHeartbeat() {
  if (millis() - lastHeartbeat > HEARTBEAT_TIMEOUT) {
    lcd.setCursor(0, 3);
    lcd.print("ESP Connection Lost!");  // ESP连接丢失 / ESP connection lost
    setMotors(0, 0);                    // 安全停止 / Safety stop
  }
}

// 更新LCD显示 / Update LCD display
void updateLCD() {
  lcd.clear();

  // 显示模式和指南针 / Display mode and compass
  lcd.setCursor(0, 0);
  lcd.print(isESPMode ? "Mode: ESP" : "Mode: Joystick");
  lcd.setCursor(0, 1);
  int heading = getCompassHeading();
  lcd.print("Heading: ");
  lcd.print(heading);
  lcd.print(" ");
  lcd.print(getCardinalDirection(heading));

  // 显示距离 / Display distance
  lcd.setCursor(0, 2);
  lcd.print("L:");
  lcd.print(leftDistance, 1);
  lcd.print("cm R:");
  lcd.print(rightDistance, 1);
  lcd.print("cm");

  // 显示最后命令或摇杆值 / Display last command or joystick values
  lcd.setCursor(0, 3);
  if (isESPMode) {
    lcd.print("Cmd: ");
    lcd.print(lastCommand);
  } else {
    lcd.print("X:");
    lcd.print(analogRead(JOYSTICK_X));
    lcd.print(" Y:");
    lcd.print(analogRead(JOYSTICK_Y));
  }
}

// 获取方向描述 / Get cardinal direction description
String getCardinalDirection(int heading) {
  const char* directions[] = { "N", "NE", "E", "SE", "S", "SW", "W", "NW" };
  int index = ((heading + 22) % 360) / 45;
  return directions[index];
}