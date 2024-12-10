/*******************************************************************
 * RC Car Control Framework
 * 
 * 功能模块:
 * 1. 指南针导航与校准
 * 2. 双模式控制（摇杆/ESP）
 * 3. 编码器距离测量
 * 4. LCD显示
 * 5. 错误处理
 *******************************************************************/

/************************* 引入区 *************************/
#include <LiquidCrystal.h>
#include <Wire.h>
#include<math.h>
#include <Bounce2.h>
#include <EEPROM.h>

/************************* 引脚区 *************************/
LiquidCrystal lcd(39, 36, 35, 34, 33, 30);

// define motor
#define LEFT_MOTOR_DIR_PIN 7
#define RIGHT_MOTOR_DIR_PIN 8    
#define LEFT_MOTOR_PWM_PIN 9
#define RIGHT_MOTOR_PWM_PIN 10

// define encoder
#define LEFT_ENCODER_PIN 2
#define RIGHT_ENCODER_PIN 3

// define joystick
#define JOYSTICK_X_PIN A2
#define JOYSTICK_Y_PIN A3
#define JOYSTICK_SW_PIN 4

// define compass
#define CMPS14_ADDRESS 0x60
#define BEARING_Register 0x01

/***********************  系统常量区 *************************/
// motion control
const int MIN_TURN_SPEED = 120;
const int MAX_TURN_SPEED = 180;
const int HEADING_TOLERANCE = 2;
const float PULSES_PER_CM = 13.31;

// mode control
#define MODE_JOYSTICK 0
#define MODE_ESP 1

// PWM
const int PWM_STOP = 0;
const int PWM_SLOW = 120;
const int PWM_MEDIUM = 180;
const int PWM_FAST = 255;

// direction control
#define FORWARD 1
#define BACKWARD 0

// define compass direction
#define NORTH 0
#define EAST 90
#define SOUTH 180
#define WEST 270

//*const unsigned long HEARTBEAT_TIMEOUT = 5000; //heartbeat timeout

/************************* 全局变量区 *************************/
// system status
int currentMode = MODE_JOYSTICK;
bool isError = false;  
String errorMessage = ""; 

// compass setting
int currentHeading = 0;
int compassOffset = 0;

// encoder count
volatile int leftEncoderPulses = 0;
volatile int rightEncoderPulses = 0;
float leftDistance = 0;
float rightDistance = 0;

// motor control
int leftMotorSpeed = 0;
int rightMotorSpeed = 0;

// ESP connection
String lastCommand = "";

// joystick control
int joystickX = 0;
int joystickY = 0;
bool joystickButtonPressed = false;
Bounce modeButton = Bounce();

// 在全局变量区添加
int currentOrientation = 0;  // 记录小车当前朝向（以北为0度）

/************************* 函数声明区 *********************/
// 初始化函数
void setupSystem();
void setupCompass();
void setupMotors();
void setupJoystick();
void setupEncoder();
void calibrateCompass();

// 错误处理
void handleLeftEncoder();
void handleRightEncoder();

// 传感器函数
int readCompass();
void readEncoders();
void readJoystick();

// 控制函数
void handleJoystickMode();
void handleESPMode();
void moveMotors(int left, int right);

// 显示函数
void updateLCD();
void showError(String msg);

// 在函数声明区添加
void handleMoveCommand(int distance);

/************************* 初始化程序 *********************/
void setup() {
    // 初始化串口
    Serial.begin(9600);
    
    // 初始化各个模块
    setupMotors();    // 电机初始化
    lcd.begin(20, 4); // LCD初始化
    setupSystem();    // 系统初始化
    setupCompass();   // 指南针初始化
    
    // ensure motors stop
    stopMotors();
    delay(100);
    stopMotors();

    // 初始化完成提示
    Serial.println("System initialized");
    lcd.print("System Ready!");
}

void setupMotors() {
    // 设置电机控制引脚为输出模式
    pinMode(LEFT_MOTOR_DIR_PIN, OUTPUT);
    pinMode(RIGHT_MOTOR_DIR_PIN, OUTPUT);
    pinMode(LEFT_MOTOR_PWM_PIN, OUTPUT);
    pinMode(RIGHT_MOTOR_PWM_PIN, OUTPUT);
    
    // 确保电机初始状态为停止，尝试修改方向引脚的默认值
    stopMotors();
    
    leftMotorSpeed = 0;
    rightMotorSpeed = 0;
}

/************************* 循环程序 *********************/
void loop() {
    // 1. 读取传感器
    currentHeading = readCompass();
    //readEncoders();
    
    // 2. 检查模式切换
    checkModeSwitch();
    
    // 3. 根据模式执行控制
    if (currentMode == MODE_JOYSTICK) {
        handleJoystickMode();
    } else {
        handleESPMode();
    }
    
    // 4. 更新显示
    updateLCD();
    
    // 5. 错误检查
    if (isError) {
        handleError();
    }
    
    // 6. 短暂延时
    delay(5);
}

/************************* 功能函数区 *********************/
// 这里将实现各个具体的功能函数...

void setupSystem() {
    // ... other setup code ...
    
    // 设置模式切换按钮（使用摇杆的按钮）
    pinMode(JOYSTICK_SW_PIN, INPUT_PULLUP);
    modeButton.attach(JOYSTICK_SW_PIN);
    modeButton.interval(50); // 设置防抖时间为50ms
}

// stop motors
void stopMotors() {
    analogWrite(LEFT_MOTOR_PWM_PIN, 0);
    analogWrite(RIGHT_MOTOR_PWM_PIN, 0);
}
// setting up compas
void setupCompass() {
    // 初始化I2C通信
    Wire.begin();
    
    // 等待指南针模块稳定
    delay(100);
    
    // 校准指南针偏移量
    calibrateCompass();
}

// using with compass
void calibrateCompass() {
    // 读取多次取平均值作为偏移量
    int sum = 0;
    for(int i = 0; i < 10; i++) {
        sum += readRawCompass();
        delay(100);
    }
    compassOffset = sum / 10;
    
    Serial.print("Compass calibrated, offset: ");
    Serial.println(compassOffset);
}

int readCompass() {
    // 向CMPS14发送寄存器地址
    Wire.beginTransmission(CMPS14_ADDRESS);
    Wire.write(BEARING_Register);
    Wire.endTransmission();
    
    // 请求读取1个字节
    Wire.requestFrom(CMPS14_ADDRESS, 1);
    
    if (Wire.available() >= 1) {
        int raw = Wire.read();  // 读取原始值 (0-255)
        // 转换为角度 (0-359)
        int heading = map(raw, 0, 255, 0, 359);
        
        // 应用偏移量校准
        heading = (heading - compassOffset + 360) % 360;
        
        return heading;
    }
    
    return -1;  // 读取失败返回-1
}

// 读取原始指南针值（用于校准）
int readRawCompass() {
    Wire.beginTransmission(CMPS14_ADDRESS);
    Wire.write(BEARING_Register);
    Wire.endTransmission();
    
    Wire.requestFrom(CMPS14_ADDRESS, 1);
    
    if (Wire.available() >= 1) {
        int raw = Wire.read();
        return map(raw, 0, 255, 0, 359);
    }
    
    return -1;
}

// checking mode switch
void checkModeSwitch() {
    modeButton.update();
    
    if (modeButton.fell()) {  // 按钮被按下
        // 切换模式
        stopMotors();
        if (currentMode == MODE_JOYSTICK) {
            currentMode = MODE_ESP;
            lcd.clear();
            Serial.println("Switched to ESP mode");
        } else {
            currentMode = MODE_JOYSTICK;
            lcd.clear();
            Serial.println("Switched to Joystick mode");
        }
    }
}

// joystick mode
void handleJoystickMode() {
    // 读取摇杆值
    joystickX = analogRead(JOYSTICK_X_PIN);
    joystickY = analogRead(JOYSTICK_Y_PIN);
    
    // 将摇杆值映射为电机速度
    int leftSpeed = 0;
    int rightSpeed = 0;
    
    // 打印摇杆原始值用于调试
    Serial.print("Joystick Raw - X: ");
    Serial.print(joystickX);
    Serial.print(" Y: ");
    Serial.println(joystickY);
    
    // Y轴控制前进后退
    int baseSpeed = map(joystickY, 0, 1023, -255, 255);
    // X轴控制转向
    int turnOffset = map(joystickX, 0, 1023, -128, 128);
    
    // 检查摇杆是否在中位
    if (abs(joystickY - 512) < 100 && abs(joystickX - 512) < 100) {
        // 摇杆在中位区域，停止电机
        stopMotors();
        return;  // 直接返回，不执行后续代码
    }
    
    // 如果摇杆不在中位，计算电机速度
    leftSpeed = baseSpeed + turnOffset;
    rightSpeed = baseSpeed - turnOffset;
    
    // 限制速度范围
    leftSpeed = constrain(leftSpeed, -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);
    
    // 添加死区控制
    if (abs(baseSpeed) < 50) {  // 添加死区
        leftSpeed = 0;
        rightSpeed = 0;
    }
    
    // 控制电机
    moveMotors(leftSpeed, rightSpeed);
}

// ESP mode
void handleESPMode() {
    // 检查是否有新的串口命令
    if (Serial.available() > 0) {
        lastCommand = Serial.readStringUntil('\n');
        lastCommand.trim();  // 移除多余的空格和换行符
        
        // 解析命令
        if (lastCommand.startsWith("Move:")) {
            // 格式: "Move:20" 或 "Move:-20"
            int distance = lastCommand.substring(5).toInt();
            handleMoveCommand(distance);
        }
        else if (lastCommand.startsWith("Turn:")) {
            // 格式: "Turn:180" 
            String value = lastCommand.substring(5);
            handleTurnCommand(value);
        }
        else if (lastCommand == "find:North") {
            handleFindNorth();
        }
        
        // 更新LCD显示最新命令
        updateLCD();
    }
}

// handling move command
void handleMoveCommand(int distance) {
    if (distance > 0) {
        // 前进
        moveMotors(200, 200);  // 使用适中的速度前进
    } else if (distance < 0) {
        // 后退
        moveMotors(-200, -200);  // 使用适中的速度后退
    }
    
    // 根据distance的绝对值来决定运行时间
    delay(abs(distance) * 100);  // 假设每厘米需要100ms
    moveMotors(0, 0);  // 停止
}
// finding heading
void findHeading(int target) {
    // 更新当前方向
    int currentHeading = readCompass();
    int turnSpeed = 150;  // 声明并初始化转向速度

    // 计算到目标角度的最短路径
    int diff = target - currentHeading;  // 顺时针差值
    int complement = diff > 0 ? diff - 360 : 360 + diff;  // 逆时针差值
    int realDiff = abs(diff) < abs(complement) ? diff : complement;
    
    // 打印调试信息
    Serial.print("Current: ");
    Serial.print(currentHeading);
    Serial.print(" Target: ");
    Serial.print(target);
    Serial.print(" Diff: ");
    Serial.println(realDiff);

    // 当差值大于容差时继续转向
    while ((abs(realDiff) * 1.0) > 2.5) {
        if (realDiff > 0) {
            // 右转
            digitalWrite(LEFT_MOTOR_DIR_PIN, FORWARD);
            digitalWrite(RIGHT_MOTOR_DIR_PIN, BACKWARD);
        } else {
            // 左转
            digitalWrite(LEFT_MOTOR_DIR_PIN, BACKWARD);
            digitalWrite(RIGHT_MOTOR_DIR_PIN, FORWARD);
        }
        
        // 应用电机速度
        analogWrite(LEFT_MOTOR_PWM_PIN, turnSpeed);
        analogWrite(RIGHT_MOTOR_PWM_PIN, turnSpeed);
        
        delay(100);
        stopMotors();
        delay(50);
        
        // 更新方向和差值
        currentHeading = readCompass();
        diff = target - currentHeading;
        complement = diff > 0 ? diff - 360 : 360 + diff;
        realDiff = abs(diff) < abs(complement) ? diff : complement;
        
        // 打印当前状态
        Serial.print("Current: ");
        Serial.print(currentHeading);
        Serial.print(" Diff: ");
        Serial.println(realDiff);
    }
    
    stopMotors();
    Serial.println("Target heading reached");
}

// handling turn command
void handleTurnCommand(String value) {
    int turnAngle = value.toInt();
    updateOrientation();  // 更新当前朝向
    
    Serial.print("Current Orientation: ");
    Serial.print(currentOrientation);
    Serial.print("° Turn Angle: ");
    Serial.println(turnAngle);
    
    // 如果turnAngle为0，说明是要转到0度（北方）
    if (turnAngle == 0) {
        turnAngle = -currentOrientation;  // 计算到0度需要转动的角度
        Serial.print("Turning to North, recalculated angle: ");
        Serial.println(turnAngle);
    }
    
    int remainingAngle = abs(turnAngle);  // 仅关注需要转动的角度大小
    
    while (remainingAngle > HEADING_TOLERANCE) {
        updateOrientation();
        
        Serial.print("Current: ");
        Serial.print(currentOrientation);
        Serial.print("° Remaining: ");
        Serial.println(remainingAngle);
        
        // 根据剩余角度动态调整转向速度
        int turnSpeed;
        if (remainingAngle < 15) {
            turnSpeed = MIN_TURN_SPEED;
        } else {
            turnSpeed = map(remainingAngle, 15, 180, MIN_TURN_SPEED, MAX_TURN_SPEED);
        }
        
        // 根据turnAngle的正负决定转向方向
        if (turnAngle > 0) {
            // 右转
            digitalWrite(LEFT_MOTOR_DIR_PIN, FORWARD);
            digitalWrite(RIGHT_MOTOR_DIR_PIN, BACKWARD);
        } else {
            // 左转
            digitalWrite(LEFT_MOTOR_DIR_PIN, BACKWARD);
            digitalWrite(RIGHT_MOTOR_DIR_PIN, FORWARD);
        }
        
        analogWrite(LEFT_MOTOR_PWM_PIN, turnSpeed);
        analogWrite(RIGHT_MOTOR_PWM_PIN, turnSpeed);
        
        // 根据剩余角度调整转向时间
        int turnTime = map(remainingAngle, 0, 180, 30, 100);
        delay(turnTime);
        stopMotors();
        delay(50);
        
        // 更新剩余需要转动的角度
        remainingAngle -= 5;  // 假设每次转动5度

         // 打印调试信息
        Serial.print("Current: ");
        Serial.print(currentOrientation);
        Serial.print("° Remaining: ");
        Serial.println(remainingAngle);
    }
    
    stopMotors();
    Serial.println("Turn completed");
}

// 更新方向的辅助函数
void updateOrientation() {
    currentOrientation = readCompass();
    Serial.print("Updated orientation: ");
    Serial.println(currentOrientation);
}

// handling find north command
void handleFindNorth() {
    Serial.println("Starting North finding sequence...");
    updateOrientation();  // 更新当前朝向
    
    // 直接使用当前方向的负值作为转动角度
    int turnValue = -currentOrientation;
    
    // 打印调试信息
    Serial.print("Current orientation: ");
    Serial.print(currentOrientation);
    Serial.print("° Calculated turn value: ");
    Serial.println(turnValue);
    
    // 确保转换为字符串时保留负号
    String turnCommand = String(turnValue);
    Serial.print("Turn command string: ");
    Serial.println(turnCommand);
    
    // 调用handleTurnCommand
    handleTurnCommand(turnCommand);
    
    Serial.println("North direction found!");
}

// 计算航向误差的辅助函数
int calculateHeadingError(int current, int target) {
    int error = target - current;
    if (error > 180) error -= 360;
    if (error < -180) error += 360;
    return error;
}

// moving motors
void moveMotors(int leftSpeed, int rightSpeed) {
    // 设置方向
    digitalWrite(LEFT_MOTOR_DIR_PIN, leftSpeed >= 0 ? FORWARD : BACKWARD);
    digitalWrite(RIGHT_MOTOR_DIR_PIN, rightSpeed >= 0 ? FORWARD : BACKWARD);
    
    // 设置速度
    analogWrite(LEFT_MOTOR_PWM_PIN, abs(leftSpeed));
    analogWrite(RIGHT_MOTOR_PWM_PIN, abs(rightSpeed));
}

// 添加错误处理函数
void handleError() {
    if (isError) {
        // 显示错误信息
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Error:");
        lcd.setCursor(0, 1);
        lcd.print(errorMessage);
        
        // 停止电机
        moveMotors(0, 0);
        
        // 等待一段时间后清除错误
        delay(2000);
        isError = false;
        errorMessage = "";
    }
}

// 更新LCD显示函数
void updateLCD() {
    lcd.clear();
    
    // 第一行：模式和方向
    lcd.setCursor(0, 0);
    lcd.print("Mode:");
    lcd.print(currentMode == MODE_JOYSTICK ? "Joy" : "ESP");
    lcd.print(" Dir:");
    lcd.print(readCompass());
    
    // 第二行：编码器计数
    lcd.setCursor(0, 1);
    lcd.print("L:");
    lcd.print(leftEncoderPulses);
    lcd.print(" R:");
    lcd.print(rightEncoderPulses);
    
    // 第三行：根据模式显示不同信息
    lcd.setCursor(0, 2);
    if (currentMode == MODE_JOYSTICK) {
        lcd.print("X:");
        lcd.print(joystickX);
        lcd.print(" Y:");
        lcd.print(joystickY);
    } else {
        lcd.print("CMD:");
        lcd.print(lastCommand);
    }
}


