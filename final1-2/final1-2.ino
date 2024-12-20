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

/************************* 引脚区 *************************/
LiquidCrystal lcd(38, 36, 35, 34, 33, 30);

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



// mode control
#define MODE_JOYSTICK 0
#define MODE_ESP 1


// direction control
#define FORWARD 1
#define BACKWARD 0

/************************* 全局变量区 *************************/

// define errors
enum ErrorType {
    ERROR_NONE = 0,
    ERROR_COMPASS = 1,
    ERROR_MOTOR = 2,
    ERROR_JOYSTICK = 3,
    ERROR_COMMUNICATION = 4
};

// 错误信息结构
struct ErrorInfo {
    bool isError;
    ErrorType type;
    String message;
    String statusMessage;  // 添加状态信息
};

// 全局错误信息对象
ErrorInfo currentError = {false, ERROR_NONE, "", ""};

// system status
int currentMode = MODE_JOYSTICK;

// compass setting
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
const int JOYSTICK_DEADZONE = 25;  // 将死区值定义为常量

/************************* 函数声明区 *********************/
// 初始化函数
void setupSystem();
void setupCompass();
void setupMotors();
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
void handleError(ErrorType type, String message);

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

    // 设置编码器中断
    pinMode(LEFT_ENCODER_PIN, INPUT_PULLUP);
    pinMode(RIGHT_ENCODER_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN), handleLeftEncoder, RISING);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN), handleRightEncoder, RISING);
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
    // 1.更新方向信息, change to north
    updateOrientation();
    handleTurnCommand(0);

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
    
    // 5.错误检查
    handleError();
    
    delay(50);
}

/************************* 功能函数区 *********************/

void setupSystem() {

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
    handleError(ERROR_COMPASS, "Compass read failed");
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
    
    if (modeButton.fell()) {  

        stopMotors();
        if (currentMode == MODE_JOYSTICK) {
            currentMode = MODE_ESP;
            leftEncoderPulses = 0;
            rightEncoderPulses = 0;
            lcd.clear();
            Serial.println("Switched to ESP mode");
        } else {
            currentMode = MODE_JOYSTICK;
            leftEncoderPulses = 0;
            rightEncoderPulses = 0;
            lcd.clear();
            Serial.println("Switched to Joystick mode");
        }
    }
}

// joystick mode
void handleJoystickMode() {
    updateStatus("Joystick Control");
    const int deadzone = 25;  // 设置死区值
    
    // 读取摇杆值并中心化到0
    int xValue = analogRead(JOYSTICK_X_PIN) - 512;
    int yValue = analogRead(JOYSTICK_Y_PIN) - 512;
    
    // 死区处理
    if(abs(xValue) < deadzone) xValue = 0;
    if(abs(yValue) < deadzone) yValue = 0;
    
    // 计算基础速度和转向速度
    int baseSpeed = constrain(map(abs(yValue), deadzone, 512, 0, 255), 0, 255);
    int turnSpeed = constrain(map(abs(xValue), deadzone, 512, 0, baseSpeed), 0, baseSpeed);
    
    // 设置初始电机速度
    int leftSpeed = baseSpeed;
    int rightSpeed = baseSpeed;
    
    // 如果摇杆在死区内，停止电机
    if (yValue == 0 && xValue == 0) {
        stopMotors();
        updateStatus("Standby");  // 更新状态为待机
        return;  // 直接返回，不执行后续代码
    }

    
    // 设置前进/后退方向
    bool forward = yValue >= 0;
    digitalWrite(LEFT_MOTOR_DIR_PIN, forward ? FORWARD : BACKWARD);
    digitalWrite(RIGHT_MOTOR_DIR_PIN, forward ? FORWARD : BACKWARD);
    
    // 转向控制
    if (xValue < 0) {
        leftSpeed -= turnSpeed;  // 左转
    } else if (xValue > 0) {
        rightSpeed -= turnSpeed; // 右转
    }
    
    // 应用电机速度
    analogWrite(LEFT_MOTOR_PWM_PIN, leftSpeed);
    analogWrite(RIGHT_MOTOR_PWM_PIN, rightSpeed);
    
    // 打印调试信息
    Serial.print("Motor Speed - Left: ");
    Serial.print(leftSpeed);
    Serial.print(" Right: ");
    Serial.print(rightSpeed);
    Serial.print(" Direction: ");
    Serial.println(forward ? "Forward" : "Backward");
}

// ESP mode
void handleESPMode() {
    updateStatus("ESP Control");
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
            // 直接调用handleTurnCommand(0)来寻找北方
            handleTurnCommand("0");
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

// handling turn command
void handleTurnCommand(String value) {
    int turnAngle = value.toInt();
    updateOrientation();
    int initialOrientation = currentOrientation;
    int accumulatedAngle = 0;  // 累计转动角度
    int lastOrientation = currentOrientation;
    
    // 如果是寻北命令
    if (turnAngle == 0) {
        // 寻北模式使用最短路径
        int diff = -currentOrientation;
        if (diff < -180) diff += 360;
        if (diff > 180) diff -= 360;
        turnAngle = diff;
    }

    while (abs(accumulatedAngle) < abs(turnAngle)) {
        updateOrientation();
        
        // 计算单次转动的角度变化
        int angleChange = currentOrientation - lastOrientation;
        
        // 处理角度环绕情况
        if (angleChange > 180) angleChange -= 360;
        if (angleChange < -180) angleChange += 360;
        
        // 只有当角度变化超过阈值时才累加
        if (abs(angleChange) > 1) {
            if (turnAngle > 0) {
                // 右转
                if (angleChange > 0) {
                    accumulatedAngle += angleChange;
                } else {
                    // 处理从359到0的情况
                    accumulatedAngle += (angleChange + 360);
                }
            } else {
                // 左转
                if (angleChange < 0) {
                    accumulatedAngle += angleChange;
                } else {
                    // 处理从0到359的情况
                    accumulatedAngle -= (360 - angleChange);
                }
            }
            lastOrientation = currentOrientation;
        }

        // 计算剩余需要转动的角度
        int remainingAngle = abs(turnAngle) - abs(accumulatedAngle);
        
        // 根据剩余角度动态调整速度
        int turnSpeed;
        if (remainingAngle < 10) {
            turnSpeed = MIN_TURN_SPEED - 30;
        } else if (remainingAngle < 30) {
            turnSpeed = MIN_TURN_SPEED - 20;
        } else {
            turnSpeed = MIN_TURN_SPEED;
        }

        // 设置电机方向
        if (turnAngle > 0) {
            // 右转
            digitalWrite(LEFT_MOTOR_DIR_PIN, FORWARD);
            digitalWrite(RIGHT_MOTOR_DIR_PIN, BACKWARD);
        } else {
            // 左转
            digitalWrite(LEFT_MOTOR_DIR_PIN, BACKWARD);
            digitalWrite(RIGHT_MOTOR_DIR_PIN, FORWARD);
        }

        // 应用转向速度
        analogWrite(LEFT_MOTOR_PWM_PIN, turnSpeed);
        analogWrite(RIGHT_MOTOR_PWM_PIN, turnSpeed);

        delay(50);
    }
    
    stopMotors();
    Serial.println("Turn completed");
}

// 更新方向的辅助函数
void updateOrientation() {
    currentOrientation = readCompass();
}

// handling find north command
// void handleFindNorth() {
//     Serial.println("Starting North finding sequence...");
//     updateOrientation();  // 更新当前朝向
    
//     // 直接使用当前方向的负值作为转动角度
//     int turnValue = -currentOrientation;
//     if (turnValue < -180) {
//         turnValue += 360;  // 选择最短的转向路径
//     } else if (turnValue > 180) {
//         turnValue -= 360;
//     }
    
//     // 打印调试信息
//     Serial.print("Current orientation: ");
//     Serial.print(currentOrientation);
//     Serial.print("° Calculated turn value: ");
//     Serial.println(turnValue);
    
//     // 调用handleTurnCommand
//     handleTurnCommand(String(turnValue));
    
//     Serial.println("North direction found!");
// }

// 计算航向误差的辅助函数
// int calculateHeadingError(int current, int target) {
//     int error = target - current;
//     if (error > 180) error -= 360;
//     if (error < -180) error += 360;
//     return error;
// }

// moving motors
void moveMotors(int leftSpeed, int rightSpeed) {
    // 设置方向
    digitalWrite(LEFT_MOTOR_DIR_PIN, leftSpeed >= 0 ? FORWARD : BACKWARD);
    digitalWrite(RIGHT_MOTOR_DIR_PIN, rightSpeed >= 0 ? FORWARD : BACKWARD);
    
    // 设置速度
    analogWrite(LEFT_MOTOR_PWM_PIN, abs(leftSpeed));
    analogWrite(RIGHT_MOTOR_PWM_PIN, abs(rightSpeed));
}

// 更新LCD显示函数
void updateLCD() {
    lcd.clear();
    

    // 第一行：模式信息
    lcd.setCursor(0, 1);
    lcd.print("Status: ");
    lcd.print(currentError.statusMessage);
    
    // 第二行：方向信息
    displayDirection(currentOrientation);
    
    // 第四行：编码器计数
    lcd.setCursor(0, 3);
    lcd.print("L:");
    lcd.print(leftEncoderPulses);
    lcd.print(" R:");
    lcd.print(rightEncoderPulses);
}

// 修改错误处理函数
void handleError(ErrorType type, String message) {
    currentError.isError = true;
    currentError.type = type;
    currentError.message = message;
    
    // 根据错误类型采取不同措施
    switch(type) {
        case ERROR_COMPASS:
            stopMotors();
            currentError.statusMessage = "Compass Error";
            break;
            
        case ERROR_MOTOR:
            stopMotors();
            currentError.statusMessage = "Motor Error";
            break;
            
        case ERROR_JOYSTICK:
            stopMotors();
            currentError.statusMessage = "Joystick Error";
            break;
            
        case ERROR_COMMUNICATION:
            currentError.statusMessage = "Comm Error";
            break;
    }
}

// 添加状态更新函数
void updateStatus(String status) {
    currentError.isError = false;
    currentError.type = ERROR_NONE;
    currentError.message = "";
    currentError.statusMessage = status;
    updateLCD();
}


// 添加方向显示函数
void displayDirection(int heading) {
    String direction;
    
    // 将��度转换为方向字符串
    if (heading >= 337.5 || heading < 22.5) {  // 处理跨越0度的北向区间
        direction = "N";
    } else if (heading >= 22.5 && heading < 67.5) {  // 使用标准45度区间
        direction = "NE";
    } else if (heading >= 67.5 && heading < 112.5) {
        direction = "E";
    } else if (heading >= 112.5 && heading < 157.5) {
        direction = "SE";
    } else if (heading >= 157.5 && heading < 202.5) {
        direction = "S";
    } else if (heading >= 202.5 && heading < 247.5) {
        direction = "SW";
    } else if (heading >= 247.5 && heading < 292.5) {
        direction = "W";
    } else if (heading >= 292.5 && heading < 337.5) {
        direction = "NW";
    }
    
    // 在LCD上显示方向信息
    lcd.setCursor(0, 2);  // 使用第三行显示方向
    lcd.print("Dir: ");
    lcd.print(heading);
    lcd.print("  ");
    lcd.print(direction);
}

// 添加中断处理函数
void handleLeftEncoder() {
    leftEncoderPulses++;
}

void handleRightEncoder() {
    rightEncoderPulses++;
}

// 添加重置编码器计数的函数
void resetEncoders() {
    leftEncoderPulses = 0;
    rightEncoderPulses = 0;
}

void handleError() {
    if (currentError.isError) {
        // 显示错误信息
        lcd.clear();
        lcd.print("Error: " + currentError.message);
        
        // 根据错误类型采取恢复措施
        switch(currentError.type) {
            case ERROR_COMPASS:
                // 尝试重新初始化指南针
                setupCompass();
                break;
            case ERROR_MOTOR:
                // 重置电机控制
                setupMotors();
                break;
            case ERROR_JOYSTICK:
                // 切换到ESP模式
                currentMode = MODE_ESP;
                break;
            case ERROR_COMMUNICATION:
                // 尝试重新建立连接
                Serial.end();
                Serial.begin(9600);
                break;
        }
        
        // 清除错误状态
        currentError.isError = false;
        currentError.message = "";
        updateLCD();
    }
}


