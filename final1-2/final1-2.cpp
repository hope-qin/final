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
LiquidCrystal lcd(39, 36, 35, 34, 33, 30)

// define motor
#define LEFT_MOTOR_DIR_PIN 7;
#define RIGHT_MOTOR_DIR_PIN 8;  
#define LEFT_MOTOR_PWM_PIN 9;
#define RIGHT_MOTOR_PWM_PIN 10;

// define encoder
#define LEFT_ENCODER_PIN 2;  
#define RIGHT_ENCODER_PIN 3;

// define joystick
#define JOYSTICK_X_PIN A2;
#define JOYSTICK_Y_PIN A3;
#define JOYSTICK_SW_PIN 4;

// define compass
#define CMPS14_ADDRESS 0x60;
#define BEARING_Register 0x01;

/***********************  系统常量区 *************************/
// motion control
const int MIN_TURN_SPEED = 120;
const int MAX_TURN_SPEED = 180;
const int HEADING_TOLERANCE = 2;
const float PULSES_PER_CM = 13.31;

// mode control
#define MODE_JOYSTICK 0;
#define MODE_ESP 1;

// define compass direction
#define NORTH 0;
#define EAST 90;
#define SOUTH 180;
#define WEST 270;    

// direction control
#define FORWARD 1;
#define BACKWARD 0;

//*const unsigned long HEARTBEAT_TIMEOUT = 5000; //heartbeat timeout

/************************* 变量区 *************************/
// 系统状态
int controlMode = 0;  // 0:摇杆, 1:ESP
bool isError = false;

// 传感器数据
int currentHeading = 0;
int compassOffset = 0;

// 编码器计数
volatile int leftEncoderPulses = 0;
volatile int rightEncoderPulses = 0;

/************************* 函数声明区 *********************/
// 初始化函数
void setupSystem();
void setupCompass();
void setupMotors();

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

/************************* 初始化程序 *********************/
void setup() {
    // 初始化串口
    Serial.begin(9600);

    // 初始化LCD
    lcd.begin(20, 4);

    // 初始化各个模块
    setupSystem();
    setupCompass();
    setupMotors();
    
    // 初始化完成提示
    Serial.println("System initialized");
}

/************************* 主循环程序 *********************/
void loop() {
    // 1. 读取传感器
    currentHeading = readCompass();
    readEncoders();
    
    // 2. 检查模式切换
    checkModeSwitch();
    
    // 3. 根据模式执行控制
    if (controlMode == 0) {
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
    delay(10);
}

/************************* 功能函数区 *********************/
// 这里将实现各个具体的功能函数...