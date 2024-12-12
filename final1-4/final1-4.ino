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

/************************* IMPORT LIBRARIES *************************/
#include <LiquidCrystal.h>
#include <Wire.h>
#include<math.h>
#include <Bounce2.h>

/************************* PIN DEFINE  *************************/
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

/***********************  SYSTEM CONSTANTS *************************/
// motion control
const int MIN_TURN_SPEED = 120;
const int MAX_TURN_SPEED = 180;



// mode control
#define MODE_JOYSTICK 0
#define MODE_ESP 1


// direction control
#define FORWARD 1
#define BACKWARD 0

/************************* GLOBAL VARIABLES *************************/

// define errors
enum ErrorType {
    ERROR_NONE = 0,
    ERROR_COMPASS = 1,
    ERROR_MOTOR = 2,
    ERROR_JOYSTICK = 3,
    ERROR_COMMUNICATION = 4
};

// ERROR INFO
struct ErrorInfo {
    bool isError;
    ErrorType type;
    String message;
    String statusMessage;  // 添加状态信息
};

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

// record current orientation
int currentOrientation = 0;  
// joystick deadzone
const int JOYSTICK_DEADZONE = 25;  // 将死区值定义为常量

/************************* DEFINEFUNCTIONS *********************/
// setup functions
void setupSystem();
void setupCompass();
void setupMotors();
void setupEncoder();
void calibrateCompass();

// pulse count
void handleLeftEncoder();
void handleRightEncoder();

// sensor functions
int readCompass();
void readEncoders();
void readJoystick();

// control functions
void handleJoystickMode();
void handleESPMode();
void moveMotors(int left, int right);


// display functions
void updateLCD();
void handleError(ErrorType type, String message);

// move command
void handleMoveCommand(int distance);

// turn command
void handleTurnCommand(String value);

/************************* INITIALIZATION *********************/
void setup() {
    // Serial setup
    Serial.begin(9600);
    
    // module setup
    setupMotors();    
    lcd.begin(20, 4); 
    setupSystem();    
    setupCompass();   
    
    // ensure motors stop
    stopMotors();
    delay(100);
    stopMotors();

    // init complete
    Serial.println("System initialized");
    lcd.print("System Ready!");

    // setup encoder interrupt
    pinMode(LEFT_ENCODER_PIN, INPUT_PULLUP);
    pinMode(RIGHT_ENCODER_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN), handleLeftEncoder, RISING);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN), handleRightEncoder, RISING);
}

void setupMotors() {
    // setup motor control pins
    pinMode(LEFT_MOTOR_DIR_PIN, OUTPUT);
    pinMode(RIGHT_MOTOR_DIR_PIN, OUTPUT);
    pinMode(LEFT_MOTOR_PWM_PIN, OUTPUT);
    pinMode(RIGHT_MOTOR_PWM_PIN, OUTPUT);
    
    // 确保电机初始状态为停止，尝试修改方向引脚的默认值
    stopMotors();
    
    leftMotorSpeed = 0;
    rightMotorSpeed = 0;
}

/************************* MAIN LOOP *********************/
void loop() {
    // 1.update orientation
    updateOrientation();

    // 2. check mode switch
    checkModeSwitch();
    
    // 3. control according to mode
    if (currentMode == MODE_JOYSTICK) {
        handleJoystickMode();
    } else {
        handleESPMode();
    }
    
    // 4. update display
    updateLCD();
    
    // 5. error check
    handleError();
    
    delay(50);
}

/************************* FUNCTIONS *********************/

void setupSystem() {

    // setup mode switch button 
    pinMode(JOYSTICK_SW_PIN, INPUT_PULLUP);
    modeButton.attach(JOYSTICK_SW_PIN);
    modeButton.interval(50); // set debounce time
}

// stop motors
void stopMotors() {
    analogWrite(LEFT_MOTOR_PWM_PIN, 0);
    analogWrite(RIGHT_MOTOR_PWM_PIN, 0);
}
// setting up compas
void setupCompass() {
    // init I2C
    Wire.begin();
    
    // wait for compass stable
    delay(100);
    
    // calibrate compass offset
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
    
    // request 1 byte
    Wire.requestFrom(CMPS14_ADDRESS, 1);
    
    if (Wire.available() >= 1) {
        int raw = Wire.read();  // read raw value (0-255)
        // convert to angle (0-359)
        int heading = map(raw, 0, 255, 0, 359);
        
        // apply offset calibration
        heading = (heading - compassOffset + 360) % 360;
        
        return heading;
    }
    handleError(ERROR_COMPASS, "Compass read failed");
    return -1;  // read failed
}

// read raw compass value (for calibration)
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
    
    // read joystick input
    int xValue = analogRead(JOYSTICK_X_PIN) - 512;
    int yValue = analogRead(JOYSTICK_Y_PIN) - 512;
    
    // deadzone
    if(abs(xValue) < JOYSTICK_DEADZONE) xValue = 0;
    if(abs(yValue) < JOYSTICK_DEADZONE) yValue = 0;
    
    // standby check
    if (yValue == 0 && xValue == 0) {
        stopMotors();
        updateStatus("Standby");
        return;
    }
    
    // calculate motor speed
    int baseSpeed = constrain(map(abs(yValue), JOYSTICK_DEADZONE, 512, 0, 255), 0, 255);
    int turnSpeed = constrain(map(abs(xValue), JOYSTICK_DEADZONE, 512, 0, baseSpeed), 0, baseSpeed);
    int leftSpeed = baseSpeed;
    int rightSpeed = baseSpeed;
    
    // set motion direction
    bool forward = yValue >= 0;
    digitalWrite(LEFT_MOTOR_DIR_PIN, forward ? FORWARD : BACKWARD);
    digitalWrite(RIGHT_MOTOR_DIR_PIN, forward ? FORWARD : BACKWARD);
    
    // apply turn adjustment
    if (xValue < 0) {
        leftSpeed -= turnSpeed;
    } else if (xValue > 0) {
        rightSpeed -= turnSpeed;
    }
    
    // control motor
    analogWrite(LEFT_MOTOR_PWM_PIN, leftSpeed);
    analogWrite(RIGHT_MOTOR_PWM_PIN, rightSpeed);
    
    // debug output
    #ifdef DEBUG_MODE
    Serial.print("Motor Speed - Left: ");
    Serial.print(leftSpeed);
    Serial.print(" Right: ");
    Serial.print(rightSpeed);
    Serial.print(" Direction: ");
    Serial.println(forward ? "Forward" : "Backward");
    #endif
}

// ESP mode
void handleESPMode() {
    updateStatus("ESP Control");
    // check new command
    if (Serial.available() > 0) {
        lastCommand = Serial.readStringUntil('\n');
        lastCommand.trim(); // remove extra spaces and new lines
        
        // 解析命令
        if (lastCommand.startsWith("Move:")) {
            // format: "Move:20" or "Move:-20"
            int distance = lastCommand.substring(5).toInt();
            handleMoveCommand(distance);
        }
        else if (lastCommand.startsWith("Turn:")) {
            // format: "Turn:180" 
            // 格式: "Turn:180" 
            String value = lastCommand.substring(5);
            handleTurnCommand(value);
        }
        else if (lastCommand == "find:North") {
            // find:North
            handleTurnCommand("0");
        }
        
        
        updateLCD();
    }
}

// handling move command
void handleMoveCommand(int distance) {
    if (distance > 0) {
        // forward
        moveMotors(200, 200);  // use medium speed
    } else if (distance < 0) {
        // backward
        moveMotors(-200, -200);  // use medium speed
    }
    
    // 根据distance的绝对值来决定前进时间
    delay(abs(distance) * 100);  
    moveMotors(0, 0);  

}

// handling turn command
void handleTurnCommand(String value) {
    int turnAngle = value.toInt();
    updateOrientation();
    int initialOrientation = currentOrientation;
    int accumulatedAngle = 0;  // calculate turn angle
    int lastOrientation = currentOrientation;
    
    // find:North
    if (turnAngle == 0) {
        // find north mode use shortest path
        int diff = -currentOrientation;
        if (diff < -180) diff += 360;
        if (diff > 180) diff -= 360;
        turnAngle = diff;
    }

    while (abs(accumulatedAngle) < abs(turnAngle)) {
        updateOrientation();
        
        // calculate single turn angle change
        int angleChange = currentOrientation - lastOrientation;
        
        // handle angle wrap
        if (angleChange > 180) angleChange -= 360;
        if (angleChange < -180) angleChange += 360;
        
        // only add when angle change exceed threshold
        if (abs(angleChange) > 1) {
            if (turnAngle > 0) {
                // right turn
                if (angleChange > 0) {
                    accumulatedAngle += angleChange;
                } else {
                    // handle wrap from 359 to 0
                    accumulatedAngle += (angleChange + 360);
                }
            } else {
                // left turn
                if (angleChange < 0) {
                    accumulatedAngle += angleChange;
                } else {
                    // handle wrap from 0 to 359
                    accumulatedAngle -= (360 - angleChange);
                }
            }
            lastOrientation = currentOrientation;
        }

        // 计算剩余需要转动的角度
        // calculate remaining turn angle
        int remainingAngle = abs(turnAngle) - abs(accumulatedAngle);
        
        // dynamic speed with remaining angle
        // 根据剩余角度动态调整速度
        int turnSpeed;
        if (remainingAngle < 10) {
            turnSpeed = MIN_TURN_SPEED - 30;
        } else if (remainingAngle < 30) {
            turnSpeed = MIN_TURN_SPEED - 20;
        } else {
            turnSpeed = MIN_TURN_SPEED;
        }

        // set motor direction
        if (turnAngle > 0) {
            // right turn
            digitalWrite(LEFT_MOTOR_DIR_PIN, FORWARD);
            digitalWrite(RIGHT_MOTOR_DIR_PIN, BACKWARD);
        } else {
            // left turn
            digitalWrite(LEFT_MOTOR_DIR_PIN, BACKWARD);
            digitalWrite(RIGHT_MOTOR_DIR_PIN, FORWARD);
        }

        // apply turn speed
        analogWrite(LEFT_MOTOR_PWM_PIN, turnSpeed);
        analogWrite(RIGHT_MOTOR_PWM_PIN, turnSpeed);

        delay(50);
    }
    
    stopMotors();
    Serial.println("Turn completed");
}

// update orientation
void updateOrientation() {
    currentOrientation = readCompass();
}

// moving motors
void moveMotors(int leftSpeed, int rightSpeed) {
    // set direction
    digitalWrite(LEFT_MOTOR_DIR_PIN, leftSpeed >= 0 ? FORWARD : BACKWARD);
    digitalWrite(RIGHT_MOTOR_DIR_PIN, rightSpeed >= 0 ? FORWARD : BACKWARD);
    
    // set speed
    analogWrite(LEFT_MOTOR_PWM_PIN, abs(leftSpeed));
    analogWrite(RIGHT_MOTOR_PWM_PIN, abs(rightSpeed));
}

// update LCD
void updateLCD() {
    lcd.clear();

    //  status info
    lcd.setCursor(0, 1);
    lcd.print("Status: ");
    lcd.print(currentError.statusMessage);
    
    // direction info
    displayDirection(currentOrientation);
    
    // encoder info
    lcd.setCursor(0, 3);
    lcd.print("L:");
    lcd.print(leftEncoderPulses);
    lcd.print(" R:");
    lcd.print(rightEncoderPulses);
}

// error handling
void handleError(ErrorType type, String message) {
    currentError.isError = true;
    currentError.type = type;
    currentError.message = message;
    
    // handle error
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

// update status
void updateStatus(String status) {
    currentError.isError = false;
    currentError.type = ERROR_NONE;
    currentError.message = "";
    currentError.statusMessage = status;
    updateLCD();
}


// display direction
void displayDirection(int heading) {
    String direction;
    
    // convert degree to direction in normal 45 degree
    if (heading >= 337.5 || heading < 22.5) {  
        direction = "N";
    } else if (heading >= 22.5 && heading < 67.5) {  
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
    
    // display direction info
    lcd.setCursor(0, 2);  
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
        // display error info
        lcd.clear();
        lcd.print("Error: " + currentError.message);
        
        // handle error
        switch(currentError.type) {
            case ERROR_COMPASS:
                setupCompass();
                break;
            case ERROR_MOTOR:
                setupMotors();
                break;
            case ERROR_JOYSTICK:
                currentMode = MODE_ESP;
                break;
            case ERROR_COMMUNICATION:
                Serial.end();
                Serial.begin(9600);
                break;
        }
        
        // clear error status
        currentError.isError = false;
        currentError.message = "";
        updateLCD();
    }
}


