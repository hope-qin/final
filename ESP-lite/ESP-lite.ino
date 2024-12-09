/*******************************************************************
 * ESP8266遥控车控制代码 / ESP8266 RC Car Control Code
 * 
 * 功能列表 / Features:
 * - WiFi连接和网页服务器 / WiFi connection and web server
 * - 移动控制命令 / Movement control commands
 * - 指南针控制命令 / Compass control commands
 * - 心跳检测 / Heartbeat detection
 *******************************************************************/

#include <ESP8266WiFi.h>        // WiFi功能库 / WiFi functionality
#include <ESP8266WebServer.h>   // Web服务器库 / Web server library
#include <FS.h>                 // 文件系统库 / File system library
#include <Ticker.h>             // 定时器库 / Timer library

// WiFi设置 / WiFi Settings
const char* ssid = "SOURCE";         // WiFi名称 / WiFi SSID
const char* password = "Pelle!23";   // WiFi密码 / WiFi password

// 系统常量 / System Constants
const int HTTP_OK = 200;             // HTTP成功代码 / HTTP success code
const int HTTP_BAD_REQUEST = 400;    // HTTP错误请求代码 / HTTP bad request code
const int HTTP_NOT_FOUND = 404;      // HTTP未找到代码 / HTTP not found code

// 移动限制 / Movement Limits
const int MAX_DISTANCE = 100;        // 最大距离(cm) / Maximum distance(cm)
const int MIN_DISTANCE = -100;       // 最小距离(cm) / Minimum distance(cm)

// 全局对象 / Global Objects
ESP8266WebServer server(80);         // Web服务器实例 / Web server instance
Ticker heartbeatTicker;              // 心跳定时器 / Heartbeat timer
bool isConnected = false;            // 连接状态 / Connection status

// 处理移动命令 / Handle movement command
void handleMove(int distance) {
    // 验证距离范围 / Validate distance range
    if (distance > MAX_DISTANCE || distance < MIN_DISTANCE) {
        server.send(HTTP_BAD_REQUEST, "text/plain", "Invalid distance value");
        return;
    }
    
    // 发送移动命令 / Send movement command
    Serial.println("Move:" + String(distance));
    server.send(HTTP_OK);
}

// 处理指南针命令 / Handle compass command
void handleCompass() {
    if (!server.hasArg("value")) {
        server.send(HTTP_BAD_REQUEST, "text/plain", "Missing value parameter");
        return;
    }

    String valueString = server.arg("value");
    
    // 处理FindNorth命令 / Handle FindNorth command
    if (valueString == "FindNorth") {
        Serial.println("Turn:FindNorth");
        server.send(HTTP_OK);
        return;
    }
    
    // 验证角度值 / Validate angle value
    int degree = valueString.toInt();
    if (degree < 0 || degree > 360) {
        server.send(HTTP_BAD_REQUEST, "text/plain", "Invalid degree value");
        return;
    }
    
    // 发送转向命令 / Send turn command
    Serial.println("Turn:" + String(degree));
    server.send(HTTP_OK);
}

// 处理404错误 / Handle 404 error
void handleNotFound() {
    server.send(HTTP_NOT_FOUND, "text/plain", "404: Not Found");
}

// 发送心跳 / Send heartbeat
void sendHeartbeat() {
    if (isConnected) {
        Serial.println("heartbeat");
    }
}

// 设置WiFi / Setup WiFi
void setupWiFi() {
    Serial.print("Connecting to ");
    Serial.println(ssid);
    
    WiFi.begin(ssid, password);
    
    // 等待连接 / Wait for connection
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    
    isConnected = true;
    Serial.println("\nWiFi connected");
    Serial.println("IP address: " + WiFi.localIP().toString());
}

// 设置服务器路由 / Setup server routes
void setupServer() {
    // 静态文件路由 / Static file routes
    server.serveStatic("/", SPIFFS, "/index.html");
    server.serveStatic("/style.css", SPIFFS, "/style.css");
    server.serveStatic("/script.js", SPIFFS, "/script.js");
    server.serveStatic("/favicon.ico", SPIFFS, "/favicon.png");

    // 移动控制路由 / Movement control routes
    server.on("/forwards5", []() { handleMove(5); });
    server.on("/forwards20", []() { handleMove(20); });
    server.on("/backwards5", []() { handleMove(-5); });
    server.on("/backwards20", []() { handleMove(-20); });
    
    // 方向控制路由 / Direction control routes
    server.on("/compass", handleCompass);
    
    // 404错误处理 / 404 error handling
    server.onNotFound(handleNotFound);
    
    // 启动服务器 / Start server
    server.begin();
    Serial.println("HTTP server started");
}

// 初始化设置 / Setup initialization
void setup() {
    // 初始化串口 / Initialize serial
    Serial.begin(9600);
    Serial.println("\nInitializing...");

    // 初始化文件系统 / Initialize file system
    if (!SPIFFS.begin()) {
        Serial.println("Error: SPIFFS Mount Failed");
        return;
    }

    // 设置WiFi / Setup WiFi
    setupWiFi();
    
    // 设置服务器 / Setup server
    setupServer();
    
    // 启动心跳 / Start heartbeat
    heartbeatTicker.attach(1, sendHeartbeat);
}

// 主循环 / Main loop
void loop() {
    // 处理客户端请求 / Handle client requests
    server.handleClient();
    
    // 检查WiFi连接 / Check WiFi connection
    if (WiFi.status() != WL_CONNECTED) {
        isConnected = false;
        Serial.println("WiFi connection lost");
        setupWiFi();  // 尝试重新连接 / Try to reconnect
    }
    
    // 防止看门狗重置 / Prevent watchdog reset
    yield();
}