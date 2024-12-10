/******************** 库引入 Library Inclusion ********************/
#include <ESP8266WiFi.h>        // 提供WiFi功能 / Provides WiFi functionality
#include <ESP8266WebServer.h>   // 提供Web服务器功能 / Provides web server functionality
#include <FS.h>                 // 提供文件系统功能 / Provides file system functionality (SPIFFS)

/******************** 全局变量 Global Variables ********************/
const char* ssid = "SOURCE";         // WiFi网络名称 / WiFi network name
const char* password = "Pelle!23";   // WiFi密码 / WiFi password

// 创建Web服务器实例，监听80端口（HTTP默认端口）
// Create web server instance on port 80 (default HTTP port)
ESP8266WebServer server(80);

/******************** 系统初始化 System Initialization ********************/
void setup() {
  // 初始化串口通信，波特率9600 / Initialize serial communication at 9600 baud
  Serial.begin(9600);

  // 初始化SPIFFS文件系统 / Initialize SPIFFS file system
  if (!SPIFFS.begin()) {
    Serial.println("Error while mounting SPIFFS");
    return;
  }

  // 连接WiFi网络 / Connect to WiFi network
  Serial.print("\nConnecting to " + (String)ssid);
  WiFi.begin(ssid, password);
  // 等待WiFi连接成功 / Wait for WiFi connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nIP address: " + WiFi.localIP().toString());

  // 设置静态文件路由 / Set up static file routes
  server.serveStatic("/", SPIFFS, "/index.html");         // 主页 / Homepage
  server.serveStatic("/style.css", SPIFFS, "/style.css"); // CSS样式文件 / CSS style file
  server.serveStatic("/script.js", SPIFFS, "/script.js"); // JavaScript文件 / JavaScript file
  server.serveStatic("/favicon.ico", SPIFFS, "/favicon.png"); // 网站图标 / Website icon

  // 设置移动控制路由 / Set up movement control routes
  server.on("/forwards5", [](){ handleMove(5); });      // 前进5厘米 / Move forward 5cm
  server.on("/forwards20", [](){ handleMove(20); });    // 前进20厘米 / Move forward 20cm
  server.on("/backwards5", [](){ handleMove(-5); });    // 后退5厘米 / Move backward 5cm
  server.on("/backwards20", [](){ handleMove(-20); });  // 后退20厘米 / Move backward 20cm
  
  // 设置方向控制路由 / Set up direction control routes
  server.on("/compass", handleCompass);    // 罗盘控制 / Compass control
  server.on("/findNorth", handleFindNorth);// 寻找北方 / Find north direction

  // 设置404错误处理 / Set up 404 error handling
  server.onNotFound(handleNotFound);

  // 启动服务器 / Start the server
  server.begin();
}

/******************** 主循环 Main Loop ********************/
void loop() {
  // 处理客户端请求 / Handle client requests
  server.handleClient();
}

/******************** 路由处理函数 Route Handlers ********************/
// 处理404错误 / Handle 404 errors
void handleNotFound() {
  server.send(404, "text/plain", "404: Not Found");
}

// 处理移动命令 / Handle movement commands
void handleMove(int distance) {
  // 发送移动指令到串口 / Send movement command to serial
  Serial.println("Move:" + String(distance));
  // 发送成功响应到客户端 / Send success response to client
  server.send(200);
}

// 处理罗盘命令 / Handle compass commands
void handleCompass() {
  if (server.hasArg("value")) {                     // 检查是否有value参数 / Check if value parameter exists
    String valueString = server.arg("value");       // 获取转向角度值 / Get turning angle value
    Serial.println("Turn:" + valueString);          // 发送转向命令到串口 / Send turn command to serial
  }
  server.send(200);
}

// 处理寻找北方命令 / Handle find north command
void handleFindNorth() {
  // 发送寻找北方命令到串口 / Send find north command to serial
  Serial.println("find:North");
  // 发送成功响应到客户端 / Send success response to client
  server.send(200);
}