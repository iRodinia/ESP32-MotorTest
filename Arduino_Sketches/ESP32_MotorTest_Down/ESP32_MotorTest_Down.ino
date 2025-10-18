#include <Arduino.h>
#include <Wire.h>  // Must include Wire here, otherwise all .h files won't include Wire
#include <WiFi.h>
#include <WiFiUdp.h>
#include <TimeLib.h>

#include "SD_manager.h"
#include "12864_display.h"
#include "IMU_GY85_manager.h"
#include "Kalman_filter_GY85.h"
#include "helper_functions.h"

/*
MCU (down) Functionality:
Measure the acceleration, gyro and gesture in the world frame (ENU), and measure angular velocity in the body frame (FLU);
Record and send the data by 10Hz;
Synchronize time through UART;
Send data through WiFi if available;
Communicate with MCU (up) using UART, receive commands and send the data;
Display the real-time data on the screen.
*/

const char* wifi_ssid = "BioInBot_Lab";
const char* wifi_pswd = "11223344";
WiFiUDP udp;
const char* udpAddress = "192.168.31.240"; // 接收端 IP 地址
const int udpPort = 8888;                 // 接收端端口

MyDisplay myscreen;  // scl-33, sda-32
MyIMU_GY85 mysensor;  // scl-22, sda-21
SDCard mysd(18, 23, 19, 5);  // sck = 18; miso = 23; mosi = 19; cs = 5;
GY85_KalmanFilter myfilter;
// Serial2： rx-16, tx-17

hw_timer_t *timer = NULL;
bool cmd_received = false;
bool start_log = false;  // data sending status
bool start_wifi_broadcast = false;  // wifi data sending status
uint32_t DEFAULT_TIME = 1357041600;  // Jan 1 2013
uint32_t startRecordLT = 0;
uint32_t lastDataLoopT = 0;
uint32_t lastSensorFastUpdate = 0;
uint32_t lastSensorSlowUpdate = 0;
float lastVx = 0; float lastVy = 0; float lastVz = 0;
float lastAx = 0; float lastAy = 0; float lastAz = 0;
float lastRoll = 0; float lastPitch = 0; float lastYaw = 0;
float lastWx = 0; float lastWy = 0; float lastWz = 0;

float lastMx = 0; float lastMy = 0; float lastMz = 0;
String up_cmd = "";  // store the received cmd from MCU (up)

void onTimer() {
  myscreen.set_Line1("T:" + getCurrentTime());
  myscreen.set_Line2("vel:"+String(lastVx,1)+","+String(lastVy,1)+","+String(lastVz,1));
  myscreen.set_Line3("acc:"+String(lastAx,1)+","+String(lastAy,1)+","+String(lastAz,1));
  myscreen.set_Line4("att:"+String(lastRoll,1)+","+String(lastPitch,1)+","+String(lastYaw,1));
  myscreen.set_Line5("gyo:"+String(lastWx,1)+","+String(lastWy,1)+","+String(lastWz,1));
  if(start_wifi_broadcast){
    myscreen.set_Checkbox(true);
  }
  myscreen.OLED_UpdateRam();
  myscreen.OLED_Refresh();
}

void setup(){
  Serial.begin(115200);  // usb
  while (!Serial)
    delay(10);

  Serial.printf("\n");
  Serial.println("##### Start the Data Recording Board (Down) #####");

  Serial.println("Starting the data serial (Serial 2: Rx-16, Tx-17).");
  Serial2.begin(115200);  // Rx-16, Tx-17
  while(!Serial2)
    delay(10);
  Serial.println("Initialized data serial.");
  up_cmd.reserve(200);

  Serial.println("Starting the OLED screen.");
  myscreen.init();
  timer = timerBegin(1000000);
  if(timer == NULL) {
    Serial.println("OLED Timer initialization failed!");
    while(1);
  }
  timerAlarm(timer, 400000, true, 0);
  timerAttachInterrupt(timer, &onTimer);
  Serial.println("Screen initialized.");
  delay(50);

  Serial.print("Trying to conncet to Lab WiFi.");
  WiFi.mode(WIFI_STA);
  uint8_t wifi_connect_tms = 0;
  WiFi.begin(wifi_ssid, wifi_pswd);
  while(WiFi.status() != WL_CONNECTED && wifi_connect_tms < 5){
    Serial.print(".");
    wifi_connect_tms += 1;
    delay(200);
  }
  if(WiFi.status() == WL_CONNECTED){
    Serial.println("\nConnected to the Lab WiFi.");
    Serial.print("Local MCU (down) IP: ");
    Serial.println(WiFi.localIP());
    udp.begin(0);
  }
  else{
    Serial.println("\nWiFi not connected.");
  }

  Serial.println("Set I2C to normal mode (100kHz).");
  Wire.setClock(100000);
  
  Serial.println("Starting the SD card and sensor.");
  int init_count = 0;
  while(!mysd.checkCardStatus() && init_count < 20){
    mysd.init();
    delay(200);
    init_count += 1;
  }
  init_count = 0;
  while(!mysensor.checkInitStatus() && init_count < 20){
    mysensor.init();
    delay(200);
    init_count += 1;
  }
  if(!(mysd.checkCardStatus() && mysensor.checkInitStatus())){
    Serial.println("Device initialization failed!");
    while(1);
  }
  Serial.println("All modules initialized.");
  delay(50);

  Serial.println("Starting the Kalman filter.");
  myfilter.init();
  Serial.println("Kalman filter initialized.");

  Serial.println("Data Recording Board (Down) Initialization Done.");
  delay(50);
}


void loop(){
  ////////////// MCU (up) command /////////////////
  if (Serial2.available()) {
    Serial2Event();
  }
  if(cmd_received){
    String _command = up_cmd;
    up_cmd = "";
    cmd_received = false;
    Process_Command(_command);
  }
  /////////////////////////////////////////////////

  if (millis() - lastDataLoopT > 100) {
    lastDataLoopT = millis();
    myfilter.getVelocity(lastVx, lastVy, lastVz);
    myfilter.getAcceleration(lastAx, lastAy, lastAz);
    myfilter.getAttitude(lastRoll, lastPitch, lastYaw);
    myfilter.getAngularVelocity(lastWx, lastWy, lastWz);
    char resultStr[200];
    sprintf(resultStr, "%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f",
          lastVx, lastVy, lastVz, lastAx, lastAy, lastAz, lastRoll, lastPitch, lastYaw, lastWx, lastWy, lastWz);

    if(start_log){
      String _glb_t = (timeStatus() == timeNotSet)? "N/A" : getCurrentHmsTime();
      String _lca_t = String((millis() - startRecordLT)/1000.0f, 2);
      if(mysd.checkFileStatus()){
        mysd.record(_glb_t + " " + _lca_t + " " + String(resultStr));
      }
      Serial2.println(String(resultStr));
    }

    if(start_wifi_broadcast && WiFi.status() == WL_CONNECTED){
      udp.beginPacket(udpAddress, udpPort);
      udp.print(resultStr);
      udp.endPacket();
    }
  }

  if(millis() - lastSensorSlowUpdate > 50) {
    lastSensorSlowUpdate = millis();
    mysensor.readMagnetRaw(lastMx, lastMy, lastMz);
  }

  uint32_t _dt = millis() - lastSensorFastUpdate;
  if(_dt > 20) {
    lastSensorFastUpdate = millis();
    float ax = 0; float ay = 0; float az = 0;
    float gx = 0; float gy = 0; float gz = 0;
    mysensor.readAccelerationRaw(ax, ay, az);
    mysensor.readGyroRaw(gx, gy, gz);
    myfilter.update(ax, ay, az, gx, gy, gz,
      lastMx, lastMy, lastMz, _dt/1000.0f);
  }
}

void Process_Command(String command) {
  command.trim();
  if (command == "Start_rcd") {
    start_log = true;
    startRecordLT = millis();
    Serial.println("Data recording started.");
  }
  else if (command == "Stop_rcd") {
    start_log = false;
    Serial.println("Data recording stopped.");
  }
  else if (command == "Create_f"){
    String data_headline = "global_t local_t vel_x vel_y vel_z acc_x acc_y acc_z roll pitch yaw omega_x omega_y omega_z";
    int file_flag;
    if(timeStatus() != timeNotSet){
      mysd.set_folder_name(getCurrentDate());
      file_flag = mysd.create_file(data_headline, "MTest_"+getCurrentHmsTime());
    }
    else{
      file_flag = mysd.create_file(data_headline);
    }
    if(file_flag < 0){
      Serial.println("SD file creation failed.");
    }
    else{
      Serial.println("SD file creation succeeded.");
    }
  }
  else if (command.startsWith("T:")) {
    String valueStr = command.substring(2);
    unsigned long time_interger = valueStr.toInt();
    if(time_interger >= DEFAULT_TIME){
      setTime(time_interger);
      Serial.println("Global time updated.");
      Serial2.println("T_ack");
    }
  }
  else if (command.length() > 0) {
    Serial.print("Unknown command: ");
    Serial.println(command);
  }
}
