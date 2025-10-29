//ESP32-WROOM-DA multi-sensor and data transmission system

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <TimeLib.h>

#include "12864_display.h"
#include "SD_manager.h"
#include "Time_manager.h"
#include "Esc_Telemetry.h"
#include "Pwm_reader.h"
#include "My_ads1115_sensor.h"
#include "helper_functions_up.h"

const char* ssid = "BioInBot_Lab";
const char* password = "11223344";
String udpAddress = "192.168.1.100";  // target udp ip address
int udpPort = 12345;  // target udp port
hw_timer_t *timer = NULL;  // refresh the OLED

MCU_Up_Data myData;
bool start_log = false;  // data sending status
bool start_wifi_broadcast = false;  // wifi data sending status
bool start_serial_echo = false;  // print data on Serial 1
uint32_t start_record_lt = 0;  // start recording local time ms
uint32_t DEFAULT_TIME = 1357041600;  // Jan 1 2013
uint32_t screen_fresh_cnt = 0;
uint32_t lastDataUpdate = 0;
uint32_t lastSensorFastUpdate = 0;
uint32_t lastSensorSlowUpdate = 0;
String log_headline = "GlobalTime,LocalTime,EscCurrent,EscVoltage,EscPower,EscTemperature,Command,MotorRpm,MotorForce,AccelerationX,AccelerationY,AccelerationZ,GyroscopeX,GyroscopeY,GyroscopeZ,MagnetX,MagnetY,MagnetZ";
float mcu_down_temperature = 0;

char serial_cmd[64];
uint8_t serial_cmd_index = 0;

WiFiUDP udp;
MyDisplay myScreen;
SDCard mySd;
MyTimer myClock;
MyADS1115Sensor myADC;
MyEscTelemetry myEsc;
MyPwmReader myReceiver;

/* Setup function */
void wifi_init() {
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  Serial.print("Local IP: ");
  Serial.println(WiFi.localIP());
}

/* Callback function */
void onTimer() {
  if (screen_fresh_cnt < 800) {
    myScreen.set_Line1("T:" + myClock.getCurrentTime());
    myScreen.set_Line2("cur:"+String(myData.lastCur,2)+","+"vol:"+String(myData.lastVol,2));
    myScreen.set_Line3("pwr:"+String(myData.lastPwr,2)+","+"thr:"+String(myData.lastThr,2));
    myScreen.set_Line4("rpm:"+String(myData.lastRpm,2)+","+"cmd:"+String(myData.lastCmd,2));
    myScreen.set_Line5("ESCtmp:"+String(myData.lastEscTmp,2));
    if (start_wifi_broadcast) {
      myScreen.set_Checkbox(true);
    }
    else {
      myScreen.set_Checkbox(false);
    }
    myScreen.OLED_UpdateRam();
    myScreen.OLED_Refresh();
    screen_fresh_cnt++;
  }
  else {
    myScreen.OLED_Reset_Display();
    screen_fresh_cnt = 0;
  }
}

void onSerialCmdEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (serial_cmd_index >= 63) {
      serial_cmd_index = 0;
      serial_cmd[0] = '\0';
      continue;
    }
    if (inChar == ' '){
      continue;
    }
    serial_cmd[serial_cmd_index] = inChar;
    serial_cmd_index++;
    if (inChar == '\n') {
      serial_cmd[serial_cmd_index] = '\0';
      parse_serial_cmd(String(serial_cmd));
      serial_cmd_index = 0;
      serial_cmd[0] = '\0';
    }
  }
}

void sendData() {
  sprintf(myData.glbT, "%02d:%02d:%02d", hour(), minute(), second());
  myData.lcaT = (millis() - start_record_lt) / 1000.0f;
  char resultStr[300];
  convert_data_to_string(myData, resultStr);
  if (start_log) {
    mySd.logMessage(resultStr);
  }
  if (start_serial_echo) {
    Serial.println(resultStr);
  }
  if (start_wifi_broadcast) {
    if (WiFi.status() == WL_CONNECTED){
      String jsonStr;
      convert_data_to_json(myData, jsonStr);
      udp.beginPacket(udpAddress.c_str(), udpPort);
      udp.print(jsonStr);
      udp.endPacket();
    }
    else{
      Serial.println("WiFi connection lost! Retry to connect.");
      WiFi.begin(ssid, password);
    }
  }
}


void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n===== ESP32 Motor Test MCU (Up) Initializing =====");

  Serial2.begin(115200);  // Rx-16, Tx-17
  while(!Serial2)
    delay(10);

  Serial.println("Initializing submodules...");
  String init_message = "";
  wifi_init();
  init_message += myScreen.init();
  init_message += mySd.init();
  init_message += myClock.init(udp);
  init_message += myADC.init();
  init_message += myEsc.init();
  myReceiver.begin();
  if(init_message.length() < 2){
    Serial.println("Submodules initialized.");
  }
  else{
    Serial.println(init_message);
    Serial.println("Abort.");
    while(1);
  }

  mySd.setFolderName(myClock.getCurrentDate());
  mySd.setFileName(myClock.getCurrentTime());
  mySd.setHeadLine(log_headline);
  delay(50);

  timer = timerBegin(1000000);
  if(timer == NULL) {
    Serial.println("OLED Timer initialization failed!");
    while(1);
  }
  timerAlarm(timer, 400000, true, 0);
  timerAttachInterrupt(timer, &onTimer);
  Serial.println("Screen timer initialized.");
  delay(50);

  Serial.println("===== System Initialization Done. =====\n");

  Serial.println("Wait for MCU (down) startup.");
  while(!Serial2.available()) delay(50);
  unsigned long time_now = now();
  Serial2.println("Time:"+String(time_now));
  Serial.println("MCU (down) connected.");
  delay(50);
  Serial2.println("Reset_Folder_Name");
  delay(50);
  Serial2.println("Reset_File_Name");
  delay(50);
  Serial2.println("UDP_IP:"+udpAddress);
  delay(50);
  Serial2.println("UDP_Port:"+String(udpPort));
  delay(50);

  start_record_lt = millis();
  flush_serial2_buffer();
}

void loop() {
  onSerialCmdEvent();

  if(millis() - lastDataUpdate > 501) {
    lastDataUpdate = millis();
    sendData();
  }

  if(millis() - lastSensorSlowUpdate > 151) {
    lastSensorSlowUpdate = millis();
    myADC.readPower(myData.lastVol, myData.lastCur, myData.lastPwr);
    myADC.readForce(myData.lastThr);

    float _tmp_ax, _tmp_ay, _tmp_az, _tmp_gx, _tmp_gy, _tmp_gz, _tmp_mx, _tmp_my, _tmp_mz, _temperature;
    if(read_serial2_data(_tmp_ax, _tmp_ay, _tmp_az, _tmp_gx, _tmp_gy, _tmp_gz, _tmp_mx, _tmp_my, _tmp_mz, _temperature)) {
      myData.lastAx = _tmp_ax; myData.lastAy = _tmp_ay; myData.lastAz = _tmp_az;
      myData.lastGx = _tmp_gx; myData.lastGy = _tmp_gy; myData.lastGz = _tmp_gz;
      myData.lastMx = _tmp_mx; myData.lastMy = _tmp_my; myData.lastMz = _tmp_mz;
      mcu_down_temperature = _temperature;
    }
  }

  if(millis() - lastSensorFastUpdate > 97) {
    lastSensorFastUpdate = millis();
    myEsc.update();
    if(myEsc.isValid()){
      myData.lastRpm = myEsc.getRPMRaw();
      myData.lastEscTmp = myEsc.getTemperature();
    }
    if(myReceiver.hasNewData()){
      myData.lastCmd = myReceiver.getThrottle();
    }
  }
}


void parse_serial_cmd(String command) {
  command.trim();
  if (command == "Start_Record") {
    start_log = true;
    start_record_lt = millis();
    Serial.println("SD data recording started.");
  }
  else if (command == "Stop_Record") {
    if(start_log){
      start_record_lt = millis();
      mySd.flushToCard();
    }
    start_log = false;
    Serial.println("SD data recording stopped.");
  }
  else if (command == "Clear_Log_Files"){
    mySd.clearLogs("/");
    Serial.println("SD card cleared.");
  }
  else if (command == "Start_UDP_Broadcast"){
    start_wifi_broadcast = true;
    Serial.println("Data broadcasting started.");
  }
  else if (command == "Stop_UDP_Broadcast"){
    start_wifi_broadcast = false;
    Serial.println("Data broadcasting stopped.");
  }
  else if (command == "Start_Echo"){
    start_serial_echo = true;
    Serial.println("Serial data echo started.");
  }
  else if (command == "Stop_Echo"){
    start_serial_echo = false;
    Serial.println("Serial data echo stopped.");
  }
  else if (command == "Get_IP") {
    if(WiFi.status() == WL_CONNECTED){
      Serial.println(WiFi.localIP());
    }
    else {
      Serial.println("WiFi not connected.");
    }
  }
  else if (command.startsWith("UDP_IP:")) {  // set udp target IP address
    String valueStr = command.substring(7);
    if(isValidIP(valueStr)){
      udpAddress = valueStr;
      Serial.println("UDP target IP updated: " + valueStr);
      Serial2.println("UDP_IP:"+udpAddress);
    }
  }
  else if (command.startsWith("UDP_Port:")) {  // set udp target port number
    String valueStr = command.substring(9);
    int port_interger = valueStr.toInt();
    if(port_interger >= 1024){
      udpPort = port_interger;
      Serial.println("UDP target port updated: " + valueStr);
      Serial2.println("UDP_Port:"+valueStr);
    }
  }
  else if (command == "H") {
    Serial.println("Supported command:");
    Serial.println("1. Start_Record  /  Stop_Record");
    Serial.println("2. Clear_Log_Files");
    Serial.println("3. Start_UDP_Broadcast  /  Stop_UDP_Broadcast");
    Serial.println("4. Get_IP");
    Serial.println("5. UDP_IP:192.168.31.109");
    Serial.println("6. UDP_Port:1234");
    Serial.println("7. Start_Echo  /  Stop_Echo");
  }
  else if (command.length() > 0) {
    Serial.println("Unknown command: " + command);
    Serial.println("Type H to get supported commands.");
  }
}