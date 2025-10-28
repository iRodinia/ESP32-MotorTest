//ESP32-WROOM-DA multi-sensor system

#include <Arduino.h>
#include <Wire.h>  // Must include Wire here, otherwise all .h files won't include Wire
#include <WiFi.h>
#include <WiFiUdp.h>
#include <TimeLib.h>

#include "SD_manager.h"
#include "12864_display.h"
#include "IMU_GY85_manager.h"
#include "helper_functions_down.h"

const char* wifi_ssid = "BioInBot_Lab";
const char* wifi_pswd = "11223344";
String udpAddress = "192.168.31.240";  // receiver IP
int udpPort = 8888;  // receiver port
hw_timer_t *timer = NULL;  // refresh the OLED

MCU_Down_Data myData;
bool start_log = false;  // data sending status
bool start_wifi_broadcast = false;  // wifi data sending status
bool start_serial_echo = false;  // print data on Serial 1
uint32_t DEFAULT_TIME = 1357041600;  // Jan 1 2013
uint32_t screen_fresh_cnt = 0;
uint32_t startRecordLT = 0;  // start recording local time
uint32_t lastDataUpdate = 0;
uint32_t lastSensorFastUpdate = 0;
uint32_t lastSensorSlowUpdate = 0;
String log_headline = "GlobalTime,LocalTime,AccelerationX,AccelerationY,AccelerationZ,GyroscopeX,GyroscopeY,GyroscopeZ,MagnetX,MagnetY,MagnetZ";

char serial_cmd[64];  // store the received cmd from main serial
uint8_t serial_cmd_index = 0;
char serial2_cmd[64];  // store the received cmd from MCU (up)
uint8_t serial2_cmd_index = 0;

WiFiUDP udp;
MyDisplay myScreen;  // scl-33, sda-32
MyIMU_GY85 mySensor;  // scl-22, sda-21
SDCard mySd(18, 23, 19, 5);  //miso = 23, mosi = 19, wrong wiring...

void onTimer() {
  if (screen_fresh_cnt < 800) {
    myScreen.set_Line1("T:" + String(myData.lcaT,2));
    myScreen.set_Line2("acc:"+String(myData.lastAx,1)+","+String(myData.lastAy,1)+","+String(myData.lastAz,1));
    myScreen.set_Line3("gyo:"+String(myData.lastGx,1)+","+String(myData.lastGy,1)+","+String(myData.lastGz,1));
    myScreen.set_Line4("mag:"+String(myData.lastMx,1)+","+String(myData.lastMy,1)+","+String(myData.lastMz,1));
    myScreen.set_Line5("tmp:"+String(myData.lastTmp,2));
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
    serial_cmd_index;
    if (inChar == '\n') {
      serial_cmd[serial_cmd_index] = '\0';
      parse_serial_cmd(String(serial_cmd));
      serial_cmd_index = 0;
      serial_cmd[0] = '\0';
    }
  }
}

void onSerial2CmdEvent() {
  while (Serial2.available()) {
    char inChar = (char)Serial2.read();
    if (serial2_cmd_index >= 63) {
      serial2_cmd_index = 0;
      serial2_cmd[0] = '\0';
      continue;
    }
    if (inChar == ' '){
      continue;
    }
    serial2_cmd[serial2_cmd_index] = inChar;
    serial2_cmd_index;
    if (inChar == '\n') {
      serial2_cmd[serial2_cmd_index] = '\0';
      parse_serial_cmd(String(serial2_cmd));
      serial2_cmd_index = 0;
      serial2_cmd[0] = '\0';
    }
  }
}

void sendData() {
  if (timeStatus() == timeNotSet) {
    sprintf(myData.glbT, "N/A");
  }
  else {
    sprintf(myData.glbT, "%02d:%02d:%02d", hour(), minute(), second());
  }
  myData.lcaT = (millis() - startRecordLT) / 1000.0f;
  
  char resultStr[250];
  convert_data_to_string(myData, resultStr);
  Serial2.println(resultStr);  // send data via Serial 2
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
      WiFi.begin(wifi_ssid, wifi_pswd);
    }
  }
}


void setup(){
  Serial.begin(115200);  // usb
  while (!Serial)
    delay(10);
  Serial.println("\n##### Start the Data Recording Board (Down) #####");

  Serial2.begin(115200);  // Rx-16, Tx-17
  while(!Serial2)
    delay(10);

  Serial.println("Set I2C to fast mode (400kHz).");
  Wire.setClock(400000);

  Serial.print("Trying to conncet to Lab WiFi.");
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
  }
  else{
    Serial.println("\nWiFi not connected.");
  }

  String init_message = "";
  init_message += myScreen.init();
  init_message += mySensor.init();
  init_message += mySd.init();
  if(init_message.length() < 2){
    Serial.println("Submodules initialized.");
  }
  else{
    Serial.println(init_message);
    Serial.println("Abort.");
    while(1);
  }
  mySd.setHeadLine(log_headline);
  mySensor.calibrateAccel();

  timer = timerBegin(1000000);
  if(timer == NULL) {
    Serial.println("OLED Timer initialization failed!");
    while(1);
  }
  timerAlarm(timer, 400000, true, 0);
  timerAttachInterrupt(timer, &onTimer);
  Serial.println("Screen timer initialized.");
  delay(50);

  Serial.println("##### All modules initialized #####");
  
  startRecordLT = millis();
}


void loop(){
  onSerialCmdEvent();
  onSerial2CmdEvent();

  if(millis() - lastDataUpdate > 203) {
    lastDataUpdate = millis();
    sendData();
  }

  if(millis() - lastSensorSlowUpdate > 97) {
    lastSensorSlowUpdate = millis();
    float _mx, _my, _mz;
    mySensor.readMagnetRaw(_mx, _my, _mz);
    myData.lastMx = _mx; myData.lastMy = -_my; myData.lastMz = -_mz;
    mySensor.readTemperatureRaw(myData.lastTmp);
  }

  if(millis() - lastSensorFastUpdate > 51) {
    lastSensorFastUpdate = millis();
    float _ax, _ay, _az, _gx, _gy, _gz;
    mySensor.readAccelerationRaw(_ax, _ay, _az);
    myData.lastAx = _ax; myData.lastAy = -_ay; myData.lastAz = -_az;
    mySensor.readGyroRaw(_gx, _gy, _gz);
    myData.lastGx = _gx; myData.lastGy = -_gy; myData.lastGz = -_gz;
  }
}

void parse_serial_cmd(String command) {
  command.trim();
  if (command == "Start_Record") {
    start_log = true;
    startRecordLT = millis();
    Serial.println("SD data recording started.");
  }
  else if (command == "Stop_Record") {
    if(start_log){
      startRecordLT = millis();
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
  else if (command == "Reset_Folder_Name"){
    if (timeStatus() != timeNotSet) {
      char timeStr[12];
      sprintf(timeStr, "%04d-%02d-%02d", year(), month(), day());
      mySd.setFolderName(String(timeStr));
      Serial.println("SD folder name reset to date.");
    }
    else {
      Serial.println("SD folder name reset unsuccessful.");
    }
  }
  else if (command == "Reset_File_Name"){
    if (timeStatus() != timeNotSet) {
      char timeStr[10];
      sprintf(timeStr, "%02d:%02d:%02d", hour(), minute(), second());
      mySd.setFileName(String(timeStr));
      Serial.println("SD file name reset to time (H:M:S).");
    }
    else {
      Serial.println("SD file name reset unsuccessful.");
    }
  }
  else if (command == "Get_IP") {
    if(WiFi.status() == WL_CONNECTED){
      Serial.println(WiFi.localIP());
    }
    else {
      Serial.println("WiFi not connected.");
    }
  }
  else if (command == "Connect_WiFi") {
    if(WiFi.status() != WL_CONNECTED){
      WiFi.begin(wifi_ssid, wifi_pswd);
    }
  }
  else if (command.startsWith("UDP_IP:")) {  // set udp target IP address
    String valueStr = command.substring(7);
    if(isValidIP(valueStr)){
      udpAddress = valueStr;
      Serial.println("UDP target IP updated to: " + valueStr);
    }
  }
  else if (command.startsWith("UDP_Port:")) {  // set udp target port number
    String valueStr = command.substring(9);
    int port_interger = valueStr.toInt();
    if(port_interger >= 1024){
      udpPort = port_interger;
      Serial.println("UDP target port updated: " + valueStr);
    }
  }
  else if (command.startsWith("Time:")) {
    String valueStr = command.substring(5);
    unsigned long time_interger = valueStr.toInt();
    if (time_interger > DEFAULT_TIME) {
      setTime(time_interger);
    }
  }
  else if (command.length() > 0) {
    Serial.println("Unknown command: " + command);
  }
}