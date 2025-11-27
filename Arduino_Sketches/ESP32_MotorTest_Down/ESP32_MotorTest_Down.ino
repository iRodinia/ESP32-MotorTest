//ESP32-WROOM-DA multi-sensor system

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <HardwareSerial.h>

#include "SD_manager.h"
#include "IMU_Info_display.h"
#include "IMU_GY85_manager.h"
#include "helper_functions_down.h"

#define SERIAL2_TX 25
#define SERIAL2_RX 4

const char* wifi_ssid = "BioInBot_Lab";
const char* wifi_pswd = "11223344";
String udpAddress = "192.168.31.240";  // receiver IP
int udpPort = 8888;  // receiver port

MCU_Down_Data myData;
bool start_log = false;  // data sending status
bool start_wifi_broadcast = false;  // wifi data sending status
bool start_serial_echo = false;  // print data on Serial 1

uint32_t startRecordLT = 0;  // start recording local time
uint32_t lastDataRecord = 0;
uint32_t lastSensorUpdate = 0;
String log_headline = "LocalTime,AccelerationX,AccelerationY,AccelerationZ,GyroscopeX,GyroscopeY,GyroscopeZ,MagnetX,MagnetY,MagnetZ";

char serial_cmd[64];  // store the received cmd from main serial
uint8_t serial_cmd_index = 0;
char serial2_cmd[64];  // store the received cmd from MCU (up)
uint8_t serial2_cmd_index = 0;

WiFiUDP udp;
InfoDisplayDown myScreen;  // scl-33, sda-32
MyIMU_GY85 mySensor;  // scl-22, sda-21
SDCard mySd(18, 23, 19, 5);  //miso = 23, mosi = 19, wrong wiring...

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
    serial2_cmd_index++;
    if (inChar == '\n') {
      serial2_cmd[serial2_cmd_index] = '\0';
      parse_serial_cmd(String(serial2_cmd));
      serial2_cmd_index = 0;
      serial2_cmd[0] = '\0';
    }
  }
}

void sendData() {
  char resultStr[250];
  convert_data_to_string(myData, resultStr);
  Serial2.printf("%s\n", resultStr);  // send data via Serial 2
  
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
  }
  if (start_log) {
    mySd.logMessage(resultStr);  // resultStr do not include '\n'
  }
}

void setup(){
  Serial.begin(115200);  // usb
  while (!Serial)
    delay(10);
  Serial.println("\n##### Start the Data Recording Board (Down) #####");

  Serial.print("Trying to conncet to Lab WiFi.");
  uint8_t wifi_connect_tms = 0;
  WiFi.begin(wifi_ssid, wifi_pswd);
  while(WiFi.status() != WL_CONNECTED && wifi_connect_tms < 10){
    Serial.print(".");
    wifi_connect_tms += 1;
    delay(200);
  }
  if(WiFi.status() == WL_CONNECTED){
    Serial.println("\nConnected to the Lab WiFi.");
    Serial.print("Local MCU (down) IP: ");
    Serial.println(WiFi.localIP());
  }

  String init_message = "";
  init_message += myScreen.init();
  init_message += mySensor.init();
  init_message += mySd.init(log_headline);
  if(init_message.length() < 2){
    Serial.println("Submodules initialized.");
  }
  else{
    Serial.println(init_message);
    Serial.println("Abort.");
    while(1);
  }
  mySensor.calibrateAccel();
  delay(50);

  Serial.println("##### All modules initialized #####");
  Serial2.begin(115200, SERIAL_8N1, SERIAL2_RX, SERIAL2_TX);
  startRecordLT = millis();
}

void loop(){
  onSerialCmdEvent();
  onSerial2CmdEvent();
  uint32_t current_time = millis();

  if(current_time - lastDataRecord > 100) {
    lastDataRecord = current_time;
    recordData();
  }

  if(current_time - lastSensorUpdate > 97) {
    lastSensorUpdate = current_time;
    float _mx, _my, _mz;
    mySensor.readMagnetRaw(_mx, _my, _mz);
    myData.lastMx = _mx; myData.lastMy = -_my; myData.lastMz = -_mz;
    mySensor.readTemperatureRaw(myData.lastTmp);

    float _ax, _ay, _az, _gx, _gy, _gz;
    mySensor.readAccelerationRaw(_ax, _ay, _az);
    myData.lastAx = _ax; myData.lastAy = -_ay; myData.lastAz = -_az;
    mySensor.readGyroRaw(_gx, _gy, _gz);
    myData.lastGx = _gx; myData.lastGy = -_gy; myData.lastGz = -_gz;
  }

  if(WiFi.status() == WL_CONNECTED) {
    myScreen.setCheckbox(true);
  }
  else {
    myScreen.setCheckbox(false);
  }
  myData.lcaT = (millis() - startRecordLT) / 1000.0f;
  myScreen.updateInfo(myData);
  myScreen.refresh();

  delay(1);
}

void parse_serial_cmd(String command) {
  command.trim();
  if (command == "Start_Record") {
    start_log = true;
    Serial.println("SD data recording started.");
  }
  else if (command == "Stop_Record") {
    if(start_log){
      mySd.flushToCard();
    }
    start_log = false;
    Serial.println("SD data recording stopped.");
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
  else if (command == "Connect_WiFi") {
    if(WiFi.status() != WL_CONNECTED){
      WiFi.disconnect();
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
  else if (command.length() > 0) {
    Serial.println("Unknown command: " + command);
  }
}