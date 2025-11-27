//ESP32-WROOM-DA multi-sensor and data transmission system

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#include "helper_functions_up.h"
#include "12864_display.h"
#include "SD_manager.h"
#include "Serial_manager.h"
#include "My_ads1115_sensor.h"

#define MOTOR_POLE_PAIR 7

const char* ssid = "BioInBot_Lab";
const char* password = "11223344";
String udpAddress = "192.168.1.100";  // target udp ip address
int udpPort = 12345;  // target udp port

MCU_Up_Data myData;
bool start_log = false;  // data sending status
bool start_wifi_broadcast = false;  // wifi data sending status

uint32_t start_record_lt = 0;  // start recording local time ms
uint32_t lastDataRecord = 0;
uint32_t lastSensorUpdate = 0;
String log_headline = "LocalTime,EscCurrent,EscVoltage,EscPower,Command,MotorRpm,MotorForce,EscTemperature";

WiFiUDP udp;
InfoDisplayUp myScreen;
SDCard mySd;
MyADS1115Sensor myADC;

void sendData() {
  char resultStr[300];
  convert_data_to_string(myData, resultStr);
  Serial.printf("%s\n", resultStr);

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
    mySd.logMessage(resultStr);
  }
}

void setup() {
  String init_message = "";
  init_message += initAllSerials();
  Serial.println("\n===== ESP32 Motor Test MCU (Up) Initializing =====");

  Serial.print("Trying to conncet to Lab WiFi.");
  uint8_t wifi_connect_tms = 0;
  WiFi.begin(ssid, password);
  while(WiFi.status() != WL_CONNECTED && wifi_connect_tms < 10){
    Serial.print(".");
    wifi_connect_tms += 1;
    delay(200);
  }
  Serial.println(" ");
  if(WiFi.status() == WL_CONNECTED){
    Serial.println("\nConnected to the Lab WiFi.");
    Serial.print("Local MCU (down) IP: ");
    Serial.println(WiFi.localIP());
  }

  init_message += myScreen.init();
  init_message += mySd.init(log_headline);
  init_message += myADC.init();
  if(init_message.length() < 2){
    Serial.println("Submodules initialized.");
  }
  else{
    Serial.println(init_message);
    Serial.println("Abort.");
    while(1);
  }

  Serial.println("===== System Initialization Done. =====\n");
  start_record_lt = millis();
}

void loop() {
  serial0CmdEvent();
  serial1DataEvent();
  serial2DataEvent();
  uint32_t current_time = millis();

  if(current_time - lastDataRecord > 100) {
    lastDataRecord = current_time;
    sendData();
  }

  if(current_time - lastSensorUpdate > 97) {
    lastSensorUpdate = current_time;
    myADC.readPower(myData.lastVol, myData.lastCur, myData.lastPwr);
    myADC.readForce(myData.lastThr);

    myData.lastRpm = myEscData.erpm / MOTOR_POLE_PAIR;
    myData.lastEscTmp = myEscData.temperature;
    myData.lastCmd = (receiver_channels[2] - CMD_MIN) / (CMD_MAX - CMD_MIN);  // throttle channel is No.3, which is channel[2]
  }

  if(WiFi.status() == WL_CONNECTED) {
    myScreen.setCheckbox(true);
  }
  else {
    myScreen.setCheckbox(false);
  }
  myData.lcaT = (millis() - start_record_lt) / 1000.0f;
  myScreen.updateInfo(myData);
  myScreen.refresh();

  delay(1);
}


void parseSerial0Cmd(String command) {
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
  else if (command == "H") {
    Serial.println("Supported command:");
    Serial.println("1. Start_Record  /  Stop_Record");
    Serial.println("2. Start_UDP_Broadcast  /  Stop_UDP_Broadcast");
    Serial.println("3. Get_IP");
    Serial.println("4. UDP_IP:192.168.31.109");
    Serial.println("5. UDP_Port:1234");
  }
  else if (command.length() > 0) {
    Serial.println("Unknown command: " + command);
    Serial.println("Type H to get supported commands.");
  }
}