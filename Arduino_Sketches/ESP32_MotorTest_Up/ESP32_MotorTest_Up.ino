//ESP32-WROOM-DA multi-sensor and data transmission system
#include <Arduino.h>
#include <Wire.h>

#include "helper_functions_up.h"
// #include "12864_display.h"
#include "SD_manager.h"
#include "Serial_manager.h"
#include "My_ads1115_sensor.h"

#define LED_PIN 2
#define LED_TOGGLE() digitalWrite(LED_PIN, digitalRead(LED_PIN) ^ 1)

#define MOTOR_POLE_PAIR 7

MCU_Up_Data myData;
bool start_log = false;  // data sending status

uint32_t start_record_lt = 0;  // start recording local time ms
// uint32_t lastScreenRefresh = 0;
uint32_t lastDataRecord = 0;
uint32_t lastSensorUpdate = 0;
String log_headline = "LocalTime,EscCurrent,EscVoltage,EscPower,Command,MotorRpm,MotorForce,EscTemperature";

// InfoDisplayUp myScreen;
SDCard mySd;
MyADS1115Sensor myADC;

void sendData() {
  char resultStr[300];
  convert_data_to_string(myData, resultStr);
  Serial.println(resultStr);
  if (start_log) {
    mySd.logMessage(resultStr);
  }

  LED_TOGGLE();  // LED blink means data sending
}

void setup() {
  String init_message = "";
  init_message += initAllSerials();
  Serial.println("\n===== ESP32 Motor Test MCU (Up) Initializing =====");

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  Wire.begin();
  delay(50);
  Wire.setClock(400000);  // fast mode

  // init_message += myScreen.init();
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

  if(current_time - lastSensorUpdate > 120) {
    lastSensorUpdate = current_time;
    myADC.readPower(myData.lastVol, myData.lastCur, myData.lastPwr);
    myADC.readForce(myData.lastThr);

    myData.lastRpm = float(myEscData.erpm) / MOTOR_POLE_PAIR;
    myData.lastEscTmp = myEscData.temperature;
    myData.lastCmd = (float(receiver_channels[2]) - CMD_MIN) / (CMD_MAX - CMD_MIN);  // throttle channel is No.3, which is channel[2]
  }

  // if(current_time - lastScreenRefresh > 150) {
  //   lastScreenRefresh = current_time;
  //   myData.lcaT = (millis() - start_record_lt) / 1000.0f;
  //   myScreen.updateInfo(myData);
  //   myScreen.refresh();
  // }

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
  else if (command == "H") {
    Serial.println("Supported command:");
    Serial.println("1. Start_Record");
    Serial.println("2. Stop_Record");
  }
  else if (command.length() > 0) {
    Serial.println("Unknown command: " + command);
    Serial.println("Type H to get supported commands.");
  }
}