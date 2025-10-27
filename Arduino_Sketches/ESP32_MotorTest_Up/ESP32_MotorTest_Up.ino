//ESP32-WROOM-DA multi-sensor and data transmission system

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <TimeLib.h>

#include "12864_display.h"
#include "SD_manager.h"
#include "Time_manager.h"
#include "Force_sensor.h"
#include "Power_monitor.h"
#include "Esc_Telemetry.h"
#include "Pwm_reader.h"
#include "helper_functions_up.h"

#define LED_PIN 2  // LED on means initializaiton done, LED blink means data sending
#define LED_TOGGLE() digitalWrite(LED_PIN, digitalRead(LED_PIN) ^ 1)

const char* ssid = "BioInBot_Lab";
const char* password = "11223344";
String udpAddress = "192.168.1.100";  // target udp ip address
int udpPort = 12345;  // target udp port
hw_timer_t *timer1 = NULL;  // record the data
hw_timer_t *timer2 = NULL;  // refresh the OLED

MCU_Up_Data myData;
bool start_log = false;  // data sending status
bool start_wifi_broadcast = false;  // wifi data sending status
uint32_t start_record_lt = 0;  // start recording local time ms
uint32_t DEFAULT_TIME = 1357041600;  // Jan 1 2013
uint32_t screen_fresh_cnt = 0;
uint32_t lastSensorFastUpdate = 0;
uint32_t lastSensorSlowUpdate = 0;
String log_headline = "GlobalTime,LocalTime,EscCurrent,EscVoltage,EscPower,EscTemperature,Command,MotorRpm,MotorForce,AccelerationX,AccelerationY,AccelerationZ,GyroscopeX,GyroscopeY,GyroscopeZ,MagnetX,MagnetY,MagnetZ";
String mcu_down_data = "";
bool cmd_received = false;

WiFiUDP udp;
MyDisplay myScreen;
SDCard mySd;
MyTimer myClock;
MyForceSensor myGauge;
MyPowerMonitor myMonitor;
MyEscTelemetry myEsc;
MyPwmReader myReceiver;

void onTimer1() {
  sprintf(myData.glbT, "%02d:%02d:%02d", hour(), minute(), second());
  myData.lcaT = (millis() - start_record_lt) / 1000.0f;
  char resultStr[400];
  convert_data_to_string(myData, resultStr);
  if (start_log) {
    mySd.record(String(resultStr));
    LED_TOGGLE();
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

void onTimer2() {
  if (screen_fresh_cnt < 800) {
    myScreen.set_Line1("T:" + myClock.getCurrentTime());
    myScreen.set_Line2("cur:"+String(myData.lastCur,2)+","+"vol:"+String(myData.lastVol,2));
    myScreen.set_Line3("pwr:"+String(myData.lastPwr,2)+","+"thr:"+String(myData.lastThr,2));
    myScreen.set_Line4("rpm:"+String(myData.lastRpm,2)+","+"cmd:"+String(myData.lastCmd,2));
    myScreen.set_Line5("ESCtmp:"+String(myData.lastEscTmp,2));
    myScreen.OLED_UpdateRam();
    myScreen.OLED_Refresh();
    screen_fresh_cnt++;
  }
  else {
    myScreen.OLED_Reset_Display();
    screen_fresh_cnt = 0;
  }
}

void wifi_init() {
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(50);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  Serial.print("Local IP: ");
  Serial.println(WiFi.localIP());
}

void Serial2Event() {
  while (Serial2.available()) {
    char inChar = (char)Serial2.read();
    mcu_down_data += inChar;
    if (inChar == '\n') {
      cmd_received = true;
    }
  }
}






void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n===== ESP32 Motor Test MCU (Up) Initializing =====");

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial2.begin(115200);  // Rx-16, Tx-17
  while(!Serial2)
    delay(10);

  Serial.println("Initializing submodules...");
  wifi_init();
  myScreen.init();
  Serial.println("OLED inited.");
  while(!myClock.getStatus()){
    myClock.init(udp);
    delay(50);
  }
  Serial.println("Time manager inited.");
  while(!mySd.checkCardStatus()){
    mySd.init();
    delay(50);
  }
  Serial.println("SD card inited.");
  myGauge.init();
  Serial.println("Force gauge inited.");
  while(!myMonitor.isInitialized()){
    myMonitor.init();
    delay(50);
  }
  Serial.println("Power monitor inited.");
  myEsc.begin();
  Serial.println("ESC telemetry inited.");
  myReceiver.begin();
  Serial.println("PWM monitor inited.");
  
  Serial.println("===== System initialization done. =====\n");

  myGauge.calibrate();

  int retryNum = 0;
  mySd.set_folder_name(myClock.getCurrentDate());
  while(mySd.create_file(log_headline, myClock.getCurrentDateTime()) < 0 && retryNum < 10){
    retryNum++;
    delay(100);
  }
  if(!mySd.checkFileStatus()){
    Serial.println("SD file creation failed. Abort.");
    while(1);
  }

  timer1 = timerBegin(1000000);
  if(timer1 == NULL) {
    Serial.println("Data broadcasting timer initialization failed!");
    while(1);
  }
  timerAlarm(timer1, 80000, true, 0);
  timerAttachInterrupt(timer1, &onTimer1);
  Serial.println("Data broadcasting timer initialized.");
  delay(50);

  timer2 = timerBegin(1000000);
  if(timer2 == NULL) {
    Serial.println("OLED Timer initialization failed!");
    while(1);
  }
  timerAlarm(timer2, 300000, true, 0);
  timerAttachInterrupt(timer2, &onTimer2);
  Serial.println("Screen timer initialized.");
  delay(50);


  unsigned long time_now = now();
  Serial2.println("T:"+String(time_now));





  LED_TOGGLE();
  delay(50);
  
}

void loop() {
  if (Serial2.available()) {
    Serial2Event();
  }
  if(cmd_received){
    String _command = mcu_down_data;
    mcu_down_data = "";
    cmd_received = false;
    // Process_Command(_command);
  }

  if(millis() - lastSensorSlowUpdate > 97) {
    lastSensorSlowUpdate = millis();
    float _volt, _curr, _force;
    myMonitor.readPower(_volt, _curr);
    myData.lastVol = _volt;
    myData.lastCur = _curr;
    myData.lastPwr = _volt * _curr;
    myGauge.getForceCalibrated(myData.lastThr);
  }

  if(millis() - lastSensorFastUpdate > 47) {
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