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

#define LED_PIN 2  // LED on means initializaiton done, LED blink means data sending
#define LED_TOGGLE() digitalWrite(LED_PIN, digitalRead(LED_PIN) ^ 1)

const char* ssid = "BioInBot_Lab";
const char* password = "11223344";
String udpAddress = "192.168.1.100";  // target udp ip address
int udpPort = 12345;  // target udp port
hw_timer_t *timer1 = NULL;
hw_timer_t *timer2 = NULL;

bool start_log = false;  // data sending status
bool start_wifi_broadcast = false;  // wifi data sending status
uint32_t start_record_lt = 0;  // start recording local time ms
uint32_t DEFAULT_TIME = 1357041600;  // Jan 1 2013
uint32_t screen_fresh_cnt = 0;

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
    mysd.record(String(resultStr));
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












void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n===== ESP32 Motor Test MCU (Up) Initializing =====");
  
  Serial.println("Starting the data serial (Serial 2: Rx-16, Tx-17).");
  Serial2.begin(115200);  // Rx-16, Tx-17
  while(!Serial2)
    delay(10);
  Serial.println("Initialized data serial.");

  wifi_init();
  myScreen.init();
  myClock.init(udp);
  myGauge.init();
  myMonitor.init();
  myEsc.begin();
  myReceiver.begin();
  
  Serial.println("===== 系统初始化完成 =====\n");
}

void loop() {
  unsigned long currentTime = millis();
  
  // 持续读取BLHeli32数据
  read_blheli32_data();
  
  // 每0.1秒采集并发送一次数据
  if(currentTime - lastSendTime >= sendInterval) {
    lastSendTime = currentTime;
    
    // 读取AD7705数据
    sensorData.ad7705_voltage = ad7705_read_voltage();
    
    // 读取PM02数据
    read_pm02_data(sensorData.battery_voltage, sensorData.battery_current);
    
    // 读取BLHeli32数据
    sensorData.esc_rpm = escData.rpm;
    sensorData.esc_voltage = escData.voltage;
    sensorData.esc_current = escData.current;
    sensorData.esc_temp = escData.temp;
    
    // 读取PWM脉宽
    uint16_t pwm = get_pwm_width();
    if(pwm > 0) {
      sensorData.pwm_width = pwm;
    }
    
    // 记录时间戳
    sensorData.timestamp = currentTime;
    
    // 如果WiFi连接正常,发送数据
    if(WiFi.status() == WL_CONNECTED) {
      send_udp_data();
    } else {
      Serial.println("WiFi未连接,尝试重连...");
      wifi_init();
    }
  }
  
  // 简短延时
  delay(1);
}