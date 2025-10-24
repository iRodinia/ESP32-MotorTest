//ESP32-WROOM-DA multi-sensor and data transmission system

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>

#include "12864_display.h"
#include "SD_manager.h"
#include "Time_manager.h"
#include "Force_sensor.h"
#include "Power_monitor.h"
#include "Esc_Telemetry.h"
#include "Pwm_reader.h"

const char* ssid = "BioInBot_Lab";
const char* password = "11223344";
WiFiUDP udp;
String udpAddress = "192.168.1.100";  // target udp ip address
int udpPort = 12345;  // target udp port
hw_timer_t *timer1 = NULL;
hw_timer_t *timer2 = NULL;

bool start_log = false;  // data sending status
bool start_wifi_broadcast = false;  // wifi data sending status
uint32_t DEFAULT_TIME = 1357041600;  // Jan 1 2013
uint32_t screen_fresh_cnt = 0;

MyDisplay myScreen;
SDCard mySd;
MyTimer myClock;
MyForceSensor myGauge;
MyPowerMonitor myMonitor;
MyEscTelemetry myEsc;
MyPwmReader myReceiver;

void onTimer1() {
  char resultStr[200];
  sprintf(resultStr, "%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f",
        lastAx, lastAy, lastAz, lastGx, lastGy, lastGz, lastMx, lastMy, lastMz, lastTmp);
  if (start_log) {
    
  }
  if (start_wifi_broadcast) {
    
  }
}

void onTimer2() {
  if (screen_fresh_cnt < 800) {
    myscreen.set_Line1("T:" + myClock.getCurrentTime());
    myscreen.set_Line2("cur:"+String(myData.lastCur,2)+","+"vol:"+String(myData.lastVol,2));
    myscreen.set_Line3("pwr:"+String(myData.lastPwr,2)+","+"thr:"+String(myData.lastThr,2));
    myscreen.set_Line4("rpm:"+String(myData.lastRpm,2)+","+"cmd:"+String(myData.lastCmd,2));
    myscreen.set_Line5("ESCtmp:"+String(myData.lastEscTmp,2));
    myscreen.OLED_UpdateRam();
    myscreen.OLED_Refresh();
    screen_fresh_cnt++;
  }
  else {
    myscreen.OLED_Reset_Display();
    screen_fresh_cnt = 0;
  }
}


void wifi_init() {
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  
  int timeout = 0;
  while (WiFi.status() != WL_CONNECTED && timeout < 30) {
    delay(50);
    Serial.print(".");
    timeout++;
  }
  
  if(WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected!");
    Serial.print("Local IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWiFi connection failed!");
  }
}

void send_udp_data() {
  char buffer[512];  // in JSON format
  snprintf(buffer, sizeof(buffer), 
    "{"
    "\"ts\":%lu,"
    "\"ad7705\":%.3f,"
    "\"bat_v\":%.3f,"
    "\"bat_i\":%.3f,"
    "\"esc_rpm\":%u,"
    "\"esc_v\":%.2f,"
    "\"esc_i\":%.2f,"
    "\"esc_temp\":%u,"
    "\"pwm\":%u"
    "}",
    sensorData.timestamp,
    sensorData.ad7705_voltage,
    sensorData.battery_voltage,
    sensorData.battery_current,
    sensorData.esc_rpm,
    sensorData.esc_voltage,
    sensorData.esc_current,
    sensorData.esc_temp,
    sensorData.pwm_width
  );

  udp.beginPacket(udpAddress, udpPort);
  udp.write((uint8_t*)buffer, strlen(buffer));
  udp.endPacket();
  
  Serial.println("Sent UDP packet.");
}











void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n===== ESP32传感器系统启动 =====");
  
  // 初始化SPI
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV16);
  
  // 初始化I2C
  Wire.begin();
  
  // 初始化AD7705
  ad7705_init();
  
  // 初始化ADS1115
  ads1115_init();
  
  // 初始化BLHeli32串口
  blheli32_init();
  
  // 初始化PWM测量
  pwm_init();
  
  // 连接WiFi
  wifi_init();
  
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