//ESP32-WROOM-DA multi-sensor and data transmission system

// Data order (comma-separated):
//   LocalTime, Current, Voltage, Command [0-1], RPM, Thrust, EscTemperature

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>

#include "Serial_manager.h"
#include "My_ads1115_sensor.h"
#include "MyMT6701_RPM.h"

#define MOTOR_POLE_PAIR 7
#define LED_PIN 2
#define LED_TOGGLE() digitalWrite(LED_PIN, digitalRead(LED_PIN) ^ 1)
#define RPM_Sensor_CS 9

struct MCU_Sensors_Data {
  float lcaT = 0; float lastCur = 0; float lastVol = 0; // in s, A, V
  float lastCmd = 0; float lastRpm = 0; float lastThr = 0;  // in [0-1], r/min, N
  float lastEscTmp = 0;  // in centidegree
};


// ─── Access Point Configuration ───────────────────────────────────────────────
const char* AP_SSID     = "BioInBot_Sensor";   // Hotspot SSID
const char* AP_PASSWORD = "11223344";       // Hotspot password (min 8 chars)
const IPAddress AP_LOCAL_IP(192, 168, 9, 1);
const IPAddress AP_GATEWAY(192, 168, 9, 1);
const IPAddress AP_SUBNET(255, 255, 255, 0);

// ─── UDP Configuration ─────────────────────────────────────────────────────────
// Broadcast to all clients on the AP subnet
const IPAddress UDP_BROADCAST_IP(192, 168, 9, 255);
const uint16_t  UDP_PORT = 8888;

WiFiUDP udp;
MCU_Sensors_Data myData;
MyADS1115Sensor myADC;
MT6701RPM myRPMSensor(RPM_Sensor_CS);

bool firstMainLoop = true;
uint32_t startRecordLt = 0;
uint32_t lastDataSend = 0;
uint32_t lastSensorFastUpdate = 0;
uint32_t lastSensorSlowUpdate = 0;

void setup() {
  String init_message = "";
  init_message += initAllSerials();
  Serial.println("\n===== ESP32 Motor Seneors (AP) =====");

  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(AP_LOCAL_IP, AP_GATEWAY, AP_SUBNET);
  if (WiFi.softAP(AP_SSID, AP_PASSWORD)) {
    Serial.printf("[WiFi] AP started. SSID: %s  IP: %s\n",
                  AP_SSID, WiFi.softAPIP().toString().c_str());
  } else {
    Serial.println("[WiFi] ERROR: Failed to start AP. Halting.");
    while (1);
  }
  udp.begin(UDP_PORT);
  Serial.printf("[UDP]  Listening on port %d, broadcasting to %s\n",
                UDP_PORT, UDP_BROADCAST_IP.toString().c_str());

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  Wire.begin();
  delay(50);
  Wire.setClock(400000);  // fast mode

  init_message += myADC.init();
  delay(100);
  init_message += myRPMSensor.begin();

  if(init_message.length() < 2){
    Serial.println("Submodules initialized.");
  }
  else{
    Serial.println(init_message);
    Serial.println("Halting.");
    while(1);
  }

  Serial.println("===== System Initialization Done. =====\n");
}

void loop() {
  if (firstMainLoop) {
    startRecordLt = millis();
    firstMainLoop = false;
  }
  else {
    uint32_t current_time = millis();

    if(current_time - lastSensorFastUpdate > 2) {
      lastSensorFastUpdate = current_time;
      myRPMSensor.update();
    }

    if(current_time - lastSensorSlowUpdate > 147) {
      lastSensorSlowUpdate = current_time;
      serial1DataEvent();
      serial2DataEvent();

      myADC.readPower(myData.lastVol, myData.lastCur);
      myADC.readForce(myData.lastThr);

      myData.lastEscTmp = myEscData.temperature;
      myData.lastCmd = (float(receiver_channels[2]) - CMD_MIN) / (CMD_MAX - CMD_MIN);  // throttle channel is No.3, which is channel[2]

      // myData.lastRpm = float(myEscData.erpm) / MOTOR_POLE_PAIR;
      myData.lastRpm = myRPMSensor.getRPM();
    }

    if(current_time - lastDataSend > 150) {
      lastDataSend = current_time;
      myData.lcaT = (current_time - startRecordLt) / 1000.0f;
      sendUDP(udp, myData);
      LED_TOGGLE();
    }
  }
}

void sendUDP(WiFiUDP &udp, MCU_Sensors_Data &_data) {
  udp.beginPacket(UDP_BROADCAST_IP, UDP_PORT);
  udp.write((const uint8_t*)&_data, sizeof(MCU_Sensors_Data));  // raw binary
  udp.endPacket();
}