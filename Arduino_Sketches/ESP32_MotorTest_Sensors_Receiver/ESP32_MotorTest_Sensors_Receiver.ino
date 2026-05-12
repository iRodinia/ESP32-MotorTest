// ESP32 Serial Bridge Node
// Connects to the sensor ESP32's WiFi hotspot, receives UDP packets,
// and forwards the data over USB Serial (comma-separated CSV).
//
// Data order in output CSV (one line per packet):
//   [0]  LocalTime  — seconds since sensor node started (float, s)
//   [1]  EscCurrent  — Current from the Esc input (float, A)
//   [2]  EscVoltage  — Voltage from the Esc input (float, V)
//   [3]  PwmCommand  — PWM command received (float, [0-1])
//   [4]  RPM  — Motor rotation per minute (float, r/min)
//   [5]  Thrust  — Thrust force (float, N)
//   [6]  EscTemperature  — Temperature of the Esc die (float, centidegree)


#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

struct MCU_Sensors_Data {
  float lcaT = 0; float lastCur = 0; float lastVol = 0; // in s, A, V
  float lastCmd = 0; float lastRpm = 0; float lastThr = 0;  // in [0-1], r/min, N
  float lastEscTmp = 0;  // in centidegree
};


// ─── Sensor AP Credentials ───────────────────
const char*    AP_SSID     = "BioInBot_Sensor";
const char*    AP_PASSWORD = "11223344";
const uint16_t UDP_PORT    = 8888;

// ─── USB Serial baud rate ──────────────────────────────────────────────────────
const uint32_t SERIAL_BAUD = 115200;

// ─── WiFi reconnect interval ───────────────────────────────────────────────────
const uint32_t RECONNECT_INTERVAL_MS = 2000;

// ─── Global State ──────────────────────────────────────────────────────────────
WiFiUDP  udp;
uint32_t lastReconnectAttempt = 0;
bool     udpStarted           = false;

// ─── Forward Declarations ──────────────────────────────────────────────────────
void connectToAP();
void handleUDP();

// ─── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(SERIAL_BAUD);
  while (!Serial) delay(10);
  Serial.println("# ESP32 Serial Bridge — waiting for sensor AP...");

  WiFi.mode(WIFI_STA);
  connectToAP();
}

// ─── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
  // Auto-reconnect if WiFi drops
  if (WiFi.status() != WL_CONNECTED) {
    udpStarted = false;
    uint32_t now = millis();
    if (now - lastReconnectAttempt >= RECONNECT_INTERVAL_MS) {
      lastReconnectAttempt = now;
      Serial.println("# WiFi disconnected, retrying...");
      connectToAP();
    }
    return;
  }

  // Start UDP socket once connected
  if (!udpStarted) {
    udp.begin(UDP_PORT);
    udpStarted = true;
    Serial.printf("# UDP listening on port %d\n", UDP_PORT);
  }

  handleUDP();
}

// ─── Connect to the sensor AP ─────────────────────────────────────────────────
void connectToAP() {
  Serial.printf("# Connecting to AP: %s ...", AP_SSID);
  WiFi.begin(AP_SSID, AP_PASSWORD);

  uint8_t attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\n# Connected! Local IP: %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\n# Connection failed, will retry.");
  }
}

// ─── Receive UDP packet and decode ────────────────────────────────
void handleUDP() {
  int packetSize = udp.parsePacket();
  if (packetSize <= 0) return;

  if (packetSize != sizeof(MCU_Sensors_Data)) {
    Serial.printf("# [WARN] Unexpected packet size: %d bytes (expected %d)\n",
                  packetSize, sizeof(MCU_Sensors_Data));
    udp.flush();
    return;
  }

  MCU_Sensors_Data data;
  udp.read((uint8_t*)&data, sizeof(MCU_Sensors_Data));  // unpack directly into struct

  Serial.printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
    data.lcaT,
    data.lastCur, data.lastVol, data.lastCmd,
    data.lastRpm, data.lastThr, data.lastEscTmp
  );
}
