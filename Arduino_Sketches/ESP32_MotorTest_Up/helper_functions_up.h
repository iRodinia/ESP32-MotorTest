#ifndef _HELPER_FCNS_UP_
#define _HELPER_FCNS_UP_

#include <Arduino.h>
#include <ArduinoJson.h>

struct MCU_Up_Data {
  char glbT[9]; float lcaT = 0;
  float lastCur = 0; float lastVol = 0; float lastPwr = 0;  // in A, V, W
  float lastCmd = 0; float lastRpm = 0; float lastThr = 0;  // in [0-1], r/s, N
  float lastAx = 0; float lastAy = 0; float lastAz = 0;  // in m/s^2
  float lastGx = 0; float lastGy = 0; float lastGz = 0;  // in rad/s
  float lastMx = 0; float lastMy = 0; float lastMz = 0;  // in μT
  float lastEscTmp = 0;  // in centidegree
};

void convert_data_to_string(MCU_Up_Data data, char* resultStr) {
  sprintf(resultStr, "%s,%.2f,%.2f,%.2f,%.2f,%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f",
        data.glbT, data.lcaT, data.lastCur, data.lastVol, data.lastPwr, 
        data.lastCmd, data.lastRpm, data.lastThr, data.lastAx, data.lastAy, data.lastAz, 
        data.lastGx, data.lastGy, data.lastGz, data.lastMx, data.lastMy, data.lastMz, data.lastEscTmp
  );
}

void convert_data_to_json(MCU_Up_Data data, String& resultStr) {
  StaticJsonDocument<512> doc;
  doc["GlobalTime"] = data.glbT;
  doc["LocalTime"] = data.lcaT;
  doc["EscCurrent"] = data.lastCur;
  doc["EscVoltage"] = data.lastVol;
  doc["EscPower"] = data.lastPwr;
  doc["EscTemperature"] = data.lastEscTmp;
  doc["Command"] = data.lastCmd;
  doc["MotorRpm"] = data.lastRpm;
  doc["MotorForce"] = data.lastThr;
  doc["AccelerationX"] = data.lastAx;
  doc["AccelerationY"] = data.lastAy;
  doc["AccelerationZ"] = data.lastAz;
  doc["GyroscopeX"] = data.lastGx;
  doc["GyroscopeY"] = data.lastGy;
  doc["GyroscopeZ"] = data.lastGz;
  doc["MagnetX"] = data.lastMx;
  doc["MagnetY"] = data.lastMy;
  doc["MagnetZ"] = data.lastMz;
  serializeJson(doc, resultStr);
}


char serial2Buffer[256];
uint16_t serial2BufferIndex = 0;
unsigned long serial2LastReceiveTime = 0;

bool read_serial2_data(float &ax, float &ay, float &az, float &gx, float &gy, float &gz, float &mx, float &my, float &mz, float &temp) {
  if (Serial2.available() <= 0) {
    if (serial2BufferIndex > 0 && (millis() - serial2LastReceiveTime > 800)) {
      serial2BufferIndex = 0;
      serial2Buffer[0] = '\0';
    }
    return false;
  }

  while (Serial2.available() > 0) {
    char c = Serial2.read();
    serial2LastReceiveTime = millis();

    if (serial2BufferIndex >= 255) {
      serial2BufferIndex = 0;
      serial2Buffer[0] = '\0';
      continue;
    }
    
    serial2Buffer[serial2BufferIndex] = c;
    serial2BufferIndex++;
    if (c == '\n') {
      serial2Buffer[serial2BufferIndex] = '\0';
      int parsedCount = sscanf(serial2Buffer, 
        "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
        &ax, &ay, &az,
        &gx, &gy, &gz,
        &mx, &my, &mz,
        &temp
      );
      if (parsedCount == 10) {
        serial2BufferIndex = 0;
        serial2Buffer[0] = '\0';
        return true;
      }
      else {
        serial2BufferIndex = 0;
        serial2Buffer[0] = '\0';
      }
    }
  }
  return false;
}

void flush_serial2_buffer() {
    while (Serial2.available() > 0) {
        Serial2.read();
    }
    serial2BufferIndex = 0;
    serial2Buffer[0] = '\0';
}

bool isValidIP(String ip) {
  int dots = 0;
  int num = 0;
  int segment[4] = {0};
  for (int i = 0; i < ip.length(); i++) {
    char c = ip[i];
    if (c == '.') {
      if (dots > 3) return false;
      segment[dots] = num;
      num = 0;
      dots++;
    } 
    else if (isdigit(c)) {
      num = num * 10 + (c - '0');
      if (num > 255) return false;
    } 
    else {
      return false;
    }
  }
  segment[dots] = num;
  if (dots != 3) return false;
  for (int i = 0; i < 4; i++) {
    if (segment[i] < 0 || segment[i] > 255) return false;
  }
  return true;
}

#endif