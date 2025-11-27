#ifndef _HELPER_FCNS_UP_
#define _HELPER_FCNS_UP_

#include <Arduino.h>
#include <ArduinoJson.h>

struct MCU_Up_Data {
  float lcaT = 0;
  float lastCur = 0; float lastVol = 0; float lastPwr = 0;  // in A, V, W
  float lastCmd = 0; float lastRpm = 0; float lastThr = 0;  // in [0-1], r/s, N
  float lastEscTmp = 0;  // in centidegree
};

void convert_data_to_string(MCU_Up_Data data, char* resultStr) {
  sprintf(resultStr, "%.2f,%.2f,%.2f,%.2f,%.3f,%.2f,%.2f,%.2f",
    data.lcaT, data.lastCur, data.lastVol, data.lastPwr, 
    data.lastCmd, data.lastRpm, data.lastThr, data.lastEscTmp
  );
}

void convert_data_to_json(MCU_Up_Data data, String& resultStr) {
  StaticJsonDocument<512> doc;
  doc["LocalTime"] = data.lcaT;
  doc["EscCurrent"] = data.lastCur;
  doc["EscVoltage"] = data.lastVol;
  doc["EscPower"] = data.lastPwr;
  doc["EscTemperature"] = data.lastEscTmp;
  doc["Command"] = data.lastCmd;
  doc["MotorRpm"] = data.lastRpm;
  doc["MotorForce"] = data.lastThr;
  serializeJson(doc, resultStr);
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