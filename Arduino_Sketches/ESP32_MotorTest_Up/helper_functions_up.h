#ifndef _HELPER_FCNS_UP_
#define _HELPER_FCNS_UP_

#include <Arduino.h>
#include <ArduinoJson.h>

struct MCU_Up_Data {
  char glbT[8]; float lcaT = 0;
  float lastCur = 0; float lastVol = 0; float lastPwr = 0;  // in A, V, W
  float lastCmd = 0; float lastRpm = 0; float lastThr = 0;  // in [0-1], r/s, N
  float lastAx = 0; float lastAy = 0; float lastAz = 0;  // in m/s^2
  float lastGx = 0; float lastGy = 0; float lastGz = 0;  // in rad/s
  float lastMx = 0; float lastMy = 0; float lastMz = 0;  // in μT
  float lastEscTmp = 0;  // in centidegree
};

void convert_data_to_string(MCU_Up_Data data, char* resultStr) {
  sprintf(resultStr, "%s,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f",
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


#endif