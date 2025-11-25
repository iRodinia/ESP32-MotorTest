#ifndef HELPER_FCNS_DOWN
#define HELPER_FCNS_DOWN

#include <Arduino.h>
#include <ArduinoJson.h>

struct MCU_Down_Data {
  char glbT[9]; float lcaT = 0;
  float lastAx = 0; float lastAy = 0; float lastAz = 0;  // in m/s^2
  float lastGx = 0; float lastGy = 0; float lastGz = 0;  // in rad/s
  float lastMx = 0; float lastMy = 0; float lastMz = 0;  // in μT
  float lastTmp = 0;  // in centigrade
};

void convert_data_to_string(MCU_Down_Data data, char* resultStr) {
  sprintf(resultStr, "%s,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f",
    data.glbT, data.lcaT, data.lastAx, data.lastAy, data.lastAz, 
    data.lastGx, data.lastGy, data.lastGz, data.lastMx, data.lastMy, data.lastMz, data.lastTmp
  );
}

void extract_data_skip_time(const char* resultStr, char* output) {
    int comma_count = 0;
    const char* ptr = resultStr;
    while (*ptr != '\0' && comma_count < 2) {
        if (*ptr == ',') {
            comma_count++;
        }
        ptr++;
    }
    strcpy(output, ptr);
}

void convert_data_to_json(MCU_Down_Data data, String& resultStr) {
  StaticJsonDocument<480> doc;
  doc["GlobalTime"] = data.glbT;
  doc["LocalTime"] = data.lcaT;
  doc["AccelerationX"] = data.lastAx;
  doc["AccelerationY"] = data.lastAy;
  doc["AccelerationZ"] = data.lastAz;
  doc["GyroscopeX"] = data.lastGx;
  doc["GyroscopeY"] = data.lastGy;
  doc["GyroscopeZ"] = data.lastGz;
  doc["MagnetX"] = data.lastMx;
  doc["MagnetY"] = data.lastMy;
  doc["MagnetZ"] = data.lastMz;
  doc["ImuTemperature"] = data.lastTmp;
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