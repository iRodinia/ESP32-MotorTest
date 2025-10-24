#ifndef _HELPER_FCNS_UP_
#define _HELPER_FCNS_UP_

struct MCU_Up_Data {
  float lastCur = 0; float lastVol = 0; float lastPwr = 0;  // in A, V, W
  float lastCmd = 0; float lastRpm = 0; float lastThr = 0;  // in [0-1], r/s, N
  float lastAx = 0; float lastAy = 0; float lastAz = 0;  // in m/s^2
  float lastGx = 0; float lastGy = 0; float lastGz = 0;  // in rad/s
  float lastMx = 0; float lastMy = 0; float lastMz = 0;  // in μT
  float lastEscTmp = 0;  // in centidegree
};

MCU_Up_Data myData;


#endif