#ifndef SER_MANAGER
#define SER_MANAGER

#include <Arduino.h>

//////////// Serial 0 ///////////////
#define SERIAL0_BUF_SIZE 64

char serial0_cmd[SERIAL0_BUF_SIZE];
uint8_t serial0_cmd_index = 0;
extern void parseSerial0Cmd(String command);

void serial0CmdEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (serial0_cmd_index >= SERIAL0_BUF_SIZE-1) {
      serial0_cmd_index = 0;
      serial0_cmd[0] = '\0';
      continue;
    }
    if (inChar == ' '){
      continue;
    }
    serial0_cmd[serial0_cmd_index] = inChar;
    serial0_cmd_index++;
    if (inChar == '\n') {
      serial0_cmd[serial0_cmd_index] = '\0';
      parseSerial0Cmd(String(serial0_cmd));
      serial0_cmd_index = 0;
      serial0_cmd[0] = '\0';
    }
  }
}
/////////////////////////////////////

//////////// Serial 1 ///////////////
#define SERIAL1_BUF_SIZE 256

char serial1_cmd[SERIAL1_BUF_SIZE];
uint8_t serial1_cmd_index = 0;
struct MCU_Sensors_Data {
  float lcaT = 0;  // in s
  float lastVol = 0;  // in V
  uint32_t lastCmd = 0;  // int [0-2047]
  float lastRpm = 0;  // in r/min
  float lastThr = 0;  // in N
} myEscData;

bool parseSerial1Cmd(String command) {
  command.trim();
  if (command.length() == 0) return false;

  const int FIELD_COUNT = 5;  // T, V, Cmd, Rpm, F
  float fields[FIELD_COUNT];
  int   fieldIdx = 0;
  int   start    = 0;

  while (fieldIdx < FIELD_COUNT) {
    int comma = command.indexOf(',', start);
    String token;
    if (comma == -1) {
      token = command.substring(start);
      token.trim();
      if (token.length() == 0) break;
      fields[fieldIdx++] = token.toFloat();
      break;
    } 
    else {
      token = command.substring(start, comma);
      token.trim();
      if (token.length() == 0) return false;  // empty field → malformed, discard
      fields[fieldIdx++] = token.toFloat();
      start = comma + 1;
    }
  }

  if (fieldIdx < FIELD_COUNT) return false;  // too few fields → discard

  myEscData.lcaT = fields[0];
  myEscData.lastVol = fields[1];
  myEscData.lastCmd = uint32_t(fields[2]);
  myEscData.lastRpm = fields[3];
  myEscData.lastThr = fields[4];

  return true;
}

void serial1CmdEvent() {
  while (Serial1.available()) {
    char inChar = (char)Serial1.read();
    if (serial1_cmd_index >= SERIAL1_BUF_SIZE-1) {
      serial1_cmd_index = 0;
      serial1_cmd[0] = '\0';
      continue;
    }
    if (inChar == ' '){
      continue;
    }
    serial1_cmd[serial1_cmd_index] = inChar;
    serial1_cmd_index++;
    if (inChar == '\n') {
      serial1_cmd[serial1_cmd_index] = '\0';
      bool _t = parseSerial1Cmd(String(serial1_cmd));
      serial1_cmd_index = 0;
      serial1_cmd[0] = '\0';
    }
  }
}
/////////////////////////////////////

#endif