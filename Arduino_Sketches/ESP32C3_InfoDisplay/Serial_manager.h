#ifndef SER_MANAGER
#define SER_MANAGER

#include <Arduino.h>

#define SERIAL1_RX 20
#define SERIAL1_TX 21

#define MAX_CHANNEL_NUM 2

// serial 0: usb serial
// serial 1: tx-21, rx-20, only rx connect to the ESC loopback
//   baudrate 115200

struct Sensor_Data {
  int channel = 0;
  float time = 0;  // in s
  float esc_current = 0;  // in A
  float esc_voltage = 0;  // in V
  float esc_command = 0;  // in %
  float esc_temperature = 0;  // in centidegree
  float motor_rpm = 0;  // in r/min
  float motor_force = 0;  // in N
} channel_data[MAX_CHANNEL_NUM];

////// Serials Initialization ///////
void initAllSerials() {
  Serial.begin(115200);
  delay(100);
  Serial1.begin(115200, SERIAL_8N1, SERIAL1_RX, SERIAL1_TX);
  delay(100);
}
/////////////////////////////////////

//////////// Serial 1 ///////////////
#define SERIAL1_BUF_SIZE 128

char serial1_cmd[SERIAL1_BUF_SIZE];
uint8_t serial1_cmd_index = 0;
bool parseSerial1Cmd(String command) {
  command.trim();
  if (command.length() == 0) return false;

  // ── Tokenise by comma ────────────────────────────────────────────────────
  const int FIELD_COUNT = 8;
  float fields[FIELD_COUNT];
  int   fieldIdx = 0;
  int   start    = 0;

  while (fieldIdx < FIELD_COUNT) {
    int comma = command.indexOf(',', start);
    String token;
    if (comma == -1) {
      // Last (or only) token — take the rest of the string
      token = command.substring(start);
      token.trim();
      if (token.length() == 0) break;
      fields[fieldIdx++] = token.toFloat();
      break;
    } else {
      token = command.substring(start, comma);
      token.trim();
      if (token.length() == 0) return false;  // empty field → malformed, discard
      fields[fieldIdx++] = token.toFloat();
      start = comma + 1;
    }
  }

  if (fieldIdx < FIELD_COUNT) return false;  // too few fields → discard

  // ── Validate channel ─────────────────────────────────────────────────────
  int ch = (int)fields[0];
  if (ch < 0 || ch >= MAX_CHANNEL_NUM) return false;  // out of range → discard

  // ── Store into the zero-based array ─────────────────────────────────────
  // channel_data index = ch (channel 0, 1, 2, ...)
  int idx = ch;
  channel_data[idx].channel         = ch;
  channel_data[idx].time            = fields[1];
  channel_data[idx].esc_voltage     = fields[2];
  channel_data[idx].esc_current     = fields[3];
  channel_data[idx].esc_command     = fields[4];
  channel_data[idx].esc_temperature = fields[5];
  channel_data[idx].motor_rpm       = fields[6];
  channel_data[idx].motor_force     = fields[7];

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