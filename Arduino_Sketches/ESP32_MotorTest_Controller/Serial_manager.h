#include <Arduino.h>

#define SERIAL0_BUF_SIZE 15

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

#define SERIAL1_RX 26
#define SERIAL1_TX 25
#define SERIAL1_BUF_SIZE 16

static const uint8_t KISS_FRAME_START = 0xC0;
static const uint8_t KISS_FRAME_END = 0xC0;
static const uint8_t KISS_TELEMETRY_SIZE = 10;

uint8_t serial1_buffer[SERIAL1_BUF_SIZE];
uint8_t serial1_buffer_index = 0;
bool serial1_frame_started = false;
unsigned long serial1_frame_start_time = 0;

// BLHeli32 KISS structure
struct EscTelemetryData {
  float temperature = 0;  // temperature (C)
  float voltage = 0;  // voltage (V)
  float current = 0;  // current (A)
  float consumption = 0;  // power cosumption (mAh)
  uint16_t erpm = 0;  // electric rpm (ERPM)
  uint8_t crc = 0;  // CRC test
};

EscTelemetryData myEscData;

uint8_t calculateCRC8(uint8_t* data, uint8_t length) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x80) {
        crc = (crc << 1) ^ 0xD5;
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

void parseSerial1Data() {
  if (serial1_buffer_index < KISS_TELEMETRY_SIZE) {
    return;
  }
  uint8_t receivedCRC = serial1_buffer[KISS_TELEMETRY_SIZE - 1];
  uint8_t calculatedCRC = calculateCRC8(serial1_buffer, KISS_TELEMETRY_SIZE - 1);
  if (receivedCRC != calculatedCRC) {
    return;
  }

  myEscData.temperature = serial1_buffer[0];
  myEscData.voltage = ((serial1_buffer[1] << 8) | serial1_buffer[2]) / 100.0;
  myEscData.current = ((serial1_buffer[3] << 8) | serial1_buffer[4]) / 100.0;
  myEscData.consumption = (serial1_buffer[5] << 8) | serial1_buffer[6];
  myEscData.erpm = ((serial1_buffer[7] << 8) | serial1_buffer[8]) * 100;
  myEscData.crc = receivedCRC;
}

void serial1DataEvent() {
  unsigned long now = millis();
  if (serial1_frame_started && (now - serial1_frame_start_time > 100)) {
    serial1_frame_started = false;
    serial1_buffer_index = 0;
  }
  while (Serial1.available()) {
    uint8_t byte = Serial1.read();
    if (byte == KISS_FRAME_START) {
      if (!serial1_frame_started) {
        serial1_frame_started = true;
        serial1_buffer_index = 0;
        serial1_frame_start_time = now;
      }
      else {
        parseSerial1Data();
        serial1_frame_started = false;
        serial1_buffer_index = 0;
      }
    }
    else if (serial1_frame_started && serial1_buffer_index < SERIAL1_BUF_SIZE) {
      serial1_buffer[serial1_buffer_index++] = byte;
    }
  }
}