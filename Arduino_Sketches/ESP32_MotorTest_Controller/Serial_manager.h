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

static const uint8_t KISS_TELEMETRY_SIZE = 10;
uint8_t serial1_buffer[SERIAL1_BUF_SIZE] = {0};
uint8_t serial1_buffer_index = 0;
bool serial1_frame_synced = false;

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

bool parseSerial1Data() {
  uint8_t receivedCRC = serial1_buffer[KISS_TELEMETRY_SIZE - 1];
  uint8_t calculatedCRC = calculateCRC8(serial1_buffer, KISS_TELEMETRY_SIZE - 1);
  if (receivedCRC != calculatedCRC) {
    Serial.println("Error: 1");
    return false;
  }

  float temperature = serial1_buffer[0];
  float voltage = int16_t((serial1_buffer[1] << 8) | serial1_buffer[2]) / 100.0;
  float current = int16_t((serial1_buffer[3] << 8) | serial1_buffer[4]) / 100.0;
  if (temperature >= 150 || temperature < 0 || 
      voltage >= 60 || voltage < 0 || 
      current >= 200 || current < 0) {
        Serial.println("Error: 2");
    return false;
  }

  myEscData.temperature = temperature;
  myEscData.voltage = voltage;
  myEscData.current = current;
  myEscData.consumption = int16_t((serial1_buffer[5] << 8) | serial1_buffer[6]);
  myEscData.erpm = int16_t((serial1_buffer[7] << 8) | serial1_buffer[8]) * 100;
  myEscData.crc = receivedCRC;
  return true;
}

void serial1DataEvent() {
  while (Serial1.available()) {
    uint8_t byte = Serial1.read();
    if (serial1_buffer_index < KISS_TELEMETRY_SIZE) {
      serial1_buffer[serial1_buffer_index++] = byte;
    }
    else {
      for (uint8_t i = 0; i < KISS_TELEMETRY_SIZE-1; i++) {
        serial1_buffer[i] = serial1_buffer[i+1];
      }
      serial1_buffer[KISS_TELEMETRY_SIZE-1] = byte;
      if (parseSerial1Data()) {
        serial1_buffer_index = 0;
      }
    }
  }
}