#ifndef SER_MANAGER
#define SER_MANAGER

#include <Arduino.h>

#define SERIAL1_RX 26
#define SERIAL1_TX 25
#define SERIAL2_RX 16
#define SERIAL2_TX 17

////// Serials Initialization ///////
String initAllSerials() {
  Serial.begin(9600);
  delay(100);
  Serial1.begin(57600, SERIAL_8N1, SERIAL1_RX, SERIAL1_TX, true);
  delay(100);
  Serial2.begin(9600, SERIAL_8N1, SERIAL2_RX, SERIAL2_TX);
  delay(100);
  if (!Serial || !Serial1 || !Serial2) {
    return "Serial initializaiton failed.";
  }
  return "";
}
/////////////////////////////////////

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
#define SPORT_FRAME_SIZE 8  // S.Port reveive data format: [0x7E + frameId] + [0x10 + 2 byte data Id + 4 byte data + CRC] = 2 + 8
#define DATA_FRAME_HEADER 0x10

#define ID_POWER_LO 0x0B50  // ESC power
#define ID_POWER_HI 0x0B5F
#define ID_ERPM_LO 0x0B60  // ESC ERPM
#define ID_ERPM_HI 0x0B6F
#define ID_TEMPERATURE_LO 0x0B70  // ESC Temperature
#define ID_TEMPERATURE_HI 0x0B7F

uint8_t serial1_buffer[SPORT_FRAME_SIZE+1];
uint8_t serial1_buffer_index = 0;
bool escape_next = false;

struct SPortTelemetryData {
  float temperature = 0;  // temperature (C)
  float voltage = 0;  // voltage (V)
  float current = 0;  // current (A)
  uint32_t erpm = 0;  // electric rpm (ERPM)
  uint32_t alive_ts = 0;
};

volatile SPortTelemetryData myEscData;

void serial1SendIdRequest(uint8_t sport_id) {
  uint8_t id = sport_id & 0x1F; 
  
  uint8_t d0 = (id >> 0) & 0x01;
  uint8_t d1 = (id >> 1) & 0x01;
  uint8_t d2 = (id >> 2) & 0x01;
  uint8_t d3 = (id >> 3) & 0x01;
  uint8_t d4 = (id >> 4) & 0x01;
  
  uint8_t c0 = d0 ^ d1 ^ d2;
  uint8_t c1 = d2 ^ d3 ^ d4;
  uint8_t c2 = d0 ^ d2 ^ d4;
  
  uint8_t pollingByte = (c2 << 7) | (c1 << 6) | (c0 << 5) | id;
    
  Serial1.write(0x7E);
  Serial1.write(pollingByte);
}

uint8_t calculateCheckSum(uint8_t* data, uint8_t length) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < length; i++) {
    crc += data[i];
  }
  return 0xFF - crc;
}

void parseSerial1Data() {
  // Serial.printf("%x %x %x %x %x %x %x %x %x \n", serial1_buffer[0], serial1_buffer[1], serial1_buffer[2],
  //   serial1_buffer[3], serial1_buffer[4], serial1_buffer[5], serial1_buffer[6], serial1_buffer[7],
  //   serial1_buffer[8]);

  if (serial1_buffer[0] != DATA_FRAME_HEADER) {
    return;
  }
  uint8_t receivedCRC = serial1_buffer[SPORT_FRAME_SIZE-1];
  uint8_t calculatedCRC = calculateCheckSum(serial1_buffer, SPORT_FRAME_SIZE-1);
  if (calculatedCRC - receivedCRC > 1) {
    Serial.println("Received invalid S.Port packet.");
    return;
  }

  uint16_t dataId = (serial1_buffer[2] << 8) | serial1_buffer[1];
  uint32_t rawValue = (serial1_buffer[6] << 24) | (serial1_buffer[5] << 16) | (serial1_buffer[4] << 8) | serial1_buffer[3];

  if (dataId >= ID_POWER_LO && dataId <= ID_POWER_HI) {
    myEscData.voltage = (rawValue & 0xFFFF) / 100.0;
    myEscData.current = ((rawValue >> 16) & 0xFFFF) / 100.0;
  }
  else if (dataId >= ID_ERPM_LO && dataId <= ID_ERPM_HI) {
    myEscData.erpm = uint32_t(rawValue & 0xFFFF) * 100;
  }
  else if (dataId >= ID_TEMPERATURE_LO && dataId <= ID_TEMPERATURE_HI) {
    myEscData.temperature = rawValue;
  }

  myEscData.alive_ts = millis();
}

void serial1DataEvent() {
  while (Serial1.available()) {
    uint8_t byte = Serial1.read();

    if (byte == 0x7E) {
      serial1_buffer_index = 0;
      escape_next = false;
      continue;
    }
    if (byte == 0x7D) {
      escape_next = true;
      continue;
    }
    if (escape_next) {
      byte ^= 0x20;
      escape_next = false;
    }

    if (serial1_buffer_index == 0 && byte == DATA_FRAME_HEADER) {
      serial1_buffer[serial1_buffer_index++] = byte;
    }
    else {
      serial1_buffer[serial1_buffer_index++] = byte;
      if (serial1_buffer_index >= SPORT_FRAME_SIZE) {
        parseSerial1Data();
        serial1_buffer_index = 0;
      }
    }
  }
}
//////////////////////////////////////

//////////// Serial 2 ///////////////
#define SERIAL2_BUF_SIZE 64

char serial2_cmd[SERIAL2_BUF_SIZE];
uint8_t serial2_cmd_index = 0;

void serial2CmdEvent() {
  while (Serial2.available()) {
    char inChar = (char)Serial2.read();
    if (serial2_cmd_index >= SERIAL2_BUF_SIZE-1) {
      serial2_cmd_index = 0;
      serial2_cmd[0] = '\0';
      continue;
    }
    if (inChar == ' '){
      continue;
    }
    serial2_cmd[serial2_cmd_index] = inChar;
    serial2_cmd_index++;
    if (inChar == '\n') {
      serial2_cmd[serial2_cmd_index] = '\0';
      parseSerial0Cmd(String(serial2_cmd));
      serial2_cmd_index = 0;
      serial2_cmd[0] = '\0';
    }
  }
}
/////////////////////////////////////

#endif