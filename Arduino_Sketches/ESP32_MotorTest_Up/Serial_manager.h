#ifndef SER_MANAGER
#define SER_MANAGER

#include <Arduino.h>

#define SERIAL1_RX 26
#define SERIAL1_TX 25
#define SERIAL2_RX 27
#define SERIAL2_TX 14

// serial 0: usb serial
// serial 1: tx-25, rx-26, only rx connect to the ESC loopback
//   baudrate 115200
// serial 2: tx-14, rx-27, only rx connect to the Telemetry SBUS/UART
//   baudrate 1000

////// Serials Initialization ///////
String initAllSerials() {
  Serial.begin(115200);
  delay(100);
  Serial1.begin(115200, SERIAL_8N1, SERIAL1_RX, SERIAL1_TX);
  delay(100);
  Serial2.begin(1000, SERIAL_8E2, SERIAL2_RX, SERIAL2_TX);
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
//////////////////////////////////////

///////////// Serial 2 ///////////////
#define SERIAL2_BUF_SIZE 30

static const uint8_t SBUS_HEADER = 0x0F;
static const uint8_t SBUS_FOOTER = 0x00;
static const uint8_t SBUS_FRAME_SIZE = 25;
static const uint16_t CMD_MIN = 172;
static const uint16_t CMD_MAX = 1810;

uint8_t serial2_buffer[SERIAL2_BUF_SIZE];
uint8_t serial2_buffer_index = 0;
uint16_t receiver_channels[16];  // 16 channel readings
uint8_t flags = 0;  // flag of the SBUS data

void parseSerial2Data() {
  receiver_channels[0]  = ((serial2_buffer[1]    | serial2_buffer[2]  << 8)                    & 0x07FF);
  receiver_channels[1]  = ((serial2_buffer[2]>>3 | serial2_buffer[3]  << 5)                    & 0x07FF);
  receiver_channels[2]  = ((serial2_buffer[3]>>6 | serial2_buffer[4]  << 2 | serial2_buffer[5]<<10) & 0x07FF);
  receiver_channels[3]  = ((serial2_buffer[5]>>1 | serial2_buffer[6]  << 7)                    & 0x07FF);
  receiver_channels[4]  = ((serial2_buffer[6]>>4 | serial2_buffer[7]  << 4)                    & 0x07FF);
  receiver_channels[5]  = ((serial2_buffer[7]>>7 | serial2_buffer[8]  << 1 | serial2_buffer[9]<<9)  & 0x07FF);
  receiver_channels[6]  = ((serial2_buffer[9]>>2 | serial2_buffer[10] << 6)                    & 0x07FF);
  receiver_channels[7]  = ((serial2_buffer[10]>>5| serial2_buffer[11] << 3)                    & 0x07FF);
  receiver_channels[8]  = ((serial2_buffer[12]   | serial2_buffer[13] << 8)                    & 0x07FF);
  receiver_channels[9]  = ((serial2_buffer[13]>>3| serial2_buffer[14] << 5)                    & 0x07FF);
  receiver_channels[10] = ((serial2_buffer[14]>>6| serial2_buffer[15] << 2 | serial2_buffer[16]<<10) & 0x07FF);
  receiver_channels[11] = ((serial2_buffer[16]>>1| serial2_buffer[17] << 7)                    & 0x07FF);
  receiver_channels[12] = ((serial2_buffer[17]>>4| serial2_buffer[18] << 4)                    & 0x07FF);
  receiver_channels[13] = ((serial2_buffer[18]>>7| serial2_buffer[19] << 1 | serial2_buffer[20]<<9) & 0x07FF);
  receiver_channels[14] = ((serial2_buffer[20]>>2| serial2_buffer[21] << 6)                    & 0x07FF);
  receiver_channels[15] = ((serial2_buffer[21]>>5| serial2_buffer[22] << 3)                    & 0x07FF);
  flags = serial2_buffer[23];
}

void serial2DataEvent() {
  while (Serial2.available() > 0) {
    uint8_t c = Serial2.read();
    if (serial2_buffer_index == 0 && c != SBUS_HEADER) {
      continue;
    }
    serial2_buffer[serial2_buffer_index++] = c;
    if (serial2_buffer_index == SBUS_FRAME_SIZE) {
      if (serial2_buffer[24] == SBUS_FOOTER || serial2_buffer[24] == 0x04) {
        parseSerial2Data();
      }
      serial2_buffer_index = 0;
    }
    
    if (serial2_buffer_index >= SBUS_FRAME_SIZE) {
      serial2_buffer_index = 0;
    }
  }
}

String parseSbusFlag() {
  if (flags & 0x80) return "CH17";
  if (flags & 0x40) return "CH18";
  if (flags & 0x20) return "LOST";
  if (flags & 0x10) return "FAILSAFE";
}
///////////////////////////////////////

#endif