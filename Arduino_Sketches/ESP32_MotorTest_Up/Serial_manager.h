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
  Serial1.begin(57600, SERIAL_8N1, SERIAL1_RX, SERIAL1_TX, true);
  delay(100);
  Serial2.begin(100000, SERIAL_8E2, SERIAL2_RX, SERIAL2_TX);
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
#define SPORT_FRAME_SIZE 9  // S.Port reveive data format: frameId 0x10 + 2 byte data Id + 4 byte data + CRC + 0x7E
#define DATA_FRAME_HEADER 0x10
#define DATA_FRAME_END 0x7E

#define ESC_POWER_FIRST_ID        0x0B50
#define ESC_POWER_LAST_ID         0x0B5F
#define ESC_RPM_CONS_FIRST_ID     0x0B60
#define ESC_RPM_CONS_LAST_ID      0x0B6F
#define ESC_TEMPERATURE_FIRST_ID  0x0B70
#define ESC_TEMPERATURE_LAST_ID   0x0B7F

#define ID_VOLTAGE 0x0210  // VFAS / Battery Voltage
#define ID_CURRENT 0x0200  // Current
#define ID_CONSUMPTION 0x0600
#define ID_ERPM 0x0B50  // ESC ERPM
#define ID_TEMPERATURE 0x0B70  // ESC Temperature

#define ESC_POWER       0x0B50  // ESC电压 (0.01V)
#define ESC_RPM         0x0500  // ESC转速 (RPM)
#define ESC_CONSUMPTION 0x0600  // ESC消耗电量 (mAh)
#define ESC_TEMP        0x0B70  // ESC温度 (°C)
#define ESC_CURRENT     0x0B60  // ESC电流 (0.1A)

uint8_t serial1_buffer[SPORT_FRAME_SIZE+1];
uint8_t serial1_buffer_index = 0;

struct SPortTelemetryData {
  float temperature = 0;  // temperature (C)
  float voltage = 0;  // voltage (V)
  float current = 0;  // current (A)
  uint32_t consumption = 0;  // power cosumption (mAh)
  uint16_t erpm = 0;  // electric rpm (ERPM)
};

SPortTelemetryData myEscData;

uint16_t calculateCRC(uint8_t* data, uint8_t length) {
  uint16_t crc = 0;
  for (uint8_t i = 0; i < length; i++) {
    crc += data[i];
    crc += crc >> 8;
    crc &= 0x0FF;
  }
  return crc;  // return (crc == 0x00ff);
}

void parseSerial1Data() {
  Serial.printf("%x %x %x %x %x %x %x %x %x \n", serial1_buffer[0], serial1_buffer[1], serial1_buffer[2],
    serial1_buffer[3], serial1_buffer[4], serial1_buffer[5], serial1_buffer[6], serial1_buffer[7],
    serial1_buffer[8]);

  if (serial1_buffer_index < SPORT_FRAME_SIZE || 
      serial1_buffer[0] != DATA_FRAME_HEADER || 
      serial1_buffer[SPORT_FRAME_SIZE-1] != DATA_FRAME_END) {
    return;
  }
  uint8_t receivedCRC = serial1_buffer[SPORT_FRAME_SIZE-2];
  uint8_t calculatedCRC = calculateCRC(serial1_buffer, SPORT_FRAME_SIZE-2);
  if (receivedCRC != calculatedCRC) {
    Serial.printf("%x != %x \n", receivedCRC, calculatedCRC);
    // return;
  }

  uint16_t dataId = (serial1_buffer[2] << 8) | serial1_buffer[1];
  uint32_t rawValue = (serial1_buffer[6] << 24) | (serial1_buffer[5] << 16) | (serial1_buffer[4] << 8) | serial1_buffer[3];

  Serial.printf("Id: %x, Value: %d \n", dataId, rawValue);

  switch (dataId) {
    case ID_VOLTAGE:
      myEscData.voltage = rawValue / 100.0;
      break;
    case ID_CURRENT:
      myEscData.current = rawValue / 10.0;
      break;
    case ID_CONSUMPTION:
      myEscData.consumption = rawValue;
      break;
    case ID_ERPM:
      myEscData.erpm = rawValue;
      break;
    case ID_TEMPERATURE:
      myEscData.temperature = rawValue;
      break;
  }
}

void serial1DataEvent() {
  while (Serial1.available()) {
    uint8_t byte = Serial1.read();
    if (serial1_buffer_index == 0 && byte != DATA_FRAME_HEADER)
      continue;
    serial1_buffer[serial1_buffer_index++] = byte;
    if (serial1_buffer_index >= SPORT_FRAME_SIZE) {
      if (serial1_buffer[SPORT_FRAME_SIZE-1] == DATA_FRAME_END) {
        parseSerial1Data();
      }
      serial1_buffer_index = 0;
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
    if (serial2_buffer_index >= SBUS_FRAME_SIZE) {
      if (serial2_buffer[SBUS_FRAME_SIZE-1] == SBUS_FOOTER || serial2_buffer[SBUS_FRAME_SIZE-1] == 0x04) {
        parseSerial2Data();
      }
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