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
  Serial1.begin(115200, SERIAL_8N1, SERIAL1_RX, SERIAL1_TX);
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

//////////// Serial 1 (KISS Telemetry) ///////////////
#define KISS_FRAME_SIZE 10  // 10 8-bit bytes per transmission

uint8_t serial1_buffer[KISS_FRAME_SIZE];
uint8_t serial1_buffer_index = 0;
uint32_t last_serial1_rx_time = 0; // 用于基于超时的帧对齐

struct KissTelemetryData {
  float temperature = 0;    // temperature (C)
  float voltage = 0;        // voltage (V)
  float current = 0;        // current (A)
  uint16_t consumption = 0; // consumption (mAh)
  uint32_t erpm = 0;        // electric rpm (ERPM)
  uint32_t alive_ts = 0;
};

// 保持变量名与你原有工程兼容，或者你可以在工程其他地方重命名它
volatile KissTelemetryData myEscData;

// === 从图片中提取的 CRC8 校验算法 ===
uint8_t update_crc8(uint8_t crc, uint8_t crc_seed) {
  uint8_t crc_u, i;
  crc_u = crc;
  crc_u ^= crc_seed;
  for (i = 0; i < 8; i++) {
    crc_u = (crc_u & 0x80) ? 0x7 ^ (crc_u << 1) : (crc_u << 1);
  }
  return (crc_u);
}

uint8_t get_crc8(uint8_t *Buf, uint8_t BufLen) {
  uint8_t crc = 0, i;
  for (i = 0; i < BufLen; i++) {
    crc = update_crc8(Buf[i], crc);
  }
  return (crc);
}
// ===================================

void parseSerial1Data() {
  // === CRC 校验 (当前已按要求注释) ===
  /*
  uint8_t calculatedCRC = get_crc8(serial1_buffer, KISS_FRAME_SIZE - 1); // 校验前9个字节
  uint8_t receivedCRC = serial1_buffer[KISS_FRAME_SIZE - 1]; // 第10个字节是CRC
  if (calculatedCRC != receivedCRC) {
    // Serial.println("Received invalid KISS packet (CRC error).");
    return;
  }
  */
  // =================================

  // Byte 0: Temperature in 1°C
  myEscData.temperature = (int8_t)serial1_buffer[0];

  // Byte 1-2: Voltage high/low byte (Volt * 100)
  uint16_t rawVoltage = (serial1_buffer[1] << 8) | serial1_buffer[2];
  myEscData.voltage = rawVoltage / 100.0f;

  // Byte 3-4: Current high/low byte (Ampere * 100)
  uint16_t rawCurrent = (serial1_buffer[3] << 8) | serial1_buffer[4];
  myEscData.current = rawCurrent / 100.0f;

  // Byte 5-6: Consumption high/low byte (Consumption in 1mAh)
  myEscData.consumption = (serial1_buffer[5] << 8) | serial1_buffer[6];

  // Byte 7-8: Rpm high/low byte (Electrical Rpm / 100)
  uint16_t rawErpm = (serial1_buffer[7] << 8) | serial1_buffer[8];
  myEscData.erpm = (uint32_t)rawErpm * 100;

  myEscData.alive_ts = millis();
}

void serial1DataEvent() {
  while (Serial1.available()) {
    uint32_t current_time = millis();
    
    // 超时重置机制：如果距离上一个字节的接收时间超过 5ms，认为开始了新的数据包
    if (current_time - last_serial1_rx_time > 5) {
      serial1_buffer_index = 0;
    }
    last_serial1_rx_time = current_time;

    uint8_t byte = Serial1.read();

    if (serial1_buffer_index < KISS_FRAME_SIZE) {
      serial1_buffer[serial1_buffer_index++] = byte;
    }

    // 收满 10 个字节后进行解析
    if (serial1_buffer_index >= KISS_FRAME_SIZE) {
      parseSerial1Data();
      serial1_buffer_index = 0; // 解析完成后重置索引
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