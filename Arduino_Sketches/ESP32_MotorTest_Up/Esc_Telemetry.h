#ifndef MY_ESC_TELEMETRY_H
#define MY_ESC_TELEMETRY_H

#include <Arduino.h>
#include <SoftwareSerial.h>

// BLHeli32 KISS structure
struct EscTelemetryData {
  float temperature;      // 温度 (°C)
  float voltage;          // 电压 (V)
  float current;          // 电流 (A)
  float consumption;      // 电量消耗 (mAh)
  uint16_t erpm;          // 电子转速 (ERPM)
  uint8_t crc;           // CRC校验值
  bool valid;            // 数据是否有效
  unsigned long timestamp; // 最后更新时间戳
};

// reading statics
struct TelemetryStats {
  uint32_t totalFrames;      // 总接收帧数
  uint32_t validFrames;      // 有效帧数
  uint32_t crcErrors;        // CRC错误数
  uint32_t timeoutErrors;    // 超时错误数
  float updateRate;          // 更新频率 (Hz)
  unsigned long lastStatsReset; // 上次统计重置时间
};

class MyEscTelemetry {
private:
  SoftwareSerial* _serial;
  EscTelemetryData _data;
  TelemetryStats _stats;

  static const uint8_t KISS_FRAME_START = 0xC0;
  static const uint8_t KISS_FRAME_END = 0xC0;
  static const uint8_t KISS_TELEMETRY_SIZE = 10;
  
  uint8_t _buffer[16];
  uint8_t _bufferIndex;
  bool _frameStarted;

  uint8_t _motor_pole_num;  // motor pole 极对数
  unsigned long _dataTimeout;
  unsigned long _lastUpdateTime;
  uint32_t _updateCount;
  unsigned long _frameTimeout;
  unsigned long _frameStartTime;
  
  // CRC8校验
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

  void parseTelemetryFrame() {
    _stats.totalFrames++;
    if (_bufferIndex < KISS_TELEMETRY_SIZE) {
      return;
    }

    uint8_t receivedCRC = _buffer[KISS_TELEMETRY_SIZE - 1];
    uint8_t calculatedCRC = calculateCRC8(_buffer, KISS_TELEMETRY_SIZE - 1);
    if (receivedCRC != calculatedCRC) {
      _data.valid = false;
      _stats.crcErrors++;
      return;
    }

    _data.temperature = _buffer[0];
    _data.voltage = ((_buffer[1] << 8) | _buffer[2]) / 100.0;
    _data.current = ((_buffer[3] << 8) | _buffer[4]) / 100.0;
    _data.consumption = (_buffer[5] << 8) | _buffer[6];
    _data.erpm = ((_buffer[7] << 8) | _buffer[8]) * 100;
    _data.crc = receivedCRC;
    _data.valid = true;
    _data.timestamp = millis();
    
    _stats.validFrames++;
    _updateCount++;

    unsigned long now = millis();
    if (now - _lastUpdateTime >= 1000) {
      _stats.updateRate = _updateCount * 1000.0 / (now - _lastUpdateTime);
      _lastUpdateTime = now;
      _updateCount = 0;
    }
  }

  void resetFrame() {
    _frameStarted = false;
    _bufferIndex = 0;
  }

public:
  MyEscTelemetry() {
    _serial = nullptr;
    _bufferIndex = 0;
    _frameStarted = false;
    _dataTimeout = 1000;
    _frameTimeout = 100;
    _lastUpdateTime = 0;
    _updateCount = 0;
    _frameStartTime = 0;
    _motor_pole_num = 7;
  }

  ~MyEscTelemetry() {
    if(_serial){
      delete(_serial);
    }
  }

  void begin(int8_t rxPin = 26, int8_t txPin = -1, long baudRate = 115200) {
    _serial = new SoftwareSerial(rxPin, txPin);
    _serial->begin(baudRate);
    Serial.print("ESC serial initializing.");
    while(!_serial){
      Serial.print(".");
      delay(50);
    }
    Serial.printf("\n");
    
    resetFrame();
    memset(&_data, 0, sizeof(EscTelemetryData));
    memset(&_stats, 0, sizeof(TelemetryStats));
    _stats.lastStatsReset = millis();
    _lastUpdateTime = millis();
  }
  
  void setTimeout(unsigned long timeout) {
    _dataTimeout = timeout;
  }

  void setFrameTimeout(unsigned long timeout) {
    _frameTimeout = timeout;
  }

  void update() {
    if (!_serial) return;
    unsigned long now = millis();
    if (_frameStarted && (now - _frameStartTime > _frameTimeout)) {
      resetFrame();
    }

    while (_serial->available()) {
      uint8_t byte = _serial->read();

      if (byte == KISS_FRAME_START) {
        if (!_frameStarted) {
          _frameStarted = true;
          _bufferIndex = 0;
          _frameStartTime = now;
        } else {
          parseTelemetryFrame();
          resetFrame();
        }
      } else if (_frameStarted && _bufferIndex < sizeof(_buffer)) {
        _buffer[_bufferIndex++] = byte;
      }
    }

    if (_data.valid && (now - _data.timestamp > _dataTimeout)) {
      _data.valid = false;
      _stats.timeoutErrors++;
    }
  }

  float getTemperature() { return _data.temperature; }
  float getVoltage() { return _data.voltage; }
  float getCurrent() { return _data.current; }
  float getConsumption() { return _data.consumption; }
  uint16_t getERPM() { return _data.erpm; }
  float getRPMRaw() { return getERPM() / float(_motor_pole_num); }
  float getPower() { return _data.voltage * _data.current; }
  bool isValid() { return _data.valid; }
  unsigned long getLastUpdateTime() { return _data.timestamp; }
  EscTelemetryData getData() { return _data; }
  TelemetryStats getStats() { return _stats; }
  float getUpdateRate() { return _stats.updateRate; }

  void resetStats() {
    memset(&_stats, 0, sizeof(TelemetryStats));
    _stats.lastStatsReset = millis();
    _lastUpdateTime = millis();
    _updateCount = 0;
  }

  int available() {
    return _serial ? _serial->available() : 0;
  }

  void printData(Stream* output = &Serial) {
    if (_data.valid) {
      output->println("=== ESC Telemetry ===");
      output->print("Temperature: "); output->print(_data.temperature); output->println(" °C");
      output->print("Voltage: "); output->print(_data.voltage, 2); output->println(" V");
      output->print("Current: "); output->print(_data.current, 2); output->println(" A");
      output->print("Power: "); output->print(getPower(), 2); output->println(" W");
      output->print("Consumption: "); output->print(_data.consumption); output->println(" mAh");
      output->print("RPM: "); output->println(_data.rpm);
      output->println("====================");
    } else {
      output->println("ESC Telemetry: No valid data");
    }
  }

  void printStats(Stream* output = &Serial) {
    output->println("=== Telemetry Stats ===");
    output->print("Total Frames: "); output->println(_stats.totalFrames);
    output->print("Valid Frames: "); output->println(_stats.validFrames);
    output->print("CRC Errors: "); output->println(_stats.crcErrors);
    output->print("Timeout Errors: "); output->println(_stats.timeoutErrors);
    output->print("Update Rate: "); output->print(_stats.updateRate, 1); output->println(" Hz");
    output->print("Buffer Available: "); output->println(available());
    if (_stats.totalFrames > 0) {
      float successRate = (_stats.validFrames * 100.0) / _stats.totalFrames;
      output->print("Success Rate: "); output->print(successRate, 1); output->println(" %");
    }
    output->println("=======================");
  }
};

#endif