#ifndef MY_ESC_TELEMETRY_H
#define MY_ESC_TELEMETRY_H

#include <Arduino.h>
#include <HardwareSerial.h>

// BLHeli32 KISS遥测数据结构
struct EscTelemetryData {
  float temperature;      // 温度 (°C)
  float voltage;          // 电压 (V)
  float current;          // 电流 (A)
  float consumption;      // 电量消耗 (mAh)
  uint16_t rpm;          // 转速 (RPM)
  uint8_t crc;           // CRC校验值
  bool valid;            // 数据是否有效
  unsigned long timestamp; // 最后更新时间戳
};

// 统计信息
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
  HardwareSerial* _serial;
  EscTelemetryData _data;
  TelemetryStats _stats;
  
  // KISS协议相关
  static const uint8_t KISS_FRAME_START = 0xC0;
  static const uint8_t KISS_FRAME_END = 0xC0;
  static const uint8_t KISS_TELEMETRY_SIZE = 10;
  
  uint8_t _buffer[16];
  uint8_t _bufferIndex;
  bool _frameStarted;
  
  // 数据更新超时时间(ms)
  unsigned long _dataTimeout;
  
  // 用于计算更新率
  unsigned long _lastUpdateTime;
  uint32_t _updateCount;
  
  // 帧超时检测(ms) - 防止不完整的帧一直占用缓冲区
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
  
  // 解析遥测数据包
  void parseTelemetryFrame() {
    _stats.totalFrames++;
    
    if (_bufferIndex < KISS_TELEMETRY_SIZE) {
      return;
    }
    
    // 验证CRC
    uint8_t receivedCRC = _buffer[KISS_TELEMETRY_SIZE - 1];
    uint8_t calculatedCRC = calculateCRC8(_buffer, KISS_TELEMETRY_SIZE - 1);
    
    if (receivedCRC != calculatedCRC) {
      _data.valid = false;
      _stats.crcErrors++;
      return;
    }
    
    // 解析数据 (BLHeli32 KISS格式)
    _data.temperature = _buffer[0];
    _data.voltage = ((_buffer[1] << 8) | _buffer[2]) / 100.0;
    _data.current = ((_buffer[3] << 8) | _buffer[4]) / 100.0;
    _data.consumption = (_buffer[5] << 8) | _buffer[6];
    _data.rpm = ((_buffer[7] << 8) | _buffer[8]) * 100;
    _data.crc = receivedCRC;
    _data.valid = true;
    _data.timestamp = millis();
    
    _stats.validFrames++;
    _updateCount++;
    
    // 计算更新率 (每秒更新一次统计)
    unsigned long now = millis();
    if (now - _lastUpdateTime >= 1000) {
      _stats.updateRate = _updateCount * 1000.0 / (now - _lastUpdateTime);
      _lastUpdateTime = now;
      _updateCount = 0;
    }
  }
  
  // 重置帧状态
  void resetFrame() {
    _frameStarted = false;
    _bufferIndex = 0;
  }

public:
  // 构造函数
  MyEscTelemetry() {
    _serial = nullptr;
    _bufferIndex = 0;
    _frameStarted = false;
    _dataTimeout = 1000;
    _frameTimeout = 100; // 帧接收超时100ms
    _lastUpdateTime = 0;
    _updateCount = 0;
    _frameStartTime = 0;
    memset(&_data, 0, sizeof(EscTelemetryData));
    memset(&_stats, 0, sizeof(TelemetryStats));
  }
  
  // 初始化接口
  void begin(HardwareSerial* serialPort, long baudRate = 115200, int8_t rxPin = 16, int8_t txPin = -1) {
    _serial = serialPort;
    
    // 设置更大的接收缓冲区 (512字节)
    // 这样即使update()调用频率低也不容易溢出
    if (txPin == -1) {
      _serial->begin(baudRate, SERIAL_8N1, rxPin, -1);
    } else {
      _serial->begin(baudRate, SERIAL_8N1, rxPin, txPin);
    }
    _serial->setRxBufferSize(512);
    
    resetFrame();
    memset(&_data, 0, sizeof(EscTelemetryData));
    memset(&_stats, 0, sizeof(TelemetryStats));
    _stats.lastStatsReset = millis();
    _lastUpdateTime = millis();
  }
  
  // 设置数据超时时间(ms)
  void setTimeout(unsigned long timeout) {
    _dataTimeout = timeout;
  }
  
  // 设置帧接收超时时间(ms)
  void setFrameTimeout(unsigned long timeout) {
    _frameTimeout = timeout;
  }
  
  // 更新函数 - 建议在loop中调用，但不需要很高频率
  // 最低10Hz (每100ms调用一次) 也可以正常工作
  void update() {
    if (!_serial) return;
    
    unsigned long now = millis();
    
    // 检查帧接收超时
    if (_frameStarted && (now - _frameStartTime > _frameTimeout)) {
      resetFrame();
    }
    
    // 一次性读取所有可用数据，避免遗漏
    while (_serial->available()) {
      uint8_t byte = _serial->read();
      
      // 检测帧起始
      if (byte == KISS_FRAME_START) {
        if (!_frameStarted) {
          _frameStarted = true;
          _bufferIndex = 0;
          _frameStartTime = now;
        } else {
          // 帧结束
          parseTelemetryFrame();
          resetFrame();
        }
      } else if (_frameStarted && _bufferIndex < sizeof(_buffer)) {
        _buffer[_bufferIndex++] = byte;
      }
    }
    
    // 检查数据超时
    if (_data.valid && (now - _data.timestamp > _dataTimeout)) {
      _data.valid = false;
      _stats.timeoutErrors++;
    }
  }
  
  // 数据读取接口
  float getTemperature() { return _data.temperature; }
  float getVoltage() { return _data.voltage; }
  float getCurrent() { return _data.current; }
  float getConsumption() { return _data.consumption; }
  uint16_t getRPM() { return _data.rpm; }
  float getPower() { return _data.voltage * _data.current; }
  bool isValid() { return _data.valid; }
  unsigned long getLastUpdateTime() { return _data.timestamp; }
  EscTelemetryData getData() { return _data; }
  
  // 获取统计信息
  TelemetryStats getStats() { return _stats; }
  
  // 获取更新频率 (Hz)
  float getUpdateRate() { return _stats.updateRate; }
  
  // 重置统计信息
  void resetStats() {
    memset(&_stats, 0, sizeof(TelemetryStats));
    _stats.lastStatsReset = millis();
    _lastUpdateTime = millis();
    _updateCount = 0;
  }
  
  // 获取串口缓冲区剩余字节数
  int available() {
    return _serial ? _serial->available() : 0;
  }
  
  // 打印所有遥测数据
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
  
  // 打印统计信息
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