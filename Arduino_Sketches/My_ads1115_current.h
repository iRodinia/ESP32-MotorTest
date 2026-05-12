#ifndef MY_ADS1115_CURRENT_H
#define MY_ADS1115_CURRENT_H

#include <Adafruit_ADS1X15.h>

/* 
 * 本模块用于通过ADS1115读取3路电流计的输出电压，并转换为电流值
 * ADDR引脚接VCC，I2C地址为0x49
 * 通道分配：
 *   A0: 电流计 #0
 *   A1: 电流计 #1
 *   A2: 电流计 #2
 * 电压→电流转换公式：A = (V - 2.5) * 50
 */

class MyADS1115Current {
private:
    Adafruit_ADS1115 ads;

    const uint8_t  MODULE_ADDRESS  = 0x49;        // ADDR接VCC时地址为0x49
    const float    ADC_LSB         = 0.0001875f;  // GAIN_TWOTHIRDS: 1 bit = 0.1875 mV
    const float    ZERO_VOLT       = 2.5f;        // 零电流对应的输出电压 (V)
    const float    VOLT_TO_AMP     = 50.0f;       // 灵敏度系数

    bool _initialized;

    /* 读取指定通道的原始ADC值 (channel: 0~2) */
    int16_t readRaw(uint8_t channel) {
        return ads.readADC_SingleEnded(channel);
    }

    /* 将原始ADC值转换为电压 (V) */
    float rawToVolt(int16_t raw) {
        return raw * ADC_LSB;
    }

    /* 将电压转换为电流 (A)，公式：A = (V - 2.5) * 50 */
    float voltToAmp(float volt) {
        return (volt - ZERO_VOLT) * VOLT_TO_AMP;
    }

public:
    MyADS1115Current();

    /* 初始化，返回空字符串表示成功，否则返回错误信息 */
    String init();

    /* 读取单通道原始电压 (V)，channel: 0~2 */
    float readVoltage(uint8_t channel);

    /* 读取单通道转换后电流 (A)，channel: 0~2 */
    float readCurrent(uint8_t channel);

    /* 一次性读取全部3路电压，结果写入 v0, v1, v2 */
    void readAllVoltages(float &v0, float &v1, float &v2);

    /* 一次性读取全部3路电流，结果写入 a0, a1, a2 */
    void readAllCurrents(float &a0, float &a1, float &a2);

    bool isInitialized();
};

/* ──────────────────── 实现 ──────────────────── */

MyADS1115Current::MyADS1115Current() : _initialized(false) {}

String MyADS1115Current::init() {
    uint8_t init_count = 0;
    while (!ads.begin(MODULE_ADDRESS) && init_count < 10) {
        init_count++;
        delay(50);
    }
    if (init_count >= 10) {
        _initialized = false;
        return "ADS1115 initialization failed. ";
    }

    // GAIN_TWOTHIRDS: 量程 ±6.144V，1 bit = 0.1875 mV
    // 覆盖电流计输出范围 0~5V，留有余量
    ads.setGain(GAIN_TWOTHIRDS);
    ads.setDataRate(RATE_ADS1115_128SPS);

    _initialized = true;
    return "";
}

float MyADS1115Current::readVoltage(uint8_t channel) {
    if (!_initialized || channel > 2) return 0.0f;
    return rawToVolt(readRaw(channel));
}

float MyADS1115Current::readCurrent(uint8_t channel) {
    if (!_initialized || channel > 2) return 0.0f;
    return voltToAmp(readVoltage(channel));
}

void MyADS1115Current::readAllVoltages(float &v0, float &v1, float &v2) {
    if (!_initialized) {
        v0 = v1 = v2 = 0.0f;
        return;
    }
    v0 = readVoltage(0);
    v1 = readVoltage(1);
    v2 = readVoltage(2);
}

void MyADS1115Current::readAllCurrents(float &a0, float &a1, float &a2) {
    if (!_initialized) {
        a0 = a1 = a2 = 0.0f;
        return;
    }
    a0 = readCurrent(0);
    a1 = readCurrent(1);
    a2 = readCurrent(2);
}

bool MyADS1115Current::isInitialized() {
    return _initialized;
}

#endif // MY_ADS1115_CURRENT_H