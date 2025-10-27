#ifndef MY_POWER_MONITOR_H
#define MY_POWER_MONITOR_H

#include <Adafruit_ADS1X15.h>

class MyPowerMonitor {
private:
    Adafruit_ADS1115 ads;
    const float VOLTAGE_MULTIPLIER = 18.182;
    const float CURRENT_MULTIPLIER = 36.364;
    const uint8_t VOLTAGE_CHANNEL = 0;  // A0 for voltage measuring
    const uint8_t CURRENT_CHANNEL = 1;  // A1 for current measuring
    bool _initialized;
    uint8_t _address;

    float readVoltage();
    float readCurrent();
    int16_t readRawADC(uint8_t channel);  // channel: 0-3, raw adc reading, maximum 65536
    float readChannelVoltage(uint8_t channel);  // channel: 0-3, raw voltage reading

public:
    MyPowerMonitor();
    bool init();
    bool readPower(float &voltage, float &current);  // read
    bool isInitialized();
};

MyPowerMonitor::MyPowerMonitor() : _initialized(false), _address(0x48) {
}

bool MyPowerMonitor::init() {
    _address = 0x48;
    if (!ads.begin(_address)) {
        _initialized = false;
        return false;
    }
    ads.setGain(GAIN_TWOTHIRDS);  // GAIN_TWOTHIRDS = ±6.144V, 1 bit = 0.1875mV
    ads.setDataRate(RATE_ADS1115_128SPS);
    _initialized = true;
    return true;
}

float MyPowerMonitor::readVoltage() {
    float channelVoltage = readChannelVoltage(VOLTAGE_CHANNEL);
    return channelVoltage * VOLTAGE_MULTIPLIER;
}

float MyPowerMonitor::readCurrent() {
    float channelVoltage = readChannelVoltage(CURRENT_CHANNEL);
    return channelVoltage * CURRENT_MULTIPLIER;
}

bool MyPowerMonitor::readPower(float &voltage, float &current) {
    if (!_initialized) {
        voltage = 0.0;
        current = 0.0;
        return false;
    }
    voltage = readVoltage();
    current = readCurrent();
    return true;
}

int16_t MyPowerMonitor::readRawADC(uint8_t channel) {
    if (channel > 3) {
        return 0;
    }
    return ads.readADC_SingleEnded(channel);
}

float MyPowerMonitor::readChannelVoltage(uint8_t channel) {
    if (channel > 3) {
        return 0.0;
    }
    int16_t adc = ads.readADC_SingleEnded(channel);
    return ads.computeVolts(adc);
}

bool MyPowerMonitor::isInitialized() {
    return _initialized;
}

#endif