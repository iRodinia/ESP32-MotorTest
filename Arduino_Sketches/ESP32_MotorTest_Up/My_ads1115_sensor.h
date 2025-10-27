#ifndef MY_ADS1115_H
#define MY_ADS1115_H

#include <Adafruit_ADS1X15.h>

/* This module is used to measure the analog output from the power module PM02 and the differential output from the force gauge */
// A0: voltage from PM02
// A1: current from PM02
// A2: force gauge voltage +
// A3: force gauge voltage -

class MyADS1115Sensor {
private:
    Adafruit_ADS1115 ads;

    const uint8_t MODULE_ADDRESS = 0x48;
    const float VOLTAGE_MULTIPLIER = 18.182;
    const float CURRENT_MULTIPLIER = 36.364;
    const float FORCE_VOLT_LIMIT = 3.3;

    float readPM02VoltRaw() {
        int16_t adc = ads.readADC_SingleEnded(0);  // channel: 0-3, raw adc reading, maximum 65536
        return ads.computeVolts(adc);
    }
    float readPM02CurrRaw() {
        int16_t adc = ads.readADC_SingleEnded(1);
        return ads.computeVolts(adc);
    }
    float readForceVoltRaw() {
        int16_t adc = ads.readADC_Differential_2_3();
        return ads.computeVolts(adc);
    }

    bool _initialized;
    float _max_force = 10 * 9.81f;
    float _zero_force_volt = 0; 

public:
    MyADS1115Sensor();
    String init(float max_force = 10 * 9.81f);
    void forceCalibrate();
    void readPower(float &voltage, float &current, float &power);
    void readForce(float &force);
    bool isInitialized();
};

MyADS1115Sensor::MyADS1115Sensor() : _initialized(false) {}

String MyADS1115Sensor::init(float max_force) {
    uint8_t init_count = 0;
    while(!ads.begin(MODULE_ADDRESS) && init_count < 10){
        init_count++;
        delay(50);
    }
    if (init_count >= 10) {
        _initialized = false;
        return "ADS1115 initialization failed. ";
    }
    ads.setGain(GAIN_TWOTHIRDS);  // GAIN_TWOTHIRDS = ±6.144V, 1 bit = 0.1875mV
    ads.setDataRate(RATE_ADS1115_128SPS);
    _max_force = max_force;

    forceCalibrate();
    if (abs(_zero_force_volt) > 3.3) {
        _initialized = false;
        return "ADS1115 force sensor calibration failed. ";
    }

    _initialized = true;
    return "";
}

void MyADS1115Sensor::forceCalibrate() {
  float sum_volt = 0.0f;
  for(int i=0; i<200; i++){
    sum_volt += readForceVoltRaw();
    delay(10);
  }
  _zero_force_volt = sum_volt / 200.0f;
}

void MyADS1115Sensor::readPower(float &voltage, float &current, float &power) {
    if (!_initialized) {
        voltage = 0.0;
        current = 0.0;
        power = 0.0;
    }
    voltage = readPM02VoltRaw() * VOLTAGE_MULTIPLIER;
    current = readPM02CurrRaw() * CURRENT_MULTIPLIER;
    power = voltage * current;
}

void MyADS1115Sensor::readForce(float &force) {
    float volt_read = readForceVoltRaw() - _zero_force_volt;
    force = volt_read / FORCE_VOLT_LIMIT * _max_force;
}

bool MyADS1115Sensor::isInitialized() {
    return _initialized;
}

#endif