#ifndef MY_PWM_READER_H
#define MY_PWM_READER_H

#include <Arduino.h>

class MyPwmReader {
private:
    uint8_t pin;
    volatile unsigned long riseTime;
    volatile unsigned long pulseWidth;
    volatile bool newData;
    
    static MyPwmReader* instance;
    static void IRAM_ATTR handleInterrupt() {
        if (instance) {
            instance->handlePulse();
        }
    }

    void IRAM_ATTR handlePulse() {
        unsigned long currentTime = micros();
        
        if (digitalRead(pin) == HIGH) {
            riseTime = currentTime;
        } else {
            if (riseTime > 0) {
                pulseWidth = currentTime - riseTime;
                newData = true;
            }
        }
    }

public:
    MyPwmReader(uint8_t inputPin = 27) : pin(inputPin), riseTime(0), pulseWidth(1500), newData(false) {
        instance = this;
    }

    void begin() {
        pinMode(pin, INPUT);
        attachInterrupt(digitalPinToInterrupt(pin), handleInterrupt, CHANGE);
    }

    unsigned long getRawPulseWidth() {  // in ms
        return pulseWidth;
    }
    
    // 获取油门值 (0.0 - 1.0)
    // Frsky X8R典型PWM范围: 988us (最小) 到 2012us (最大)
    // 中点约为 1500us
    float getThrottle() {
        const float PWM_MIN = 988.0;   // 最小PWM值
        const float PWM_MAX = 2012.0;  // 最大PWM值

        float pw = constrain(pulseWidth, PWM_MIN, PWM_MAX);
        float throttle = (pw - PWM_MIN) / (PWM_MAX - PWM_MIN);
        
        return throttle;
    }

    bool hasNewData() {
        if (newData) {
            newData = false;
            return true;
        }
        return false;
    }

    bool isSignalValid(unsigned long timeoutMs = 100) {
        static unsigned long lastUpdate = 0;
        
        if (hasNewData()) {
            lastUpdate = millis();
            return true;
        }
        
        return (millis() - lastUpdate) < timeoutMs;
    }

    void end() {
        detachInterrupt(digitalPinToInterrupt(pin));
    }
};

MyPwmReader* MyPwmReader::instance = nullptr;

#endif // MY_PWM_READER_H