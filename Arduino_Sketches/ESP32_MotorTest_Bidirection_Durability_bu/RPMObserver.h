#ifndef RPM_OBSERVER_H
#define RPM_OBSERVER_H

#include <Arduino.h>

class RpmObserver {
private:
    float alpha;         // 位置（转速）增益，信任最新测量值的程度
    float beta;          // 速度（加速度）增益，信任加速度变化的程度
    int polePairs;       // 电机极对数
    
    float estRpm;        // 估计的当前转速 (RPM)
    float estAccel;      // 估计的加速度 (RPM/s)
    unsigned long lastUpdateMicros; // 上次收到数据的微秒时间戳
    bool initialized;

public:
    /**
     * @brief 构造函数
     * @param polePairs 电机极对数 (例如: 14极电机为7)
     * @param alpha 取值 0~1。越小越平滑，但响应越慢。建议 0.1 ~ 0.3
     * @param beta  取值 0~1。越小加速度越稳定。建议 0.001 ~ 0.05
     */
    RpmObserver(int polePairs, float alpha = 0.15f, float beta = 0.005f) {
        this->polePairs = polePairs;
        this->alpha = alpha;
        this->beta = beta;
        this->estRpm = 0.0f;
        this->estAccel = 0.0f;
        this->lastUpdateMicros = 0;
        this->initialized = false;
    }

    /**
     * @brief 每次收到 BLHeli32 遥测的 ERPM 时调用此函数
     * @param rawErpm 电调传回的原始 ERPM 数据
     */
    void update(int rawErpm) {
        unsigned long now = micros();
        float measuredRpm = (float)rawErpm / polePairs;

        // 初始化第一次数据
        if (!initialized) {
            estRpm = measuredRpm;
            estAccel = 0.0f;
            lastUpdateMicros = now;
            initialized = true;
            return;
        }

        // 计算两次数据接收的间隔时间 (秒)
        float dt = (now - lastUpdateMicros) / 1000000.0f;
        if (dt <= 0.0001f) return; // 避免除以0或极小值

        // 1. 状态预测 (State Prediction)
        float predRpm = estRpm + estAccel * dt;

        // 2. 计算误差 (Innovation)
        float error = measuredRpm - predRpm;

        // 3. 状态更新 (State Update)
        estRpm = predRpm + alpha * error;
        estAccel = estAccel + (beta / dt) * error;

        lastUpdateMicros = now;
    }

    /**
     * @brief 在控制循环中随时调用，获取平滑且无延迟的实时转速
     * @return 预测的当前实际转速
     */
    float getRpm() {
        if (!initialized) return 0.0f;
        
        unsigned long now = micros();
        float dt = (now - lastUpdateMicros) / 1000000.0f;

        // 安全机制：如果超过 100ms 没收到新数据，限制预测时间，防止加速度过大导致数据跑飞
        if (dt > 0.1f) {
            dt = 0.1f; 
            estAccel *= 0.95f; // 逐渐衰减加速度，模拟阻力
        }

        // 结合当前时间与加速度，向前预测此时此刻的转速
        return estRpm + estAccel * dt;
    }
    
    // 获取当前估计的加速度 (可用于前馈控制)
    float getAcceleration() {
        return estAccel;
    }
};

#endif