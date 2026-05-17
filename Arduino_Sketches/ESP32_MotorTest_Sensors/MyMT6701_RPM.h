#ifndef MT6701_RPM_H__
#define MT6701_RPM_H__

#define MT6701_SSI_CLOCK 4000000  // default 1000000, must be defined before include "MT6701.h"

#include <Arduino.h>
#include <SPI.h>
#include "mt6701/MT6701.h"

// Maximum angle change threshold. If greater than 180 maybe inverse.
#ifndef MT6701RPM_WRAP_THRESHOLD_DEG
#define MT6701RPM_WRAP_THRESHOLD_DEG  180.0f
#endif

/** 低通滤波系数 α（0 < α ≤ 1）。
 * α = 1 表示不滤波，直接输出原始转速；
 * α 越小，滤波越强，响应越慢。
 * 推荐值：0.1 ~ 0.3（中等平滑）。 */
#ifndef MT6701RPM_LPF_ALPHA
#define MT6701RPM_LPF_ALPHA  0.5f
#endif

/** 最小有效采样间隔（µs）。
 * 若两次 update() 调用间隔小于此值，则跳过本次计算以避免除零或噪声放大。 */
#ifndef MT6701RPM_MIN_DT_US
#define MT6701RPM_MIN_DT_US  500UL
#endif

/* ------------------------------------------------------------------ */

/**
 * @brief MT6701 角度 + 转速测量类（已优化支持多核并发读取）
 */
class MT6701RPM {
public:
    /* ===================== 构造 / 初始化 ========================== */

    /**
     * @param cs_pin        SPI 片选引脚（可为任意 GPIO）
     * @param lpf_alpha     低通滤波系数，默认使用宏定义值
     * @param wrap_threshold 跨零检测阈值（°），默认使用宏定义值
     */
    explicit MT6701RPM(int cs_pin,
                       float lpf_alpha     = MT6701RPM_LPF_ALPHA,
                       float wrap_threshold = MT6701RPM_WRAP_THRESHOLD_DEG)
        : _cs_pin(cs_pin)
        , _alpha(lpf_alpha)
        , _wrapThreshold(wrap_threshold)
        , _angleRaw(0.0f)
        , _rpmFiltered(0.0f)
        , _lastAngle(0.0f)
        , _lastTimestamp(0)
        , _initialized(false)
    {}

    /**
     * @brief  初始化 SPI 总线并使能传感器
     * @param  initSPI  是否需要初始化SPI，默认需要
     * @return 初始化信息
     */
    String begin(bool initSPI = true) {
        if (initSPI) {
            SPI.begin();
        }

        if (!_encoder.initializeSSI(_cs_pin)) {
            return "MT6701 initialization failed.";
        }

        // 读取一次作为基准，避免首次 update() 产生虚假转速
        _angleRaw    = _encoder.angleRead();
        _lastAngle   = _angleRaw;
        _lastTimestamp = micros();
        _initialized = true;
        return "";
    }

    /* ===================== 核心更新函数 =========================== */

    /**
     * @brief  读取最新角度并更新转速估算。
     * 建议以固定周期（1 ~ 5 ms）调用，可放在 loop() 或硬件定时器 ISR 中。
     *
     * @note   若两次调用间隔 < MT6701RPM_MIN_DT_US，本次计算将被跳过。
     */
    void update() {
        if (!_initialized) return;

        uint32_t now = micros();
        uint32_t dt  = now - _lastTimestamp; // 无符号差值，自动处理溢出

        if (dt < MT6701RPM_MIN_DT_US) return;

        // --- 读取当前角度 ---
        float angle = _encoder.angleRead();
        _angleRaw   = angle;

        // --- 计算角度差（跨零点处理）---
        float delta = angle - _lastAngle;

        // 跨零点修正：将 delta 归一化到 (-180, +180]
        if (delta >  _wrapThreshold) delta -= 360.0f;
        if (delta < -_wrapThreshold) delta += 360.0f;

        // --- 瞬时转速（RPM）= (度/周期) / 360° × 60 s/min × (1e6 µs/s / dt µs) ---
        //     = delta × 60,000,000 / (360 × dt)
        //     = delta × 166666.67 / dt
        float rpmInstant = delta * (166666.667f / (float)dt);

        // --- 一阶低通滤波 ---
        _rpmFiltered = _alpha * rpmInstant + (1.0f - _alpha) * _rpmFiltered;

        _lastAngle     = angle;
        _lastTimestamp = now;
    }

    /* ===================== 数据读取接口 =========================== */

    /**
     * @brief  获取最新原始角度（°）
     * @return [0, 360) 范围内的角度值
     */
    float getAngle() const { return _angleRaw; }

    /**
     * @brief  获取经低通滤波后的转速（跨核读取安全）
     * @return 转速（RPM），正值为正转，负值为反转
     */
    float getRPM() const { return _rpmFiltered; }

    /**
     * @brief  获取转速绝对值（无符号 RPM）
     */
    float getRPMAbs() const { return fabsf(_rpmFiltered); }

    /**
     * @brief  获取旋转方向
     * @return  1 = 正转（角度递增方向）
     * -1 = 反转
     * 0 = 静止（|RPM| < deadband）
     * @param  deadbandRPM  认为静止的转速门限，默认 1 RPM
     */
    int getDirection(float deadbandRPM = 1.0f) const {
        if (_rpmFiltered >  deadbandRPM) return  1;
        if (_rpmFiltered < -deadbandRPM) return -1;
        return 0;
    }

    /**
     * @brief  读取磁场状态（需要 SPI 接口）
     * @return MT6701_STATUS_NORM=0x0 / FIELD_STRONG=0x1 / FIELD_WEAK=0x2 / FIELD_ERROR=0x3
     */
    mt6701_status_t getFieldStatus() {
        return _encoder.fieldStatusRead();
    }

    /* ===================== 配置接口 ================================ */

    /**
     * @brief  运行时修改低通滤波系数
     * @param  alpha  (0, 1]，越大响应越快，越小越平滑
     */
    void setLPFAlpha(float alpha) {
        _alpha = constrain(alpha, 0.001f, 1.0f);
    }

    /**
     * @brief  清零滤波器状态（换挡或重启后调用）
     */
    void resetFilter() {
        _rpmFiltered = 0.0f;
    }

    /**
     * @brief  返回底层 MT6701 对象引用，供高级用户直接访问
     */
    MT6701& raw() { return _encoder; }

private:
    int          _cs_pin;
    float        _alpha;
    float        _wrapThreshold;

    // 关键修改：加上 volatile 确保跨核并发读取时的数据即时可见性
    volatile float        _angleRaw;
    volatile float        _rpmFiltered;
    volatile float        _lastAngle;
    volatile uint32_t     _lastTimestamp;
    bool                  _initialized;

    MT6701       _encoder;
};

#endif // MT6701_RPM_H__