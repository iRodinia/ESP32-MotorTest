/**
 * @file    ESP32_WROOM_DA_RPM.ino
 * @brief   MT6701 转速测量示例 —— ESP32-WROOM-DA
 *
 * 硬件连接（ESP32-WROOM-DA 默认 VSPI）：
 * ┌──────────────────┬────────────┬──────────────────────────────┐
 * │   MT6701 引脚    │  ESP32引脚 │          说明                │
 * ├──────────────────┼────────────┼──────────────────────────────┤
 * │  CLK (时钟)      │   GPIO18   │  VSPI SCK（硬件固定）        │
 * │  DO  (数据输出)  │   GPIO19   │  VSPI MISO（硬件固定）       │
 * │  CSN (片选)      │   GPIO5    │  可改为任意空闲 GPIO         │
 * │  VDD             │   3.3V     │                              │
 * │  GND             │   GND      │                              │
 * └──────────────────┴────────────┴──────────────────────────────┘
 *
 * 注意：
 *  1. ESP32-WROOM-DA 的 GPIO6~11 内部连接 Flash，禁止用作 SPI。
 *  2. 若板子上已有其他 SPI 设备，共用 CLK/MISO 即可，CS 各自独立。
 *  3. 本示例使用硬件定时器（timerAttachInterrupt）实现精确 2 ms 采样，
 *     比 delay() 轮询精度高得多，低速时也能准确测量。
 *
 * 依赖库：MT6701（含 MT6701.h/.cpp/mt6701_utils.h/.c）、MT6701RPM.h
 */

#include <Arduino.h>
#include <SPI.h>
#include "MyMT6701_RPM.h"

/* -------------------- 引脚定义 -------------------- */
#define CS_PIN      5       // 片选，可改为其他空闲 GPIO

/* -------------------- 采样周期 -------------------- */
#define SAMPLE_PERIOD_MS   2    // 2 ms 采样间隔 → 最高可分辨约 0.04 RPM
#define PRINT_PERIOD_MS   100

/* -------------------- 定时器相关 ------------------- */
hw_timer_t *sampleTimer = nullptr;
volatile bool sampleFlag = false;   // 定时器置位，主循环消费

// 定时器 ISR：仅置位标志，避免在 ISR 中执行 SPI（SPI 非中断安全）
void IRAM_ATTR onSampleTimer() {
    sampleFlag = true;
}

/* -------------------- 对象实例化 ------------------- */
MT6701RPM motorEncoder(CS_PIN);

uint32_t lastPrintMs = 0;

/* ================================================== */
void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    Serial.println("\n===== MT6701 RPM Demo | ESP32-WROOM-DA =====");

    // 初始化编码器（内部调用 SPI.begin()）
    if (!motorEncoder.begin()) {
        Serial.println("[ERROR] MT6701 初始化失败！请检查接线。");
        while (true) delay(1000);
    }
    Serial.println("[INFO]  MT6701 初始化成功");

    // 创建定时器，计数频率 1 MHz（即每个 tick = 1 µs）
    // timerBegin() 内部自动选择时钟源和分频系数，无需手动指定
    sampleTimer = timerBegin(1000000);
    if (sampleTimer == nullptr) {
        Serial.println("[ERROR] 定时器创建失败！");
        while (true) delay(1000);
    }
    // 绑定 ISR，新版 API 不再需要第三个 edge 参数
    timerAttachInterrupt(sampleTimer, &onSampleTimer);
    // 设置 alarm：每 SAMPLE_PERIOD_MS × 1000 tick 触发一次（= SAMPLE_PERIOD_MS ms）
    // 参数：timer, alarm计数值, 自动重载, 重载起始值(0=从头计数)
    timerAlarm(sampleTimer, (uint64_t)SAMPLE_PERIOD_MS * 1000UL, true, 0);

    Serial.println("[INFO]  采样定时器已启动，周期 = " + String(SAMPLE_PERIOD_MS) + " ms");
    Serial.println("格式: 角度(°) | 转速(RPM) | 方向 | 磁场状态");
}

void loop() {
    /* ---------- 定时采样 ---------- */
    if (sampleFlag) {
        sampleFlag = false;
        motorEncoder.update();  // 读角度 + 刷新 RPM 滤波器
    }

    /* ---------- 定时打印 ---------- */
    uint32_t now = millis();
    if (now - lastPrintMs >= PRINT_PERIOD_MS) {
        lastPrintMs = now;

        float angle = motorEncoder.getAngle();
        float rpm   = motorEncoder.getRPM();
        int   dir   = motorEncoder.getDirection(2.0f); // 2 RPM 以下视为静止

        mt6701_status_t fieldSt = motorEncoder.getFieldStatus();
        const char* fieldStr = "UNKNOWN";
        switch (fieldSt) {
            case MT6701_STATUS_NORM:         fieldStr = "NORM";   break;
            case MT6701_STATUS_FIELD_STRONG: fieldStr = "STRONG"; break;
            case MT6701_STATUS_FIELD_WEAK:   fieldStr = "WEAK";   break;
            default:                         fieldStr = "ERROR";  break;
        }

        const char* dirStr = (dir > 0) ? "CW " : (dir < 0) ? "CCW" : "---";

        // 格式化输出，方便 Serial Plotter 绘图
        Serial.printf("Angle:%.2f\tRPM:%.1f\tDir:%s\tField:%s\n",
                      angle, rpm, dirStr, fieldStr);
    }

}
