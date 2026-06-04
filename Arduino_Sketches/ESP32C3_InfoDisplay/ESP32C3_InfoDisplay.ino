// ESP32-C3 — 两块 SSD1306 128x64 OLED + UART1 数据显示
//
// 接线：
//   SCL  → GPIO0       SDA  → GPIO1
//   RX1  → GPIO20      TX1  → GPIO21
//   LED  → GPIO8       BUTTON → GPIO10（另端接 GND，按下低电平）
//   屏幕 INPUT  I2C 地址: 0x7A
//   屏幕 OUTPUT I2C 地址: 0x78

#include <Arduino.h>
#include <Wire.h>
#include "12864_display.h"
#include "Serial_manager.h"

// ── 引脚定义 ──────────────────────────────────────────────────────────────────
#define PIN_LED     8
#define PIN_BUTTON  10
#define PIN_SCL     0
#define PIN_SDA     1

// ── 屏幕实例（地址已按实物对应）─────────────────────────────────────────────
SingleScreen screenA(PIN_SCL, PIN_SDA, 0x7A);   // INPUT  面板
SingleScreen screenB(PIN_SCL, PIN_SDA, 0x78);   // OUTPUT 面板

// ── 运行状态 ──────────────────────────────────────────────────────────────────
int  current_channel = 0;   // 当前显示的通道（0 ~ MAX_CHANNEL_NUM-1）
bool led_state       = false;

// ── 按钮消抖 ──────────────────────────────────────────────────────────────────
namespace Btn {
  bool     lastRaw      = HIGH;
  bool     lastStable   = HIGH;
  uint32_t lastChangeMs = 0;
  const uint32_t DEBOUNCE_MS = 30;

  // 返回 true 表示检测到一次上升沿（按钮松开）
  bool risingEdge() {
    bool raw = digitalRead(PIN_BUTTON);
    uint32_t now = millis();

    if (raw != lastRaw) {
      lastRaw      = raw;
      lastChangeMs = now;
    }

    if ((now - lastChangeMs) >= DEBOUNCE_MS && raw != lastStable) {
      lastStable = raw;
      if (raw == HIGH) return true;   // LOW→HIGH 上升沿
    }
    return false;
  }
}

// ═════════════════════════════════════════════════════════════════════════════
void setup() {
  pinMode(PIN_LED,    OUTPUT);
  pinMode(PIN_BUTTON, INPUT_PULLUP);   // 按下接 GND → 低电平
  digitalWrite(PIN_LED, LOW);

  initAllSerials();
  Wire.begin(PIN_SDA, PIN_SCL);

  screenA.setChannel(current_channel);
  screenA.setHeadLine("INPUT");
  screenA.setLineLabels("Curr:", "Volt:", "CMD :", "Rsrv:");
  screenA.init();

  screenB.setChannel(current_channel);
  screenB.setHeadLine("OUTPUT");
  screenB.setLineLabels("TDeg:", "Forc:", "RPM :", "Rsrv:");
  screenB.init();

}

// ═════════════════════════════════════════════════════════════════════════════
void loop() {
  static float prev_time = 0.0;
  serial1CmdEvent();

  bool data_changed = false;
  if (channel_data[current_channel].time != prev_time) {
    prev_time = channel_data[current_channel].time;
    data_changed = true;
  }

  if (Btn::risingEdge()) {
    current_channel = (current_channel + 1) % MAX_CHANNEL_NUM;
    screenA.setChannel(current_channel);
    screenA.clear();
    screenB.setChannel(current_channel);
    screenB.clear();
    prev_time = 0.0;
  }

  Sensor_Data& d = channel_data[current_channel];
  screenA.updateData(d.esc_current, d.esc_voltage, d.esc_command, 0.0);
  screenB.updateData(d.esc_temperature, d.motor_force, d.motor_rpm, 0.0);
  screenA.refresh();
  screenB.refresh();

  if (data_changed) {
    led_state = !led_state;
    digitalWrite(PIN_LED, led_state ? HIGH : LOW);

    Serial.printf(
      "CH%d | t=%.2fs | V=%.2fV | I=%.2fA | CMD=%.1f%% | T=%.1fdegC | RPM=%.0f | F=%.2fN\n",
      d.channel,
      d.time,
      d.esc_voltage,
      d.esc_current,
      d.esc_command,
      d.esc_temperature,
      d.motor_rpm,
      d.motor_force
    );
  }
}
