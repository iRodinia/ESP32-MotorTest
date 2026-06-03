// dual_oled_test.ino
// ESP32-C3 — 两块 SSD1306 128x64 OLED 共享同一 I2C 总线测试例程
//
// 接线：
//   SCL → GPIO0
//   SDA → GPIO1
//   屏幕 A I2C 地址: 0x78
//   屏幕 B I2C 地址: 0x7A
//
// 依赖库：U8g2 (by olikraus)

#include <Arduino.h>
#include <Wire.h>
#include "12864_display.h"

// ── 引脚定义 ─────────────────────────────────────────────────────────────────
#define PIN_SCL  0
#define PIN_SDA  1

// ── 实例化两块屏幕 ────────────────────────────────────────────────────────────
SingleScreen screenA(PIN_SCL, PIN_SDA, 0x78);
SingleScreen screenB(PIN_SCL, PIN_SDA, 0x7A);

// ── 模拟数据 ──────────────────────────────────────────────────────────────────
double tick = 0.0;  // 每次 loop 递增，用于生成变化的测试数值

void setup() {
  Serial.begin(115200);

  // ESP32-C3 需要在 begin() 之前指定自定义引脚
  Wire.begin(PIN_SDA, PIN_SCL);

  Serial.println("Initialising screens...");

  // ── 屏幕 A ────────────────────────────────────────────────────────────────
  screenA.setChannel(1);
  screenA.setHeadLine("SensorA");
  screenA.setLineLabels("Temp :", "Humi :", "Pres :", "Volt :");
  screenA.init();  // init 内部会完成首次全量刷新

  // ── 屏幕 B ────────────────────────────────────────────────────────────────
  screenB.setChannel(2);
  screenB.setHeadLine("SensorB");
  screenB.setLineLabels("Curr :", "Powr :", "Freq :", "RSSI :");
  screenB.init();

  Serial.println("Done.");
}

void loop() {
  tick += 0.1;

  // ── 生成模拟数据（用三角函数制造平滑变化的波形）────────────────────────────
  double tempA  =  25.00 + 5.0  * sin(tick);
  double humiA  =  60.00 + 10.0 * cos(tick);
  double presA  = 101.30 + 1.5  * sin(tick * 0.5);
  double voltA  =   3.30 + 0.2  * cos(tick * 2.0);

  double currB  =   1.20 + 0.5  * sin(tick * 1.3);
  double powrB  =   3.96 + 0.8  * cos(tick * 0.7);
  double freqB  =  50.00 + 0.05 * sin(tick * 3.0);
  double rssiB  = -65.00 + 10.0 * cos(tick * 0.4);

  // ── 更新 buffer（无 I2C 传输）────────────────────────────────────────────
  screenA.updateData(tempA, humiA, presA, voltA);
  screenB.updateData(currB, powrB, freqB, rssiB);

  // ── 局部刷新两块屏幕（只传数据行）────────────────────────────────────────
  screenA.refresh();
  screenB.refresh();

  // ── 串口调试输出 ──────────────────────────────────────────────────────────
  Serial.printf("[A] Temp=%.2f  Humi=%.2f  Pres=%.2f  Volt=%.2f\n",
                tempA, humiA, presA, voltA);
  Serial.printf("[B] Curr=%.2f  Powr=%.2f  Freq=%.2f  RSSI=%.2f\n",
                currB, powrB, freqB, rssiB);

  delay(100);  // 约 5 fps 刷新率
}
