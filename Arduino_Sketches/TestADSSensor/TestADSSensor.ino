#include <Wire.h>
#include <Adafruit_ADS1X15.h>

Adafruit_ADS1115 ads;  // 创建ADS1115对象

void setup() {
  // 初始化串口通信，波特率115200
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  
  Serial.println("ADS1115 传感器测试开始");
  Serial.println("-------------------------------");
  
  // 初始化I2C通信，ESP32默认引脚：SDA=21, SCL=22
  if (!ads.begin()) {
    Serial.println("未找到ADS1115传感器，请检查接线!");
    while (1);
  }
  
  // 设置增益为 ±6.144V 范围
  // GAIN_TWOTHIRDS: ±6.144V, 1 bit = 0.1875mV
  ads.setGain(GAIN_TWOTHIRDS);
  
  // 设置数据采样率为128 SPS
  ads.setDataRate(RATE_ADS1115_128SPS);
  
  Serial.println("ADS1115 初始化成功!");
  Serial.println("A0, A1: Single-ended模式");
  Serial.println("A2-A3: Differential模式");
  Serial.println("-------------------------------");
  delay(1000);
}

void loop() {
  int16_t adc0, adc1, adc2_3;
  float volts0, volts1, volts2_3;
  
  // 读取A0单端模式
  adc0 = ads.readADC_SingleEnded(0);
  volts0 = ads.computeVolts(adc0);
  
  // 读取A1单端模式
  adc1 = ads.readADC_SingleEnded(1);
  volts1 = ads.computeVolts(adc1);
  
  // 读取A2-A3差分模式 (A2为正极, A3为负极)
  adc2_3 = ads.readADC_Differential_2_3();
  volts2_3 = ads.computeVolts(adc2_3);
  
  // 打印结果
  Serial.println("-------------------------------");
  Serial.print("A0 (Single): ");
  Serial.print(adc0);
  Serial.print(" | ");
  Serial.print(volts0, 4);
  Serial.println(" V");
  
  Serial.print("A1 (Single): ");
  Serial.print(adc1);
  Serial.print(" | ");
  Serial.print(volts1, 4);
  Serial.println(" V");
  
  Serial.print("A2-A3 (Diff): ");
  Serial.print(adc2_3);
  Serial.print(" | ");
  Serial.print(volts2_3, 4);
  Serial.println(" V");
  
  // 延迟500ms后继续下一次读取
  delay(500);
}