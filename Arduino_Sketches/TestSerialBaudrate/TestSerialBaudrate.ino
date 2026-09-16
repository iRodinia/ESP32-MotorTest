#include <Arduino.h>

#define SERIAL1_RX 26
#define SERIAL1_TX 25

// 常见的电调/飞控波特率列表
const long baudRates[] = {115200};
const int numBaudRates = sizeof(baudRates) / sizeof(baudRates[0]);

void setup() {
  Serial.begin(9600);  // 电脑串口监视器波特率请设置为 115200
  delay(1000);
  Serial.println("\n--- 启动波特率扫描测试 ---");
}

void loop() {
  for (int i = 0; i < numBaudRates; i++) {
    long currentBaud = baudRates[i];
    
    Serial.printf("\n\n>>> 正在测试波特率: %ld <<<\n", currentBaud);
    
    // 初始化 Serial1，正向信号（默认）
    Serial1.begin(currentBaud, SERIAL_8N1, SERIAL1_RX, SERIAL1_TX);
    
    // 在该波特率下监听 3 秒钟
    unsigned long startTime = millis();
    int byteCount = 0;
    
    while (millis() - startTime < 10000) {
      if (Serial1.available()) {
        uint8_t inByte = Serial1.read();
        
        // 打印十六进制格式（补齐0）
        if (inByte < 16) Serial.print("0");
        Serial.print(inByte, HEX);
        Serial.print(" ");
        
        byteCount++;
        // 每 10 个字节换行（KISS回传标准帧通常是 10 字节）
        if (byteCount % 10 == 0) {
          Serial.println();
        }
      }
    }
    
    if (byteCount == 0) {
      Serial.println("无数据接收。");
    }
    
    Serial1.end(); // 关闭当前串口，准备切换下一个波特率
    delay(500);
  }
  
  Serial.println("\n--- 扫描周期结束，即将重新开始 ---");
  delay(3000);
}