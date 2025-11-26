// ESP32串口波特率检测脚本
// 用于测试Serial1和Serial2的实际波特率

#define SERIAL1_RX 26
#define SERIAL1_TX 25

#define SERIAL2_RX 27
#define SERIAL2_TX 14

// 常见的波特率列表
const long baudRates[] = {
  300, 600, 1200, 2400, 4800, 9600, 14400, 19200, 
  28800, 38400, 57600, 115200, 230400, 460800, 921600
};
const int numBaudRates = sizeof(baudRates) / sizeof(baudRates[0]);

int testSerialPort = 0; // 0=未选择, 1=测试Serial1, 2=测试Serial2
bool testing = false;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  
  Serial.println("\n=== ESP32串口波特率检测工具 ===");
  Serial.println("输入 '1' 测试Serial1 (GPIO26)");
  Serial.println("输入 '2' 测试Serial2 (GPIO27)");
  Serial.println("\n请确保目标串口正在发送数据！");
}

void loop() {
  // 检查用户输入
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    
    if (cmd == '1') {
      testSerialPort = 1;
      Serial.println("\n>> 开始测试Serial1波特率...");
      testBaudRate(1);
    } 
    else if (cmd == '2') {
      testSerialPort = 2;
      Serial.println("\n>> 开始测试Serial2波特率...");
      testBaudRate(2);
    }
  }
}

void testBaudRate(int serialNum) {
  Serial.println("请确保目标串口正在持续发送数据！");
  Serial.println("开始扫描常见波特率...\n");
  
  for (int i = 0; i < numBaudRates; i++) {
    long baud = baudRates[i];
    
    // 根据选择初始化对应的串口
    if (serialNum == 1) {
      Serial1.end(); // 先关闭
      delay(100);
      Serial1.begin(baud, SERIAL_8N1, SERIAL1_RX, SERIAL1_TX);
    } else {
      Serial2.end();
      delay(100);
      Serial2.begin(baud, SERIAL_8N1, SERIAL2_RX, SERIAL2_TX);
    }
    
    delay(200); // 等待串口稳定
    
    // 清空缓冲区
    HardwareSerial* testSerial = (serialNum == 1) ? &Serial1 : &Serial2;
    while (testSerial->available()) {
      testSerial->read();
    }
    
    delay(100);
    
    // 统计接收到的字节数和可打印字符数
    int totalBytes = 0;
    int printableChars = 0;
    int errorChars = 0;
    
    unsigned long startTime = millis();
    while (millis() - startTime < 500) { // 采样500ms
      if (testSerial->available()) {
        char c = testSerial->read();
        totalBytes++;
        
        // 检查是否为可打印字符或常见控制字符
        if ((c >= 32 && c <= 126) || c == '\r' || c == '\n' || c == '\t') {
          printableChars++;
        } else if (c == 0xFF || c == 0x00) {
          errorChars++; // 常见的错误字节
        }
      }
      delay(1);
    }
    
    // 显示测试结果
    Serial.print("波特率: ");
    Serial.print(baud);
    Serial.print(" - 接收字节: ");
    Serial.print(totalBytes);
    
    if (totalBytes > 0) {
      float quality = (float)printableChars / totalBytes * 100;
      Serial.print(" | 可读性: ");
      Serial.print(quality, 1);
      Serial.print("%");
      
      // 判断可能的正确波特率
      if (totalBytes >= 10 && quality >= 70) {
        Serial.print(" ✓✓✓ 【可能匹配】");
      } else if (totalBytes >= 5 && quality >= 50) {
        Serial.print(" ✓ 【部分匹配】");
      }
    }
    Serial.println();
  }
  
  Serial.println("\n测试完成！");
  Serial.println("提示：接收字节数多且可读性高的波特率最有可能是正确的。");
  Serial.println("\n输入 '1' 或 '2' 可重新测试\n");
}