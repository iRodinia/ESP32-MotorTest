#include <Arduino.h>
#include <Wire.h>

// ESP32WroomDA: SCL-22, SDA-21

void setup()
{
  Wire.begin();
  
  Serial.begin(115200);
  while (!Serial);             // 等待Serial连接
  delay(100);
  
  Serial.println("\nI2C Scanner");
  Serial.println("开始扫描I2C设备...\n");
}

void loop()
{
  int nDevices = 0;
  
  // 扫描I2C地址范围：8 到 119 (0x08 到 0x77)
  // 0-7 为保留地址，120-127 为保留地址
  for (byte i = 8; i < 120; i++)
  {
    Wire.beginTransmission(i);
    byte error = Wire.endTransmission();
    
    if (error == 0)
    {
      Serial.print("I2C设备发现: 0x");
      if (i < 16)
        Serial.print("0");
      Serial.println(i, HEX);
      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("地址 0x");
      if (i < 16)
        Serial.print("0");
      Serial.print(i, HEX);
      Serial.println(" 发生未知错误");
    }
  }
  
  Serial.print("\n找到设备数: ");
  Serial.println(nDevices);
  Serial.println("----------------------------\n");
  
  delay(3000);  // 每3秒扫描一次
}