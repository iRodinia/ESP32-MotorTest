// ESP32串口转发脚本 - 支持SBUS协议解析
// Serial0 (USB): 主控制串口
// Serial1 (GPIO26 RX): 数据源1
// Serial2 (GPIO27 RX): SBUS接收器（100K波特率，8E2）

#define SERIAL1_RX 26
#define SERIAL1_TX 25

#define SERIAL2_RX 27
#define SERIAL2_TX 14

// 波特率配置
long serial1Baud = 115200;
long serial2Baud = 100000;  // SBUS标准波特率为100K

// SBUS协议参数
#define SBUS_FRAME_SIZE 25
#define SBUS_HEADER 0x0F
#define SBUS_FOOTER 0x00

// 当前转发模式: 0=无, 1=转发Serial1, 2=转发Serial2(SBUS解析)
int forwardMode = 0;

// SBUS数据缓冲区
uint8_t sbusBuffer[SBUS_FRAME_SIZE];
int sbusIndex = 0;
uint16_t channels[16];  // 16个通道数据
uint8_t flags = 0;      // 标志位

void setup() {
  // 初始化Serial0 (USB串口)
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  
  Serial.println("\n=== ESP32串口转发系统（支持SBUS）===");
  Serial.println("当前波特率配置：");
  Serial.print("  Serial1 (GPIO26): ");
  Serial.println(serial1Baud);
  Serial.print("  Serial2 (GPIO27): ");
  Serial.print(serial2Baud);
  Serial.println(" [SBUS模式: 100K, 8E2]");
  Serial.println("\n命令列表：");
  Serial.println("  '1' - 转发Serial1原始数据");
  Serial.println("  '2' - 转发Serial2 SBUS解析数据");
  Serial.println("  '0' - 停止转发");
  Serial.println("  'b1:xxxx' - 设置Serial1波特率");
  Serial.println();
  
  // 初始化Serial1（标准配置）
  Serial1.begin(serial1Baud, SERIAL_8N1, SERIAL1_RX, SERIAL1_TX);
  
  // 初始化Serial2（SBUS配置：100K波特率，8位数据，偶校验，2位停止位）
  Serial2.begin(serial2Baud, SERIAL_8E2, SERIAL2_RX, SERIAL2_TX);
  
  Serial.println("系统就绪！");
}

void loop() {
  // 检查Serial0是否有控制命令
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    
    if (cmd == "1") {
      forwardMode = 1;
      Serial.println(">> 切换到转发Serial1模式（原始数据）");
    } 
    else if (cmd == "2") {
      forwardMode = 2;
      sbusIndex = 0;  // 重置SBUS缓冲区
      Serial.println(">> 切换到转发Serial2模式（SBUS解析）");
    }
    else if (cmd == "0") {
      forwardMode = 0;
      Serial.println(">> 停止转发");
    }
    else if (cmd.startsWith("b1:")) {
      long newBaud = cmd.substring(3).toInt();
      if (newBaud > 0) {
        serial1Baud = newBaud;
        Serial1.end();
        delay(100);
        Serial1.begin(serial1Baud, SERIAL_8N1, SERIAL1_RX, SERIAL1_TX);
        Serial.print(">> Serial1波特率已设置为: ");
        Serial.println(serial1Baud);
      } else {
        Serial.println(">> 错误：无效的波特率");
      }
    }
  }
  
  // 根据当前模式转发数据
  if (forwardMode == 1) {
    // 转发Serial1的原始数据
    while (Serial1.available() > 0) {
      char c = Serial1.read();
      Serial.write(c);
    }
  } 
  else if (forwardMode == 2) {
    // 接收并解析SBUS数据
    while (Serial2.available() > 0) {
      uint8_t c = Serial2.read();
      
      // 查找帧头
      if (sbusIndex == 0 && c != SBUS_HEADER) {
        continue;  // 等待帧头
      }
      
      // 存储数据
      sbusBuffer[sbusIndex++] = c;
      
      // 检查是否接收完整帧
      if (sbusIndex == SBUS_FRAME_SIZE) {
        // 验证帧尾
        if (sbusBuffer[24] == SBUS_FOOTER || sbusBuffer[24] == 0x04) {
          // 解析SBUS数据
          parseSBUS();
          // 显示通道数据
          displayChannels();
        }
        // 重置缓冲区
        sbusIndex = 0;
      }
      
      // 防止缓冲区溢出
      if (sbusIndex >= SBUS_FRAME_SIZE) {
        sbusIndex = 0;
      }
    }
  }
  
  delay(1);
}

// 解析SBUS数据帧
void parseSBUS() {
  // 根据文档的解码算法解析16个通道
  channels[0]  = ((sbusBuffer[1]    | sbusBuffer[2]  << 8)                    & 0x07FF);
  channels[1]  = ((sbusBuffer[2]>>3 | sbusBuffer[3]  << 5)                    & 0x07FF);
  channels[2]  = ((sbusBuffer[3]>>6 | sbusBuffer[4]  << 2 | sbusBuffer[5]<<10) & 0x07FF);
  channels[3]  = ((sbusBuffer[5]>>1 | sbusBuffer[6]  << 7)                    & 0x07FF);
  channels[4]  = ((sbusBuffer[6]>>4 | sbusBuffer[7]  << 4)                    & 0x07FF);
  channels[5]  = ((sbusBuffer[7]>>7 | sbusBuffer[8]  << 1 | sbusBuffer[9]<<9)  & 0x07FF);
  channels[6]  = ((sbusBuffer[9]>>2 | sbusBuffer[10] << 6)                    & 0x07FF);
  channels[7]  = ((sbusBuffer[10]>>5| sbusBuffer[11] << 3)                    & 0x07FF);
  channels[8]  = ((sbusBuffer[12]   | sbusBuffer[13] << 8)                    & 0x07FF);
  channels[9]  = ((sbusBuffer[13]>>3| sbusBuffer[14] << 5)                    & 0x07FF);
  channels[10] = ((sbusBuffer[14]>>6| sbusBuffer[15] << 2 | sbusBuffer[16]<<10) & 0x07FF);
  channels[11] = ((sbusBuffer[16]>>1| sbusBuffer[17] << 7)                    & 0x07FF);
  channels[12] = ((sbusBuffer[17]>>4| sbusBuffer[18] << 4)                    & 0x07FF);
  channels[13] = ((sbusBuffer[18]>>7| sbusBuffer[19] << 1 | sbusBuffer[20]<<9) & 0x07FF);
  channels[14] = ((sbusBuffer[20]>>2| sbusBuffer[21] << 6)                    & 0x07FF);
  channels[15] = ((sbusBuffer[21]>>5| sbusBuffer[22] << 3)                    & 0x07FF);
  
  // 解析标志位（第23字节）
  flags = sbusBuffer[23];
}

// 显示通道数据
void displayChannels() {
  Serial.print("CH:");
  for (int i = 0; i < 16; i++) {
    Serial.print(" ");
    Serial.print(i + 1);
    Serial.print("=");
    Serial.print(channels[i]);
    if (i < 15) Serial.print(",");
  }
  
  // 显示标志位信息
  Serial.print(" | Flags:");
  if (flags & 0x80) Serial.print(" CH17");
  if (flags & 0x40) Serial.print(" CH18");
  if (flags & 0x20) Serial.print(" LOST");
  if (flags & 0x10) Serial.print(" FAILSAFE");
  
  Serial.println();
}