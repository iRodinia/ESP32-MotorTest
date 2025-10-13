/*
 * ESP32-WROOM-DA 多传感器数据采集与WiFi传输系统
 * 功能：
 * 1. AD7705采集模拟电压(0-3.3V)
 * 2. ADS1115配合Holybro PM02采集6S电池电压和电流
 * 3. 串口读取BLHeli32电调数据(转速、电流、电压)
 * 4. 读取外部PWM信号宽度
 * 5. 通过UDP每0.1秒发送一次数据
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <SPI.h>

// ========== WiFi配置 ==========
const char* ssid = "Your_WiFi_SSID";           // 修改为你的WiFi名称
const char* password = "Your_WiFi_Password";   // 修改为你的WiFi密码
const char* udpAddress = "192.168.1.100";      // 修改为目标IP地址
const int udpPort = 12345;                      // 修改为目标端口

// ========== 引脚定义 ==========
// AD7705 SPI引脚
#define AD7705_CS    5      // 片选
#define AD7705_DRDY  4      // 数据就绪引脚
#define AD7705_RESET 2      // 复位引脚
// 使用默认SPI引脚: MOSI=23, MISO=19, SCK=18

// ADS1115 I2C引脚 (使用默认I2C)
// SDA = 21, SCL = 22

// BLHeli32串口引脚
#define ESC_RX_PIN   16     // 连接到电调TX (实际不用接收)
#define ESC_TX_PIN   17     // 如果需要向电调发送命令

// PWM信号输入引脚
#define PWM_INPUT_PIN 15    // 读取PWM信号

// ========== AD7705寄存器定义 ==========
#define AD7705_COMM_REG        0x00
#define AD7705_SETUP_REG       0x10
#define AD7705_CLOCK_REG       0x20
#define AD7705_DATA_REG        0x30
#define AD7705_WRITE           0x00
#define AD7705_READ            0x08
#define AD7705_CH_AIN1         0x00

// ========== PM02电流计参数 ==========
const float VOLTAGE_DIVIDER = 18.0;      // 电压分压比 (V = ADC * 18)
const float CURRENT_SCALE = 36.6;        // 电流比例 (I = ADC * 36.6)
const float ADC_TO_VOLTAGE = 0.0001875;  // ADS1115: 6.144V/32768 = 0.0001875V/bit

// ========== BLHeli32协议定义 ==========
// BLHeli32数据帧格式 (Telemetry数据)
#define BLHELI_FRAME_LENGTH 10
#define BLHELI_HEADER 0x9B

// ========== 对象实例 ==========
WiFiUDP udp;
Adafruit_ADS1X15 ads;
HardwareSerial ESCSerial(2);  // 使用Serial2

// ========== PWM测量变量 ==========
volatile unsigned long pwm_rise_time = 0;
volatile unsigned long pwm_pulse_width = 0;
volatile bool pwm_new_data = false;

// ========== 测量数据结构 ==========
struct SensorData {
  float ad7705_voltage;      // AD7705电压 (V)
  float battery_voltage;     // 电池电压 (V)
  float battery_current;     // 电池电流 (A)
  uint16_t esc_rpm;          // 电机转速 (RPM)
  float esc_voltage;         // 电调电压 (V)
  float esc_current;         // 电调电流 (A)
  uint16_t esc_temp;         // 电调温度 (℃)
  uint16_t pwm_width;        // PWM脉冲宽度 (us)
  unsigned long timestamp;   // 时间戳 (ms)
} sensorData;

// BLHeli32数据缓冲
struct ESCTelemetry {
  uint16_t rpm;
  float voltage;
  float current;
  uint16_t temp;
  unsigned long lastUpdate;
} escData;

unsigned long lastSendTime = 0;
const unsigned long sendInterval = 100;  // 100ms = 0.1s

// ========== PWM中断服务函数 ==========
void IRAM_ATTR pwm_isr() {
  unsigned long currentTime = micros();
  
  if(digitalRead(PWM_INPUT_PIN) == HIGH) {
    // 上升沿 - 记录开始时间
    pwm_rise_time = currentTime;
  } else {
    // 下降沿 - 计算脉冲宽度
    if(pwm_rise_time > 0) {
      pwm_pulse_width = currentTime - pwm_rise_time;
      pwm_new_data = true;
    }
  }
}

// ========== AD7705函数 ==========
void ad7705_write(uint8_t data) {
  digitalWrite(AD7705_CS, LOW);
  SPI.transfer(data);
  digitalWrite(AD7705_CS, HIGH);
}

uint8_t ad7705_read() {
  digitalWrite(AD7705_CS, LOW);
  uint8_t data = SPI.transfer(0xFF);
  digitalWrite(AD7705_CS, HIGH);
  return data;
}

uint16_t ad7705_read_data() {
  digitalWrite(AD7705_CS, LOW);
  ad7705_write(AD7705_COMM_REG | AD7705_READ | AD7705_DATA_REG);
  uint8_t highByte = SPI.transfer(0xFF);
  uint8_t lowByte = SPI.transfer(0xFF);
  digitalWrite(AD7705_CS, HIGH);
  return (highByte << 8) | lowByte;
}

void ad7705_init() {
  pinMode(AD7705_CS, OUTPUT);
  pinMode(AD7705_DRDY, INPUT);
  pinMode(AD7705_RESET, OUTPUT);
  
  digitalWrite(AD7705_CS, HIGH);
  
  // 复位AD7705
  digitalWrite(AD7705_RESET, LOW);
  delay(10);
  digitalWrite(AD7705_RESET, HIGH);
  delay(10);
  
  // 写入至少32个1来复位通信
  digitalWrite(AD7705_CS, LOW);
  for(int i = 0; i < 4; i++) {
    SPI.transfer(0xFF);
  }
  digitalWrite(AD7705_CS, HIGH);
  delay(10);
  
  // 配置时钟寄存器
  ad7705_write(AD7705_COMM_REG | AD7705_WRITE | AD7705_CLOCK_REG);
  ad7705_write(0x0C);
  
  // 配置设置寄存器
  ad7705_write(AD7705_COMM_REG | AD7705_WRITE | AD7705_SETUP_REG | AD7705_CH_AIN1);
  ad7705_write(0x40);
  
  delay(500);
  
  Serial.println("AD7705初始化完成");
}

float ad7705_read_voltage() {
  int timeout = 1000;
  while(digitalRead(AD7705_DRDY) == HIGH && timeout-- > 0) {
    delayMicroseconds(100);
  }
  
  if(timeout <= 0) {
    return -1;
  }
  
  uint16_t rawData = ad7705_read_data();
  float voltage = (rawData / 65535.0) * 3.3;
  
  return voltage;
}

// ========== ADS1115 & PM02函数 ==========
void ads1115_init() {
  if (!ads.begin()) {
    Serial.println("ADS1115初始化失败!");
    while (1);
  }
  
  ads.setGain(GAIN_TWOTHIRDS);
  Serial.println("ADS1115初始化完成");
}

void read_pm02_data(float &voltage, float &current) {
  int16_t adc0 = ads.readADC_SingleEnded(0);
  int16_t adc1 = ads.readADC_SingleEnded(1);
  
  float voltage_adc = adc0 * ADC_TO_VOLTAGE;
  float current_adc = adc1 * ADC_TO_VOLTAGE;
  
  voltage = voltage_adc * VOLTAGE_DIVIDER;
  current = current_adc * CURRENT_SCALE;
  
  if(current < 0.1) current = 0.0;
}

// ========== BLHeli32电调数据读取 ==========
void blheli32_init() {
  ESCSerial.begin(115200, SERIAL_8N1, ESC_RX_PIN, ESC_TX_PIN);
  Serial.println("BLHeli32串口初始化完成");
  
  escData.rpm = 0;
  escData.voltage = 0;
  escData.current = 0;
  escData.temp = 0;
  escData.lastUpdate = 0;
}

// 解析BLHeli32 telemetry数据帧
bool parse_blheli32_frame(uint8_t* frame) {
  // 验证帧头
  if(frame[0] != BLHELI_HEADER) return false;
  
  // 计算校验和
  uint8_t crc = 0;
  for(int i = 0; i < BLHELI_FRAME_LENGTH - 1; i++) {
    crc ^= frame[i];
  }
  
  if(crc != frame[BLHELI_FRAME_LENGTH - 1]) return false;
  
  // 解析数据
  // BLHeli32标准格式: [Header][Temp][Voltage_H][Voltage_L][Current_H][Current_L][RPM_H][RPM_L][CRC]
  escData.temp = frame[1];
  escData.voltage = ((frame[2] << 8) | frame[3]) / 100.0;  // 单位: 0.01V
  escData.current = ((frame[4] << 8) | frame[5]) / 100.0;  // 单位: 0.01A
  escData.rpm = (frame[6] << 8) | frame[7];
  escData.lastUpdate = millis();
  
  return true;
}

void read_blheli32_data() {
  static uint8_t frameBuffer[BLHELI_FRAME_LENGTH];
  static uint8_t frameIndex = 0;
  
  while(ESCSerial.available()) {
    uint8_t byte = ESCSerial.read();
    
    // 检测帧头
    if(byte == BLHELI_HEADER) {
      frameIndex = 0;
    }
    
    frameBuffer[frameIndex++] = byte;
    
    // 接收完整帧
    if(frameIndex >= BLHELI_FRAME_LENGTH) {
      parse_blheli32_frame(frameBuffer);
      frameIndex = 0;
    }
  }
}

// ========== PWM测量函数 ==========
void pwm_init() {
  pinMode(PWM_INPUT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PWM_INPUT_PIN), pwm_isr, CHANGE);
  Serial.println("PWM测量初始化完成");
}

uint16_t get_pwm_width() {
  if(pwm_new_data) {
    pwm_new_data = false;
    return pwm_pulse_width;
  }
  return 0;  // 没有新数据
}

// ========== WiFi函数 ==========
void wifi_init() {
  Serial.print("连接WiFi: ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);
  
  int timeout = 0;
  while (WiFi.status() != WL_CONNECTED && timeout < 30) {
    delay(500);
    Serial.print(".");
    timeout++;
  }
  
  if(WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi连接成功!");
    Serial.print("IP地址: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWiFi连接失败!");
  }
}

// ========== 数据发送函数 ==========
void send_udp_data() {
  // 构建JSON格式数据
  char buffer[512];
  snprintf(buffer, sizeof(buffer), 
    "{"
    "\"ts\":%lu,"
    "\"ad7705\":%.3f,"
    "\"bat_v\":%.3f,"
    "\"bat_i\":%.3f,"
    "\"esc_rpm\":%u,"
    "\"esc_v\":%.2f,"
    "\"esc_i\":%.2f,"
    "\"esc_temp\":%u,"
    "\"pwm\":%u"
    "}",
    sensorData.timestamp,
    sensorData.ad7705_voltage,
    sensorData.battery_voltage,
    sensorData.battery_current,
    sensorData.esc_rpm,
    sensorData.esc_voltage,
    sensorData.esc_current,
    sensorData.esc_temp,
    sensorData.pwm_width
  );
  
  // 发送UDP数据包
  udp.beginPacket(udpAddress, udpPort);
  udp.write((uint8_t*)buffer, strlen(buffer));
  udp.endPacket();
  
  // 串口打印
  Serial.println(buffer);
}

// ========== 主函数 ==========
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n===== ESP32传感器系统启动 =====");
  
  // 初始化SPI
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV16);
  
  // 初始化I2C
  Wire.begin();
  
  // 初始化AD7705
  ad7705_init();
  
  // 初始化ADS1115
  ads1115_init();
  
  // 初始化BLHeli32串口
  blheli32_init();
  
  // 初始化PWM测量
  pwm_init();
  
  // 连接WiFi
  wifi_init();
  
  Serial.println("===== 系统初始化完成 =====\n");
}

void loop() {
  unsigned long currentTime = millis();
  
  // 持续读取BLHeli32数据
  read_blheli32_data();
  
  // 每0.1秒采集并发送一次数据
  if(currentTime - lastSendTime >= sendInterval) {
    lastSendTime = currentTime;
    
    // 读取AD7705数据
    sensorData.ad7705_voltage = ad7705_read_voltage();
    
    // 读取PM02数据
    read_pm02_data(sensorData.battery_voltage, sensorData.battery_current);
    
    // 读取BLHeli32数据
    sensorData.esc_rpm = escData.rpm;
    sensorData.esc_voltage = escData.voltage;
    sensorData.esc_current = escData.current;
    sensorData.esc_temp = escData.temp;
    
    // 读取PWM脉宽
    uint16_t pwm = get_pwm_width();
    if(pwm > 0) {
      sensorData.pwm_width = pwm;
    }
    
    // 记录时间戳
    sensorData.timestamp = currentTime;
    
    // 如果WiFi连接正常,发送数据
    if(WiFi.status() == WL_CONNECTED) {
      send_udp_data();
    } else {
      Serial.println("WiFi未连接,尝试重连...");
      wifi_init();
    }
  }
  
  // 简短延时
  delay(1);
}