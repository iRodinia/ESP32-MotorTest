//ESP32-WROOM-DA multi-sensor and data transmission system

// Data order (comma-separated):
//   LocalTime, Current, Voltage, Command [0-1], RPM, Thrust, EscTemperature

#include <Arduino.h>
#include <Wire.h>

#include "Serial_manager.h"
#include "My_ads1115_sensor.h"

#define MOTOR_POLE_PAIR 7
#define RPM_LPF_ALPHA 0.3  // [0,1], alpha > 0, 1 means no filter, 0.3 is a medium filtering value
#define LED_PIN 2
#define LED_TOGGLE() digitalWrite(LED_PIN, digitalRead(LED_PIN) ^ 1)

struct MCU_Sensors_Data {
  float lcaT = 0;
  float lastCur = 0; float lastVol = 0; // in s, A, V
  float lastCmd = 0;
  float lastRpm = 0; float lastThr = 0;  // in [0-1], r/min, N
  float lastEscTmp = 0;
  // in centidegree
};

void convert_data_to_string(const MCU_Sensors_Data& data, char* resultStr) {
  sprintf(resultStr, "%.2f,%.2f,%.2f,%.3f,%.1f,%.2f,%.2f",
    data.lcaT, data.lastCur, data.lastVol,
    data.lastCmd, data.lastRpm, data.lastThr, data.lastEscTmp
  );
}

MCU_Sensors_Data myData;
MyADS1115Sensor myADC;
TaskHandle_t SerialTaskHandle;

uint32_t startRecordLt = 0;
uint32_t lastDataSend = 0;
uint32_t lastSensorSlowUpdate = 0;

void SerialTask(void *pvParameters) {
  for (;;) {
    serial1DataEvent();
    serial2DataEvent();
    vTaskDelay(pdMS_TO_TICKS(1)); 
  }
}

void setup() {
  String init_message = "";
  init_message += initAllSerials();
  Serial.println("\n===== ESP32 Motor Seneors (AP) =====");

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  Wire.begin();
  delay(50);
  Wire.setClock(400000);  // fast mode

  xTaskCreatePinnedToCore(
    SerialTask,         /* 任务函数 */
    "SerialTask",       /* 任务名称 */
    4096,               /* 任务栈大小 (字节) */
    NULL,               /* 传递给任务的参数 */
    1,                  /* 任务优先级 (1 为默认优先级，比 loop 的 1 优先级低一点以防卡死，也可设为 1) */
    &SerialTaskHandle,  /* 任务句柄 */
    0                   /* 核心编号 (0 = PRO_CPU) */
  );

  init_message += myADC.init();
  if(init_message.length() < 2){
    Serial.println("Submodules initialized.");
  }
  else{
    Serial.println(init_message);
    Serial.println("Halting.");
    while(1);
  }

  Serial.println("===== System Initialization Done. =====\n");
  startRecordLt = millis();
}

void loop() {
  serial1DataEvent();
  serial2DataEvent();

  uint32_t current_time = millis();

  if(current_time - lastSensorSlowUpdate > 97) {
    lastSensorSlowUpdate = current_time;
    myData.lcaT = (current_time - startRecordLt) / 1e3;

    myADC.readPower(myData.lastVol, myData.lastCur);
    myADC.readForce(myData.lastThr);

    myData.lastEscTmp = myEscData.temperature;
    float tmp_rpm = myEscData.erpm / MOTOR_POLE_PAIR;
    myData.lastRpm = RPM_LPF_ALPHA*tmp_rpm + (1.0f-RPM_LPF_ALPHA)*myData.lastRpm;
    
    myData.lastCmd = (float(receiver_channels[2]) - CMD_MIN) / (CMD_MAX - CMD_MIN);  // throttle is channel [2]
  }

  if(current_time - lastDataSend > 100) {
    lastDataSend = current_time;
    char resultStr[300];
    convert_data_to_string(myData, resultStr);
    Serial.println(resultStr);
    Serial.flush();
    
    LED_TOGGLE();
  }
}