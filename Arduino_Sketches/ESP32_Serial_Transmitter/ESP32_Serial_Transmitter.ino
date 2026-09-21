// ESP32 Serial Bridge Node
// From Serial 1 to Serial 0

#include <Arduino.h>
#include <Wire.h>
#include "12864_display.h"
#include "Serial_manager.h"

#define SERIAL1_RX 26
#define SERIAL1_TX 25
#define SERIAL1_BAUDRATE 9600

#define SCREEN_SCL_PIN 21
#define SCREEN_SDA_PIN 22  // note: this is not my regular setting

#define PIN_LED 2

bool led_state = false;
bool data_changed = false;
float prev_time = 0.0;
SingleScreen myScreen(SCREEN_SCL_PIN, SCREEN_SDA_PIN);
TaskHandle_t SerialTaskHandle;

void SerialTask(void *pvParameters) {
  for (;;) {
    serial0CmdEvent();
    serial1CmdEvent();

    if (myEscData.lcaT != prev_time) {
      prev_time = myEscData.lcaT;
      data_changed = true;
    }

    vTaskDelay(pdMS_TO_TICKS(2)); 
  }
}

void setup() {
  Serial.begin(115200);
  delay(50);
  Serial1.begin(SERIAL1_BAUDRATE, SERIAL_8N1, SERIAL1_RX, SERIAL1_TX);
  delay(50);
  while (!Serial || !Serial1){
    delay(10);
  }
  
  Wire.setPins(SCREEN_SDA_PIN, SCREEN_SCL_PIN);
  delay(50);
  Wire.begin();
  delay(50);
  Wire.setClock(400000);
  delay(50);

  myScreen.setChannel(0);
  myScreen.setHeadLine("Motor Control");
  myScreen.setLineLabels("Volt:", "CMD :", "RPM :", "Forc:");
  myScreen.init();

  xTaskCreatePinnedToCore(
    SerialTask,
    "SerialTask",
    4096,
    NULL,
    1,
    &SerialTaskHandle,
    0
  );

  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);
}


void loop() {

  if (data_changed) {
    led_state = !led_state;
    digitalWrite(PIN_LED, led_state ? HIGH : LOW);
    Serial.printf(
      "t=%.2fs | V=%.2fV | CMD=%d | RPM=%.1f | F=%.2fN\n",
      myEscData.lcaT,
      myEscData.lastVol,
      myEscData.lastCmd,
      myEscData.lastRpm,
      myEscData.lastThr
    );

    data_changed = false;
  }

  myScreen.updateData(myEscData.lastVol, myEscData.lastCmd, myEscData.lastRpm, myEscData.lastThr);
  myScreen.refresh();

  delay(5);
}

void parseSerial0Cmd(String command) {
    int target_speed = command.toInt();
    Serial1.printf("%d\n", target_speed);
}

