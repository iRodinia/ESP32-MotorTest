#include <Arduino.h>
#include <Wire.h>

// #include "12864_display_soft.h"
#include "12864_display_hard.h"

displayData myData;
InfoDisplayHard myScreen;

uint32_t start_time = 0;

void setup() {
  Serial.begin(115200);
  delay(50);

  Wire.begin();
  delay(50);
  Wire.setClock(400000);  // fast mode

  myScreen.init();

  Serial.println("===== Start OLED Test =====");
  start_time = millis();
}

void loop() {
  uint32_t currT = millis();
  myData.localT = (currT-start_time) / 1000.0;
  myData.data1 = random(10000) / 1000.0;
  myData.data2 = random(10000) / 1000.0;
  myData.data3 = random(10000) / 1000.0;
  myData.data4 = random(10000) / 1000.0;
  myScreen.updateInfo(myData);

  myScreen.refresh();

  uint32_t currEndT = millis();
  Serial.printf("Loop time ms: %d \n", currEndT-currT);

  delay(1);
}
