#include <Arduino.h>

void setup() {
  Serial.begin(115200);
  delay(50);

  Serial2.begin(115200);
  delay(50);

}

void loop() {
  Serial2.println("DDDDDDDD");
  delay(300);

}
