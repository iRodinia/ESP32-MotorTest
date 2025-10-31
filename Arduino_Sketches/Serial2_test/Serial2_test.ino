#include <Arduino.h>

void setup() {
  Serial.begin(115200);
  delay(50);

  Serial1.begin(115200);
  delay(50);

}

void loop() {
  Serial1.println("DDDDDDDD");
  delay(300);

}
