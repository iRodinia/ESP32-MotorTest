#include <Arduino.h>

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600, SERIAL_8N1, 16, 17);  // rxPin=16, txPin=17
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("message sent.");
  delay(2000);

}
