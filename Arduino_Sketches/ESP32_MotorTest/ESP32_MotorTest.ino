#include <Arduino.h>

#include "SD_manager.h"
#include "Time_manager.h"

const char* ssid = "BioInBot_Lab";
const char* password = "11223344";
WiFiUDP espUDP;

void setup(){
  Serial.print("Connecting to WiFi.");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected.");
  MyTimer timer(espUDP);
  SDCard mysd(18, 19, 23, 5);


  delay(1000);
}

void loop(){

}