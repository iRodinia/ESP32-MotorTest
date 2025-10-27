#ifndef TIME_MANAGER
#define TIME_MANAGER

#include <Arduino.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <TimeLib.h>

class MyTimer {
public:
  MyTimer(){};
  ~MyTimer();
  String init(WiFiUDP &myudp);
  String getCurrentTime();
  String getCurrentDate();
  String getCurrentDateTime();
  int resetTimer();

private:
  NTPClient* timeSyncPt;
};

String MyTimer::init(WiFiUDP &myudp){
  uint8_t initNum = 0;
  while(!timeSyncPt && initNum < 10){
    timeSyncPt = new NTPClient(myudp, "pool.ntp.org", 28800);  // GMT+8
    initNum++;
    delay(50);
  }
  if(initNum >= 20){
    return "Timer initialization failed.";
  }
  Serial.println("Timer initialized. Wait for network sync.");
  timeSyncPt->begin();
  while (!timeSyncPt->update()) {
    delay(500);
    Serial.print(".");
  }
  setTime(timeSyncPt->getEpochTime());
  Serial.println("\nLocal timer synced.");
  return "";
}

MyTimer::~MyTimer(){
  if(!timeSyncPt){
    delete(timeSyncPt);
  }
}

String MyTimer::getCurrentTime(){
  time_t t = now();
  char timeStr[9];
  snprintf(timeStr, sizeof(timeStr), "%02d:%02d:%02d", hour(t), minute(t), second(t));
  return String(timeStr);
}

String MyTimer::getCurrentDate(){
  time_t t = now();
  char timeStr[11];
  snprintf(timeStr, sizeof(timeStr), "%04d-%02d-%02d", year(t), month(t), day(t));
  return String(timeStr);
}

String MyTimer::getCurrentDateTime(){
  time_t t = now();
  char timeStr[19];
  snprintf(timeStr, sizeof(timeStr), "%04d-%02d-%02d_%02d:%02d:%02d", year(t), month(t), day(t), hour(t), minute(t), second(t));
  return String(timeStr);
}

int MyTimer::resetTimer(){
  if(timeSyncPt == nullptr){
    Serial.println("Local timer not exist.");
    return -1;
  }
  Serial.println("Wait for network sync.");
  while (!timeSyncPt->update()) {
    delay(500);
    Serial.print(".");
  }
  setTime(timeSyncPt->getEpochTime());
  Serial.println("\nTimer synced.");
  return 0;
}

#endif