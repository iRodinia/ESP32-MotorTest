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
  bool init(const WiFiUDP &myudp);
  bool getStatus();
  String getCurrentDateTime();
  unsigned long getLocalTimeMs();
  int resetTimer();

private:
  bool timer_avaliable = false;
  unsigned long localStartTimeMs = 0;
  NTPClientPt* timeSyncPt;
};

bool MyTimer::init(const WiFiUDP &myudp){
  timeSyncPt = new NTPClientPt(myudp, "pool.ntp.org", 28800);  // GMT+8
  if(timeSyncPt == nullptr){
    Serial.println("Initialize local timer failed.");
    timer_avaliable = false;
    return false;
  }
  Serial.println("Local timer initialized. Wait for sync.");
  timeSyncPt->begin();
  while (!timeSyncPt->update()) {
    delay(500);
    Serial.print(".");
  }
  setTime(timeSyncPt->getEpochTime());
  Serial.println("\nLocal timer synced.");
  localStartTimeMs = millis();
  timer_avaliable = true;
  return true;
}

MyTimer::~MyTimer(){
  if(!timeSyncPt){
    delete(timeSyncPt);
  }
}

bool MyTimer::getStatus(){
  return timer_avaliable;
}

String MyTimer::getCurrentDateTime(){
  if(!timer_avaliable){
    Serial.println("Unable to get time.");
    return String("ND");
  }
  time_t t = now();
  char timeStr[19];
  snprintf(timeStr, sizeof(timeStr), "%04d-%02d-%02d_%02d:%02d:%02d", year(t), month(t), day(t), hour(t), minute(t), second(t));
  return String(timeStr);
}

unsigned long MyTimer::getLocalTimeMs(){
  if(!timer_avaliable){
    Serial.println("Unable to get time.");
    return -1;
  }
  return millis() - localStartTimeMs;
}

int MyTimer::resetTimer(){
  if(timeSyncPt == nullptr){
    Serial.println("Local timer not exist.");
    timer_avaliable = false;
    return -1;
  }
  Serial.println("Wait for local timer to sync.");
  while (!timeSyncPt->update()) {
    delay(500);
    Serial.print(".");
  }
  setTime(timeSyncPt->getEpochTime());
  Serial.println("\nLocal timer synced.");
  localStartTimeMs = millis();
  timer_avaliable = true;
}

#endif