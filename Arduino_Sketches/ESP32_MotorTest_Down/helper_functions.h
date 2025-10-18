#ifndef HELPER_FCNS
#define HELPER_FCNS

#include <Arduino.h>
#include <Wire.h>
#include <TimeLib.h>

extern String up_cmd;
extern bool cmd_received;
extern bool start_log;
extern bool start_wifi_broadcast;
extern uint32_t startRecordLT;

void Serial2Event() {
  while (Serial2.available()) {
    char inChar = (char)Serial2.read();
    up_cmd += inChar;
    if (inChar == '\n') {
      cmd_received = true;
    }
  }
}

String getCurrentHmsTime(){
  char timeStr[8];
  snprintf(timeStr, 8, "%02d:%02d:%02d", hour(), minute(), second());
  return String(timeStr);
}

String getCurrentTime(){
  if(start_log){
    if(timeStatus() != timeNotSet){
      return getCurrentHmsTime();
    }
    else{
      return String((millis()-startRecordLT)/1000.0f, 2) + "L";
    }
  }
  else{
    return "0";
  }
}

String getCurrentDate(){
  if(timeStatus() != timeNotSet){
    char timeStr[10];
    snprintf(timeStr, 10, "%04d-%02d-%02d", year(), month(), day());
    return String(timeStr);
  }
  return "";
}



#endif