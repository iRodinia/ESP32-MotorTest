#ifndef __SER_DOWN_LIB__
#define __SER_DOWN_LIB__

#include <Arduino.h>
#include <TimeLib.h>

#include "SD_manager.h"

String up_cmd = "";  // store the received cmd
volatile bool cmd_received = false;
const unsigned long DEFAULT_TIME = 1357041600;  // Jan 1 2013
volatile bool data_sending = false;  // data sending status
volatile bool file_created = false;  // file existing status

extern SDCard mysd;
extern unsigned long start_record_local_time;

void Init_Data_Serial(){
  Serial2.begin(115200);  // Rx-16, Tx-17
  while(!Serial2)
    delay(10);
  Serial.println("Initialized data serial.");
  up_cmd.reserve(200);
}

void IRAM_ATTR Serial2Event() {
  while (Serial2.available()) {
    char inChar = (char)Serial2.read();
    up_cmd += inChar;
    if (inChar == '\n') {
      cmd_received = true;
    }
  }
}

String getCurrentHmsTime(){
  if(timeStatus() != timeNotSet){
    char timeStr[8];
    snprintf(timeStr, 8, "%02d:%02d:%02d", hour(), minute(), second());
    return String(timeStr);
  }
  return "";
}

String getCurrentDate(){
  if(timeStatus() != timeNotSet){
    char timeStr[10];
    snprintf(timeStr, 10, "%04d-%02d-%02d", year(), month(), day());
    return String(timeStr);
  }
  return "";
}

void Process_Command(String command) {
  command.trim();
  if (command == "Start_rcd") {
    data_sending = true;
    start_record_local_time = millis();
    Serial.println("Data transfer started.");
  }
  else if (command == "Stop_rcd") {
    data_sending = false;
    Serial.println("Data transfer stopped.");
  }
  else if (command == "Create_f"){
    String data_headline = "global_t local_t_ms vel_z acc_z";
    int file_flag;
    if(timeStatus() != timeNotSet){
      mysd.set_folder_name(getCurrentDate());
      file_flag = mysd.create_file(data_headline, "MTest_"+getCurrentHmsTime());
    }
    else{
      file_flag = mysd.create_file(data_headline);
    }
    if(file_flag < 0){
      Serial.println("SD file creation failed.");
      file_created = false;
    }
    else{
      Serial.println("SD file creation succeeded.");
      file_created = true;
    }
  }
  else if (command.startsWith("T:")) {
    String valueStr = command.substring(2);
    unsigned long time_interger = valueStr.toInt();
    if(time_interger >= DEFAULT_TIME){
      setTime(time_interger);
      Serial.println("Global time updated.");
      Serial2.println("T_ack");
    }
  }
  else if (command.length() > 0) {
    Serial.print("Unknown command: ");
    Serial.println(command);
  }
}

#endif