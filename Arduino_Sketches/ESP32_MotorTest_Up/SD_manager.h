#ifndef SD_MANAGER
#define SD_MANAGER

#include <Arduino.h>
#include <FS.h>
#include <SD.h>
#include <SPI.h>

#include "helper_functions_up.h"

#define MAX_LINE_NUM 300
#define MAX_LINE_LEN 250

#define LED_PIN 2  // LED blink means data recording, LED on means flushing to SD card
#define LED_TOGGLE() digitalWrite(LED_PIN, digitalRead(LED_PIN) ^ 1)

// SD card 
// ESP32WroomDA module default pin assignments:
// sck = 18; miso = 19; mosi = 23; cs = 5;
// Log frequency recommendation: 3~5 Hz
// Maximum buffer: 1000 lines * 280 characters

class SDCard {
public:
  SDCard(uint8_t sck=18, uint8_t miso=19, uint8_t mosi=23, uint8_t cs=5);
  String init();
  bool cardReady();
  bool checkFileStatus();
  int setFolderName(String folder_name = "default_folder");
  int setFileName(String file_name = "default_log");
  int setHeadLine(String head_line = "");

  int logMessage(char* message);
  int flushToCard();
  int clearLogs(String folder_path = "/default_folder");

private:
  const uint64_t MIN_FREE_SPACE = 2 * 1024 * 1024;  // byte
  uint8_t _sck, _miso, _mosi, _cs;
  bool _card_mounted = false;
  bool _file_created = false;
  String _folder_name = "default_folder";
  String _file_name = "default_log";
  String _head_line = "";

  char _log_buffer[MAX_LINE_NUM * MAX_LINE_LEN];
  unsigned long _log_current_pos;
};

SDCard::SDCard(uint8_t sck, uint8_t miso, uint8_t mosi, uint8_t cs){
  _sck = sck;
  _miso = miso;
  _mosi = mosi;
  _cs = cs;

  _card_mounted = false;
  _file_created = false;
}

String SDCard::init(){
  _log_current_pos = 0;

  SPI.begin(_sck, _miso, _mosi, _cs);
  SPI.setFrequency(4000000);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  uint8_t initNum = 0;
  while(!SD.begin() && initNum < 10) {
    initNum++;
    delay(50);
  }
  if(initNum >= 10){
    _card_mounted = false;
    return "Card not mounted. SD initializaiton failed.";
  }
  _card_mounted = true;
  return "";
}

bool SDCard::cardReady(){
  if(_card_mounted){
    if(SD.totalBytes() - SD.usedBytes() > 2e5){
      return true;
    }
  }
  return false;
}

int SDCard::setFolderName(String folder_name){
  if(folder_name.length() > 0){
    _folder_name = folder_name;
    return 0;
  }
  return -1;
}

int SDCard::setFileName(String file_name){
  if(file_name.length() > 0){
    _file_name = file_name;
    return 0;
  }
  return -1;
}

int SDCard::setHeadLine(String head_line){
  if(head_line.length() > 0){
    _head_line = head_line;
  }
  return 0;
}

int SDCard::logMessage(char* message){  // message[280]
  if (!_log_buffer || _log_current_pos >= MAX_LINE_NUM * (MAX_LINE_LEN-1)) {
    return -1;
  }

  int char_num = sprintf(_log_buffer+_log_current_pos, "%s\n", message);
  _log_current_pos += char_num;
  LED_TOGGLE();
  return 0;
}

int SDCard::flushToCard(){
  if(!cardReady()){
    Serial.println("Unable to write logs to SD card.");
    return -1;
  }

  String _file_path = "/" + _folder_name + "/" + _file_name + ".txt";
  while(SD.exists(_file_path.c_str())){
    _file_name += "+";
    _file_path = "/" + _folder_name + "/" + _file_name + ".txt";
  }

  digitalWrite(LED_PIN, HIGH);

  File myfile = SD.open(_file_path, FILE_WRITE, true);
  if (!myfile) {
    Serial.println("Failed to create log file.");
    return -1;
  }
  if(_head_line.length() > 0){
    myfile.println(_head_line);
  }
  myfile.write((uint8_t*)_log_buffer, _log_current_pos);
  myfile.close();
  _log_current_pos = 0;

  digitalWrite(LED_PIN, LOW);
  return 0;
}

int SDCard::clearLogs(String folder_path){
  if(!_card_mounted){
    return -1;
  }

  File root = SD.open(folder_path);
  if(!root || !root.isDirectory()) {
    Serial.printf("Error: unable to load directory '%s'.\n", folder_path);
    return -1;
  }
  
  File file = root.openNextFile();
  while (file) {
    String filePath = String(folder_path) + "/" + String(file.name());
    if(file.isDirectory()){
      file.close();
      this->clearLogs(filePath.c_str());
      SD.rmdir(filePath.c_str());
    }
    else{
      file.close();
      Serial.printf("Delete file: %s\n", filePath.c_str());
      SD.remove(filePath.c_str());
    }
    file = root.openNextFile();
  }
  root.close();
  _file_created = false;
  return 0;
}

#endif