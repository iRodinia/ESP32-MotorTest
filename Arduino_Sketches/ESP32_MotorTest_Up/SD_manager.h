#ifndef SD_MANAGER
#define SD_MANAGER

#include <Arduino.h>
#include <FS.h>
#include <SD.h>
#include <SPI.h>

// SD card 
// ESP32WroomDA module default pin assignments:
// sck = 18; miso = 19; mosi = 23; cs = 5;

class SDCard {
public:
  SDCard(uint8_t sck=18, uint8_t miso=19, uint8_t mosi=23, uint8_t cs=5);
  String init();
  bool checkCardStatus();
  bool checkFileStatus();
  int set_folder_name(String folder_name = "default_folder");
  int create_file(String head_line, String file_name = "default_file");
  int record(String message);
  int clear_logs(String folder_path = "/default_folder");

private:
  const uint64_t MIN_FREE_SPACE = 2 * 1024 * 1024;
  uint8_t _sck, _miso, _mosi, _cs;
  bool _card_mounted = false;
  bool _file_created = false;
  String _folder_name = "default_folder";
  String _file_name = "default_file";
  String _file_path = "/default_folder/default_file.txt";
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
  SPI.begin(_sck, _miso, _mosi, _cs);
  SPI.setFrequency(4000000);

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

  if(SD.totalBytes() - SD.usedBytes() <= MIN_FREE_SPACE){
    Serial.println("SD Card Free Space not enough. Clearing existing log files.");
    clear_logs("/");
  }
  return "";
}

bool SDCard::checkCardStatus(){
  if(_card_mounted){
    if(SD.totalBytes() - SD.usedBytes() > 1024){
      return true;
    }
  }
  return false;
}

bool SDCard::checkFileStatus(){
  return _file_created;
}

int SDCard::set_folder_name(String folder_name){
  if(folder_name.length() > 0){
    _folder_name = folder_name;
    return 0;
  }
  return -1;
}

int SDCard::create_file(String head_line, String file_name){
  if(!_card_mounted){
    return -1;
  }
  if(_file_created){
    return 0;
  }

  _file_path = "/" + _folder_name + "/" + file_name + ".txt";
  while(SD.exists(_file_path.c_str())){
    file_name += "+";
    _file_path = "/" + _folder_name + "/" + file_name + ".txt";
  }
  File myfile = SD.open(_file_path, FILE_WRITE, true);
  if (!myfile) {
    Serial.println("Failed to create log file.");
    _file_created = false;
    return -1;
  }

  if(myfile.println(head_line)){
    Serial.println("Log file created.");
    _file_name = file_name;
  }
  else{
    Serial.println("Creat log file failed. Unable to write.");
  }
  myfile.close();
  _file_created = true;
  return 0;
}

int SDCard::record(String message){
  if(!_card_mounted || !_file_created){
    Serial.println("SD Log file not created. Log failed.");
    return -1;
  }

  File myfile = SD.open(_file_path, FILE_APPEND);
  if(!myfile.println(message)){
    Serial.println("Log failed unexpected.");
  }
  myfile.close();
  return 0;
}

int SDCard::clear_logs(String folder_path){
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
      this->clear_logs(filePath.c_str());
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