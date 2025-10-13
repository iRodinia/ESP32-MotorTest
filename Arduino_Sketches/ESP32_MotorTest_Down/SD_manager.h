#ifndef SD_MANAGER
#define SD_MANAGER

#include <Arduino.h>
#include <FS.h>
#include <SD.h>
#include <SPI.h>

// ESP32WroomDA module default pin assignments:
// sck = 18; miso = 19; mosi = 23; cs = 5;

class SDCard {
public:
  SDCard();
  SDCard(int8_t sck, int8_t miso, int8_t mosi, int8_t cs);
  bool init();
  bool checkCardStatus();
  int set_folder_name(String folder_name);
  int create_file(String file_name, String head_line);
  int record(String message);
  int clear_logs(String folder_path = "/" + _folder_name);

private:
  uint8_t _sck, _miso, _mosi, _cs;
  bool reassign_pins = false;
  bool _card_mounted = false;
  bool _file_created = false;
  String _folder_name = "default_folder";
  String _file_name = "default_file";
  String _file_path = "/default_folder/default_file.txt";
};

SDCard::SDcard(){
  reassign_pins = false;
}

SDCard::SDCard(uint8_t sck, uint8_t miso, uint8_t mosi, uint8_t cs){
  reassign_pins = true;
  _sck = sck;
  _miso = miso;
  _mosi = mosi;
  _cs = cs;
}

bool SDCard::init(){
  bool init_flag;
  if(reassign_pins){
    SPI.begin(sck, miso, mosi, cs);
    SPI.setFrequency(4000000);
    init_flag = SD.begin(cs);
  }
  else{
    init_flag = SD.begin();
  }
  if(!init_flag){
    Serial.println("Card mount failed");
    _card_mounted = false;
    return false;
  }
  _card_mounted = true;
  return true;
}

bool SDCard::checkCardStatus(){
  return _card_mounted;
}

int SDCard::set_folder_name(Strign folder_name){
  if(folder_name.length() > 0){
    _folder_name = folder_name;
    return 0;
  }
  return -1;
}

int SDCard::create_file(String file_name, String head_line){
  if(!_card_mounted){
    return -1;
  }

  uint64_t card_free_space = (SD.totalBytes() - SD.usedBytes()) / (1024 * 1024);
  if(card_free_space <= 2){
    Serial.printf("SD Card Free Space: %lluMB\n", card_free_space);
    Serial.println("Exit due to insufficient card space.");
    _file_created = false;
    return -1;
  }
  
  _file_path = "/" + _folder_name + "/" + file_name + ".txt";
  if(SD.exists(_file_path.c_str())){
    SD.remove(_file_path.c_str());
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
    Serial.println("Log file not created. Log failed.");
    return -1;
  }

  File myfile = SD.open(_file_path, FILE_APPEND);
  if(!myfile.println(message)){
    Serial.println("Log failed unexpected.");
  }
  myfile.close();
  return 0;
}

int SDCard::clear_logs(String folder_path = "/" + _folder_name){
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