#ifndef SD_MANAGER
#define SD_MANAGER

#include <Arduino.h>
#include <FS.h>
#include <SD.h>
#include <SPI.h>


class SDCard {

public:
  SDCard();
  SDCard(int8_t sck, int8_t miso, int8_t mosi, int8_t cs);
  int set_folder_name(String folder_name);
  int create_file(String file_name, String head_line);
  int record(String message);

private:
  bool reassign_pins = false;
  bool _file_created = false;
  String _folder_name = "default_folder";
  String _file_name = "default_file";
  String _file_path = "/default_folder/default_file.txt";
  fs::FS _fs;

};

SDCard::SDcard(){
  reassign_pins = false;
  if(!SD.begin()){
    Serial.println("Card Mount Failed");
    return;
  }
  _fs = SD;
}

SDCard::SDCard(int8_t sck, int8_t miso, int8_t mosi, int8_t cs){
  reassign_pins = true;
  SPI.begin(sck, miso, mosi, cs);
  SPI.setFrequency(4000000);
  if (!SD.begin(cs)) {
    Serial.println("Card Mount Failed");
    return;
  }
  _fs = SD;
}

int SDCard::set_folder_name(Strign folder_name){
  if(folder_name.length() > 0){
    _folder_name = folder_name;
    return 0;
  }
  return -1;
}

int SDCard::create_file(String file_name, String head_line){
  uint64_t card_free_space = (SD.totalBytes() - SD.usedBytes()) / (1024 * 1024);
  if(card_free_space <= 2){
    Serial.printf("SD Card Free Space: %lluMB\n", card_free_space);
    Serial.println("Exit due to insufficient card space.");
    _file_created = false;
    return -1;
  }
  
  _file_path = target_folder_path + "/" + file_name + ".txt";
  if(_fs.exists(_file_path.c_str())){
    _fs.remove(_file_path.c_str());
  }
  File myfile = _fs.open(_file_path, FILE_WRITE, true);
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
  File myfile = _fs.open(_file_path, FILE_APPEND);
  if(!myfile.println(message)){
    Serial.println("Log failed unexpected.");
    myfile.close();
    return -1;
  }
  myfile.close();
  return 0;
}

#endif