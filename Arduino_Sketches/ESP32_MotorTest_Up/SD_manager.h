#ifndef SD_MANAGER
#define SD_MANAGER

#include <Arduino.h>
#include <FS.h>
#include <SD.h>
#include <SPI.h>

#define LED_PIN 2  // LED blink means data recording, LED on means flushing to SD card
#define LED_TOGGLE() digitalWrite(LED_PIN, digitalRead(LED_PIN) ^ 1)

#define MAX_LINE_NUM 600
#define MAX_LINE_LEN 120

class SDCard {
public:
  SDCard(uint8_t sck=18, uint8_t miso=19, uint8_t mosi=23, uint8_t cs=5);
  String init(String headline = "");
  int logMessage(char* message);
  int flushToCard();
  String getCurrentFileName();

private:
  uint8_t sck_pin;
  uint8_t miso_pin;
  uint8_t mosi_pin;
  uint8_t cs_pin;
  String currentFileName;
  const size_t MIN_FREE_SPACE = 2 * 1024 * 1024;
  const size_t MAX_BUFFER_SIZE = MAX_LINE_NUM * (MAX_LINE_LEN-1);

  char _log_buffer[MAX_LINE_NUM * MAX_LINE_LEN];
  unsigned long _log_current_pos;

  String getNextFileName();
  void deleteAllFiles();
  bool hasEnoughSpace();
};

SDCard::SDCard(uint8_t sck, uint8_t miso, uint8_t mosi, uint8_t cs) 
  : sck_pin(sck), miso_pin(miso), mosi_pin(mosi), cs_pin(cs) {}

String SDCard::init(String headline){
  SPI.begin(sck_pin, miso_pin, mosi_pin, cs_pin);
  SPI.setFrequency(4000000);
  delay(20);

  int retryNum = 0;
  while (!SD.begin() && retryNum <= 10) {
    retryNum++;
    delay(50);
  }
  if (retryNum > 10) {
    return "SD card mount failed.";
  }
  uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE) {
    return "SD card not detected.";
  }

  if (!hasEnoughSpace()) {
    deleteAllFiles();
  }
  currentFileName = getNextFileName();
  File file = SD.open(currentFileName, FILE_WRITE);
  if (!file) {
    return "Unable to create file: " + currentFileName;
  }

  if (headline.length() > 0) {
    if (!headline.endsWith("\n")) {
      headline += "\n";
    }
    file.print(headline);
  }
  file.close();
  _log_current_pos = 0;
  return "";
}

int SDCard::logMessage(char* message){
  if (!_log_buffer || _log_current_pos >= MAX_BUFFER_SIZE) {
    return -1;
  }

  int char_num = sprintf(_log_buffer+_log_current_pos, "%s\n", message);
  _log_current_pos += char_num;
  LED_TOGGLE();
  return 0;
}

int SDCard::flushToCard(){
  if(_log_current_pos == 0){
    return 0;
  }

  digitalWrite(LED_PIN, HIGH);

  File myfile = SD.open(currentFileName, FILE_APPEND);
  if (!myfile) {
    Serial.println("Failed to open log file.");
    return -1;
  }
  myfile.write((uint8_t*)_log_buffer, _log_current_pos);
  myfile.close();

  _log_current_pos = 0;
  digitalWrite(LED_PIN, LOW);
  return 0;
}

String SDCard::getCurrentFileName() {
  return currentFileName;
}

String SDCard::getNextFileName() {
  int fileNum = 1;
  while (SD.exists("/" + String(fileNum) + ".txt")) {
    fileNum++;
  }
  return "/" + String(fileNum) + ".txt";
}

void SDCard::deleteAllFiles() {
  File root = SD.open("/");
  if (!root) return;
  
  File file = root.openNextFile();
  while (file) {
    String fileName = String(file.name());
    file.close();
    SD.remove(fileName);
    file = root.openNextFile();
  }
  root.close();
}

bool SDCard::hasEnoughSpace() {
  return SD.cardSize() - SD.usedBytes() >= MIN_FREE_SPACE;
}

#endif