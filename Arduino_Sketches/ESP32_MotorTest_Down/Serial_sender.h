#ifndef __SER_SENDER__
#define __SER_SENDER__

#include <SoftwareSerial.h>

// Suggested pins setup for ESP32-Wroom-DA: 
// TX=17, RX=16; TX=25, RX=26; TX=32, RX=33

class mySender {
public:
  mySender(uint8_t rx_p=26, uint8_t tx_p=25, uint32_t rate=115200);
  int establishConnection();
  int sendString(String mystr);
  bool getStatus();

private:
  SoftwareSerial _mySer;
  bool _ser_inited = false;

};

mySender::mySender(uint8_t rx_p, uint8_t tx_p, uint32_t rate=115200){
  _mySer.begin(rate, SWSERIAL_8N1, rx_p, tx_p, false);
  if(!_mySer){
    Serial.println("Software serial initialization failed.");
    _ser_inited = false;
  }
  _ser_inited = true;
  Serial.println("Software serial successfully initialized.");
}

mySender::establishConnection(){
  
}

bool mySender::getStatus(){
  return _ser_inited;
}

int mySender::sendString(String mystr){
  if(_ser_inited && mystr){
    _mySer.println(mystr);
  }
}

#endif