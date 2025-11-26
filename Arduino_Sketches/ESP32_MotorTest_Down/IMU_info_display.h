#ifndef __OLED12864__
#define __OLED12864__
// Declaration for a 0.96" 12864 display connected to I2C (SDA, SCL pins)
// Using U8g2 library for display control

#include <Arduino.h>
#include <U8g2lib.h>

#include "helper_functions_down.h"

// Coordinates of the display is originated from up-left corner (0,0)
class InfoDisplayDown {
public:
  InfoDisplayDown(uint8_t scl=33, uint8_t sda=32);
  String init();  // init the display
  void clear(void);  // clear display
  void refresh(void);  // upload the buffer to OLED
  void updateInfo(MCU_Down_Data data);
  void setCheckbox(bool flag);

private:
  U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2;
  String _line1, _line2, _line3, _line4, _line5;
  bool _checkbox;
  
};

InfoDisplayDown::InfoDisplayDown(uint8_t scl, uint8_t sda)
  : u8g2(U8G2_R0, scl, sda, U8X8_PIN_NONE)
{
  _line1 = "00.00";
  _line2 = " ";
  _line3 = " ";
  _line4 = " ";
  _line5 = " ";
  _checkbox = false;
}

String InfoDisplayDown::init(){
  u8g2.begin();
  u8g2.setDrawColor(1);
  u8g2.setDisplayRotation(U8G2_R0);

  clear();
  return "";
}

void InfoDisplayDown::updateInfo(MCU_Down_Data data){
  String line1 = String(data.lcaT, 2);
  if(line1.length() > 18){
    line1 = line1.substring(0, 18);
  }
  _line1 = line1;
  _line2 = "acc:"+String(data.lastAx,1)+","+String(data.lastAy,1)+","+String(data.lastAz,1);
  _line3 = "gyo:"+String(data.lastGx,1)+","+String(data.lastGy,1)+","+String(data.lastGz,1);
  _line4 = "mag:"+String(data.lastMx,1)+","+String(data.lastMy,1)+","+String(data.lastMz,1);
  _line5 = "tmp:"+String(data.lastTmp,2);
}

void InfoDisplayDown::setCheckbox(bool flag){
  _checkbox = flag;
}

void InfoDisplayDown::refresh(void){
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_t0_15b_tr);  // 6x12 font for size 12
  u8g2.drawStr(0, 11, _line1.c_str());  // y position is baseline
  u8g2.drawLine(0, 15, 127, 15);

  u8g2.setFont(u8g2_font_6x13B_tr);
  u8g2.drawStr(0, 27, _line2.c_str());
  u8g2.drawStr(0, 39, _line3.c_str());
  u8g2.drawStr(0, 51, _line4.c_str());
  u8g2.drawStr(0, 63, _line5.c_str());

  u8g2.drawCircle(117, 6, 5, U8G2_DRAW_ALL);
  if(_checkbox){
    u8g2.drawDisc(117, 6, 3, U8G2_DRAW_ALL);  // filled circle
  }

  u8g2.sendBuffer();
}

void InfoDisplayDown::clear(void){
  u8g2.clearBuffer();
  u8g2.sendBuffer();
}

#endif