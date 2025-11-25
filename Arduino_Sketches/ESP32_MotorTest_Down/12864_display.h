#ifndef __OLED12864__
#define __OLED12864__
// Declaration for a 0.96" 12864 display connected to I2C (SDA, SCL pins)
// Using U8g2 library for display control

#include <Arduino.h>
#include <U8g2lib.h>

// Coordinates of the display is originated from up-left corner (0,0)
class MyDisplay {
public:
  MyDisplay(uint8_t scl=33, uint8_t sda=32);
  String init();  // init the display
  void OLED_Reset_Display(void);  // reset to fix pixel shift
  void OLED_Clear(void);  // clear display
  void OLED_UpdateRam(void);  // upload the strings to RAM
  void OLED_Refresh(void);  // upload the buffer to OLED
  void OLED_ColorTurn(uint8_t i);  // set the color (normal or reverse)
  void OLED_DisplayTurn(uint8_t i);  // set the direction (normal or 180 deg reverse)
  void set_Line1(String line1);  // head line, lenght <= 18
  void set_Line2(String line2);  // content line, length <= 21
  void set_Line3(String line3);  // content line, length <= 21
  void set_Line4(String line4);  // content line, length <= 21
  void set_Line5(String line5);  // content line, length <= 21
  void set_Checkbox(bool flag);

protected:
  // These functions won't take effect immediately, need to call OLED_Refresh() afterwards
  void OLED_DrawPoint(uint8_t x,uint8_t y);  // x:0~127, y:0~63
  void OLED_ClearPoint(uint8_t x,uint8_t y);  // x:0~127, y:0~63
  void OLED_DrawLine(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2);  // x:0~128, y:0~64
  void OLED_DrawCircle(uint8_t x,uint8_t y,uint8_t r);  // (x,y) center position, r radius
  void OLED_ShowChar(uint8_t x,uint8_t y,const char chr,uint8_t size1);  // x:0~127, y:0~63, size1:12/16/24
  void OLED_ShowString(uint8_t x,uint8_t y,const char *chr,uint8_t size1);  // (x,y) start position, size1:12/16/24
  void OLED_ShowNum(uint8_t x,uint8_t y,int num,uint8_t len,uint8_t size1);  // (x,y) start position, len:number length, size1:12/16/24
  void OLED_ShowPicture(uint8_t x0,uint8_t y0,uint8_t x1,uint8_t y1,const uint8_t BMP[]);  // (x0,y0) start position, (x1,y1) end position, BMP[] pic buffer

private:
  U8G2_SSD1306_128X64_NONAME_F_SW_I2C *u8g2;
  uint8_t _scl, _sda;
  String _line1, _line2, _line3, _line4, _line5;
  bool _checkbox;
  bool _color_inverted;
  bool _display_flipped;
  
  u32_t OLED_Pow(uint8_t m,uint8_t n);  // self-defined power function. result = m^n
  const uint8_t* getFontBySize(uint8_t size);
};

MyDisplay::MyDisplay(uint8_t scl, uint8_t sda){
  _scl = scl;
  _sda = sda;
  _color_inverted = false;
  _display_flipped = false;
  u8g2 = nullptr;
}

String MyDisplay::init(){
  // Initialize U8g2 with software I2C
  // U8G2_SSD1306_128X64_NONAME_F_SW_I2C(rotation, clock, data, reset)
  u8g2 = new U8G2_SSD1306_128X64_NONAME_F_SW_I2C(U8G2_R0, _scl, _sda, U8X8_PIN_NONE);
  
  u8g2->begin();
  u8g2->setDrawColor(1);
  
  OLED_Clear();
  set_Line1("T:0");
  set_Line2(" ");
  set_Line3(" ");
  set_Line4(" ");
  set_Line5(" ");
  set_Checkbox(false);
  return "";
}

void MyDisplay::OLED_Reset_Display(void){
  // U8g2 handles reset internally, but we can reinitialize if needed
  if(u8g2){
    u8g2->clear();
    u8g2->sendBuffer();
  }
}

void MyDisplay::OLED_ColorTurn(uint8_t i){
  _color_inverted = (i != 0);
  if(u8g2){
    if(_color_inverted){
      u8g2->setDrawColor(0);  // inverted
    } else {
      u8g2->setDrawColor(1);  // normal
    }
  }
}

void MyDisplay::OLED_DisplayTurn(uint8_t i){
  _display_flipped = (i != 0);
  if(u8g2){
    if(_display_flipped){
      u8g2->setDisplayRotation(U8G2_R2);  // 180 degree rotation
    } else {
      u8g2->setDisplayRotation(U8G2_R0);  // normal
    }
  }
}

void MyDisplay::set_Line1(String line1){
  if(line1.length() > 18){
    line1 = line1.substring(0, 18);
  }
  _line1 = line1;
}

void MyDisplay::set_Line2(String line2){
  if(line2.length() > 21){
    line2 = line2.substring(0, 21);
  }
  _line2 = line2;
}

void MyDisplay::set_Line3(String line3){
  if(line3.length() > 21){
    line3 = line3.substring(0, 21);
  }
  _line3 = line3;
}

void MyDisplay::set_Line4(String line4){
  if(line4.length() > 21){
    line4 = line4.substring(0, 21);
  }
  _line4 = line4;
}

void MyDisplay::set_Line5(String line5){
  if(line5.length() > 21){
    line5 = line5.substring(0, 21);
  }
  _line5 = line5;
}

void MyDisplay::set_Checkbox(bool flag){
  _checkbox = flag;
}

void MyDisplay::OLED_Refresh(void){
  if(u8g2){
    u8g2->sendBuffer();
  }
}

void MyDisplay::OLED_UpdateRam(void){
  if(!u8g2) return;
  
  u8g2->clearBuffer();
  
  // Draw content using the stored strings
  u8g2->setFont(u8g2_font_6x12_tr);  // 6x12 font for size 12
  
  // Line 1 - header
  u8g2->drawStr(0, 11, _line1.c_str());  // y position is baseline
  
  // Draw horizontal line
  u8g2->drawLine(0, 15, 127, 15);
  
  // Lines 2-5 - content
  u8g2->drawStr(0, 27, _line2.c_str());
  u8g2->drawStr(0, 39, _line3.c_str());
  u8g2->drawStr(0, 51, _line4.c_str());
  u8g2->drawStr(0, 63, _line5.c_str());
  
  // Draw checkbox circle
  u8g2->drawCircle(117, 6, 5, U8G2_DRAW_ALL);
  
  // Fill checkbox if checked
  if(_checkbox){
    u8g2->drawDisc(117, 6, 3, U8G2_DRAW_ALL);  // filled circle
  }
}

void MyDisplay::OLED_Clear(void){
  if(u8g2){
    u8g2->clearBuffer();
    u8g2->sendBuffer();
  }
}

void MyDisplay::OLED_DrawPoint(uint8_t x, uint8_t y){
  if(u8g2){
    u8g2->drawPixel(x, y);
  }
}

void MyDisplay::OLED_ClearPoint(uint8_t x, uint8_t y){
  if(u8g2){
    u8g2->setDrawColor(0);  // set to background color
    u8g2->drawPixel(x, y);
    u8g2->setDrawColor(_color_inverted ? 0 : 1);  // restore draw color
  }
}

void MyDisplay::OLED_DrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2){
  if(u8g2){
    u8g2->drawLine(x1, y1, x2, y2);
  }
}

void MyDisplay::OLED_DrawCircle(uint8_t x, uint8_t y, uint8_t r){
  if(u8g2){
    u8g2->drawCircle(x, y, r, U8G2_DRAW_ALL);
  }
}

const uint8_t* MyDisplay::getFontBySize(uint8_t size){
  switch(size){
    case 12:
      return u8g2_font_6x12_tr;  // 6x12 font
    case 16:
      return u8g2_font_8x13_tr;  // 8x13 font (closest to 8x16)
    case 24:
      return u8g2_font_12x24_tr;  // 12x24 font (needs to be available)
    default:
      return u8g2_font_6x12_tr;
  }
}

void MyDisplay::OLED_ShowChar(uint8_t x, uint8_t y, const char chr, uint8_t size1){
  if(!u8g2) return;
  
  const uint8_t* font = getFontBySize(size1);
  u8g2->setFont(font);
  
  char str[2] = {chr, '\0'};
  // U8g2 uses baseline for y coordinate, adjust accordingly
  u8g2->drawStr(x, y + size1 - 1, str);
}

void MyDisplay::OLED_ShowString(uint8_t x, uint8_t y, const char *chr, uint8_t size1){
  if(!u8g2) return;
  
  const uint8_t* font = getFontBySize(size1);
  u8g2->setFont(font);
  
  // U8g2 uses baseline for y coordinate
  u8g2->drawStr(x, y + size1 - 1, chr);
}

u32_t MyDisplay::OLED_Pow(uint8_t m, uint8_t n){
  u32_t result = 1;
  while(n--){
    result *= m;
  }
  return result;
}

void MyDisplay::OLED_ShowNum(uint8_t x, uint8_t y, int num, uint8_t len, uint8_t size1){
  if(!u8g2) return;
  
  char buffer[12];  // enough for most numbers
  snprintf(buffer, sizeof(buffer), "%0*d", len, num);
  OLED_ShowString(x, y, buffer, size1);
}

void MyDisplay::OLED_ShowPicture(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, const uint8_t BMP[]){
  if(!u8g2) return;
  
  // U8g2 uses XBM format, but we can draw pixel by pixel for compatibility
  uint8_t width = x1 - x0;
  uint8_t height = y1 - y0;
  
  for(uint8_t y = 0; y < height; y++){
    for(uint8_t x = 0; x < width; x++){
      uint8_t byte_index = y * width + x;
      uint8_t pixel = BMP[byte_index];
      
      // Draw pixels based on the byte value
      for(uint8_t bit = 0; bit < 8; bit++){
        if(pixel & (1 << bit)){
          u8g2->drawPixel(x0 + x, y0 + y * 8 + bit);
        }
      }
    }
  }
}

#endif