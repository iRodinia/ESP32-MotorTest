#ifndef __OLED12864__
#define __OLED12864__
// Declaration for a 0.96" 12864 display connected to I2C (SDA, SCL pins)

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "12864_font.h"

#define OLED_CMD  0
#define OLED_DATA 1

uint8_t OLED_GRAM[128][8]; // display buffer

class myI2C {
public:
  void init(uint8_t scl, uint8_t sda){
    _scl_pin = scl;
    _sda_pin = sda;
  }

  void I2C_Start(void){
    OLED_SDIN_Set();
    OLED_SCLK_Set();
    OLED_SDIN_Clr();
    OLED_SCLK_Clr();
  }

  void I2C_Stop(void){
    OLED_SCLK_Set();
    OLED_SDIN_Clr();
    OLED_SDIN_Set();
  }

  void I2C_WaitAck(void){
    OLED_SCLK_Set();
    OLED_SCLK_Clr();
  }

  void Send_Byte(uint8_t dat){
    uint8_t i;
    for(i=0;i<8;i++){
      OLED_SCLK_Clr();
      if(dat&0x80){
        OLED_SDIN_Set();
      }
      else{
        OLED_SDIN_Clr();
      }
      OLED_SCLK_Set();
      OLED_SCLK_Clr();
      dat<<=1;
    }
  }

private:
  uint8_t _scl_pin, _sda_pin;
  void OLED_SCLK_Clr(){digitalWrite(_scl_pin, LOW);}
  void OLED_SCLK_Set(){digitalWrite(_scl_pin, HIGH);}
  void OLED_SDIN_Clr(){digitalWrite(_sda_pin, LOW);}
  void OLED_SDIN_Set(){digitalWrite(_sda_pin, HIGH);}
};

// Coordinates of the display is originated from up-left corner (0,0)
class MyDisplay {
public:
  MyDisplay(uint8_t scl=33, uint8_t sda=32);
  bool init();  // init the display
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
  // Size of the font: 12-6x12, 16-8x16, 24-12x24
  void OLED_DrawPoint(uint8_t x,uint8_t y);  // x:0~127, y:0~63
  void OLED_ClearPoint(uint8_t x,uint8_t y);  // x:0~127, y:0~63
  void OLED_DrawLine(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2);  // x:0~128, y:0~64
  void OLED_DrawCircle(uint8_t x,uint8_t y,uint8_t r);  // (x,y) center position, r radius
  void OLED_ShowChar(uint8_t x,uint8_t y,const char chr,uint8_t size1);  // x:0~127, y:0~63, size1:12/16/24
  void OLED_ShowString(uint8_t x,uint8_t y,const char *chr,uint8_t size1);  // (x,y) start position, size1:12/16/24
  void OLED_ShowNum(uint8_t x,uint8_t y,int num,uint8_t len,uint8_t size1);  // (x,y) start position, len:number length, size1:12/16/24
  void OLED_ShowPicture(uint8_t x0,uint8_t y0,uint8_t x1,uint8_t y1,const uint8_t BMP[]);  // (x0,y0) start position, (x1,y1) end position, BMP[] pic buffer

private:
  void OLED_WR_Byte(uint8_t dat,uint8_t mode);  // send a byte to ssd1306. mode: 0-command, 1-data
  u32_t OLED_Pow(uint8_t m,uint8_t n);  // self-defined power function. result = m^n
  void OLED_WR_BP(uint8_t x,uint8_t y);  // set start position of writing oled buffer

  myI2C _i2c_port;
  uint8_t _scl, _sda;
  String _line1, _line2, _line3, _line4, _line5;
  bool _checkbox;
};

MyDisplay::MyDisplay(uint8_t scl, uint8_t sda){
  _scl = scl;
  _sda = sda;
}

bool MyDisplay::init(){
  pinMode(_scl, OUTPUT);
  pinMode(_sda, OUTPUT);
  _i2c_port.init(_scl, _sda);
  delay(100);

  OLED_WR_Byte(0xAE,OLED_CMD);  //--turn off oled panel
  OLED_WR_Byte(0x00,OLED_CMD);  //---set low column address
  OLED_WR_Byte(0x10,OLED_CMD);  //---set high column address
  OLED_WR_Byte(0x40,OLED_CMD);  //--set start line address  Set Mapping RAM Display Start Line (0x00~0x3F)
  OLED_WR_Byte(0x81,OLED_CMD);  //--set contrast control register
  OLED_WR_Byte(0xCF,OLED_CMD);  // Set SEG Output Current Brightness
  OLED_WR_Byte(0xA1,OLED_CMD);  //--Set SEG/Column Mapping     0xa0:left-right reverse, 0xa1:normal
  OLED_WR_Byte(0xC8,OLED_CMD);  //Set COM/Row Scan Direction   0xc0:up-down reverse, 0xc8:normal
  OLED_WR_Byte(0xA6,OLED_CMD);  //--set normal display
  OLED_WR_Byte(0xA8,OLED_CMD);  //--set multiplex ratio(1 to 64)
  OLED_WR_Byte(0x3f,OLED_CMD);  //--1/64 duty
  OLED_WR_Byte(0xD3,OLED_CMD);  //-set display offset Shift Mapping RAM Counter (0x00~0x3F)
  OLED_WR_Byte(0x00,OLED_CMD);  //-not offset
  OLED_WR_Byte(0xd5,OLED_CMD);  //--set display clock divide ratio/oscillator frequency
  OLED_WR_Byte(0x80,OLED_CMD);  //--set divide ratio, Set Clock as 100 Frames/Sec
  OLED_WR_Byte(0xD9,OLED_CMD);  //--set pre-charge period
  OLED_WR_Byte(0xF1,OLED_CMD);  //Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
  OLED_WR_Byte(0xDA,OLED_CMD);  //--set com pins hardware configuration
  OLED_WR_Byte(0x12,OLED_CMD);
  OLED_WR_Byte(0xDB,OLED_CMD);  //--set vcomh
  OLED_WR_Byte(0x40,OLED_CMD);  //Set VCOM Deselect Level
  OLED_WR_Byte(0x20,OLED_CMD);  //-Set Page Addressing Mode (0x00/0x01/0x02)
  OLED_WR_Byte(0x02,OLED_CMD);  //
  OLED_WR_Byte(0x8D,OLED_CMD);  //--set Charge Pump enable/disable
  OLED_WR_Byte(0x14,OLED_CMD);  //--set(0x10) disable
  OLED_WR_Byte(0xA4,OLED_CMD);  // Disable Entire Display On (0xa4/0xa5)
  OLED_WR_Byte(0xA6,OLED_CMD);  // Disable Inverse Display On (0xa6/a7) 
  OLED_WR_Byte(0xAF,OLED_CMD);

  OLED_Clear();
  set_Line1("T:0");
  set_Line2(" ");
  set_Line3(" ");
  set_Line4(" ");
  set_Line5(" ");
  set_Checkbox(false);
  return true;
}

void MyDisplay::OLED_ColorTurn(uint8_t i){
  if(!i)
    OLED_WR_Byte(0xA6,OLED_CMD);  // 0:normal
  else
    OLED_WR_Byte(0xA7,OLED_CMD);  // 1:reverse
}

void MyDisplay::OLED_DisplayTurn(uint8_t i){
  if(i==0){
    OLED_WR_Byte(0xC8,OLED_CMD);  // 0:normal
    OLED_WR_Byte(0xA1,OLED_CMD);
  }
  else{
    OLED_WR_Byte(0xC0,OLED_CMD);  // 1:reverse
    OLED_WR_Byte(0xA0,OLED_CMD);
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

void MyDisplay::OLED_Refresh(void)
{
  uint8_t i,n;
  for(i=0;i<8;i++){
    OLED_WR_Byte(0xb0+i,OLED_CMD);  // row start addr
    OLED_WR_Byte(0x00,OLED_CMD);    // low column start addr
    OLED_WR_Byte(0x10,OLED_CMD);    // high column start addr
    for(n=0;n<128;n++)
      OLED_WR_Byte(OLED_GRAM[n][i],OLED_DATA);
  }
}

void MyDisplay::OLED_UpdateRam(void){
  uint8_t i,n;
  for(i=0;i<8;i++){
    for(n=0;n<128;n++){
      OLED_GRAM[n][i]=0;  //clear all data
    }
  }
  OLED_ShowString(0, 1, _line1.c_str(), 12);
  OLED_DrawLine(0,15,128,15);
  OLED_ShowString(0, 16, _line2.c_str(), 12);
  OLED_ShowString(0, 28, _line3.c_str(), 12);
  OLED_ShowString(0, 40, _line4.c_str(), 12);
  OLED_ShowString(0, 52, _line5.c_str(), 12);

  OLED_DrawCircle(117,6,5);
  if(_checkbox){
    uint8_t i,j;
    for(i=116;i<=118;i++){
      for(j=5;j<=7;j++){
        OLED_DrawPoint(i,j);
      }
    }
  }
}

void MyDisplay::OLED_Clear(void){
  uint8_t i,n;
  for(i=0;i<8;i++){
    for(n=0;n<128;n++){
      OLED_GRAM[n][i]=0;  //clear all data
    }
  }
  OLED_Refresh();  // refresh
}

void MyDisplay::OLED_DrawPoint(uint8_t x,uint8_t y){
  uint8_t i,m,n;
  i=y/8;
  m=y%8;
  n=1<<m;
  OLED_GRAM[x][i]|=n;
}

void MyDisplay::OLED_ClearPoint(uint8_t x,uint8_t y){
  uint8_t i,m,n;
  i=y/8;
  m=y%8;
  n=1<<m;
  OLED_GRAM[x][i]=~OLED_GRAM[x][i];
  OLED_GRAM[x][i]|=n;
  OLED_GRAM[x][i]=~OLED_GRAM[x][i];
}

void MyDisplay::OLED_DrawLine(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2){
  uint8_t i,k,k1,k2,y0;
  if(x1==x2){
    for(i=0;i<(y2-y1);i++){
      OLED_DrawPoint(x1,y1+i);
    }
  }
  else if(y1==y2){
    for(i=0;i<(x2-x1);i++){
      OLED_DrawPoint(x1+i,y1);
    }
  }
  else{
    k1=y2-y1;
    k2=x2-x1;
    k=k1*10/k2;
    for(i=0;i<(x2-x1);i++){
      OLED_DrawPoint(x1+i,y1+i*k/10);
    }
  }
}

void MyDisplay::OLED_DrawCircle(uint8_t x,uint8_t y,uint8_t r){
  int a, b,num;
  a = 0;
  b = r;
  while(2 * b * b >= r * r){
    OLED_DrawPoint(x + a, y - b);
    OLED_DrawPoint(x - a, y - b);
    OLED_DrawPoint(x - a, y + b);
    OLED_DrawPoint(x + a, y + b);
    OLED_DrawPoint(x + b, y + a);
    OLED_DrawPoint(x + b, y - a);
    OLED_DrawPoint(x - b, y - a);
    OLED_DrawPoint(x - b, y + a);
    a++;
    num = (a * a + b * b) - r*r;  // calc the distance to center
    if(num > 0){
      b--;
      a--;
    }
  }
}

void MyDisplay::OLED_ShowChar(uint8_t x,uint8_t y,const char chr,uint8_t size1){
  uint8_t i,m,temp,size2,chr1;
  uint8_t y0=y;
  size2=(size1/8+((size1%8)?1:0))*(size1/2);
  chr1=chr-' ';
  for(i=0;i<size2;i++){
    if(size1==12){ 
      temp=pgm_read_byte(&asc2_1206[chr1][i]);
    }  // use 1206 font
    else if(size1==16){
      temp=pgm_read_byte(&asc2_1608[chr1][i]);
    }  // use 1608 font
    else if(size1==24){
      temp=pgm_read_byte(&asc2_2412[chr1][i]);
    }  // use 2412 font
    else
      return;
    
    for(m=0;m<8;m++){  // write buffer
      if(temp&0x80)
        OLED_DrawPoint(x,y);
      else
        OLED_ClearPoint(x,y);
      temp<<=1;
      y++;
      if((y-y0)==size1){
        y=y0;
        x++;
        break;
      }
    }
  }
}

void MyDisplay::OLED_ShowString(uint8_t x,uint8_t y,const char *chr,uint8_t size1)
{
  while((*chr>=' ')&&(*chr<='~')){  // check if illigal character
    OLED_ShowChar(x,y,*chr,size1);
    x+=size1/2;
    if(x>128-size1/2){  // automatic new line
      x=0;
      y+=size1;
    }
    chr++;
  }
}

u32_t MyDisplay::OLED_Pow(uint8_t m,uint8_t n){
  u32_t result=1;
  while(n--){
    result*=m;
  }
  return result;
}

void MyDisplay::OLED_ShowNum(uint8_t x,uint8_t y,int num,uint8_t len,uint8_t size1)
{
  uint8_t t,temp;
  for(t=0;t<len;t++){
    temp=(num/OLED_Pow(10,len-t-1))%10;
      if(temp==0){
        OLED_ShowChar(x+(size1/2)*t,y,'0',size1);
      }
      else{
        OLED_ShowChar(x+(size1/2)*t,y,temp+'0',size1);
      }
  }
}

void MyDisplay::OLED_WR_BP(uint8_t x,uint8_t y){
  OLED_WR_Byte(0xb0+y,OLED_CMD);  // set start addr
  OLED_WR_Byte(((x&0xf0)>>4)|0x10,OLED_CMD);
  OLED_WR_Byte((x&0x0f),OLED_CMD);
}

void MyDisplay::OLED_ShowPicture(uint8_t x0,uint8_t y0,uint8_t x1,uint8_t y1,const uint8_t BMP[]){
  int j=0;
  uint8_t t;
  uint8_t x,y;
  for(y=y0;y<y1;y++){
    OLED_WR_BP(x0,y);
    for(x=x0;x<x1;x++){
      t=pgm_read_byte(&BMP[j++]);
      OLED_WR_Byte(t,OLED_DATA);
    }
  }
}

void MyDisplay::OLED_WR_Byte(uint8_t dat,uint8_t mode){
  _i2c_port.I2C_Start();
  _i2c_port.Send_Byte(0x78);
  _i2c_port.I2C_WaitAck();
  if(mode){
    _i2c_port.Send_Byte(0x40);
  }
  else{
    _i2c_port.Send_Byte(0x00);
  }
  _i2c_port.I2C_WaitAck();
  _i2c_port.Send_Byte(dat);
  _i2c_port.I2C_WaitAck();
  _i2c_port.I2C_Stop();
}

#endif