#ifndef __OLED12864__
#define __OLED12864__
// Declaration for a 0.96" 12864 display connected to I2C (SDA, SCL pins)
// Using U8g2 library for display control
//
// Layout:
//   Row 0 (headline): font u8g2_font_squeezed_b7_tr, baseline y=16
//   Rows 1-4 (data):  font u8g2_font_6x13B_tr, baseline y = 27/39/51/63
//
// Partial refresh strategy:
//   - init()            : full init + full sendBuffer (once at power-on)
//   - updateData()      : writes values into RAM buffer only, no I2C
//   - refresh()         : partial push — tile rows 1-7 (data area) only
//   - drawStaticAndPush(): redraws headline+labels then full sendBuffer

#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>

// ── Layout constants ──────────────────────────────────────────────────────────
static const uint8_t ROW_Y[5]   = { 16, 27, 39, 51, 63 };
static const uint8_t SCREEN_W   = 128;
static const uint8_t FONT_H     = 13;   // u8g2_font_6x13B_tr cap height
static const uint8_t DATA_COL   = 70;   // left edge of value erase box
static const uint8_t DATA_RIGHT = 127;  // right-align digits to this x
// ─────────────────────────────────────────────────────────────────────────────

class SingleScreen {
public:
  SingleScreen(uint8_t _scl, uint8_t _sda, uint8_t addr);

  void init();
  void setChannel(int _ch);
  void setHeadLine(String _hdl);
  void setLineLabels(String _l1, String _l2, String _l3, String _l4);
  void updateData(double _d1, double _d2, double _d3, double _d4);

  void clear();             // clearBuffer + full sendBuffer (clears OLED)
  void refresh();           // update screen
  
private:
  U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2;
  uint8_t _addr;

  int    _channel;
  String _head_line;
  String _line1, _line2, _line3, _line4;
  double _data[4];

  void _drawStatic();
  void _drawValue(uint8_t rowIdx);
};

// ── Constructor ───────────────────────────────────────────────────────────────
SingleScreen::SingleScreen(uint8_t _scl, uint8_t _sda, uint8_t addr)
  : u8g2(U8G2_R0, U8X8_PIN_NONE, _scl, _sda), _addr(addr)
{
  u8g2.setI2CAddress(_addr);
  _channel = 0;
  for (uint8_t i = 0; i < 4; i++) _data[i] = 0.0;
}

// ── Public methods ────────────────────────────────────────────────────────────
void SingleScreen::init() {
  u8g2.begin();
  u8g2.setDisplayRotation(U8G2_R0);
  u8g2.setDrawColor(1);
  u8g2.clearBuffer();
  u8g2.sendBuffer();        // clear OLED
}

void SingleScreen::setChannel(int _ch) {
  if (_ch >= 0) _channel = _ch;
}

void SingleScreen::setHeadLine(String _hdl) {
  _head_line = _hdl;
}

void SingleScreen::setLineLabels(String _l1, String _l2, String _l3, String _l4) {
  _line1 = _l1; _line2 = _l2; _line3 = _l3; _line4 = _l4;
}

void SingleScreen::updateData(double _d1, double _d2, double _d3, double _d4) {
  _data[0] = _d1; _data[1] = _d2; _data[2] = _d3; _data[3] = _d4;
}

void SingleScreen::clear() {
  u8g2.clearBuffer();
  u8g2.sendBuffer();
}

void SingleScreen::refresh() {
  u8g2.clearBuffer();
  _drawStatic();
  u8g2.setFont(u8g2_font_6x13B_tr);
  for (uint8_t i = 0; i < 4; i++) _drawValue(i);
  u8g2.sendBuffer();
}

void SingleScreen::_drawStatic() {
  u8g2.setFont(u8g2_font_squeezed_b7_tr);
  u8g2.drawStr(0, ROW_Y[0], _head_line.c_str());
  char chBuf[8];
  snprintf(chBuf, sizeof(chBuf), "CH%d", _channel);
  uint8_t chWidth = u8g2.getStrWidth(chBuf);
  u8g2.drawStr(SCREEN_W - chWidth - 2, ROW_Y[0], chBuf);

  u8g2.setFont(u8g2_font_6x13B_tr);
  const String* labels[4] = { &_line1, &_line2, &_line3, &_line4 };
  for (uint8_t i = 0; i < 4; i++)
    u8g2.drawStr(0, ROW_Y[i + 1], labels[i]->c_str());
}

void SingleScreen::_drawValue(uint8_t rowIdx) {
  uint8_t y = ROW_Y[rowIdx + 1];

  double v = _data[rowIdx];
  bool negative = (v < 0.0);
  char digBuf[16];
  snprintf(digBuf, sizeof(digBuf), "%.2f", negative ? -v : v);
  uint8_t digW = u8g2.getStrWidth(digBuf);
  uint8_t digX = DATA_RIGHT - digW + 1;
  u8g2.drawStr(digX, y, digBuf);

  if (negative) {
    uint8_t signW = u8g2.getStrWidth("-");
    u8g2.drawStr(digX - signW, y, "-");
  }
}

#endif  // __OLED12864__