#ifndef __OLED12864__
#define __OLED12864__
// Declaration for a 0.96" 12864 display connected to I2C (SDA, SCL pins)
// Using U8g2 library for display control
//
// Layout:
//   Row 0 (headline): font u8g2_font_t0_15b_tr, baseline y=15  — left: head_line | right: "CHx"
//   Rows 1-4 (data):  font u8g2_font_6x13B_tr,  each char ~6px wide
//   Row 1: y=27  — label + data (d1)
//   Row 2: y=39  — label + data (d2)
//   Row 3: y=51  — label + data (d3)
//   Row 4: y=63  — label + data (d4)
//
// Partial refresh strategy:
//   - init() draws static content (headline, labels) and does a full sendBuffer().
//   - updateData() stores new values and redraws only the value area in the RAM buffer.
//   - refresh() pushes only the data tile rows (y=14..63) to the OLED via updateDisplayArea.

#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>

// ── Layout constants ─────────────────────────────────────────────────────────
static const uint8_t ROW_Y[5]      = { 16, 27, 39, 51, 63 };  // baseline y per row
                                                                // row 0 raised to 15 for the taller headline font
static const uint8_t SCREEN_W      = 128;
static const uint8_t FONT_H        = 13;  // pixel height of u8g2_font_6x13B_tr (data rows)
static const uint8_t DATA_COL      = 70;  // x where the value area begins (erase box left edge)
static const uint8_t DATA_RIGHT    = 127; // x right boundary for right-aligned digits
                                          // negative sign is drawn to the left of the digits
// ─────────────────────────────────────────────────────────────────────────────

class SingleScreen {
public:
  // addr: I2C write address, typically 0x78 or 0x7A
  SingleScreen(uint8_t _scl, uint8_t _sda, uint8_t addr);

  void init();                                                      // initialise display
  void setChannel(int _ch);                                         // set channel number shown as "CHx"
  void setHeadLine(String _hdl);                                    // set headline string
  void setLineLabels(String _l1, String _l2, String _l3, String _l4); // set per-row labels

  // Store new values and update the value area in the RAM buffer (no I2C transfer).
  // Call refresh() afterwards to push changes to the display.
  void updateData(double _d1, double _d2, double _d3, double _d4);

  void clear();    // clear display (buffer + OLED)
  void refresh();  // partial push: sends only the data rows (y=14..63) to the OLED

private:
  U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2;

  int    _channel;
  String _head_line;
  String _line1, _line2, _line3, _line4;
  double _data[4];

  // Draw the static parts (headline row + label column) into the buffer.
  void _drawStatic();

  // Erase and redraw one data value at the given row index (1-4 → array 0-3).
  void _drawValue(uint8_t rowIdx);
};

// ── Constructor ───────────────────────────────────────────────────────────────
SingleScreen::SingleScreen(uint8_t _scl, uint8_t _sda, uint8_t addr)
  : u8g2(U8G2_R0, U8X8_PIN_NONE, _scl, _sda)
{
  u8g2.setI2CAddress(addr);
  _channel = 0;
  for (uint8_t i = 0; i < 4; i++) _data[i] = 0.0;
}

// ── Public methods ────────────────────────────────────────────────────────────
void SingleScreen::init() {
  u8g2.begin();
  u8g2.setDisplayRotation(U8G2_R0);
  u8g2.setDrawColor(1);
  clear();
  _drawStatic();
  refresh();
}

void SingleScreen::setChannel(int _ch) {
  if (_ch >= 0) _channel = _ch;
}

void SingleScreen::setHeadLine(String _hdl) {
  _head_line = _hdl;
}

void SingleScreen::setLineLabels(String _l1, String _l2, String _l3, String _l4) {
  _line1 = _l1;
  _line2 = _l2;
  _line3 = _l3;
  _line4 = _l4;
}

void SingleScreen::updateData(double _d1, double _d2, double _d3, double _d4) {
  _data[0] = _d1;
  _data[1] = _d2;
  _data[2] = _d3;
  _data[3] = _d4;

  // Redraw value areas in the RAM buffer only — no I2C transfer here.
  // Call refresh() to push the changes to the display.
  u8g2.setFont(u8g2_font_6x13B_tr);
  for (uint8_t i = 0; i < 4; i++) {
    _drawValue(i);
  }
}

void SingleScreen::clear() {
  u8g2.clearBuffer();
  u8g2.sendBuffer();
}

void SingleScreen::refresh() {
  // Partial transfer: only tile rows covering the data area (y=14..63).
  // Headline row (y=0..13, tile rows 0-1) is static and not retransmitted.
  // u8g2 tile size = 8px.  Data rows start at y=14 → tile row 1 (with 2px overlap,
  // harmless since that region is the separator line which never changes).
  // Tile row 1 = y8..15, through tile row 7 = y56..63 → ty=1, th=7.
  u8g2.updateDisplayArea(0, 1, 16, 7);  // (tx, ty, tw, th) in 8-px tiles
}

// ── Private helpers ───────────────────────────────────────────────────────────

void SingleScreen::_drawStatic() {
  // ── Row 0: headline font ──────────────────────────────────────────────────
  u8g2.setFont(u8g2_font_squeezed_b7_tr);

  u8g2.drawStr(0, ROW_Y[0], _head_line.c_str());

  // Build "CHx" string and right-align it
  char chBuf[8];
  snprintf(chBuf, sizeof(chBuf), "CH%d", _channel);
  uint8_t chWidth = u8g2.getStrWidth(chBuf);
  u8g2.drawStr(SCREEN_W - chWidth - 2, ROW_Y[0], chBuf);

  // ── Rows 1-4: label font ──────────────────────────────────────────────────
  u8g2.setFont(u8g2_font_6x13B_tr);

  const String* labels[4] = { &_line1, &_line2, &_line3, &_line4 };
  for (uint8_t i = 0; i < 4; i++) {
    u8g2.drawStr(0, ROW_Y[i + 1], labels[i]->c_str());
  }
}

void SingleScreen::_drawValue(uint8_t rowIdx) {
  // rowIdx: 0-3 → display rows 1-4
  uint8_t y = ROW_Y[rowIdx + 1];

  // Erase the entire value area
  u8g2.setDrawColor(0);
  u8g2.drawBox(DATA_COL, y - FONT_H + 2, SCREEN_W - DATA_COL, FONT_H);
  u8g2.setDrawColor(1);

  double v = _data[rowIdx];
  bool negative = (v < 0.0);

  // Format the absolute value — digits + decimal point only, no sign
  char digBuf[16];
  snprintf(digBuf, sizeof(digBuf), "%.2f", negative ? -v : v);

  // Right-align the digit string against DATA_RIGHT
  uint8_t digW = u8g2.getStrWidth(digBuf);
  uint8_t digX = DATA_RIGHT - digW + 1;
  u8g2.drawStr(digX, y, digBuf);

  // Draw the minus sign immediately to the left of the digit block
  if (negative) {
    uint8_t signW = u8g2.getStrWidth("-");
    u8g2.drawStr(digX - signW, y, "-");
  }
}

#endif  // __OLED12864__