#ifndef _THINKINK_420_GRAY4_T2_H
#define _THINKINK_420_GRAY4_T2_H

#include "Adafruit_EPD.h"

// clang-format off

static const uint8_t ti_420t2_gray4_init_code[] {
    IL0398_POWER_SETTING, 5, 0x03, 0x00, 0x2b, 0x2b, 0x13,
    IL0398_BOOSTER_SOFT_START, 3, 0x17, 0x17, 0x17,
    IL0398_POWER_ON, 0,
    0xFF, 200,
    IL0398_PANEL_SETTING, 1, 0x3F,
    IL0398_PLL, 1, 0x3C,    
    IL0398_VCM_DC_SETTING, 1, 0x12,
    IL0398_VCOM, 1, 0x97,
    0xFE // EOM
};

static const uint8_t ti_420t2_gray4_lut_code[] = {
  // const unsigned char lut_vcom[]PROGMEM =
  IL0398_LUT1, 42,
  0x00, 0x0A, 0x00, 0x00, 0x00, 0x01,
  0x60, 0x14, 0x14, 0x00, 0x00, 0x01,
  0x00, 0x14, 0x00, 0x00, 0x00, 0x01,
  0x00, 0x13, 0x0A, 0x01, 0x00, 0x01,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  // const unsigned char lut_ww[]PROGMEM ={
  IL0398_LUTWW, 42, 
  0x40, 0x0A, 0x00, 0x00, 0x00, 0x01,
  0x90, 0x14, 0x14, 0x00, 0x00, 0x01,
  0x10, 0x14, 0x0A, 0x00, 0x00, 0x01,
  0xA0, 0x13, 0x01, 0x00, 0x00, 0x01,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  // const unsigned char lut_bw[]PROGMEM ={
  IL0398_LUTBW, 42,
  0x40, 0x0A, 0x00, 0x00, 0x00, 0x01,
  0x90, 0x14, 0x14, 0x00, 0x00, 0x01,
  0x00, 0x14, 0x0A, 0x00, 0x00, 0x01,
  0x99, 0x0C, 0x01, 0x03, 0x04, 0x01,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  // const unsigned char lut_wb[]PROGMEM ={
  IL0398_LUTWB, 42,
  0x40, 0x0A, 0x00, 0x00, 0x00, 0x01,
  0x90, 0x14, 0x14, 0x00, 0x00, 0x01,
  0x00, 0x14, 0x0A, 0x00, 0x00, 0x01,
  0x99, 0x0B, 0x04, 0x04, 0x01, 0x01,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  // const unsigned char lut_bb[]PROGMEM ={
  IL0398_LUTBB, 42,
  0x80, 0x0A, 0x00, 0x00, 0x00, 0x01,
  0x90, 0x14, 0x14, 0x00, 0x00, 0x01,
  0x20, 0x14, 0x0A, 0x00, 0x00, 0x01,
  0x50, 0x13, 0x01, 0x00, 0x00, 0x01,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

// clang-format on

class ThinkInk_420_Grayscale4_T2 : public Adafruit_IL0398 {
private:
public:
  ThinkInk_420_Grayscale4_T2(int8_t SID, int8_t SCLK, int8_t DC, int8_t RST,
                             int8_t CS, int8_t SRCS, int8_t MISO,
                             int8_t BUSY = -1)
      : Adafruit_IL0398(300, 400, SID, SCLK, DC, RST, CS, SRCS, MISO, BUSY){};

  ThinkInk_420_Grayscale4_T2(int8_t DC, int8_t RST, int8_t CS, int8_t SRCS,
                             int8_t BUSY = -1, SPIClass *spi = &SPI)
      : Adafruit_IL0398(300, 400, DC, RST, CS, SRCS, BUSY, spi){};

  void begin(thinkinkmode_t mode = THINKINK_MONO) {
    Adafruit_EPD::begin(true);
    setColorBuffer(0, true); // layer 0 uninverted
    setBlackBuffer(1, true); // layer 1 uninverted

    if (mode == THINKINK_MONO) {
      _epd_init_code = NULL;
      _epd_lut_code = NULL;

      layer_colors[EPD_WHITE] = 0b00;
      layer_colors[EPD_BLACK] = 0b01;
      layer_colors[EPD_RED] = 0b01;
      layer_colors[EPD_GRAY] = 0b01;
      layer_colors[EPD_LIGHT] = 0b00;
      layer_colors[EPD_DARK] = 0b01;
    }

    if (mode == THINKINK_GRAYSCALE4) {
      _epd_init_code = ti_420t2_gray4_init_code;
      _epd_lut_code = ti_420t2_gray4_lut_code;

      layer_colors[EPD_WHITE] = 0b00;
      layer_colors[EPD_BLACK] = 0b11;
      layer_colors[EPD_RED] = 0b01;
      layer_colors[EPD_GRAY] = 0b10;
      layer_colors[EPD_LIGHT] = 0b01;
      layer_colors[EPD_DARK] = 0b10;
    }

    default_refresh_delay = 1000;
    setRotation(1);
    powerDown();
  };
};

#endif // _THINKINK_420_GRAY4_T2_H
