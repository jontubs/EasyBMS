#include "display.h"

// Option 1 (recommended): must use the hardware SPI pins
// (for UNO thats sclk = 13 and sid = 11) and pin 10 must be
// an output. This is much faster - also required if you want
// to use the microSD card (see the image drawing example)

// For 1.44" and 1.8" TFT with ST7735 use
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS,  TFT_DC, TFT_RST); // Fast
//Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST); // Slow

void drawtext(const char *text, uint16_t color) {
  tft.setCursor(0, 0);
  tft.setTextColor(color);
  tft.setTextWrap(true);
  tft.print(text);
}

/*
void draw_cell_voltage(const char* text, int row, int column) {
    uint8_t text_size = 1;
    tft.setTextSize(text_size);
    tft.setCursor(column*8*4, row*8*text_size);
    tft.setTextWrap(true);
    tft.print(text);
}
*/

void draw_cell_voltages(const DisplayData& data) {
  /*
  tft.invertDisplay(true);
  delay(500);
  tft.invertDisplay(false);
  delay(500);
  */
  tft.fillScreen(ST77XX_BLACK);

  uint8_t text_size = 1;
  tft.setTextSize(text_size);
  tft.setRotation(1);
  tft.setTextWrap(true);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextWrap(true);

  for (int i = 0; i < 12; i++) {
    String display_text = String(data.measurements.cell_voltages[i], 3) + "-" + "99mV" + " " + "1:37.5C";
    tft.setCursor(0*8*4*text_size, i*8*text_size);
    tft.print(display_text.c_str());
  }

  //tft.drawLine(60,0,60, 128, ST77XX_WHITE);
  //testdrawtext(display_text.c_str(), ST77XX_WHITE);
  delay(10000);
}

void setup_display(void) {
  // Use this initializer if you're using a 1.8" TFT
  tft.initR(INITR_BLACKTAB);   // initialize a ST7735S chip, black tab
}
