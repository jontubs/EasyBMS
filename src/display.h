#pragma once

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>
#include "Measurements.hpp"


#define TFT_CS     D0
#define TFT_RST    -1  // TFT RST = Arduino RST
#define TFT_DC   D3
#define TFT_SCLK D5
#define TFT_MOSI D7

struct DisplayData {
    Measurements measurements;
    std::bitset<12> balance_bits;
    long uptime_seconds;
};

void setup_display(void);
void draw_cell_voltages(const DisplayData& data);