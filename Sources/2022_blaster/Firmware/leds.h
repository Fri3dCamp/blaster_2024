#ifndef LEDS_H
#define LEDS_H

#include <Arduino.h>

class _leds
{
private:
  _leds();
  uint32_t color;

public:
  static _leds &getInstance();
  void init();
  void setPixelColor(byte pixel, uint32_t color);
  void setDiskColor(byte disk, uint32_t color);
  void update();
  static uint32_t wheel(byte index);
  void clear();
  void stealth(bool status);
  void setBrightness(uint8_t brightness);
};

extern _leds &Leds;

#endif
