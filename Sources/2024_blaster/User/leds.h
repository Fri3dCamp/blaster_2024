#ifndef LEDS_H
#define LEDS_H

#include <cstdint>

class _leds
{
private:
  _leds();
  uint32_t color;

public:
  static _leds &getInstance();
  void init();
  void setPixelColor(uint8_t pixel, uint32_t color);
  void setDiskColor(uint8_t disk, uint32_t color);
  void update();
  static uint32_t wheel(uint8_t index);
  void clear();
  void stealth(bool status);
  void setBrightness(uint8_t brightness);
};

//extern _leds &Leds;

#endif
