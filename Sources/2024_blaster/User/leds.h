#ifndef LEDS_H
#define LEDS_H

void init_leds();
uint32_t color(int r, int g, int b);
void set_led(int led, uint32_t color);
void fill(uint32_t color);
void write_leds(void);

#endif
