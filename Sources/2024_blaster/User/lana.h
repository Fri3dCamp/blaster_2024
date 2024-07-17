#ifndef LANA_H
#define LANA_H

#include "debug.h"

#define PIN_PA0  0
#define PIN_PA1  1
#define PIN_PA2  2
#define PIN_PA3  3
#define PIN_PA4  4
#define PIN_PA5  5
#define PIN_PA6  6
#define PIN_PA7  7
#define PIN_PA9  9
#define PIN_PA13 13
#define PIN_PA14 14
#define PIN_PA15 15

#define PIN_PB0  16+0
#define PIN_PB1  16+1
#define PIN_PB3  16+3
#define PIN_PB4  16+4
#define PIN_PB5  16+5
#define PIN_PB6  16+6
#define PIN_PB7  16+7
#define PIN_PB8  16+8

#define PIN_PD0  48+0
#define PIN_PD1  48+1

#define HIGH 0x1
#define LOW  0x0

#define INPUT 0x0
#define OUTPUT 0x1
#define INPUT_PULLUP 0x2
#define INPUT_PULLDOWN 0x3
#define OUTPUT_AF_PP 0x4

extern volatile uint32_t ir_ticks;


uint32_t micros(void);
uint32_t millis(void);
void delay_ms(uint32_t delay);
void SYSTICK_Init_Config(u64 ticks);

void enable_ir_carrier(void);
void ir_on(void);
void ir_off(void);

void enable_ir_rx(void);
void disable_ir_rx(void);

//void LED_SendBit(uint8_t bit);
//void LED_SendColour(uint8_t red, uint8_t green, uint8_t blue);
//void SetLed(int i, uint8_t r,uint8_t g,uint8_t b);
//void Write(void);
GPIO_TypeDef* PinToPort(int pin);
uint32_t PinToPeriph(int pin);
uint16_t PinToBitMask(int pin);
void DisableSWD_UsePinsAsGPIO(void);
void EnableSWD_UsePinsAsGPIO(void);
void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, int value);
uint8_t digitalRead(uint8_t pin);
//void initNeopixel(void);

void tone(uint16_t frequency);
void notone(void);

#endif
