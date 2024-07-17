#include "stdint.h"
#include "ch32v20x.h"

#define LedCount 5
uint8_t LedBuffer[3 * LedCount]; // Each color is 3 bytes

void init_leds(void)
{
    //pinMode(Pin_D0, OUTPUT);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    GPIO_PinRemapConfig(AFIO_PCFR1_PD01_REMAP, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure = {0};
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    for (int i=0; i< LedCount*3; i++) LedBuffer[i] = 0;
}

void led_sendbit(uint8_t bit)
{
    if (bit)
    {
        GPIOD->BSHR = 1 << 0; // put pin high and wait for 800nS
        __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
        __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
        __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
        __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
        __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
        __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
        __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
        __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
        __asm__("nop");__asm__("nop");
        GPIOD->BCR = 1 << 0; // put pin low and exit, 400nS is taken up by other functions
        return;
    }
    GPIOD->BSHR = 1 << 0; // put pin high and wait for 400nS
    __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
    __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
    __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
    __asm__("nop");__asm__("nop");
    GPIOD->BCR = 1 << 0; // put pin low and wait for 400nS, 400nS is taken up by other functions
    __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
    __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
    __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
    __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
    __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
}

void send_color(uint8_t red, uint8_t green, uint8_t blue)
{
    // Send the green component first (MSB)
    for (int i = 7; i >= 0; i--) {
        led_sendbit((green >> i) & 1);
    }
    // Send the red component next
    for (int i = 7; i >= 0; i--) {
        led_sendbit((red >> i) & 1);
    }
    // Send the blue component last (LSB)
    for (int i = 7; i >= 0; i--) {
        led_sendbit((blue >> i) & 1);
    }
}

uint32_t color(int r, int g, int b) {
    uint32_t c = (r<<16)+(g<<8)+b;
    return c;
}

int red(uint32_t color){
    return (color >> 16) & 0xFF;
}

int green(uint32_t color){
    return (color >> 8) & 0xFF;
}

int blue(uint32_t color){
    return (color >> 0) & 0xFF;
}

void set_led(int led, uint32_t color)
{
    if (led == 0)
    {
        LedBuffer[led*3] = red(color);
        LedBuffer[led*3+1] = green(color);
    } else {
        LedBuffer[led*3] = green(color);
        LedBuffer[led*3+1] = red(color);
    }
    LedBuffer[led*3+2] = blue(color);
}

void fill(uint32_t color){
    for (int i=0; i< LedCount; i++) set_led(i, color);
}

void write_leds(void)
{
    __disable_irq();
    for (int i = 0; i< LedCount; i++){
        send_color(LedBuffer[i*3],LedBuffer[i*3+1],LedBuffer[i*3+2]);
    }
    for (int i = 0; i < 3500; i++) {
       __asm volatile ("nop");  // ASM nop (no operation) to prevent the compiler from optimizing the loop away
    }
   __enable_irq();
}
