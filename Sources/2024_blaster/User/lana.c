#include "lana.h"
#include "stdlib.h"

#define N 14
uint8_t rgbArray[3 * N]; // Each color is 3 bytes
volatile uint32_t ir_ticks = 0;

uint32_t micros(void)
{
    return (ir_ticks * 560) + (SysTick->CNT / 48);
}

uint32_t millis(void)
{
    return micros() / 1000;
}

void delay_ms(uint32_t delay)
{
    uint32_t ref = millis()+delay;
    while (millis() < ref) __NOP();
}

void SYSTICK_Init_Config(u64 ticks)
{
    SysTick->SR = 0;
    SysTick->CNT = 0;
    SysTick->CMP = ticks;
    SysTick->CTLR =0xF;

    NVIC_SetPriority(SysTicK_IRQn, 1);
    NVIC_EnableIRQ(SysTicK_IRQn);
}


void SysTick_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void SysTick_Handler(void)
{
    // clear IRQ
    SysTick->SR = 0;

    // update counter
    ir_ticks++;



}

void enable_ir_carrier(void)
{
    // Enable GPIO and Timer Clocks
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // Alternate Function Push Pull Mode
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    // Time base configuration
    TIM_TimeBaseStructure.TIM_Period = 1266;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    // PWM1 Mode configuration: Channel 2 (Assuming PA1 is connected to TIM2 CH2)
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 600; // 50% duty cycle
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

    TIM_OC4Init(TIM3, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM3, ENABLE);
}

void ir_on(void){
    pinMode(PIN_PB1, OUTPUT_AF_PP);
    TIM_Cmd(TIM3, ENABLE);
}

void ir_off(void) {
    TIM_Cmd(TIM3, DISABLE);
    pinMode(PIN_PB1, OUTPUT);
    digitalWrite(PIN_PB1, LOW);
}

void LED_SendBit(uint8_t bit)
{
    if (bit) {
    //// Send a 1 bit
        GPIOD->BSHR = 1 << 0; // put pin C4 high and wait for 800nS
            __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
            __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
            __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
            __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
            __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
            __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
            __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
            __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
            __asm__("nop");__asm__("nop");
        GPIOD->BCR = 1 << 0; // put pin C4 low and exit, 400nS is taken up by other functions
        return;
        }
//    else {
        // Send a 0 bit
        GPIOD->BSHR = 1 << 0; // put pin C4 high and wait for 400nS
            __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
            __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
            __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
            __asm__("nop");__asm__("nop");
        GPIOD->BCR = 1 << 0; // put pin C4 low and wait for 400nS, 400nS is taken up by other functions
            __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
            __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
            __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
            __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
            __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
//    }
}

// Send a single colour for a single LED
//WS2812B LEDs want 24 bits per led in the string
void LED_SendColour(uint8_t red, uint8_t green, uint8_t blue)
{
    // Send the green component first (MSB)
    for (int i = 7; i >= 0; i--) {
        LED_SendBit((green >> i) & 1);
    }
    // Send the red component next
    for (int i = 7; i >= 0; i--) {
        LED_SendBit((red >> i) & 1);
    }
    // Send the blue component last (LSB)
    for (int i = 7; i >= 0; i--) {
        LED_SendBit((blue >> i) & 1);
    }
}

void Write(void){
    for (int i = 0; i< N; i++){
        LED_SendColour(rgbArray[i*3],rgbArray[i*3+1],rgbArray[i*3+2]);
    }
    for (int i = 0; i < 3500; i++) {
       __asm volatile ("nop");  // ASM nop (no operation) to prevent the compiler from optimizing the loop away
   }
}

void SetLed(int i, uint8_t r,uint8_t g,uint8_t b){
    if (i == 0)
    {
        rgbArray[i*3] = r;
        rgbArray[i*3+1] = g;
    } else {
        rgbArray[i*3] = g;
        rgbArray[i*3+1] = r;
    }
    rgbArray[i*3+2] = b;
}

GPIO_TypeDef* PinToPort(int pin)
{
    if(pin <= 15) return GPIOA;
    if(pin <= 31) return GPIOB;
    if(pin <= 63) return GPIOD;
    return 0;
}

uint32_t PinToPeriph(int pin)
{
    if(pin <= 15) return RCC_APB2Periph_GPIOA;
    if(pin <= 31) return RCC_APB2Periph_GPIOB;
    if(pin <= 63) return RCC_APB2Periph_GPIOD;
    return 0;
}

uint16_t PinToBitMask(int pin)
{
    return 1<<(pin%16);
}

void DisableSWD_UsePinsAsGPIO(void) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void EnableSWD_UsePinsAsGPIO(void) {
       RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA, ENABLE);
       GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, DISABLE);
}

void pinMode(uint8_t pin, uint8_t mode)
{
    if (pin == PIN_PA13 || pin == PIN_PA14){
        DisableSWD_UsePinsAsGPIO();
    }

    RCC_APB2PeriphClockCmd(PinToPeriph(pin), ENABLE);
    GPIO_InitTypeDef gpio_pin = {0};
    gpio_pin.GPIO_Pin = PinToBitMask(pin);
    switch (mode) {
        case OUTPUT:
            gpio_pin.GPIO_Mode = GPIO_Mode_Out_PP;
            break;
        case INPUT_PULLUP:
            gpio_pin.GPIO_Mode = GPIO_Mode_IPU;
            break;
        case INPUT_PULLDOWN:
             gpio_pin.GPIO_Mode = GPIO_Mode_IPD;
             break;
        case INPUT:
             gpio_pin.GPIO_Mode = GPIO_Mode_IN_FLOATING;
             break;
        case OUTPUT_AF_PP:
             gpio_pin.GPIO_Mode = GPIO_Mode_AF_PP;
             break;
        default:
            gpio_pin.GPIO_Mode = GPIO_Mode_IN_FLOATING;
            break;
    }

    gpio_pin.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(PinToPort(pin), &gpio_pin);
}

void digitalWrite(uint8_t pin, int value)
{
    GPIO_WriteBit(PinToPort(pin), PinToBitMask(pin), value);
}

uint8_t digitalRead(uint8_t pin){
    return GPIO_ReadInputDataBit(PinToPort(pin), PinToBitMask(pin));
}

void initNeopixel(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    GPIO_PinRemapConfig(AFIO_PCFR1_PD01_REMAP, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure = {0};
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
}

void notone(void){
    TIM_Cmd(TIM2, DISABLE); // Disable timer to stop PWM output
    digitalWrite(PIN_PA1, 0);
}

void tone(uint16_t frequency){
    //Buzzer:  PA1, T2_CH2
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    pinMode(PIN_PA1, OUTPUT_AF_PP);

    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    uint32_t test = SystemCoreClock/frequency;

    uint16_t PrescalerValue;
    if (test / 100 > 65000) PrescalerValue = 65000;
    else PrescalerValue = (uint16_t) test/100;

    uint16_t period = (uint16_t)(SystemCoreClock/PrescalerValue/frequency);

    // Time base configuration
    TIM_TimeBaseStructure.TIM_Period = period;
    TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue-1;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV2;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    // PWM1 Mode configuration: Channel 2 (Assuming PA1 is connected to TIM2 CH2)
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = period/2; // 50% duty cycle
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

    TIM_OC2Init(TIM2, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM2, ENABLE);
    TIM_Cmd(TIM2, ENABLE); // Enable timer
}
