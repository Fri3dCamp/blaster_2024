#include "data.h"
#include "lana.h"

const int ir_bit_lenght = 32;
const int ir_start_high_time = 16;
const int ir_start_low_time = 8;
const int ir_zero_high_time = 1;
const int ir_zero_low_time = 1;
const int ir_one_high_time = 1;
const int ir_one_low_time = 3;
const int ir_stop_high_time = 1;
const int ir_stop_low_time = 1;
#define pulse_train_lenght 68 // 2 + ir_bit_lenght * 2 + 2;

volatile DataReader ir1_reader;
volatile DataReader ir2_reader;
volatile int rx_enabled = 0;

volatile int pulse_train[pulse_train_lenght];
volatile int8_t pulse_pointer;
volatile int transmit_ir = 0;

uint8_t get_channel(uint32_t raw)      { return (raw & 0b00000000000000000000000000000001) >> 0; }
uint8_t get_team(uint32_t raw)         { return (raw & 0b00000000000000000000000000001110) >> 1; }
uint8_t get_action(uint32_t raw)       { return (raw & 0b00000000000000000000000000110000) >> 4; }
uint8_t get_action_param(uint32_t raw) { return (raw & 0b00000000000000000000001111000000) >> 6; }
uint16_t get_player_id(uint32_t raw)   { return (raw & 0b00000000001111111111110000000000) >> 10; }
uint8_t get_unused(uint32_t raw)       { return (raw & 0b00000000110000000000000000000000) >> 22; }
uint8_t get_crc(uint32_t raw)          { return (raw & 0b11111111000000000000000000000000) >> 24; }

uint32_t set_channel(uint32_t raw, uint8_t channel)           { raw &= ~(0b1 << 0); raw |= (channel & 0b1) << 0; return raw;}
uint32_t set_team(uint32_t raw, uint8_t team)                 { raw &= ~(0b111 << 1); raw |= (team & 0b111) << 1; return raw;}
uint32_t set_action(uint32_t raw, uint8_t action)             { raw &= ~(0b11 << 4); raw |= (action & 0b11) << 4; return raw;}
uint32_t set_action_param(uint32_t raw, uint8_t action_param) { raw &= ~(0b1111 << 6); raw |= (action_param & 0b1111) << 6; return raw;}
uint32_t set_player_id(uint32_t raw, uint16_t player_id)      { raw &= ~((uint32_t)0b111111111111 << 10); raw |= (player_id & 0b111111111111) << 10; return raw;}
uint32_t set_unused(uint32_t raw, uint8_t unused)             { raw &= ~((uint32_t)0b11 << 22); raw |= (unused & (uint32_t)0b11) << 22; return raw;}
uint32_t set_crc(uint32_t raw, uint8_t crc)                   { raw &= ~((uint32_t)0b11111111 << 24); raw |= (crc & (uint32_t)0b11111111) << 24; return raw;}



void handle_pulse(volatile DataReader *dr, uint32_t time){
    if (dr->bits_read >= 32) {
       return;
   }
   uint32_t delta = time - dr->last_interrupt;
   if (delta > 896 && delta < 1400) {
       dr->raw = dr->raw >> 1;
       dr->raw &= 0x7FFFFFFF;
       dr->bits_read++;
   }
   else if (delta > 1792 && delta < 2800) {
       dr->raw = dr->raw >> 1;
       dr->raw |= 0x80000000;
       dr->bits_read++;
   }
   else{
       dr->raw = 0;
       dr->bits_read = 0;
   }
   dr->last_interrupt = time;
}

void handle_ir_interrupt(int channel)
{
    uint32_t time = micros();
    if (channel == 0){
       handle_pulse(&ir1_reader, time);
    }
    if (channel == 1){
        handle_pulse(&ir2_reader, time);
    }
}

uint32_t calculateCRC(uint32_t raw_packet){
  uint32_t raw = raw_packet;
  uint32_t checksum =  ((raw <<  2) & 0b10000000111111110111111100) ^
            ((raw <<  1) & 0b01111111100000000111111110) ^
            ((raw <<  0) & 0b00000000111111111111111111) ^
            ((raw >>  1) & 0b00000000100000000000000000) ^
            ((raw >>  2) & 0b00000000011111110000000000) ^
            ((raw >>  3) & 0b00000000111111111000000000) ^
            ((raw >>  4) & 0b00000011100000001111111100) ^
            ((raw >>  5) & 0b00000000111111111000000010);
  checksum = checksum ^ (checksum >> 8) ^ (checksum >> 16) ^ (checksum >> 24);
  checksum = checksum & 0xFF;
  raw ^= checksum << 24;
  return raw;
}

int ir_data_ready(){
   return (ir1_reader.bits_read == 32 || ir2_reader.bits_read == 32);
}

uint32_t get_ir_packet(){
    uint32_t p = 0;
    if (ir1_reader.bits_read == 32) {
        p = ir1_reader.raw;
    }
    else if (ir2_reader.bits_read == 32) {
        p = ir2_reader.raw;
    }
    if (p > 0) {
        ir1_reader.bits_read = 0;
        ir2_reader.bits_read = 0;
        p = calculateCRC(p);
        if (get_crc(p) != 0) {
            p = 0;
            printf("CRC ERROR\r\n");
        }
    }
    return p;
}

void enable_rx() {
    ir1_reader.bits_read=0;
    ir2_reader.bits_read=0;
    rx_enabled = 1;
}

void disable_rx() {
    rx_enabled = 0;
}

int bitRead(uint32_t p, int index){
    uint32_t mask = (uint32_t)1 << index;

    // Extract the bit value (0 or 1)
    return (p & mask) ? 1 : 0;
}

void prepare_pulse_train(uint32_t raw_packet)
{
  int index = 0;
  pulse_train[index++] = ir_start_high_time;
  pulse_train[index++] = ir_start_low_time;
  for (int i = 0; i < ir_bit_lenght; i++)
  {
    if (bitRead(raw_packet, i))
    {
      pulse_train[index++] = ir_one_high_time;
      pulse_train[index++] = ir_one_low_time;
    }
    else
    {
      pulse_train[index++] = ir_zero_high_time;
      pulse_train[index++] = ir_zero_low_time;
    }
  }
  pulse_train[index++] = ir_stop_high_time;
  pulse_train[index++] = ir_stop_low_time;

  pulse_pointer = 0;
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



void send_ir_packet(uint32_t p)
{
    disable_rx();
    p = set_crc(p, 0);
    p = calculateCRC(p);

    printf("raw: %u, ", p);
    printf("CH: %u, ", get_channel(p));
    printf("T: %u, ", get_team(p));
    printf("A: %u, ", get_action(p));
    printf("AP: %u, ", get_action_param(p));
    printf("PID: %u, ", get_player_id(p));
    printf("_: %u, ", get_unused(p));
    printf("CRC: %u, ", get_crc(p));
    printf("\r\n");

    prepare_pulse_train(p);

    enable_ir_carrier();
    ir_off();
    transmit_ir = 1;
    while (transmit_ir);

    enable_rx();
}



void transmit_ISR()
{
  if (transmit_ir)
  {
    if (pulse_pointer % 2 == 1) // would & 0b1 be faster?
    {
      if (transmit_ir) ir_off();
    }
    else
    {
      if (transmit_ir) ir_on();

    }
    pulse_train[pulse_pointer]--; // count down

    // if we reached the end go to the next pulse
    if (pulse_train[pulse_pointer] <= 0)
      pulse_pointer++;

    // unless we already were on the last pulse
    if (pulse_pointer >= pulse_train_lenght)
    {
      transmit_ir = 0;
    }
  }
}



void enable_ir_interupt(){
    // Enable GPIOA clock
    RCC_AHBPeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);



    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource3);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource5);

    // Configure EXTI Line3
    EXTI_InitTypeDef EXTI_InitStructure;
    EXTI_InitStructure.EXTI_Line = EXTI_Line3;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; // Trigger on button press (falling edge)
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // Configure EXTI Line5
    EXTI_InitStructure.EXTI_Line = EXTI_Line5;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; // Trigger on button press (falling edge)
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void EXTI3_IRQHandler( void ) __attribute__((interrupt));

void EXTI3_IRQHandler(void) {
   // Check if the interrupt was from PA3
   if (EXTI_GetITStatus(EXTI_Line3) != RESET) {
       // Clear the interrupt flag
       EXTI_ClearITPendingBit(EXTI_Line3);
       handle_ir_interrupt(0);
   }
}

void EXTI9_5_IRQHandler( void ) __attribute__((interrupt));

void EXTI9_5_IRQHandler(void) {
    // Check if the interrupt was from PA5
    if (EXTI_GetITStatus(EXTI_Line5) != RESET) {
        // Clear the interrupt flag
        EXTI_ClearITPendingBit(EXTI_Line5);
        handle_ir_interrupt(1);
    }
}







