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
const int pulse_train_lenght = 68; // 2 + ir_bit_lenght * 2 + 2;

DataReader ir1_reader;
DataReader ir2_reader;

void handle_pulse(DataReader *dr, uint32_t time){
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

IrDataPacket get_ir_packet()
{
    IrDataPacket p;
    p.raw = 0;
    if (ir1_reader.bits_read == 32) {
        p.raw = ir1_reader.raw;
    }
    else if (ir2_reader.bits_read == 32) {
        p.raw = ir2_reader.raw;
    }
    if (p.raw > 0) {
        ir1_reader.bits_read = 0;
        ir2_reader.bits_read = 0;
        p.raw = calculateCRC(p.raw);
        if (p.crc != 0) p.raw = 0;
    }
    return p;
}

void enable_ir_interupt(){
    // Enable GPIOA clock
    RCC_AHBPeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    pinMode(PIN_PA3, INPUT_PULLUP);
    pinMode(PIN_PA5, INPUT_PULLUP);

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







