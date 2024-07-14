/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2021/06/06
 * Description        : Main program body.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

/*
 *@Note
 *USART Print debugging routine:
 *USART1_Tx(PA9).
 *This example demonstrates using USART1(PA9) as a print debug port output.
 *
 */

#include "debug.h"
#include "lana.h"
#include "ch32v20x.h"


volatile int32_t x = 0;

// Timer 1 Interrupt Service Routine
void TIM1_UP_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM1_UP_IRQHandler(void) {
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
        // Your code to handle the interrupt
        x++;
    }
}

//1786 hz for .560 ms speed.
//this timer will is used for IT TX
void Timer1_Init(void) {
    // Enable the clock for TIM1
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // Timer base configuration
    TIM_TimeBaseStructure.TIM_Period = 7999;            // 866
    TIM_TimeBaseStructure.TIM_Prescaler = 0;         // 0
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    // Enable Timer update interrupt
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);

    // Enable TIM1 interrupt in NVIC
    NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // Start the timer
    TIM_Cmd(TIM1, ENABLE);
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
    // move the compare further ahead in time.
    // as a warning, if more than this length of time
    // passes before triggering, you may miss your
    // interrupt.
    //SysTick->CMP += (48000000/1);

    // clear IRQ
    SysTick->SR = 0;

    // update counter
    x++;
}

void enable_ir_carrier()
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

    TIM_Cmd(TIM3, ENABLE); // Enable timer
    //Delay_Ms(100); // Implement a Delay function or use another timer to wait for 1 second
    //TIM_Cmd(TIM3, DISABLE); // Disable timer to stop PWM output

    //GPIO_WriteBit(GPIOB, GPIO_Pin_1, Bit_RESET);
}


int main(void)
{
    SystemInit();
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    Delay_Init();
    USART_Printf_Init(115200);
    printf("SystemClk:%d\r\n", SystemCoreClock);
    printf( "ChipID:%08x\r\n", DBGMCU_GetCHIPID() );
    printf("This is printf example\r\n");

    //enable_ir_carrier();

    //Delay_Ms(1000);

    pinMode(PIN_PB1, OUTPUT);
    SYSTICK_Init_Config(SystemCoreClock/20000);
    //__disable_irq();
    //Timer1_Init();
    //__enable_irq();

    x = 0;
    while(1)
    {

        if (x%2==0) digitalWrite(PIN_PB1, LOW);
        else digitalWrite(PIN_PB1, HIGH);
        //digitalWrite(PIN_PB1, LOW);
        //digitalWrite(PIN_PB1, HIGH);

        /*
        //Delay_Ms(1000);
        while(x < 1786);
        pinMode(PIN_PB1, OUTPUT_AF_PP);
        TIM_Cmd(TIM3, ENABLE);
        x=0;
        //printf("IR ON\r\n");

        //Delay_Ms(1000);
        while(x < 1786);
        TIM_Cmd(TIM3, DISABLE);
        pinMode(PIN_PB1, OUTPUT);
        digitalWrite(PIN_PB1, LOW);
        x=0;
        //printf("IR OFF\r\n");
       /* Delay_Ms(1000);
        int d = x;
        printf( "x:%d\r\n", d );
*/
    }
}


