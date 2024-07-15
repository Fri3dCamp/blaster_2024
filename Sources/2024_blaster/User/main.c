#include "debug.h"
#include "lana.h"
#include "ch32v20x.h"


volatile uint32_t ir_ticks = 0;

uint32_t micros()
{
    return (ir_ticks * 560) + (SysTick->CNT / 48);
}

uint32_t millis()
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

    /*if (x%2==0)
    {
        pinMode(PIN_PB1, OUTPUT_AF_PP);
        TIM_Cmd(TIM3, ENABLE);
    }
    else {
        TIM_Cmd(TIM3, DISABLE);
        pinMode(PIN_PB1, OUTPUT);
        digitalWrite(PIN_PB1, LOW);
    }*/

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
    USART_Printf_Init(115200);
    printf("SystemClk:%d\r\n", SystemCoreClock);
    printf( "ChipID:%08x\r\n", DBGMCU_GetCHIPID() );

    //enable_ir_carrier();


    pinMode(PIN_PB1, OUTPUT);
    SYSTICK_Init_Config(26876);
    //__disable_irq();
    //__enable_irq();

    while(1)
    {
        delay_ms(1000);
        printf("%d\r\n",millis());

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


