#include "debug.h"
#include "lana.h"
#include "data.h"
#include "ch32v20x.h"

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
        handle_ir_interrupt(1); //TODO
    }
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

    enable_ir_interupt();


    while(1)
    {
        delay_ms(1000);
        uint32_t p = get_ir_packet();
        if (p > 0) printf("%d\r\n",p);
    }
}


