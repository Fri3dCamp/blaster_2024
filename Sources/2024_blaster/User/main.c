#include "debug.h"
#include "lana.h"
#include "ch32v20x.h"

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
    }
}


