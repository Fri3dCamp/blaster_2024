#include "debug.h"
#include "lana.h"
#include "data.h"
#include "leds.h"
#include "ch32v20x.h"



int hitpoints = 4;
int fixed_color = 0;

int main(void)
{
    SystemInit();
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    USART_Printf_Init(115200);
    printf("SystemClk:%d\r\n", SystemCoreClock);
    printf( "ChipID:%08x\r\n", DBGMCU_GetCHIPID() );

    enable_ir_carrier();


    pinMode(PIN_PB1, OUTPUT);
    SYSTICK_Init_Config(26876);

    enable_ir_interupt();

    init_leds();
    fill(color(50,50,0));
    set_led(1,color(0,50,0));
    set_led(2,color(0,0,50));
    write_leds();


    while(1)
    {
        delay_ms(1000);
        uint32_t p = get_ir_packet();
        if (p > 0) printf("%d\r\n",p);

    }
}


