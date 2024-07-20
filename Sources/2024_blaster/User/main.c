#include "debug.h"
#include "lana.h"
#include "data.h"
#include "leds.h"
#include "ch32v20x.h"


int hitpoints = 4;
int fixed_color = 0;
int hardware_team = 0;

int get_hw_team() {
    if (!digitalRead(PIN_PB6)) return 1;
    if (!digitalRead(PIN_PB7)) return 2;
    if (!digitalRead(PIN_PA0)) return 4;
    else return 0;
}

void setup()
{
    SystemInit();
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    USART_Printf_Init(115200);
    printf("SystemClk:%d\r\n", SystemCoreClock);
    printf( "ChipID:%08x\r\n", DBGMCU_GetCHIPID());

    //team selector
    pinMode(PIN_PB6, INPUT_PULLUP);
    pinMode(PIN_PB7, INPUT_PULLUP);
    pinMode(PIN_PA0, INPUT_PULLUP);

    //Ir out
    pinMode(PIN_PB1, OUTPUT);
    digitalWrite(PIN_PB1, LOW);

    //delay timer and TX ISR
    SYSTICK_Init_Config(26876);

    enable_ir_interupt();
    enable_rx();

    init_leds();
}

void startup_animation()
{
    #define animation_delay 200
    fill(color(0,0,0));
    set_led(1, color(255,0,0));
    write_leds();
    delay_ms(animation_delay);
    set_led(1, color(0,255,0));
    set_led(2, color(255,0,0));
    write_leds();
    delay_ms(animation_delay);
    set_led(1, color(0,0,255));
    set_led(2, color(0,255,0));
    set_led(3, color(255,0,0));
    write_leds();
    delay_ms(animation_delay);
    set_led(1, color(128,128,0));
    set_led(2, color(0,0,255));
    set_led(3, color(0,255,0));
    set_led(4, color(255,0,0));
    write_leds();
    delay_ms(animation_delay);
    fill(color(80,80,80));
    write_leds();
    tone(1025);
    delay_ms(70);
    tone(2090);
    delay_ms(500);
    notone();
    delay_ms(500);
    fill(color(0,0,0));
    write_leds();

    set_led(1,color(0,50,0));
    set_led(2,color(0,0,50));
    write_leds();
}

int main(void)
{
    setup();

    startup_animation();

    int player = 0;

    while(1){
        printf("%d\r\n",get_hw_team());
        switch (get_hw_team()) {
            case 1:
                fill(color(50,0,0));
                break;
            case 2:
                fill(color(0,50,0));
                break;
            case 4:
                fill(color(0,0,50));
                break;
        }

        write_leds();
        if (get_hw_team() != hardware_team)
        {
            //tone(400);
            delay_ms(200);
            notone();
            hardware_team = get_hw_team();
        }
        delay_ms(100);
    }

    while(1)
    {
        player++;
        delay_ms(1000);
        IrDataPacket p = get_ir_packet();
        if (p.raw > 0) printf("%d\r\n",p.team);
        p.raw = 0;
        p.team=1;
        p.action=1;
        p.channel=0;
        p.player_id = player;
        send_ir_packet(p);



    }
}


