#include "debug.h"
#include "lana.h"
#include "data.h"
#include "leds.h"
#include "ch32v20x.h"

#define TRIGGER PIN_PA6

int hitpoints = 4;
int last_hw_team = -1; //to detect team change

uint32_t team_to_color(int team){
    uint32_t team_color = color(
            team & 1?128:0,
            team & 2?128:0,
            team & 4?128:0
                    );
   return team_color;
}

int get_hw_team() {
    if (!digitalRead(PIN_PB6)) return 1;
    if (!digitalRead(PIN_PB7)) return 2;
    if (!digitalRead(PIN_PA0)) return 4;
    else return 0;
}

int hw_team_changed(){
    if (get_hw_team() != last_hw_team) {
        last_hw_team = get_hw_team();
        return last_hw_team;
    }
    return 0;
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

    //trigger button
    pinMode(TRIGGER, INPUT_PULLUP);

    //delay timer and TX ISR
    SYSTICK_Init_Config(26876);

    // IR Receivers
    pinMode(PIN_PA3, INPUT_PULLUP);
    pinMode(PIN_PA5, INPUT_PULLUP);
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
}

void team_change_animation()
{
    tone(G * (1L << 4) / 100);
    delay_ms(35);
    tone(G * (1L << 5) / 100);
    delay_ms(35);
    tone(G * (1L << 6) / 100);
    delay_ms(35);
    notone();
}

void shoot_animation(){ //needs work
    uint32_t color = team_to_color(last_hw_team);

    fill(0);
    set_led(1, color);
    write_leds();
    delay_ms(10);

    fill(0);
    set_led(2, color);
    write_leds();
    delay_ms(50);

    fill(0);
    set_led(3, color);
    write_leds();
    delay_ms(80);

    fill(0);
    set_led(4, color);
    write_leds();

    int team = last_hw_team;
    int mod = team * 2;

    for (int i = 0; i < 4; i++)
    {
        tone(5000);
      for (int f = 5000; f > 1000; f -= 300)
      {
        change_tone(f+team*500);
        delay_ms(10);
      }
      notone();
    }

    /*tone(5000);
    for (int f = 8000 / mod; f > 100 * mod; f -= 50)
    {
      change_tone(f);
      delay_ms(5+mod);
    }

   notone();*/
}

void display_status(){
    fill(color(0,0,0));
    for (int i=0; i<= hitpoints; i++)
        set_led(i, team_to_color(last_hw_team));
    write_leds();
}

void game_loop();

int main(void)
{
    setup();

    startup_animation();

    game_loop();


        /*IrDataPacket p = get_ir_packet();
        if (p.raw > 0) printf("%d\r\n",p.team);
        p.raw = 0;
        p.team=1;
        p.action=1;
        p.channel=0;
        p.player_id = player;
        send_ir_packet(p);*/

}

void game_loop() {
    while (1) {
        if (hw_team_changed()){
            // debounce [C0Dq4jE-Ixw]
            delay_ms(10);
            while (hw_team_changed()) delay_ms(10);
            team_change_animation();
        }

        if (triggered) {
            while (triggered) {
                triggered = 0;
                delay_ms(10);
            };
            //send_packet();
            shoot_animation();
        }
        /*
        else if (triggered() > 3000) ChangeMode(Chatter);
        if (ir_data_ready())
        {
            handle_ir_data(); //validate, animate
        }
        */
        display_status();
    }
}

