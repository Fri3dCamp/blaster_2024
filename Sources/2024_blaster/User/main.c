#include "debug.h"
#include "lana.h"
#include "data.h"
#include "leds.h"
#include "ch32v20x.h"
#include "stdlib.h"

#define TRIGGER PIN_PA6

#define DEFAULTHP 4
int hitpoints;

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

void crash_animation(uint8_t team, uint32_t hit_timeout)
{
  fill(team_to_color(team));
  write_leds();

  for (int i = 1000; i > 0; i -= 20)
  {
    int min = 500 - i / 3;
    int max = 8000 - i * 9;
    tone(min+rand()%(max-min));
    delay_ms(4);
  }
  for (int i = 0; i < 1000; i+=2)
  {
      int min = 500 - i / 3;
      int max = 8000 - i * 9;
      tone(min+rand()%(max-min));
      delay_ms(1);
  }
  notone();
  fill(team_to_color(team));
  write_leds();
  for (int i = 4; i > 0; i--){
      set_led(i, team_to_color(last_hw_team));
      delay_ms(hit_timeout/4);
      write_leds();
  }

}

void display_status(){
    fill(color(0,0,0));
    for (int i=0; i<= hitpoints; i++)
        set_led(i, team_to_color(last_hw_team));
    write_leds();
}

void game_loop();
void no_blaster_loop();

int main(void)
{
    setup();

    no_blaster_loop();

    startup_animation();

    game_loop();
}

/* run this loop as long a the blaster is not detected.
 * detection works by means of the team selector. If no team is selected then
 * there is no blaster (or a broken switch)
 * while in this mode, blink the onboard led GREEN at 2Hz
 */
void no_blaster_loop(){
    while (!get_hw_team()) {
        fill(color(0,255,0));
        write_leds();
        delay_ms(250);
        fill(color(0,0,0));
        write_leds();
        delay_ms(250);
    }

}

void game_loop() {
    hitpoints = DEFAULTHP;
    while (1) {
        if (hw_team_changed()){
            delay_ms(10);
            while (hw_team_changed()) delay_ms(10);
            team_change_animation();
        }

        if (triggered) {
            while (triggered) {
                triggered = 0;
                delay_ms(10);
            };
            uint32_t p = 0;
            p = set_team(p, last_hw_team);
            p = set_action(p, 1);
            p = set_channel(p, 0);
            p = set_player_id(p, 0);
            send_ir_packet(p);
            shoot_animation();
        }
        if (ir_data_ready())
        {
            uint32_t p = get_ir_packet();
            if (p != 0 &&
                get_team(p) != last_hw_team &&
                get_action(p) == 1){
                if (hitpoints) {
                    hitpoints--;
                    if (hitpoints) crash_animation(get_team(p), 2000);
                    else crash_animation(get_team(p), 20);
                }
                get_ir_packet();
            }

        }
        if (hitpoints == 0) {
            for (int k=1; k< 5; k++){
                for (int j =0; j< 50; j++)
                {
                    set_led(1+rand()%4, team_to_color(rand()%8));
                    for (int kk=1; kk<k; kk++) set_led(kk, team_to_color(last_hw_team));
                    write_leds();
                    delay_ms(50);
                }
            }
            hitpoints = DEFAULTHP;
            get_ir_packet();
            team_change_animation();
            triggered = 0;
        }
        display_status();
    }
}

