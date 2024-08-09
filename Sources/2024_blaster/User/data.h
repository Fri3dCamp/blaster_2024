
#ifndef DATA_H
#define DATA_H

#include <stdint.h>

typedef struct {
    uint32_t raw;
    uint8_t bits_read;
    uint8_t data_ready;
    uint32_t last_interrupt;
} DataReader;

typedef union
{
  uint32_t raw;
  struct
  {
    uint8_t channel: 1;
    uint8_t team: 3;
    uint8_t action: 2;
    uint8_t action_param: 4;
    uint16_t player_id: 12;
    uint8_t crc: 8;
  };
} IrDataPacket;

void enable_ir_interupt();
void handle_ir_interrupt(int channel);
int ir_data_ready();
IrDataPacket get_ir_packet();
void send_ir_packet(IrDataPacket p);
void enable_rx();
void disable_rx();
void transmit_ISR();

#endif
