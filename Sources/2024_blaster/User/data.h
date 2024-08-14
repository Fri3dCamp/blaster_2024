
#ifndef DATA_H
#define DATA_H

#include <stdint.h>

typedef struct {
    uint32_t raw;
    uint8_t bits_read;
    uint32_t last_interrupt;
} DataReader;


uint8_t get_channel(uint32_t raw);
uint8_t get_team(uint32_t raw);
uint8_t get_action(uint32_t raw);
uint8_t get_action_param(uint32_t raw);
uint16_t get_player_id(uint32_t raw);
uint8_t get_unused(uint32_t raw);
uint8_t get_crc(uint32_t raw);

uint32_t set_channel(uint32_t raw, uint8_t channel);
uint32_t set_team(uint32_t raw, uint8_t team);
uint32_t set_action(uint32_t raw, uint8_t action);
uint32_t set_action_param(uint32_t raw, uint8_t action_param);
uint32_t set_player_id(uint32_t raw, uint16_t player_id);
uint32_t set_unused(uint32_t raw, uint8_t unused);
uint32_t set_crc(uint32_t raw, uint8_t crc);


void enable_ir_interupt();
void handle_ir_interrupt(int channel);
int ir_data_ready();
uint32_t get_ir_packet();
void send_ir_packet(uint32_t p);
void enable_rx();
void disable_rx();
void transmit_ISR();
uint32_t calculateCRC(uint32_t raw_packet);

#endif
