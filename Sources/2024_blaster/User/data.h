
#ifndef DATA_H
#define DATA_H

#include <stdint.h>

void enable_ir_interupt();
void handle_ir_interrupt(int channel);
uint32_t get_ir_packet();


#endif
