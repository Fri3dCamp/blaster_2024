#include "data.h"
#include "lana.h"

const int ir_bit_lenght = 32;
const int ir_start_high_time = 16;
const int ir_start_low_time = 8;
const int ir_zero_high_time = 1;
const int ir_zero_low_time = 1;
const int ir_one_high_time = 1;
const int ir_one_low_time = 3;
const int ir_stop_high_time = 1;
const int ir_stop_low_time = 1;
const int pulse_train_lenght = 68; // 2 + ir_bit_lenght * 2 + 2;

typedef struct {
    uint32_t raw;
    uint8_t bits_read;
    uint32_t last_interrupt;
} DataReader;

DataReader ir1_reader;
DataReader ir2_reader;

void handle_ir_interrupt(int channel)
{
    uint32_t time = micros();
    if (channel == 0){
        //nothing to do when all data was read
        if (ir1_reader.bits_read >= 32) {
            printf("packet %d\r\n", ir1_reader.raw);
            ir1_reader.bits_read = 0;
            return;
        }
        uint32_t delta = time - ir1_reader.last_interrupt;
        if (delta > 896 && delta < 1400) {
            ir1_reader.raw = ir1_reader.raw >> 1;
            ir1_reader.raw &= 0x7FFFFFFF;
            ir1_reader.bits_read++;
        }
        else if (delta > 1792 && delta < 2800) {
          ir1_reader.raw = ir1_reader.raw >> 1;
          ir1_reader.raw |= 0x80000000;
          ir1_reader.bits_read++;
        }
        else{
            ir1_reader.raw = 0;
            ir1_reader.bits_read = 0;
        }
        ir1_reader.last_interrupt = time;
    }
}

uint32_t get_ir_packet()
{
    if (ir1_reader.bits_read == 32) {
        uint32_t packet = ir1_reader.raw;
        ir1_reader.bits_read = 0;
        ir2_reader.bits_read = 0;
        return packet;
    }
    if (ir2_reader.bits_read == 32) {
        uint32_t packet = ir2_reader.raw;
        ir1_reader.bits_read = 0;
        ir2_reader.bits_read = 0;
        return packet;
    }
    return 0;
}













