
#ifndef DATA_H
#define DATA_H

#include "stdlib.h"

const int ir_bit_lenght = 32;
const int ir_start_high_time = 16;
const int ir_start_low_time = 8;
const int ir_zero_high_time = 1;
const int ir_zero_low_time = 1;
const int ir_one_high_time = 1;
const int ir_one_low_time = 3;
const int ir_stop_high_time = 1;
const int ir_stop_low_time = 1;
const int pulse_train_lenght =  2 + ir_bit_lenght * 2 + 2;

/*
 * Ir1_last_pulse_time
 * Ir1_bits_read
 * Ir1_packet
 * */

#define uint32_t unsigned int
#define uint8_t unsigned char
#define uint16_t unsigned short

enum TeamColor
{
  eNoTeam = 0b000,
  eTeamRex = 0b001,
  eTeamGiggle = 0b010,
  eTeamBuzz = 0b100,
  eTeamYellow = eTeamRex | eTeamGiggle,
  eTeamMagenta = eTeamRex | eTeamBuzz,
  eTeamCyan = eTeamGiggle | eTeamBuzz,
  eTeamWhite = eTeamRex | eTeamGiggle | eTeamBuzz
};

enum Action
{
  eActionNone = 0,
  eActionDamage = 1,
  eActionHeal = 2,
};

#define CH_STANDALONE 0
#define CH_MANAGED 1
#define CMD_ACK 0
#define CMD_MODE 1
#define CMD_SHOT 2
#define CMD_TRIGGER 3
#define CMD_TEAM 4
#define CMD_ANIMATION 5

union IrDataPacket
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
};

union LinkDataPacket
{
  uint32_t raw;
  struct
  {
    uint8_t command: 3;
    uint32_t parameter: 21;
    uint8_t crc: 8;
  };
};

union ShotParameter
{
  uint32_t raw;
  struct
  {
    uint8_t team: 3;
    uint8_t action: 2;
    uint16_t id: 12;
    uint8_t hp: 3;  //new HP amount
  };
};

union TriggerParameter
{
  uint32_t raw;
  struct
  {
    uint8_t new_team: 3;
  };
};

union ModeParameter
{
  uint32_t raw;
  struct
  {
    uint8_t mode: 1;
    uint8_t team: 3;
    uint8_t action: 2;
    uint16_t id: 11;
    uint8_t hp: 3;
    uint8_t ready: 1;
  };
};



enum DeviceType
{
  eInfrared = 0b01,
  eBadge = 0b10,
  eAllDevices = 0b11,
};

class DataReader
{
private:
  volatile uint32_t refTime;
  volatile bool oldState;
  volatile uint32_t rawData;
  volatile uint8_t bitsRead;
  volatile bool dataReady;

public:
  DataReader()
     : refTime(0), oldState(true), rawData(0), bitsRead(0), dataReady(false)
   {
   }

  void handlePinChange(bool state);
  void reset();           // clear buffer
  bool isDataReady();     // check buffer, if valid True, if invalid ResetBuffer
  uint32_t getPacket(); // return packet and reset; Dataclass then needs to calculate CRC
};

class _data
{
private:
  _data();
  DataReader ir1_reader;
  DataReader ir2_reader;
  DataReader badge_reader;

  volatile bool transmitting;
  volatile bool transmit_badge;
  volatile bool transmit_ir;
  volatile int pulse_train[pulse_train_lenght];
  volatile uint8_t pulse_pointer;

  void setup_ir_carrier();
  void setup_data_timer();
  void prepare_pulse_train(uint32_t raw_packet);
  void enableReceive(DeviceType device);
  void disableReceive(DeviceType device);

public:
  IrDataPacket readIr();
  LinkDataPacket readBadge();

  static _data &getInstance();
  uint32_t calculateCRC(uint32_t raw_packet);
  void transmit(LinkDataPacket packet);
  void transmit(IrDataPacket packet);
  void transmit_ISR();             // function called by ISR
  void receive_ISR(bool ir1, bool ir2, bool badge); // function called by ISR
  void init();
};


#endif
