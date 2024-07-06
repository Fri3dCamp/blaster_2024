
#ifndef DATA_H
#define DATA_H
#include <Arduino.h>

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
#define IR_IN1_PIN 5 // PD PCINT21
#define IR_IN2_PIN 6 // PD PCINT22
#define BADGELINK_PIN 7

enum TeamColor : uint8_t
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

enum Action : uint8_t
{
  eActionNone = 0,
  eActionDamage = 1,
  eActionHeal = 2,
};

enum CommandType : uint8_t
{
  eCommandShoot = 1,
  eCommandHeal = 2,
  eCommandSetChannel = 3,
  eCommandSetTriggerAction = 4,
  eCommandSetGameMode = 5,
  eCommandSetHitTimeout = 6,
  eCommandPlayAnimation = 7,
  eCommandTeamChange = 8,
  eCommandChatter = 9,
  eCommandPullTrigger = 10,
  eCommandSetSettings = 11,
  //eCommandSetFlagsB = 12,
  //eCommandReservedA = 13,
  //eCommandReservedB = 14,
  eCommandBlasterAck = 15,
};

union IrDataPacket
{
  uint32_t raw;
  struct
  {
    uint8_t channel: 2;
    uint8_t team: 3;
    uint8_t action: 3;
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
    uint32_t todo: 24;
    uint8_t crc: 8;
  };
};

enum DeviceType: uint8_t
{
  eInfrared = 0b01,
  eBadge = 0b10,
  eAllDevices = 0b11,
};

class DataReader
{
private:
  volatile uint32_t refTime;
  volatile bool oldState = 1;
  volatile uint32_t rawData;
  volatile uint8_t bitsRead;
  volatile bool dataReady;

public:
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
  volatile int8_t pulse_pointer;

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

extern _data &Data;
#endif
