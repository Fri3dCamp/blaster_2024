
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

#define CH_STANDALONE 0
#define CH_MANAGED 1
#define CMD_ACK 0
#define CMD_MODE 1
#define CMD_SHOT 2
#define CMD_TRIGGER 3
#define CMD_TEAM 4
#define CMD_ANIMATION 5

/**
 * @brief IR data packet
 * holds the 32 ir bits in a private uint32_t
 * structure: return type - field name - nr of bits
 *  uint8_t channel: 1;
 *  uint8_t team: 3;
 *  uint8_t action: 2;
 *  uint8_t action_param: 4;
 *  uint16_t player_id: 12;
 *  uint8_t crc: 8;
 *  uint8_t unused: 2;
 */
class IrDataPacket
{
private:
  uint32_t raw;
public:
  IrDataPacket();
  IrDataPacket(uint32_t raw);
  uint32_t get_raw();
  void set_raw(uint32_t raw);
  uint8_t get_channel();
  void set_channel(uint8_t channel);
  uint8_t get_team();
  void set_team(uint8_t team);
  uint8_t get_action();
  void set_action(uint8_t action);
  uint8_t get_action_param();
  void set_action_param(uint8_t action_param);
  uint16_t get_player_id();
  void set_player_id(uint16_t player_id);
  uint8_t get_crc();
  void set_crc(uint8_t crc);
  uint8_t get_unused();
  void set_unused(uint8_t unused);
};

/**
 * @brief Link Data Packet on badge blaster link communication
 * holds the 32 ir bits in a private uint32_t
 * structure: return type - field name - nr of bits
 * uint8_t command: 3;
 * uint32_t parameter: 21;
 * uint8_t crc: 8;
 */
class LinkDataPacket
{
private:
  uint32_t raw;
public:
  LinkDataPacket();
  LinkDataPacket(uint32_t raw);
  uint32_t get_raw();
  void set_raw(uint32_t raw);
  uint8_t get_command();
  void set_command(uint8_t command);
  uint32_t get_parameter();
  void set_parameter(uint32_t parameter);
  uint8_t get_crc();
  void set_crc(uint8_t crc);
};

/**
 * @brief Mode Parameter: 21 bits of LinkDataPacket parameter part
 * holds the 21 bits in a private uint32_t
 * structure: return type - field name - nr of bits
 * uint8_t mode: 1;
 * uint8_t team: 3;
 * uint8_t action: 2;
 * uint16_t id: 11;
 * uint8_t hp: 3;
 * uint8_t ready: 1;
 */
class ModeParameter
{
private:
  uint32_t raw;
public:
  ModeParameter();
  ModeParameter(uint32_t raw);
  uint32_t get_raw();
  void set_raw(uint32_t raw);
  uint8_t get_mode();
  void set_mode(uint8_t mode);
  uint8_t get_team();
  void set_team(uint8_t team);
  uint8_t get_action();
  void set_action(uint8_t action);
  uint16_t get_id();
  void set_id(uint16_t id);
  uint8_t get_hp();
  void set_hp(uint8_t hp);
  uint8_t get_ready();
  void set_ready(uint8_t ready);
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
