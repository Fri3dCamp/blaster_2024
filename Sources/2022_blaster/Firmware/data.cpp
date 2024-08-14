#include "data.h"
#include <Arduino.h>

IrDataPacket::IrDataPacket(){};
IrDataPacket::IrDataPacket(uint32_t raw){this->raw=raw;};

uint32_t IrDataPacket::get_raw()         { return this->raw; }
uint8_t IrDataPacket::get_channel()      { return (this->raw & 0b00000000000000000000000000000001) >> 0; }
uint8_t IrDataPacket::get_team()         { return (this->raw & 0b00000000000000000000000000001110) >> 1; }
uint8_t IrDataPacket::get_action()       { return (this->raw & 0b00000000000000000000000000110000) >> 4; }
uint8_t IrDataPacket::get_action_param() { return (this->raw & 0b00000000000000000000001111000000) >> 6; }
uint16_t IrDataPacket::get_player_id()   { return (this->raw & 0b00000000001111111111110000000000) >> 10; }
uint8_t IrDataPacket::get_unused()       { return (this->raw & 0b00000000110000000000000000000000) >> 22; }
uint8_t IrDataPacket::get_crc()          { return (this->raw & 0b11111111000000000000000000000000) >> 24; }

void IrDataPacket::set_raw(uint32_t raw)                  { this->raw = raw; }
void IrDataPacket::set_channel(uint8_t channel)           { this->raw &= ~(0b1 << 0); this->raw |= (channel & 0b1) << 0;}
void IrDataPacket::set_team(uint8_t team)                 { this->raw &= ~(0b111 << 1); this->raw |= (team & 0b111) << 1;}
void IrDataPacket::set_action(uint8_t action)             { this->raw &= ~(0b11 << 4); this->raw |= (action & 0b11) << 4;}
void IrDataPacket::set_action_param(uint8_t action_param) { this->raw &= ~(0b1111 << 6); this->raw |= (action_param & 0b1111) << 6;}
void IrDataPacket::set_player_id(uint16_t player_id)      { this->raw &= ~((uint32_t)0b111111111111 << 10); this->raw |= (player_id & 0b111111111111) << 10;}
void IrDataPacket::set_unused(uint8_t unused)             { this->raw &= ~((uint32_t)0b11 << 22); this->raw |= (unused & (uint32_t)0b11) << 22;}
void IrDataPacket::set_crc(uint8_t crc)                   { this->raw &= ~((uint32_t)0b11111111 << 24); this->raw |= (crc & (uint32_t)0b11111111) << 24;}

LinkDataPacket::LinkDataPacket(){};
LinkDataPacket::LinkDataPacket(uint32_t raw){this->raw=raw;};

uint32_t LinkDataPacket::get_raw()         { return this->raw; }
uint8_t LinkDataPacket::get_command()      { return (this->raw & 0b00000000000000000000000000000111) >> 0; }
uint32_t LinkDataPacket::get_parameter()   { return (this->raw & 0b00000000111111111111111111111000) >> 3; }
uint8_t LinkDataPacket::get_crc()          { return (this->raw & 0b11111111000000000000000000000000) >> 24; }

void LinkDataPacket::set_raw(uint32_t raw)             { this->raw = raw; }
void LinkDataPacket::set_command(uint8_t command)      { this->raw &= ~(0b111 << 0); this->raw |= (command & 0b111) << 0;}
void LinkDataPacket::set_parameter(uint32_t parameter) { this->raw &= ~(0b111111111111111111111 << 3); this->raw |= (parameter & 0b111111111111111111111) << 3;}
void LinkDataPacket::set_crc(uint8_t crc)              { this->raw &= ~((uint32_t)0b11111111 << 24); this->raw |= (crc & (uint32_t)0b11111111) << 24;}


ModeParameter::ModeParameter(){};
ModeParameter::ModeParameter(uint32_t raw){this->raw=raw;};

uint32_t ModeParameter::get_raw()   { return (this->raw & 0b00000000000111111111111111111111) >> 0; }
uint8_t ModeParameter::get_mode()   { return (this->raw & 0b00000000000000000000000000000001) >> 0; }
uint8_t ModeParameter::get_team()   { return (this->raw & 0b00000000000000000000000000001110) >> 1; }
uint8_t ModeParameter::get_action() { return (this->raw & 0b00000000000000000000000000110000) >> 4; }
uint16_t ModeParameter::get_id()    { return (this->raw & 0b00000000000000011111111111000000) >> 6; }
uint8_t ModeParameter::get_hp()     { return (this->raw & 0b00000000000011100000000000000000) >> 17; }
uint8_t ModeParameter::get_ready()  { return (this->raw & 0b00000000000100000000000000000000) >> 20; }

void ModeParameter::set_raw(uint32_t raw)      { this->raw &= ~(0b111111111111111111111 << 0); this->raw |= (raw & 0b111111111111111111111) << 0;}
void ModeParameter::set_mode(uint8_t mode)     { this->raw &= ~(0b1 << 0); this->raw |= (mode & 0b1) << 0;}
void ModeParameter::set_team(uint8_t team)     { this->raw &= ~(0b111 << 1); this->raw |= (team & 0b111) << 1;}
void ModeParameter::set_action(uint8_t action) { this->raw &= ~(0b11 << 4); this->raw |= (action & 0b11) << 4;}
void ModeParameter::set_id(uint16_t id)        { this->raw &= ~((uint32_t)0b11111111111 << 6); this->raw |= (id & (uint32_t)0b11111111111) << 6;}
void ModeParameter::set_hp(uint8_t hp)         { this->raw &= ~((uint32_t)0b111 << 17); this->raw |= (hp & (uint32_t)0b111) << 17;}
void ModeParameter::set_ready(uint8_t ready)   { this->raw &= ~((uint32_t)0b1 << 20); this->raw |= (ready & (uint32_t)0b1) << 20;}


/* #region DataReader */
void DataReader::handlePinChange(bool state)
{
  if (dataReady) return; // don't read more data until the "buffer" is empty
  if (state == oldState) return;  // if the state didn't change then don't do anything. this happens if an other pin caused the interrupt
  oldState = state; // update the oldState value so we can detect the next pin change.
  if (state) return;   // we are looking for a rising edge, but the signal is inverted so a falling edge is what we want.
  uint32_t time = micros(); // check time passed since boot up.
  uint32_t delta_time = time - refTime;
  refTime = time;

  /* if delta_time == 4500 set Ack state to true 
     ack state resets after a send
  */

  /* Check total pulse length (rising to rising edge) allow for some deviation*/
  if (delta_time > (uint32_t)(13500 * 0.8) && delta_time < (uint32_t)(13500 / 0.8))
  {
    bitsRead = 1;
    rawData = 0;
    return;
  }
  if (bitsRead == 0) return;
  
  if (delta_time > (uint32_t)(2250 * 0.8) && delta_time < (uint32_t)(2250 / 0.8))
  {
    rawData = rawData >> 1; // make room for an extra bit
    rawData |= 0x80000000;      // set left bit high
    if (++bitsRead == (ir_bit_lenght+1))
    {
      dataReady = 1;
    }
  }
  else if (delta_time > (uint32_t)(1120 * 0.8) && delta_time < (uint32_t)(1120 / 0.8))
  {
    rawData = rawData >> 1; // make room for an extra bit
    if (++bitsRead == (ir_bit_lenght+1))
    {
      dataReady = 1;
    }
  }  
}
void DataReader::reset()
{
  rawData = 0;
  bitsRead = 0;
  dataReady = 0;
}
bool DataReader::isDataReady()
{
  return dataReady;
}

uint32_t DataReader::getPacket()
{
  uint32_t p;
  p = rawData;
  reset();
  return p;
}
/* #endregion */

/* #region Data */

// Private
_data::_data()
{
  ir1_reader.reset();
  ir2_reader.reset();
  badge_reader.reset();

  enableReceive(eAllDevices);
}

void _data::setup_ir_carrier()
{
  /* Setup Timer 1 to toggle the IR-LED D1 at 38khz
     This will generate a wave of approximately 38.1khz.
     http://www.8bit-era.cz/arduino-timer-interrupts-calculator.html
  */
  cli();              // Stop interrupts while we set up the timer
  TCCR1B = B00000000; // Stop Timer/Counter1 clock by setting the clock source to none.
  TCCR1A = B00000000; // Set Timer/Counter1 to normal mode.
  TCNT1 = 0;          // Set Timer/Counter1 to 0
  OCR1A = 209;        // = 16000000 / (1 * 76190.47619047618) - 1 (must be <65536)
  TCCR1A = B01000100; // Set Timer/Counter1 to CTC mode. Set OC1A to toggle.
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (0 << CS12) | (0 << CS11) | (1 << CS10);
  sei(); // allow interrupts
}

void _data::enableReceive(DeviceType device)
{
  PCICR |= 0b00000100; // turn on port D
  
  if (device & eInfrared)
  {
    pinMode(IR_IN1_PIN, INPUT);
    pinMode(IR_IN2_PIN, INPUT);
    PCMSK2 |= 0b01100000; // turn on pins 5 & 6
  }
  if (device & eBadge)
  {
    pinMode(BADGELINK_PIN, INPUT_PULLUP);
    PCMSK2 |= 0b10000000; // tun on pin 7
    badge_reader.reset();
  }
}

void _data::disableReceive(DeviceType device)
{
  if (device & eInfrared)
  {
    PCMSK2 &= ~0b01100000; // turn off pins 5 & 6
    ir1_reader.reset();
    ir2_reader.reset();
  }
  if (device & eBadge)
  {
    PCMSK2 &= ~0b10000000; // tun off pin 7
    badge_reader.reset();
  }
}

void _data::setup_data_timer()
{
  /* Set up timer 2 for data transfer.
      The NEC protocol has pulses that are multiples of 560 µs
      So we need a frequency of 1786 Hz (aproximatly)
      CPU: 16.000.000 Hz
      Prescaler: 64
      Timer freq: 16.000.000 / 64 = 250.000 Hz
      OCRA2: 250.000 / 1786 - 1 = ~ 139

      This gives us an interrupt frequency of 1786 Hz which is good enough

      Ref: http://www.8bit-era.cz/arduino-timer-interrupts-calculator.html
   */

  // TIMER 2 for interrupt frequency 1786 Hz:
  cli();      // stop interrupts
  TCCR2A = 0; // set entire TCCR2A register to 0
  TCCR2B = 0; // same for TCCR2B
  TCNT2 = 0;  // initialize counter value to 0
  // set compare match register for 8khz increments
  OCR2A = 139;
  // turn on CTC mode
  TCCR2A |= (1 << WGM21);
  // Set CS22 bit for /64 prescaler
  TCCR2B |= (1 << CS22);
  // enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A);
  sei(); // allow interrupts

  // Stop timer
  // TIMSK2 &= ~(1 << OCIE2A);
}

void _data::prepare_pulse_train(uint32_t raw_packet)
{
  byte index = 0;
  pulse_train[index++] = ir_start_high_time;
  pulse_train[index++] = ir_start_low_time;
  for (int i = 0; i < ir_bit_lenght; i++)
  {
    if (bitRead(raw_packet, i))
    {
      pulse_train[index++] = ir_one_high_time;
      pulse_train[index++] = ir_one_low_time;
    }
    else
    {
      pulse_train[index++] = ir_zero_high_time;
      pulse_train[index++] = ir_zero_low_time;
    }
  }
  pulse_train[index++] = ir_stop_high_time;
  pulse_train[index++] = ir_stop_low_time;

  pulse_pointer = 0;
}


// Public
IrDataPacket _data::readIr() //add overload to bypass command type validation?
{
  if (ir1_reader.isDataReady())
  {
    IrDataPacket p(ir1_reader.getPacket());
    Serial.println(p.get_raw());
    p.set_raw(calculateCRC(p.get_raw()));
    if (p.get_crc() == 0) 
    {
      ir2_reader.reset();
      return p;
      }
  }

  if (ir2_reader.isDataReady())
  {
    IrDataPacket p(ir2_reader.getPacket());
    Serial.println(p.get_raw());
    p.set_raw(calculateCRC(p.get_raw()));
    if (p.get_crc() == 0) 
    {
      ir1_reader.reset();
      return p;
      }
  }

  auto emptyPacket = IrDataPacket(0);
  return emptyPacket;
}

LinkDataPacket _data::readBadge()
{
  if (badge_reader.isDataReady())
  {
    LinkDataPacket p;
    p.set_raw(badge_reader.getPacket());
    p.set_raw(calculateCRC(p.get_raw()));
    if (p.get_crc() == 0) 
    {
      return p;   
    }
  }
  auto emptyPacket = LinkDataPacket(0);
  return emptyPacket;
}

_data &_data::getInstance()
{
  static _data data;
  return data;
}

void _data::init()
{
  pinMode(BADGELINK_PIN, INPUT_PULLUP);
  transmitting = false;
  transmit_badge = false;
  transmit_ir = false;
  pulse_pointer = 0;
  setup_data_timer();
}

uint32_t _data::calculateCRC(uint32_t raw_packet){
  uint32_t raw = raw_packet;
  uint32_t checksum =  ((raw <<  2) & 0b10000000111111110111111100) ^
            ((raw <<  1) & 0b01111111100000000111111110) ^
            ((raw <<  0) & 0b00000000111111111111111111) ^
            ((raw >>  1) & 0b00000000100000000000000000) ^
            ((raw >>  2) & 0b00000000011111110000000000) ^
            ((raw >>  3) & 0b00000000111111111000000000) ^
            ((raw >>  4) & 0b00000011100000001111111100) ^
            ((raw >>  5) & 0b00000000111111111000000010);
  checksum = checksum ^ (checksum >> 8) ^ (checksum >> 16) ^ (checksum >> 24);
  checksum = checksum & 0xFF;
  raw ^= checksum << 24;
  return raw;
}


void _data::transmit(IrDataPacket packet)
{
  Serial.println("*");
  disableReceive(eAllDevices);

  packet.set_crc(0);
  packet.set_raw(calculateCRC(packet.get_raw()));

  prepare_pulse_train(packet.get_raw());
  transmit_ir = true;
  setup_ir_carrier();
  transmitting = true;
  while (transmitting)
  {
  }
  enableReceive(eAllDevices);
}

void _data::transmit(LinkDataPacket packet)
{
  Serial.println("Link SEnd");
  disableReceive(eAllDevices);

  packet.set_crc(0);
  packet.set_raw(calculateCRC(packet.get_raw()));

  prepare_pulse_train(packet.get_raw());
  transmit_badge = true;
  pinMode(BADGELINK_PIN, OUTPUT);
  transmitting = true;
  while (transmitting)
  {
  }
  pinMode(BADGELINK_PIN, INPUT_PULLUP);
  enableReceive(eAllDevices);
}

/*
void _data::transmit(DataPacket packet, DeviceType device)
{
  //Fixed delay to give the badge time to switch to receive mode
  if (device & eBadge) delay(8);

  //Serial.println("TRANSMITTING PACKET");
  // 1) Disable Receiving for each device
  disableReceive(device);

  // 2) Clear and recalculate CRC
  packet.crc = 0;
  //packet = calculateCRC(packet);

  prepare_pulse_train(packet.raw);

  if (device & eBadge)
  {
    //Serial.println("TO BADGE");
    // stop RX
    // set badge send
    transmit_badge = true;
  }
  if (device & eInfrared)
  {
    //Serial.println("TO IR");
    // stop RX
    // set ir send
    transmit_ir = true;
    setup_ir_carrier();
  }

  // enable transmit
  pinMode(BADGELINK_PIN, OUTPUT);
  transmitting = true;
  while (transmitting)
  {
  }
  pinMode(BADGELINK_PIN, INPUT_PULLUP);

  enableReceive(device);
}*/

/* Keep this as short (in time) as possible.
 * This ISR is called 1908 times per second.
 */
void _data::transmit_ISR()
{
  if (transmitting)
  {
    if (pulse_pointer % 2 == 1) // would & 0b1 be faster?
    {
      if (transmit_ir)
        DDRB &= ~B00000010;
      if (transmit_badge)
        digitalWrite(BADGELINK_PIN, HIGH);
    }
    else
    {
      if (transmit_ir)
        DDRB |= B00000010;
      if (transmit_badge)
        digitalWrite(BADGELINK_PIN, LOW);
    }
    pulse_train[pulse_pointer]--; // count down

    // if we reached the end go to the next pulse
    if (pulse_train[pulse_pointer] <= 0)
      pulse_pointer++;

    // unless we allready were on the last pulse
    if (pulse_pointer >= pulse_train_lenght)
    {
      transmitting = false;
      transmit_badge = false;
      transmit_ir = false;
    }
  }
}

void _data::receive_ISR(bool ir1, bool ir2, bool badge)
{
  ir1_reader.handlePinChange(ir1);
  ir2_reader.handlePinChange(ir2);
  badge_reader.handlePinChange(badge);
}

/* #endregion */

_data &Data = Data.getInstance();

ISR(TIMER2_COMPA_vect)
{
  Data.transmit_ISR();
}

ISR(PCINT2_vect)
{
  bool IR1 = (PIND & 0b01000000);
  bool IR2 = (PIND & 0b00100000);
  bool Badge = !(PIND & 0b10000000);
  Data.receive_ISR(IR1, IR2, Badge);
}
