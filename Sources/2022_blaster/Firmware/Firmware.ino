#include "animations.h"
#include "data.h"
#include "EEPROM.h"

#define VERSION 20240626

#define R_TEAM_PIN A5
#define G_TEAM_PIN 4
#define B_TEAM_PIN 8
#define TRIGGER_PIN 3

//add eGMhitpoints
enum GameMode : uint8_t
{
  eGMTimeout = 0,
  eGMZombie = 1,
  eGMSuddenDeath = 2,
  eGMChatterMaster = 3
};

/* Blaster state variables */
// if false the trigger is disabled. Set by the badge (trigger field)
bool can_shoot = true;

uint8_t channel = CH_STANDALONE; // default channel

uint8_t badge_team = 0;  // the team set by the badge (overrules the blaster if != 0)
uint8_t zombie_team = 0;  // indicates the team that infected you
uint8_t hardware_team = 0; // in case the hardware switch is in between values
uint8_t trigger_action = eActionDamage; //damage, heal
uint16_t player_id = 0;
uint8_t hitpoints = 4;

//flags
bool stealth_mode = false;
bool single_shot_mode = false;
uint8_t game_mode = eGMTimeout;

uint8_t last_active_team = 0;  // to detect team changes

uint8_t chatter_limit = 10;
uint8_t hit_timeout = 3;

void setup()
{
  setupButtons();
  setupCommunication();
  setupRandomSeed();
  setupAnimations();
  blinkIfNoTeamSelector();
  modeSelection();

  //Animations::mute(); //TODO
  Animations::blaster_start();

  player_id = EEPROM.read(0);
  player_id <<= 8;
  player_id += EEPROM.read(1);
  if (player_id <= 2048 || player_id >= 3072) {
    player_id = 0b100000000000;
    player_id += random(1024);
    EEPROM.write(0,player_id >> 8);
    EEPROM.write(1,player_id & 0xFF);
  }
  Serial.print("Player ID: ");
  Serial.println(player_id);


  Serial.println(" * Blaster Ready\n\n");
  //sendACK();
  delay(500);
  //sendACK(); //blaster is listening 
}

void loop()
{
  //dirty but works for now
  if (game_mode == eGMChatterMaster) chatter_master_loop();

  if (teamChanged()){
    Animations::team_switch(activeTeam(), hitpoints);
  }

  if (triggerPressed())
  {
    if (hitpoints)
    {
        damageShot();
        Animations::shoot(activeTeam());
        Animations::set_team_status(activeTeam(), hitpoints);
    }
    else
    {
      Animations::error();
    }
    if (single_shot_mode)
    {
      single_shot_mode = false;
      can_shoot = false;
      Animations::set_team_status(activeTeam(), hitpoints);
    }
    Animations::stealth(false);
  }

  handle_badge_packet(Data.readBadge());
  handle_ir_packet(Data.readIr());

  if (game_mode == eGMZombie && zombie_team > 0)
  {
    Animations::FlickerTeam(zombie_team, hardware_team);
  }

  if (hitpoints==0) {
    for (int i =0; i< 4; i++){
      hitpoints++;
      Animations::set_team_status(activeTeam(), hitpoints);
      delay(1000);
    }

  }
}

void chatter_master_loop(){
  Animations::chatter();
  Animations::clear();

  while (true){
    if (triggerPressed()){
      /*DataPacket d;
      d.team = eTeamRex;
      d.trigger_state = 1;
      d.command = eCommandChatter;
      d.parameter = 9;

      Data.transmit(d, eInfrared);*/
      Serial.println("boom");
    }
  }
}

void handle_badge_packet(LinkDataPacket packet)
{
  if (packet.raw == 0) return;

  switch (packet.command){
    case CMD_ACK:
      Serial.println("Link_in ACK");
      //not supposed to happen
      break;
    case CMD_MODE:
      ModeParameter payload;
      payload.raw = packet.parameter;
      Serial.print("  Mode: ");
      Serial.println(payload.mode);
      Serial.print("  Team: ");
      Serial.println(payload.team);
      Serial.print("  Action: ");
      Serial.println(payload.action);
      Serial.print("  ID: ");
      Serial.println(payload.id);
      Serial.print("  hp: ");
      Serial.println(payload.hp);
      Serial.print("  ready: ");
      Serial.println(payload.ready);

      sendAck();
      break;
  }

 
}

void handle_set_hit_timeout(LinkDataPacket packet){
  //hit_timeout = packet.parameter;
}

void handle_set_settings(LinkDataPacket packet){
 /* bool mute = packet.parameter & 0b0001;
  if (mute) {
    Animations::mute();
  } else {
    Animations::unmute();
  }
  uint8_t brightness = (packet.parameter & 0b1110) >> 1; // brightness 
  uint8_t result = 0b00000000;
  if (brightness & 0b100) result |= 0b11000000;
  if (brightness & 0b010) result |= 0b00111000;
  if (brightness & 0b001) result |= 0b00000111;
  Leds.setBrightness(result);
  Leds.update();
  */
}

void handle_play_chatter(IrDataPacket packet){
  /*
  if (packet.parameter <= 0) return;
  if (packet.parameter >= chatter_limit) return;
  //chatter_limit = packet.parameter;
  delay(random(500,1000));
  if (random(30) == 0) Animations::wolfWhistle();
  else Animations::chatter();
  packet.parameter--;
  if (packet.parameter > 0){
    delay(random(250,500));
    //Data.transmit(packet, eInfrared);
  }
  Animations::set_team_status(activeTeam(),can_shoot);
  */
}

void handle_play_animation(LinkDataPacket packet) {
  /*
  switch (packet.parameter){
    case Animations::eAnimationBlasterStart:
      Animations::blaster_start();
      break;
    case Animations::eAnimationError:
          Animations::error();
      break;
    case Animations::eAnimationCrash:
      Animations::crash(7,5);
      break;
    case Animations::eAnimationFireball:
      Animations::fireball();
      break;
    case Animations::eAnimationOneUp:
      Animations::one_up();
      break;
    case Animations::eAnimationCoin:
      Animations::coin();
      break;
    case Animations::eAnimationVoice:
      Animations::voice();
      break;
    case Animations::eAnimationWolfWhistle:
      Animations::wolfWhistle();
      break;
    case Animations::eAnimationChatter:
      Animations::chatter();
      break;    
    case Animations::eAnimationBlinkTeamLed:
      Animations::blink_team_led();
      break;
  }
  Animations::set_team_status(activeTeam(),can_shoot);
*/
}

void handle_ir_packet(IrDataPacket packet)
{
  if (packet.raw == 0)
    return;
  Serial.println("Received message from IR");
  switch (packet.action)
  {
  case eActionDamage:
    Serial.println("eActionDamage");
    if (hitpoints) handle_damage_received(packet);   
    break;
  /*case eCommandHeal:
    Serial.println("eCommandHeal");
    handle_healing_received(packet);
    break;
  case eCommandChatter:
    sendACK();
    Serial.println("eCommandChatter");
    handle_play_chatter(packet);
    sendACK(true);
    break;*/
  default:
    break;
  }
}

void handle_damage_received(IrDataPacket packet)
{
  if (packet.channel == channel && packet.team != activeTeam())
  {
    Animations::stealth(false);
   
    /*
    // send data to badge
    packet.trigger_state = 0; // we are not fireing
    //Data.transmit(packet, eBadge);  TODO
    */
    switch (game_mode)
    {
    case eGMTimeout:
      // action here depends on game mode
      Animations::crash(packet.team, hit_timeout);
      if (hitpoints) hitpoints--;
      Serial.print("Hitpoints: ");
      Serial.println(hitpoints);
      Data.readIr();    // clear buffer
      Data.readBadge(); // clear badge (in case the badge received the same packet) //todo: improve so that we only remove shoot commands
      Animations::clear();
      Animations::team_switch(activeTeam(), hitpoints);
      break;
    /*case eGMZombie:
      // action here depends on game mode
      Animations::crash(packet.team, hit_timeout);
      Data.readIr();    // clear buffer
      Data.readBadge(); // clear badge (in case the badge received the same packet) //todo: improve so that we only remove shoot commands
      Animations::clear();
      zombie_team = packet.team;
      healing_mode = false; // no such thing as healing zombies
      if (packet.team == nonZombieTeam())
        zombie_team = 0;
      Animations::team_switch(activeTeam(), can_shoot);
      break;*/
    }
  }
}

void handle_healing_received(IrDataPacket packet) {
  /*
    packet.trigger_state = 0; // we are not fireing
    //Data.transmit(packet, eBadge); TODO
    zombie_team = 0;
    */
}

void setGameMode(LinkDataPacket packet)
{
  /*
  Serial.print("Received eCommandSetGameMode M:");
  Serial.print(packet.parameter);
  Serial.print(" T:");
  Serial.println(packet.team);
  if (packet.parameter == eGMTimeout)
  {
    game_mode = packet.parameter;
    can_shoot = true;
    healing_mode = false;
    single_shot_mode = false;
    zombie_team = 0;
    badge_team = packet.team;
  }
  else if (packet.parameter == eGMZombie)
  {
    game_mode = packet.parameter;
    can_shoot = true;
    healing_mode = false;
    single_shot_mode = false;
    zombie_team = 0;
    badge_team = packet.team;
  }
  else if (packet.parameter == eGMSuddenDeath)
  {
    game_mode = packet.parameter;
    can_shoot = true;
    healing_mode = false;
    single_shot_mode = false;
    zombie_team = 0;
    badge_team = packet.team;
  }
  */
}

void setTeamColor(LinkDataPacket packet)
{
  /*
  Serial.println("Received eCommandTeamChange");
  badge_team = packet.team;
  */
}

void setTriggerAction(LinkDataPacket packet)
{
  /*
  Serial.println("Received eCommandSetTriggerAction");
  stealth_mode = packet.parameter & 8;
  single_shot_mode = packet.parameter & 4;
  healing_mode = packet.parameter & 2;
  can_shoot = !(packet.parameter & 1);

  Animations::stealth(stealth_mode);
  Animations::set_team_status(activeTeam(), can_shoot);
  */
}

void setChannel(LinkDataPacket packet)
{
  Serial.println("Received eCommandSetChannel");
  //ir_channel = packet.parameter;
}

void damageShot()
{
  Serial.println("Pang!");
  Serial.println(activeTeam());

  IrDataPacket packet;
  packet.raw = 0;
  packet.team = activeTeam();
  packet.channel = channel;
  packet.player_id = player_id;
  packet.action = trigger_action;
  packet.action_param = 1;
  Data.transmit(packet);

  delay(100);

  LinkDataPacket dp;
  dp.raw = 0;
  dp.command = CMD_TRIGGER;
  Data.transmit(dp);

/*
  DataPacket d;
  d.team = TeamColor(activeTeam());
  d.trigger_state = 1;
  d.command = eCommandShoot;
  d.parameter = ir_channel;

  Data.transmit(d, eAllDevices);
  */
}

/*
  Configure all switch inputs as input_pullup
*/
void setupButtons(){
  pinMode(TRIGGER_PIN, INPUT_PULLUP);
  pinMode(R_TEAM_PIN, INPUT_PULLUP);
  pinMode(G_TEAM_PIN, INPUT_PULLUP);
  pinMode(B_TEAM_PIN, INPUT_PULLUP);
}

void setupCommunication(){
  Serial.begin(115200);
  Serial.println();
  Serial.println("___ _ _  _ ____    ___  _    ____ ____ ___ ____ ____");
  Serial.println(" |  | |\\/| |___    |__] |    |__| [__   |  |___ |__/ ");
  Serial.println(" |  | |  | |___    |__] |___ |  | ___]  |  |___ |  \\______\\/_");
  Serial.println("                                                          /\\");
  Serial.println();                                                
  Serial.println(" * Serial Ready.");
  Serial.print(" * Firmware version: ");
  Serial.println(VERSION);
  Data.init();
  Serial.println(" * Communication Ready.");
}

void setupRandomSeed(){
  auto analogValue = analogRead(A4);
  randomSeed(analogValue);
  Serial.print(" * Random seed: ");
  Serial.println(analogValue);
}

void setupAnimations(){
  Animations::setup();
  Serial.println(" * Animations ready.");
}

/* This is to give a visual signal that the firmware is flashed successfully when bulk uploading.
 * This has no use after flashing.
 */
void blinkIfNoTeamSelector()
{
  //Don't use gethardwareteam() here as it ignores team 0
  pinMode(R_TEAM_PIN, INPUT_PULLUP);
  pinMode(G_TEAM_PIN, INPUT_PULLUP);
  pinMode(B_TEAM_PIN, INPUT_PULLUP);
  pinMode(A0, OUTPUT);
  if (digitalRead(R_TEAM_PIN) && digitalRead(G_TEAM_PIN) && digitalRead(B_TEAM_PIN))  {
    Serial.println(" * No team switch detected, entering Post Flash Mode");
    while (digitalRead(R_TEAM_PIN) && digitalRead(G_TEAM_PIN) && digitalRead(B_TEAM_PIN))    {
      digitalWrite(A0, HIGH);
      delay(100);
      digitalWrite(A0, LOW);
      delay(500);
    }
    Serial.println(" * Exit Flash Test Mode");
  } else{
    Serial.println(" * Team switch detected");
  }
  pinMode(A0, INPUT);
}


void modeSelection(){
  if (triggerPressed())  {
    Serial.println(" * Trigger pressed, starting in mode selection mode.");
    switch (getHardwareTeam())
    {
    case 1:
      Animations::mute();
      break;
    case 2:
      break;
    case 4:
      break;
    default:
      break;
    }

    Serial.print(" * Waiting for trigger to be released ");
    while (triggerPressed()) {
      Animations::blink_team_led();
    }
  }
}



void sendAck()
{
  delay(10);
  LinkDataPacket dp;
  dp.raw = 0;
  dp.command = CMD_ACK;
  dp.parameter = 1;  // Something to ensure we don't send out a 00000 packet
  Data.transmit(dp);
}


bool triggerPressed()
{
  return !digitalRead(TRIGGER_PIN);
}

bool teamChanged()
{
  if (last_active_team != activeTeam())
  {
    last_active_team = activeTeam();
    return true;
  }
  return false;
}

// Get the team from the hardware switch.
uint8_t getHardwareTeam() {
  if (zombie_team) return hardware_team;
  uint8_t r = (1 - digitalRead(R_TEAM_PIN)) * 1;
  uint8_t g = (1 - digitalRead(G_TEAM_PIN)) * 2;
  uint8_t b = (1 - digitalRead(B_TEAM_PIN)) * 4;

  uint8_t team = r + g + b;
  if (team != hardware_team && team != 0){
    hardware_team = team;
    LinkDataPacket d;
    /*
    d.team = TeamColor(hardware_team);
    d.trigger_state = 0;
    d.command = eCommandTeamChange;
    d.parameter = 0;
    Serial.print("Sending Changed HW Team (=");
    Serial.print(hardware_team);
    Serial.println(") to badge.");*/
    //Data.transmit(d, eBadge);  TODO
  }

  return hardware_team;
}

uint8_t activeTeam() //The team you shoot as
{
  if (zombie_team > 0 && game_mode == eGMZombie)
    return zombie_team;
  if (badge_team > 0)
    return badge_team;
  return getHardwareTeam();
}

uint8_t nonZombieTeam(){
  if (badge_team > 0)
    return badge_team;
  return getHardwareTeam();
}
