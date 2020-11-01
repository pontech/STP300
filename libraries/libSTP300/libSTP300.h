#ifndef _libSTP300_H_
#define _libSTP300_H_

//////////////////////////////////////////////////////////
// Include
//////////////////////////////////////////////////////////

// C/C++
#include <stdint.h>
// chipKIT
#include <Wire.h>
#include <SPI.h>
#include <EEPROM.h>
// PONTECH pic32lib
#include "pic32lib/DetectEdge.h"
#include "pic32lib/TokenParser.h"
#include "pic32lib/Variant.h"
#include "pic32lib/Properties.h"
#include "pic32lib/Cron.h"
// Application Specific
#include "Half_Duplex_Turnaround.h"
#include <dSPIN_L6472.h>

//////////////////////////////////////////////////////////
// GPIO Definitions Outputs
//////////////////////////////////////////////////////////

uint8_t Enable485 = 68;
uint8_t Relay1Out = 28;
uint8_t Relay2Out = 29;

uint8_t pJP0 = 46;
uint8_t pJP1 = 45;
uint8_t pJP2 = 48;
uint8_t pJP3 = 59;

//////////////////////////////////////////////////////////
// GPIO Definitions Inputs
//////////////////////////////////////////////////////////

uint8_t posHome 	= 69;
uint8_t negHome 	= 70;
uint8_t DEBUGLED 	= 80;

//////////////////////////////////////////////////////////
// NVM Settings
//////////////////////////////////////////////////////////

typedef struct {
  uint8_t BoardId;
  bool homeswitchnc;
  bool stophomingonly;
  uint16_t maxSpeed;
  uint16_t minSpeed;
  uint16_t acceleration;
  uint8_t currentMoving;
  uint8_t currentHolding;
  int16_t stepspastsenorpos;
  int16_t stepspastsensorneg;
  uint8_t structend;
} ram_struct;

extern ram_struct ram;

// Stepper motor object
extern L6472 axis;

extern TokenParser usb;
extern SerialHalf MySerial0;
extern TokenParser rs485;

//////////////////////////////////////////////////////////
// Function Prototypes
//////////////////////////////////////////////////////////

void set_default_ram();
void eeprom_in(uint8_t* data, uint16_t eeprom_adress, uint16_t bytes);
void eeprom_out(uint16_t eeprom_adress,uint8_t* data, uint16_t bytes);
void copySettingsToRam();
void copySettingsToDevice();

unsigned char ReadJumper();
bool safeToMove(bool directionPositive);

// Setup
void setup_STP300();

// Loops
void loop_stp300_homing_task();
void processInput(TokenParser& parser);
void loop_stp300_serial_parser();

// Reset
void Reset();

#endif
