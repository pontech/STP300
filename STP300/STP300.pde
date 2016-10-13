// MPIDE Version 20140316
#include <Wire.h>
#include "Half_Duplex_Turnaround.h"
#include <SPI.h>
#include <dSPIN_L6472.h>
#include <EEPROM.h>

#include "pic32lib/DetectEdge.h"
#include "pic32lib/TokenParser.h"
#include "pic32lib/Variant.h"
#include "pic32lib/Properties.h"
#include "pic32lib/Cron.h"

Cron cron(micros);
#line 16 "STP300.pde"

typedef struct {
  us8 BoardId;
  bool homeswitchnc;
  bool stophomingonly;
  us16 maxSpeed;
  us16 minSpeed;
  us16 acceleration;
  us8 currentMoving;
  us8 currentHolding;
  s16 stepspastsenorpos;
  s16 stepspastsensorneg;
  us8 structend;
} ram_struct;

ram_struct ram;

void set_default_ram() {
  ram.BoardId = 0;
  ram.homeswitchnc = true;
  ram.stophomingonly = false;
  ram.maxSpeed = 63;
  ram.minSpeed = 42;
  ram.acceleration = 55;
  ram.currentMoving = 47;
  ram.currentHolding = 6;
  ram.stepspastsenorpos = 0;
  ram.stepspastsensorneg = 0;
  ram.structend = 0xA5;
}
void eeprom_in(us8* Data,us16 eeprom_adress,us16 bytes) {
  int i;
  for(i=0;i<bytes;i++){
    EEPROM.write(eeprom_adress++, *Data++);
  }
}
void eeprom_out(us16 eeprom_adress,us8* Data,us16 bytes) {
  int i;
  for(i=0;i<bytes;i++){
    *(Data++) = EEPROM.read(eeprom_adress++);
  }
}
//  //save setting to eeprom
//  eeprom_in((us8*)&ram,0,sizeof(ram));
//  eeprom_out(0,(us8*)&ram,sizeof(ram)); //get structure from memory
//  if (ram.structend != 42)
//    set_default_ram();

#define Enable485 68
#define Relay1Out 28
#define Relay2Out 29

TokenParser usb(&Serial);
SerialHalf MySerial0(&Serial0, Enable485, true);
TokenParser rs485(&MySerial0);

const int pJP0 = 46;
const int pJP1 = 45;
const int pJP2 = 48;
const int pJP3 = 59;

L6472 axis(ReadJumper(), 104, 103, 102, 18, 71); //Board_ID, Response stream, MOSI, MISO, SCK, SS, RST
//L6472 axis(ReadJumper(), &Serial0, 104, 103, 102, 18, 71); //Board_ID, Response stream, MOSI, MISO, SCK, SS, RST
//L6472 axis(ReadJumper(), &Serial, 104, 103, 102, 18, 71); //Board_ID, Response stream, MOSI, MISO, SCK, SS, RST

Properties properties(24);

// inputs
#define posHome    69
#define negHome    70
#define DEBUGLED 80
#define Enable485 68

unsigned char ReadJumper()
{
  pinMode(pJP0, INPUT);
  pinMode(pJP1, INPUT);
  pinMode(pJP2, INPUT);
  unsigned char val = 0;
  if (digitalRead(pJP0) == LOW)
  {
    val += 1;
  }
  if (digitalRead(pJP1) == LOW)
  {
    val += 2;
  }
  if (digitalRead(pJP2) == LOW)
  {
    val += 4;
  }
  return val + ram.BoardId;
}

void flash() {
  Cron::CronDetail *self = cron.self();

  digitalWrite(DEBUGLED, self->temp);

  self->temp ^= 1;
  self->yield = micros() + 1000000;
}

void Setup_dSPIN(L6472 &motor)
{
  int res = 0;
  
  motor.setupPort();
  SPI.setClockDivider(SPI_CLOCK_DIV16); // or 2, 8, 16, 32, 64
  SPI.setDataMode(SPI_MODE3);	

  res = motor.init(1.5, 0.2f);
      
  motor.setMicroSteps(16); //1,2,4,8,16

  motor.setAcc(400); //set acceleration
  motor.setMaxSpeed(1000);  

  motor.setMinSpeed(10);
  motor.setThresholdSpeed(1000);
  motor.setOverCurrent(6000); //set overcurrent protection
}

void setup() {
  RCON = 2;
  Serial.begin(115200); // enable usb communication
  MySerial0.begin(115200); // enable rs485 communication
  Wire.begin();
  pinMode(DEBUGLED,OUTPUT);
  pinMode(Enable485,OUTPUT);
  digitalWrite(Enable485, LOW);
  pinMode(Relay1Out,OUTPUT);
  digitalWrite(Relay1Out,LOW);
  pinMode(Relay2Out,OUTPUT);
  digitalWrite(Relay2Out,LOW);

  eeprom_out(0,(us8*)&ram,sizeof(ram)); //get structure from memory
  if (ram.structend != 0xA5)
    set_default_ram();

  Setup_dSPIN(axis);
  copySettingsToDevice();
  axis.hardStop();
  
  pinMode(posHome,INPUT);
  pinMode(negHome,INPUT);

  cron.add(flash);
}

void copySettingsToRam()
{
  ram.maxSpeed = axis.GetParam(MAX_SPEED);
  ram.minSpeed = axis.GetParam(MIN_SPEED);
  ram.acceleration = axis.GetParam(ACC);
  ram.currentMoving = axis.GetParam(TVAL_RUN);
  ram.currentHolding = axis.GetParam(TVAL_HOLD);
}

void copySettingsToDevice()
{
  axis.SetParam(MAX_SPEED, ram.maxSpeed);
  axis.SetParam(MIN_SPEED, ram.minSpeed);
  axis.SetParam(ACC, ram.acceleration);
  axis.SetParam(DECEL, ram.acceleration);
  axis.SetParam(TVAL_RUN, ram.currentMoving);
  axis.SetParam(TVAL_ACC, ram.currentMoving);
  axis.SetParam(TVAL_DEC, ram.currentMoving);
  axis.SetParam(TVAL_HOLD, ram.currentHolding);
}
void loop() {
  cron.scheduler();
  if((!ram.stophomingonly || axis.getHRunning()) && axis.isBusy()) //if motor is homing or homingonly not set
  {
    if(axis.sensorStop(posHome, ram.homeswitchnc, true))
    {
      axis.move(ram.stepspastsenorpos);
      while(axis.isBusy()){};
    }
    if(axis.sensorStop(negHome, ram.homeswitchnc, false))
    {
      axis.move(ram.stepspastsensorneg*(-1));
      while(axis.isBusy()){};
    }
  }
  
  if(usb.scan()) {
    axis.BoardId(ReadJumper());
    usb.save(); //save head and tail
    usb.advanceHead(usb.remaining());
    String stufftosend = usb.toString();
    axis.command(&stufftosend[0], &Serial);
    if (Serial){
      rs485.print(&stufftosend[0]);
      rs485.print("\r");
    }
    usb.restore(); //restore head and tail
    processInput(usb);
  }
  if(rs485.scan()) {
    axis.BoardId(ReadJumper());
    rs485.save(); //save head and tail
    rs485.advanceHead(rs485.remaining());
    String stufftosend = rs485.toString();
    axis.command(&stufftosend[0], &MySerial0);
    if (Serial) //if USB Serial is connected
    {
      Serial.print(&stufftosend[0]);
      Serial.print("\r");
    }
    rs485.restore(); //restore head and tail
    processInput(rs485);
  }
}
char spbuf[20];
void processInput(TokenParser& parser)
{
  if((parser.toString()[0] == '/') && (axis.parseNumber(&(parser.toString()[1])) == ReadJumper())) {
    parser.nextToken();
    if(parser.compare("relay")){
      parser.nextToken();
      int hold = parser.toVariant().toInt();
      digitalWrite(Relay1Out,hold);
      parser.print("OK\r");
    }
    else if(parser.compare("boardid")){
      parser.nextToken();
      ram.BoardId = parser.toVariant().toInt();
      parser.print("OK\r");
    }
    else if(parser.compare("switchtype")){
      parser.nextToken();
      int hold = parser.toVariant().toInt();
      if(hold > 0)
      {
        ram.homeswitchnc = true;
        parser.print("1\r");
      }
      else
      {
        ram.homeswitchnc = false;
        parser.print("0\r");
      }
    }
    else if(parser.compare("stoptype")){
      parser.nextToken();
      int hold = parser.toVariant().toInt();
      if(hold > 0)
      {
        ram.stophomingonly = true;
        parser.print("1\r");
      }
      else
      {
        ram.stophomingonly = false;
        parser.print("0\r");
      }
    }
    else if(usb.compare("wss")){
      parser.print("OK\r");
      copySettingsToRam();
      //save setting to eeprom
      eeprom_in((us8*)&ram,0,sizeof(ram));
    }
    else if(parser.compare("getparam")){
      parser.nextToken();
      int hold = parser.toVariant().toInt();
      snprintf(spbuf,20,"%d\r",axis.GetParam((us8)hold));
      parser.print(spbuf);
    }
    else if(usb.compare("setparam")){
      parser.nextToken();
      int hold = parser.toVariant().toInt();
      parser.nextToken();
      axis.SetParam((us8)hold,parser.toVariant().toInt());
      parser.print("OK\r");
    }
    else if(parser.compare("spsp")){
      if(parser.nextToken())
      {
        ram.stepspastsenorpos = parser.toVariant().toInt();
      }
      snprintf(spbuf,20,"%d\r",ram.stepspastsenorpos);
      parser.print(spbuf);
    }
    else if(parser.compare("spsn")){
      if(parser.nextToken())
      {
        ram.stepspastsensorneg = parser.toVariant().toInt();
      }
      snprintf(spbuf,20,"%d\r",ram.stepspastsensorneg);
      parser.print(spbuf);
    }
    else if(parser.compare("reset")){
      parser.print("Close serial terminal, resetting board in...\r");
      us8 sec;
      for( sec = 5; sec >= 1; sec-- ) {
        snprintf(spbuf,20,"%d seconds...\r",sec);
        parser.print(spbuf);
        delay(1000);
      }
      Reset();
    }
  }
}
void Reset()
{
  // The VIRTUAL PROGRAM BUTTONS are not defined in the variants
  // so its done here (for now)
#ifndef VIRTUAL_PROGRAM_BUTTON_TRIS

#define USE_VIRTUAL_PROGRAM_BUTTON      1
#define VIRTUAL_PROGRAM_BUTTON_TRIS     TRISDbits.TRISD4
#define VIRTUAL_PROGRAM_BUTTON          LATDbits.LATD4

#endif

#ifdef VIRTUAL_PROGRAM_BUTTON_TRIS
  VIRTUAL_PROGRAM_BUTTON_TRIS = 0; //Set virtual button as output
  VIRTUAL_PROGRAM_BUTTON = 1; //push virtual button
#endif
  SYSKEY = 0x00000000;  //write invalid key to force lock
  SYSKEY = 0xAA996655;  //write key1 to SYSKEY
  SYSKEY = 0x556699AA;  //write key2 to SYSKEY  // OSCCON is now unlocked
  RSWRSTSET = 1; //set SWRST bit to arm reset
  unsigned int dummy;
  dummy = RSWRST; //read RSWRST register to trigger reset
  while(1); //prevent any unwanted code execution until reset occurs
}

