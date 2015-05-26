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
#line 15 "STP300.pde"

typedef struct {
  us8 movingCurrent; //TVAL is seven bits us8 is plenty
  us8 holdingCurrent;
  bool homeswitchnc;
  bool stophomingonly;
  us8 structend;
} ram_struct;

ram_struct ram;

void set_default_ram() {
  ram.movingCurrent = 47; //TVAL is seven bits us8 is plenty
  ram.holdingCurrent = 15;
  ram.homeswitchnc = true;
  ram.stophomingonly = false;
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
#define RelayOut 28

TokenParser usb(&Serial);
SerialHalf MySerial0(&Serial0, Enable485, true);
TokenParser rs485(&MySerial0);

unsigned char BOARD_ID = 0;
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
  return val;
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
  pinMode(RelayOut,OUTPUT);
  digitalWrite(RelayOut,LOW);

  eeprom_out(0,(us8*)&ram,sizeof(ram)); //get structure from memory
  if (ram.structend != 0xA5)
    set_default_ram();

  Setup_dSPIN(axis);
  axis.hardStop();
  
  pinMode(posHome,INPUT);
  pinMode(negHome,INPUT);

  cron.add(flash);
}

void loop() {
  cron.scheduler();
  if(!ram.stophomingonly || axis.getHRunning()) //if motor is homing or homingonly not set
  {
    axis.sensorStop(posHome, ram.homeswitchnc, true);
    axis.sensorStop(negHome, ram.homeswitchnc, false);
  }
  
  if(usb.scan()) {
//    properties.evaluate(usb);
    usb.save(); //save head and tail
    usb.advanceHead(usb.remaining());
    String stufftosend = usb.toString();
    axis.command(&stufftosend[0], &Serial);
    usb.restore(); //restore head and tail
    if((usb.toString()[0] == '/') && (axis.parseNumber(&(usb.toString()[1])) == ReadJumper())) {
      usb.nextToken();
      if(usb.compare("relay")){
        usb.nextToken();
        int hold = usb.toVariant().toInt();
        digitalWrite(RelayOut,hold);
        usb.print("OK\r");
      }
      else if(usb.compare("switchtype")){
        usb.nextToken();
        int hold = usb.toVariant().toInt();
        if(hold > 0)
        {
          ram.homeswitchnc = true;
          usb.print("1\r");
        }
        else
        {
          ram.homeswitchnc = false;
          usb.print("0\r");
        }
        //save setting to eeprom
        eeprom_in((us8*)&ram,0,sizeof(ram));
      }
      else if(usb.compare("stoptype")){
        usb.nextToken();
        int hold = usb.toVariant().toInt();
        if(hold > 0)
        {
          ram.stophomingonly = true;
          usb.print("1\r");
        }
        else
        {
          ram.stophomingonly = false;
          usb.print("0\r");
        }
        //save setting to eeprom
        eeprom_in((us8*)&ram,0,sizeof(ram));
      }
      else if(usb.compare("reset")){
        Serial.println("Close serial terminal, resetting board in...");
        us8 sec;
        for( sec = 5; sec >= 1; sec-- ) {
          Serial.print(sec, DEC);
          Serial.println(" seconds...");
          delay(1000);
        }
        Reset();
      }
    }
  }
  if(rs485.scan()) {
//    properties.evaluate(rs485);
    rs485.save(); //save head and tail
    rs485.advanceHead(rs485.remaining());
    String stufftosend = rs485.toString();
    //Serial.println(stufftosend);
    axis.command(&stufftosend[0], &MySerial0);
    rs485.restore(); //restore head and tail
    if((rs485.toString()[0] == '/') && (axis.parseNumber(&(rs485.toString()[1])) == ReadJumper())) {
      rs485.nextToken();
      if(rs485.compare("relay")){
        rs485.nextToken();
        int hold = rs485.toVariant().toInt();
        digitalWrite(RelayOut,hold);
        rs485.print("OK\r");
      }
      else if(rs485.compare("switchtype")){
        rs485.nextToken();
        int hold = rs485.toVariant().toInt();
        if(hold > 0)
        {
          ram.homeswitchnc = true;
          rs485.print("1\r");
        }
        else
        {
          ram.homeswitchnc = false;
          rs485.print("0\r");
        }
        //save setting to eeprom
        eeprom_in((us8*)&ram,0,sizeof(ram));
      }
      else if(rs485.compare("stoptype")){
        rs485.nextToken();
        int hold = rs485.toVariant().toInt();
        if(hold > 0)
        {
          ram.stophomingonly = true;
          rs485.print("1\r");
        }
        else
        {
          ram.stophomingonly = false;
          rs485.print("0\r");
        }
        //save setting to eeprom
        eeprom_in((us8*)&ram,0,sizeof(ram));
      }
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

