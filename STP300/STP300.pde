// MPIDE Version 20140316
#include <Wire.h>
#include "Half_Duplex_Turnaround.h"
#include <SPI.h>
#include <dSPIN_L6472.h>

#include "pic32lib/DetectEdge.h"
#include "pic32lib/TokenParser.h"
#include "pic32lib/Variant.h"
#include "pic32lib/Properties.h"
#include "pic32lib/Cron.h"

Cron cron(micros);
#line 15 "YZ_Unit.pde"
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
  //possible values 0,2,3,4,5,6,7,8,9,10,11,12,13,15(6,9 have 2 methods)
  pinMode(pJP0, INPUT);
  pinMode(pJP1, INPUT);
  pinMode(pJP2, INPUT);
  pinMode(pJP3, INPUT);
  unsigned char val = 0;
  if (digitalRead(pJP0) == LOW)
  {
    val += 2;
  }
  if (digitalRead(pJP1) == LOW)
  {
    val += 3;
  }
  if (digitalRead(pJP2) == LOW)
  {
    val += 4;
  }
  if (digitalRead(pJP3) == LOW)
  {
    val += 6;
  }
  return val;
}

// catch ZeroYAxis property updates
void zeroAxisHandler(String var) {
  if((var == "true") && (properties.value("Power") == "true")) {
    Setup_dSPIN(axis);
    cron.add(zeroAxisTask);
  }
  else {
    axis.softStop();
    cron.stopByFunction(zeroAxisTask);
  }
}

// catch Power property updates
void powerHandler(String var) {
  if(var == "true") {
    // enable motor power
    axis.hardStop();
  }
  else {
    // disable motor power
    axis.free();
  }
}

void axisHandler(String var) {
  if(properties.value("Power") == "true") {
    axis.goTo(Variant::fromString(var).toInt());
  }
}

void zeroAxisTask() {
  Cron::CronDetail *self = cron.self();
  
  switch(self->temp) {
  case 0:
    if(!axis.isBusy()) {
      //usb.println("axis backoff flag");
      axis.move(100);
      self->temp++;
    }
    break;
  case 1:
    if(!axis.isBusy()) {
      //usb.println("axis approach flag");
      axis.move(-2000000);
      self->temp++;
    }
    break;
  case 2:
    if(!axis.isBusy()) {
      if(digitalRead(posHome)) { //if(getHomeSensorStatus()) {
        //usb.println("axis homed");
      }
      else {
        //usb.println("axis timeout");
      }
      //setHomeSensor(0);
      axis.move(0.1);
      self->temp++;
//    } else {
//      if(digitalRead(XAxisHome)) { //if(getHomeSensorStatus()) {
//        xaxis.hardStop();
//      }
    }
    break;
  case 3:
    if(!axis.isBusy()) {
      properties.update("ZeroAxis", "false");
      axis.setAsHome();
      return;
    }
  }
  self->yield = micros() + 10000;
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

  res = motor.init(1.5, 0.1f);
      
  motor.setMicroSteps(16); //1,2,4,8,16

  motor.setAcc(400); //set acceleration
  motor.setMaxSpeed(1000);  

  motor.setMinSpeed(10);
  motor.setThresholdSpeed(1000);
  motor.setOverCurrent(6000); //set overcurrent protection
}

void setup() {
  Serial.begin(115200); // enable usb communication
  MySerial0.begin(115200); // enable rs485 communication
  Wire.begin();
  pinMode(DEBUGLED,OUTPUT);
  pinMode(Enable485,OUTPUT);
  digitalWrite(Enable485, LOW);
  pinMode(RelayOut,OUTPUT);
  digitalWrite(RelayOut,LOW);

  properties.addBool("Power", false, powerHandler);
  properties.addNumber("Axis", 0, axisHandler);
  properties.addBool("ZeroAxis", false, zeroAxisHandler);

  Setup_dSPIN(axis);
  axis.hardStop();
  
  pinMode(posHome,INPUT);
  pinMode(negHome,INPUT);

  cron.add(flash);
}

void loop() {
  cron.scheduler();
  axis.sensorStop(posHome, true, true);
  axis.sensorStop(negHome, true, false);
  
  if(usb.scan()) {
    properties.evaluate(usb);
    usb.save(); //save head and tail
    usb.advanceHead(usb.remaining());
    String stufftosend = usb.toString();
    axis.command(&stufftosend[0], &Serial);
    usb.restore(); //restore head and tail
  }
  if(rs485.scan()) {
    properties.evaluate(rs485);
    rs485.save(); //save head and tail
    rs485.advanceHead(rs485.remaining());
    String stufftosend = rs485.toString();
    //Serial.println(stufftosend);
    axis.command(&stufftosend[0], &MySerial0);
    rs485.restore(); //restore head and tail
    if((rs485.toString()[0] == '/') && (rs485.toString()[1] == ReadJumper() + '0')) {
      rs485.nextToken();
      if(rs485.compare("relay")){
        rs485.nextToken();
        int hold = rs485.toVariant().toInt();
        digitalWrite(RelayOut,hold);
        rs485.print("OK\r");
      }
    }
  }
}

