#include <libSTP300.h>

//////////////////////////////////////////////////////////
// Token Parser and Serial Communication Objects
//////////////////////////////////////////////////////////
TokenParser usb(&Serial);
SerialHalf MySerial0(&Serial0, Enable485, true);
TokenParser rs485(&MySerial0);

//Properties properties(24);  // Does not appear to be being used

// NVM Settings
ram_struct ram;

//////////////////////////////////////////////////////////
// NVM Code
//////////////////////////////////////////////////////////

void eeprom_in(uint8_t* data, uint16_t eeprom_adress, uint16_t bytes) {
  int i;
  for(i=0;i<bytes;i++){
    EEPROM.write(eeprom_adress++, *data++);
  }
}
void eeprom_out(uint16_t eeprom_adress, uint8_t* data, uint16_t bytes) {
  int i;
  for(i=0;i<bytes;i++){
    *(data++) = EEPROM.read(eeprom_adress++);
  }
}

void set_default_ram() {
  ram.BoardId = 0;
  ram.homeswitchnc = true;
  ram.stophomingonly = true; // If this is false the board can hang without proper hardware
  ram.maxSpeed = 63;
  ram.minSpeed = 42;
  ram.acceleration = 55;
  ram.currentMoving = 47;
  ram.currentHolding = 6;
  ram.stepspastsenorpos = 0;
  ram.stepspastsensorneg = 0;
  ram.structend = 0xA5;
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
  axis.SetParam(DECEL, ram.acceleration); // todo: 3 Ask Mike S. I think we use acceleration here becuase we do not have a decel
  axis.SetParam(TVAL_RUN, ram.currentMoving);
  axis.SetParam(TVAL_ACC, ram.currentMoving);
  axis.SetParam(TVAL_DEC, ram.currentMoving);
  axis.SetParam(TVAL_HOLD, ram.currentHolding);
}

//////////////////////////////////////////////////////////
// Stepper motor object
//////////////////////////////////////////////////////////

L6472 axis(ReadJumper(), 104, 103, 102, 18, 71, &safeToMove); //Board_ID, Response stream, MOSI, MISO, SCK, SS, RST
//L6472 axis(ReadJumper(), &Serial0, 104, 103, 102, 18, 71); //Board_ID, Response stream, MOSI, MISO, SCK, SS, RST
//L6472 axis(ReadJumper(), &Serial, 104, 103, 102, 18, 71); //Board_ID, Response stream, MOSI, MISO, SCK, SS, RST

//////////////////////////////////////////////////////////
// Read Board ID Jumppers
//////////////////////////////////////////////////////////

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

//////////////////////////////////////////////////////////
// Setup
//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////
// Setup Code
//////////////////////////////////////////////////////////

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

void setup_STP300() {
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
}

//////////////////////////////////////////////////////////
// Loops
//////////////////////////////////////////////////////////

void loop_stp300_homing()
{
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
}

void loop_stp300_homing_task(int id, void* tptr)
{
	loop_stp300_homing();
}

char spbuf[40];
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
    else if(parser.compare("wss")){
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
    else if(parser.compare("setparam")){
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
    else if(parser.compare("ram?")){
      snprintf(spbuf,sizeof(spbuf),"\r\n");           parser.print(spbuf);
      snprintf(spbuf,sizeof(spbuf),"           ram.BoardId: %d\r\n",ram.BoardId);           parser.print(spbuf);
      snprintf(spbuf,sizeof(spbuf),"      ram.homeswitchnc: %d\r\n",ram.homeswitchnc);      parser.print(spbuf);
      snprintf(spbuf,sizeof(spbuf),"    ram.stophomingonly: %d\r\n",ram.stophomingonly);    parser.print(spbuf);
      snprintf(spbuf,sizeof(spbuf),"          ram.maxSpeed: %d\r\n",ram.maxSpeed);          parser.print(spbuf);
      snprintf(spbuf,sizeof(spbuf),"          ram.minSpeed: %d\r\n",ram.minSpeed);          parser.print(spbuf);
      snprintf(spbuf,sizeof(spbuf),"      ram.acceleration: %d\r\n",ram.acceleration);      parser.print(spbuf);
      snprintf(spbuf,sizeof(spbuf),"     ram.currentMoving: %d\r\n",ram.currentMoving);     parser.print(spbuf);
      snprintf(spbuf,sizeof(spbuf),"    ram.currentHolding: %d\r\n",ram.currentHolding);    parser.print(spbuf);
      snprintf(spbuf,sizeof(spbuf)," ram.stepspastsenorpos: %d\r\n",ram.stepspastsenorpos); parser.print(spbuf);
      snprintf(spbuf,sizeof(spbuf),"ram.stepspastsensorneg: %d\r\n",ram.stepspastsensorneg);parser.print(spbuf);
      snprintf(spbuf,sizeof(spbuf),"         ram.structend: %x\r\n",ram.structend);         parser.print(spbuf);
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

void loop_stp300_serial_parser()
{
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

void loop_stp300_serial_parser_task(int id, void* tptr)
{
	loop_stp300_serial_parser();
}



//////////////////////////////////////////////////////////
// Reset (this should be part of the boards.c file
//////////////////////////////////////////////////////////

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


