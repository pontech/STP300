#ifndef _Half_Duplex_Turnaround_H_
#define _Half_Duplex_Turnaround_H_

#if (ARDUINO >= 100)
#define RETURN_TYPE size_t
#define RETURN_STATEMENT return n;
#define RETURN_SAVE_VALUE size_t n =
#else
#define RETURN_TYPE void
#define RETURN_STATEMENT
#define RETURN_SAVE_VALUE
#endif
#include <wprogram.h>
#include <HardwareSerial.h>

class SerialHalf : public Stream {
  protected:
  unsigned char _turnaroudpin;
  bool _normaly_low;
  public:
  HardwareSerial& _thisSerial;
  SerialHalf(HardwareSerial* thisSerial, unsigned char turnaroudpin, bool normaly_low):
    _thisSerial(*thisSerial)
  {
    _turnaroudpin = turnaroudpin;
    _normaly_low = normaly_low;
  };
  void begin(unsigned long baudRate)
  {
    _thisSerial.begin(baudRate);
    pinMode(_turnaroudpin,OUTPUT);
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
  };
  RETURN_TYPE write(uint8_t in) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    RETURN_SAVE_VALUE _thisSerial.write(in);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
    RETURN_STATEMENT
  };
  RETURN_TYPE write(const char *str) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    RETURN_SAVE_VALUE _thisSerial.write(str);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
    RETURN_STATEMENT
  };
  RETURN_TYPE write(const uint8_t *buffer, size_t size) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    RETURN_SAVE_VALUE _thisSerial.write(buffer,size);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
    RETURN_STATEMENT
  };
  
  int available() {
    return _thisSerial.available();
  }
  
  int read() {
    return _thisSerial.read();
  }

  int peek() {
    return _thisSerial.peek();
  }

  void flush() {
    _thisSerial.flush();
  }
  
  RETURN_TYPE print(const String &in) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    RETURN_SAVE_VALUE _thisSerial.print(in);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
    RETURN_STATEMENT
  };
  RETURN_TYPE print(const char* in) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    RETURN_SAVE_VALUE _thisSerial.print(in);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,((HIGH^_normaly_low)&1));
    RETURN_STATEMENT
  };
  RETURN_TYPE print(char in, int base) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    RETURN_SAVE_VALUE _thisSerial.print(in, base);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
    RETURN_STATEMENT
  };
  RETURN_TYPE print(unsigned char in, int base) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    RETURN_SAVE_VALUE _thisSerial.print(in,base);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
    RETURN_STATEMENT
  };
  RETURN_TYPE print(int in, int base) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    RETURN_SAVE_VALUE _thisSerial.print(in,base);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
    RETURN_STATEMENT
  };
  RETURN_TYPE print(unsigned int in, int base) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    RETURN_SAVE_VALUE _thisSerial.print(in,base);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
    RETURN_STATEMENT
  };
  RETURN_TYPE print(long in, int base) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    RETURN_SAVE_VALUE _thisSerial.print(in,base);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
    RETURN_STATEMENT
  };
  RETURN_TYPE print(unsigned long in, int base) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    RETURN_SAVE_VALUE _thisSerial.print(in,base);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
    RETURN_STATEMENT
  };
  RETURN_TYPE print(double in, int type = 2) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    RETURN_SAVE_VALUE _thisSerial.print(in,type);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
    RETURN_STATEMENT
  };
  
  RETURN_TYPE println(const String &s) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    delayMicroseconds(5);
    RETURN_SAVE_VALUE _thisSerial.println(s);
    waitForTransmitToComplete();
    delay(100);
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
    RETURN_STATEMENT
  };
  RETURN_TYPE println(const char* in) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    RETURN_SAVE_VALUE _thisSerial.println(in);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
    RETURN_STATEMENT
  };
  RETURN_TYPE println(char in, int base) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    RETURN_SAVE_VALUE _thisSerial.println(in,base);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
    RETURN_STATEMENT
  };
  RETURN_TYPE println(unsigned char in, int base) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    RETURN_SAVE_VALUE _thisSerial.println(in,base);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
    RETURN_STATEMENT;
  };
  RETURN_TYPE println(int in, int base) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    RETURN_SAVE_VALUE _thisSerial.println(in,base);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
    RETURN_STATEMENT
  };
  RETURN_TYPE println(unsigned int in, int base) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    RETURN_SAVE_VALUE _thisSerial.println(in,base);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
    RETURN_STATEMENT
  };
  RETURN_TYPE println(long in, int base) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    RETURN_SAVE_VALUE _thisSerial.println(in,base);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
    RETURN_STATEMENT
  };
  RETURN_TYPE println(unsigned long in, int base) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    RETURN_SAVE_VALUE _thisSerial.println(in,base);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
    RETURN_STATEMENT
  };
  RETURN_TYPE println(double in, int digits = 2) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    RETURN_SAVE_VALUE _thisSerial.println(in,digits);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
    RETURN_STATEMENT
  };
  void waitForTransmitToComplete()
  {
     _thisSerial.flush();
     //delayMicroseconds(1);
     asm("nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n");
     asm("nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n");
     asm("nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n");
     asm("nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n");
  };
};

#endif
