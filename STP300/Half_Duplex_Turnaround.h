#if (ARDUINO >= 100)
#define returntype size_t
#define returnstatement return n;
#define returnsaveval size_t n =
#else
#define returntype void
#define returnstatement
#define returnsaveval
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
  returntype write(uint8_t in) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    returnsaveval _thisSerial.write(in);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
    returnstatement
  };
  returntype write(const char *str) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    returnsaveval _thisSerial.write(str);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
    returnstatement
  };
  returntype write(const uint8_t *buffer, size_t size) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    returnsaveval _thisSerial.write(buffer,size);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
    returnstatement
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
  
  returntype print(const String &in) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    returnsaveval _thisSerial.print(in);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
    returnstatement
  };
  returntype print(const char* in) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    returnsaveval _thisSerial.print(in);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,((HIGH^_normaly_low)&1));
    returnstatement
  };
  returntype print(char in, int base) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    returnsaveval _thisSerial.print(in, base);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
    returnstatement
  };
  returntype print(unsigned char in, int base) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    returnsaveval _thisSerial.print(in,base);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
    returnstatement
  };
  returntype print(int in, int base) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    returnsaveval _thisSerial.print(in,base);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
    returnstatement
  };
  returntype print(unsigned int in, int base) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    returnsaveval _thisSerial.print(in,base);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
    returnstatement
  };
  returntype print(long in, int base) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    returnsaveval _thisSerial.print(in,base);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
    returnstatement
  };
  returntype print(unsigned long in, int base) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    returnsaveval _thisSerial.print(in,base);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
    returnstatement
  };
  returntype print(double in, int type = 2) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    returnsaveval _thisSerial.print(in,type);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
    returnstatement
  };
  
  returntype println(const String &s) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    delayMicroseconds(5);
    returnsaveval _thisSerial.println(s);
    waitForTransmitToComplete();
    delay(100);
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
    returnstatement
  };
  returntype println(const char* in) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    returnsaveval _thisSerial.println(in);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
    returnstatement
  };
  returntype println(char in, int base) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    returnsaveval _thisSerial.println(in,base);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
    returnstatement
  };
  returntype println(unsigned char in, int base) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    returnsaveval _thisSerial.println(in,base);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
    returnstatement;
  };
  returntype println(int in, int base) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    returnsaveval _thisSerial.println(in,base);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
    returnstatement
  };
  returntype println(unsigned int in, int base) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    returnsaveval _thisSerial.println(in,base);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
    returnstatement
  };
  returntype println(long in, int base) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    returnsaveval _thisSerial.println(in,base);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
    returnstatement
  };
  returntype println(unsigned long in, int base) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    returnsaveval _thisSerial.println(in,base);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
    returnstatement
  };
  returntype println(double in, int digits = 2) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    returnsaveval _thisSerial.println(in,digits);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
    returnstatement
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
