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
  void write(uint8_t in) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    _thisSerial.write(in);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
  };
  void write(const char *str) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    _thisSerial.write(str);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
  };
  void write(const uint8_t *buffer, size_t size) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    _thisSerial.write(buffer,size);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
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
  
  void print(const String &in) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    _thisSerial.print(in);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
  };
  void print(const char* in) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    _thisSerial.print(in);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,((HIGH^_normaly_low)&1));
  };
  void print(char in, int type = PRINT_BYTE) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    _thisSerial.print(in, type);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
  };
  void print(unsigned char in, int type = PRINT_BYTE) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    _thisSerial.print(in,type);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
  };
  void print(int in, int type = PRINT_DEC) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    _thisSerial.print(in,type);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
  };
  void print(unsigned int in, int type = PRINT_DEC) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    _thisSerial.print(in,type);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
  };
  void print(long in, int type = PRINT_DEC) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    _thisSerial.print(in,type);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
  };
  void print(unsigned long in, int type = PRINT_DEC) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    _thisSerial.print(in,type);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
  };
  void print(double in, int type = 2) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    _thisSerial.print(in,type);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
  };
  
  void println(const String &s) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    delayMicroseconds(5);
    _thisSerial.println(s);
    waitForTransmitToComplete();
    delay(100);
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
  };
  void println(const char* in) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    _thisSerial.println(in);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
  };
  void println(char in, int type = PRINT_BYTE) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    _thisSerial.println(in,type);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
  };
  void println(unsigned char in, int type = PRINT_BYTE) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    _thisSerial.println(in,type);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
  };
  void println(int in, int type = PRINT_DEC) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    _thisSerial.println(in,type);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
  };
  void println(unsigned int in, int type = PRINT_DEC) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    _thisSerial.println(in,type);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
  };
  void println(long in, int type = PRINT_DEC) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    _thisSerial.println(in,type);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
  };
  void println(unsigned long in, int type = PRINT_DEC) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    _thisSerial.println(in,type);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
  };
  void println(double in, int type = 2) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    _thisSerial.println(in,type);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
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
