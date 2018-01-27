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
  size_t write(uint8_t in) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    size_t n = _thisSerial.write(in);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
    return n;
  };
  size_t write(const char *str) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    size_t n = _thisSerial.write(str);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
    return n;
  };
  size_t write(const uint8_t *buffer, size_t size) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    size_t n = _thisSerial.write(buffer,size);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
    return n;
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
  
  size_t print(const String &in) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    size_t n = _thisSerial.print(in);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
    return n;
  };
  size_t print(const char* in) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    size_t n = _thisSerial.print(in);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,((HIGH^_normaly_low)&1));
    return n;
  };
  size_t print(char in, int base) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    size_t n = _thisSerial.print(in, base);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
    return n;
  };
  size_t print(unsigned char in, int base) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    size_t n = _thisSerial.print(in,base);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
    return n;
  };
  size_t print(int in, int base) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    size_t n = _thisSerial.print(in,base);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
    return n;
  };
  size_t print(unsigned int in, int base) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    size_t n = _thisSerial.print(in,base);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
    return n;
  };
  size_t print(long in, int base) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    size_t n = _thisSerial.print(in,base);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
    return n;
  };
  size_t print(unsigned long in, int base) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    size_t n = _thisSerial.print(in,base);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
    return n;
  };
  size_t print(double in, int type = 2) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    size_t n = _thisSerial.print(in,type);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
    return n;
  };
  
  size_t println(const String &s) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    delayMicroseconds(5);
    size_t n = _thisSerial.println(s);
    waitForTransmitToComplete();
    delay(100);
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
    return n;
  };
  size_t println(const char* in) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    size_t n = _thisSerial.println(in);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
    return n;
  };
  size_t println(char in, int base) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    size_t n = _thisSerial.println(in,base);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
    return n;
  };
  size_t println(unsigned char in, int base) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    size_t n = _thisSerial.println(in,base);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
    return n;
  };
  size_t println(int in, int base) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    size_t n = _thisSerial.println(in,base);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
    return n;
  };
  size_t println(unsigned int in, int base) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    size_t n = _thisSerial.println(in,base);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
    return n;
  };
  size_t println(long in, int base) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    size_t n = _thisSerial.println(in,base);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
    return n;
  };
  size_t println(unsigned long in, int base) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    size_t n = _thisSerial.println(in,base);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
    return n;
  };
  size_t println(double in, int digits = 2) {
    digitalWrite(_turnaroudpin,LOW^_normaly_low);
    size_t n = _thisSerial.println(in,digits);
    waitForTransmitToComplete();
    digitalWrite(_turnaroudpin,HIGH^_normaly_low);
    return n;
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
