#ifndef CYTRONDRIVER_H
#define CYTRONDRIVER_H

#include<Arduino.h>
#define header 85
#define dummy 128


class CytronDriver {
  private:
    HardwareSerial* serial = NULL;
    byte address, command, checksum;
    void send_data(byte command);
  public:
    CytronDriver(HardwareSerial* _serial, byte _address);
    void initiate();
    void clockwise(byte speeed);
    void antiClockwise(byte speeed);
    void brake();
};

#endif
