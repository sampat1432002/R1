/*
* @FileName : CytronDriver
* @Author : Shreyash Patel 
* @Version : 2.0
*/


#include"CytronDriver.h"
CytronDriver::CytronDriver(HardwareSerial* _serial, byte _address){
  serial = _serial;
  address = _address;
}
void CytronDriver::initiate(){
  delay(1000);
  serial->write(dummy);
}

void CytronDriver::clockwise(byte speeed){
  command = map(speeed, 0, 100, 127, 255);
  send_data(command);
}

void CytronDriver::antiClockwise(byte speeed){
  command = map(speeed, 0, 100, 127, 0);
  send_data(command);
}
void CytronDriver::brake()
{
  command = 127;
  send_data(command);
}

void CytronDriver::send_data(byte command){
  checksum = header + address + command;
  serial->write(header);
  serial->write(address);
  serial->write(command);
  serial->write(checksum);
}
