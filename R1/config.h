#include "QuadOmmniCytron.h"
#include<Ps3Controller.h>
#include<analogWrite.h>
#include <PID_v1.h>
#include <Encoder.h>
#include <Ticker.h>
#include "RoboClaw.h"

//Cytron Motor
#define load 0b00000000
#define CIM 0b00001000
#define address 0x80
#define encPPR 10000
//#define DEBUG
#define READING
#define endl "\n"

template<class T> inline Print& operator<<(Print& serial, T args) {
  serial.print(args);
  return serial;
}
void controlCIM();

//Hercules
RoboClaw roboclaw(&Serial2, 10000);
Encoder enc(18, 19);
Ticker call;
CytronDriver CLoad(&Serial1, load);
CytronDriver CCIM(&Serial1, CIM);

byte dataFromESP;
int battery = 0, player = 0;
unsigned long psMillis;
float angleM2 = 3925, angleM1 = 6090, posM2 = 0, posM1 = 0, reading = 0;
boolean status_CIM = false, initiate = true, decimal = false;

// speed for storing current speed(rpm),
float speed;
int Cspeed = 65;

// encPos - for current encoder position, oldEncPos - for last encoder position before given time
int encPos = 0, oldEncPos = 0;

//vFilt - for storing filtered speed
double vFilt = 0, vPrev = 0;
float Kp = 0.5, Kd = 0.05, Ki = 2;
double input, output, setpoint = 100;
PID controlVelocity(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

char data;

float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void homing() {
  roboclaw.BackwardM1(address, 50);
  roboclaw.BackwardM2(address, 96);
  delay(3000);
  roboclaw.ResetEncoders(address);
  roboclaw.SpeedAccelDeccelPositionM1(address, 40000, 32000, 40000, angleM1, 1);
  roboclaw.SpeedAccelDeccelPositionM2(address, 65000, 32000, 65000, angleM2, 1);
  delay(2000);
}

void LagoriBrake(){
  controlCIM();
  delay(2000);
  CLoad.antiClockwise(100);
  delay(300);
  CLoad.brake();
  delay(2000);
  CLoad.antiClockwise(100);
  delay(300);
  CLoad.brake();
  delay(2000);
  CLoad.antiClockwise(100);
  delay(300);
  CLoad.brake();
  delay(500);
  controlCIM();
}
  
//Setting encoder position according to angle for M1
void setEncoderM1(float angleM1) {
  roboclaw.SpeedAccelDeccelPositionM1(address, 65000, 32000, 65000, angleM1, 1);
#ifdef DEBUG
  Serial << "Turret Changes..." << endl;
#endif
}

void turretAntiClock() {
  angleM1 = angleM1 > 0 ? angleM1 - 15 : 0;
  setEncoderM1(angleM1);
}

void turretClock() {
  angleM1 = angleM1 < 10340 ? angleM1 + 15 : 10340;
  setEncoderM1(angleM1);
}


void setEncoderM2(float angleM2) {
  roboclaw.SpeedAccelDeccelPositionM2(address, 65000, 32000, 65000, angleM2, 1);
#ifdef DEBUG
  Serial << "Turret Changes..." << endl;
#endif
}

void decrAngle() {
  angleM2 = angleM2 < 11000 ? angleM2 + 35 : 11000;
  if (angleM2 > 11000) {
    angleM2 = 11000;
  }
  setEncoderM2(angleM2);
}

void incrAngle() {
  angleM2 = angleM2 > -1 ? angleM2 - 35 : 0;
  if (angleM2 < 0) {
    angleM2 = 0;
  }
  setEncoderM2(angleM2);
}

void controlCIM() {
  if (!status_CIM) {
    status_CIM = true;
    for (int i = 10; i <= Cspeed; i += 5) {
      CCIM.antiClockwise(i);
      delay(60);
    }
    setEncoderM2(angleM2);
#ifdef DEBUG
    Serial << "CIM Started..." << endl;
#endif
  } else {
    status_CIM = false;
    for (int i = Cspeed; i > -1; i -= 10) {
      CCIM.antiClockwise(i);
      delay(60);
    }
    setEncoderM2(angleM2);
    CCIM.brake();
#ifdef DEBUG
    Serial << "CIM Stopped..." << endl;
#endif
  }
}

void decreaseCIM() {
  if (status_CIM) {
    Cspeed = Cspeed > 49 ? Cspeed - 5 : 50;
    if (Cspeed < 49) {
      Cspeed = 50;
    }
#ifdef DEBUG
    Serial << "Turret Speed Decrease..." << endl;
#endif
    CCIM.antiClockwise(Cspeed);
  }
}

void increaseCIM() {
  if (status_CIM) {
    Cspeed = Cspeed < 100 ? Cspeed + 5 : 100;
    if (Cspeed > 101) {
      Cspeed = 100;
    }
#ifdef DEBUG
    Serial << "Turret Speed Increase..." << endl;
#endif
    CCIM.antiClockwise(Cspeed);
  }
}

void onPSDisconnect() {
  CLoad.brake();
  CCIM.brake();
}

void getSpeed() {// Calculating speed = (Encoder Pulse Diff in given time) * (given time(sec))/(Encoder PPR);
  encPos = enc.read();
  speed = (float)(encPos - oldEncPos) * 6000 / encPPR;
  vFilt = 0.969 * vFilt + 0.0155 * speed + 0.0155 * vPrev;
  vPrev = speed;
  oldEncPos = encPos;
}

void setVelocity(float speed) {
  setpoint = speed;
  input = vFilt;
  controlVelocity.Compute();
  if (vFilt >= 0)
    CCIM.clockwise(output);
  else
    CCIM.antiClockwise(output);
}
