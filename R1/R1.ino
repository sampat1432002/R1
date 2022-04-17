#include "config.h"

void notify()
{
  if ( Ps3.data.button.up && initiate) {
    incrAngle();
#ifdef DEBUG
    Serial << "Turret Up..." << endl;
#endif
  }

  if ( Ps3.data.button.down && initiate) {
    decrAngle();
#ifdef DEBUG
    Serial << "Turret Down..." << endl;
#endif
  }

  if ( Ps3.data.button.left && initiate) {
    turretAntiClock();
#ifdef DEBUG
    Serial << "Turret Backward..." << endl;
#endif
  }

  if ( Ps3.data.button.right && initiate) {
    turretClock();
#ifdef DEBUG
    Serial << "Turret Backward..." << endl;
#endif
  }

  //CIM Speed Increase
  if ( Ps3.event.button_down.triangle && initiate) {
//    increaseCIM();
  }

  // CIM Speed Decrease
  if ( Ps3.event.button_down.cross && initiate) {
//    decreaseCIM();
  }

  //CIM Initiates
  if ( Ps3.event.button_down.circle && initiate) {
    controlCIM();
  }

  //Ball Loading
  if ( Ps3.event.button_down.square && initiate) {
    CLoad.antiClockwise(100);
#ifdef DEBUG
    Serial << "Loading..." << endl;
#endif
    delay(300);
    CLoad.brake();
#ifdef DEBUG
    Serial << "Brake..." << endl;
#endif
  }

  // For Using PS
  if ( Ps3.event.button_down.start && !initiate) {
    initiate = true;
#ifdef DEBUG
    Serial << "All Initiate..." << endl;
#endif
  } else if (Ps3.event.button_down.start && initiate) {
    initiate = false;
    onPSDisconnect();
#ifdef DEBUG
    Serial << "All Stops..." << endl;
#endif
  }

  //Distance Reading
  if (Ps3.event.button_down.select) {
    if (!decimal)
      decimal = true;
  }
  if (Ps3.event.button_down.r2) {
    reading += 1;
  }
  if (Ps3.event.button_down.l2) {
    if (!decimal)
      reading += 0.1;
    else
      reading += 0.01;
  }
  //Reading
  if (Ps3.event.button_down.l1) {
    if (reading != 0) {
#ifdef READING
      Serial << angleM1 << ", " << angleM2 << ", " << Cspeed << ", " << roboclaw.ReadMainBatteryVoltage(address) << ", No, " << reading << endl;
#endif
    } else {
#ifdef READING
      Serial << angleM1 << ", " << angleM2 << ", " << Cspeed << ", " << roboclaw.ReadMainBatteryVoltage(address) << ", No" << endl;
#endif
    }
    reading = 0;
    decimal = false;
  }
  if (Ps3.event.button_down.r1) {
    if (reading != 0) {
#ifdef READING
      Serial << angleM1 << ", " << angleM2 << ", " << Cspeed << ", " << roboclaw.ReadMainBatteryVoltage(address) << ", Yes, " << reading << endl;
#endif
    } else {
#ifdef READING
      Serial << angleM1 << ", " << angleM2 << ", " << Cspeed << ", " << roboclaw.ReadMainBatteryVoltage(address) << ", Yes" << endl;
#endif
    }
    reading = 0;
    decimal = false;
  }
  psMillis = millis();
}

void onConnect() {
#ifdef DEBUG
  Serial << "Connected." << endl;
#endif
  psMillis = millis();
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, 32, 26);
  Serial2.begin(115200, SERIAL_8N1, 14, 27);
  delay(1000);

  CLoad.initiate();

#ifdef DEBUG
  Serial << "Ready." << endl;
#endif

  //Setting PID
  controlVelocity.SetMode(AUTOMATIC);
  controlVelocity.SetOutputLimits(255, 0);
  //  call.attach_ms(10, getSpeed);

  Ps3.attach(notify);
  Ps3.attachOnConnect(onConnect);
  Ps3.begin("11:11:11:11:11:11");

  homing();
  //  delay(500);
  //  LagoriBrake();

#ifdef READING
  Serial << "EncPosM1, " << "EncPosM2, " << "Speed(in PWM), " << "Battery Voltage, " << "Remark, " << "Distance Reading"  << endl;
#endif

}

void loop() {
  setEncoderM1(angleM1);
  setEncoderM2(angleM2);
  //  if (millis() - psMillis > 50) {
  //    onPSDisconnect();
  //    return;
  //  }

  //  if (Serial.available()) {
  //    data = Serial.read();
  //    if (data == 'S') {
  //      while (!Serial.available());
  //      speed = Serial.parseFloat();
  //    } else if (data == 'P') {
  //      while (!Serial.available());
  //      Kp = Serial.parseFloat();
  //    } else if (data == 'D') {
  //      while (!Serial.available());
  //      Kd = Serial.parseFloat();
  //    } else if (data == 'I') {
  //      while (!Serial.available());
  //      Ki = Serial.parseFloat();
  //    } else if (data == 'R') {
  //      Kp = 0.5, Kd = 0.05, Ki = 2;
  //    }
  //  }

  //  setVelocity(speed);
  //  Serial.print(setpoint);
  //  Serial.print(" ");
  //  delay(10000);
  //  Serial.println(speed * 3);
  //  CCIM.antiClockwise(50);
  //  delay(10000);
  //  Serial.println(speed * 3);

  //  Serial.print(output);
  //  Serial.print(" ");
  //  Serial.print(Kp);
  //  Serial.print(" ");
  //  Serial.print(Kd);
  //  Serial.print(" ");
  //  Serial.print(Ki);
  //  Serial.print(" ");
  //  controlVelocity.SetTunings(Kp, Ki, Kd);

  //---------------------- Battery events ---------------------
  if ( battery != Ps3.data.status.battery ) {
    battery = Ps3.data.status.battery;
#ifdef DEBUG
    Serial.print("The controller battery is ");
#endif
    if ( battery == ps3_status_battery_charging )
    {
      player += 1;
      if (player > 8) player = 0;
#ifdef DEBUG
      Serial << "Charging" << endl;
#endif
    }
    else if ( battery == ps3_status_battery_full )
    {
      Ps3.setPlayer(10);
#ifdef DEBUG
      Serial << "FULL" << endl;
#endif
    }
    else if ( battery == ps3_status_battery_high )
    {
      Ps3.setPlayer(9);
#ifdef DEBUG
      Serial << "HIGH" << endl;
#endif
    }
    else if ( battery == ps3_status_battery_low)
    {
      Ps3.setPlayer(7);
#ifdef DEBUG
      Serial << "LOW" << endl;
#endif
    }
    else if ( battery == ps3_status_battery_dying )
    {
      Ps3.setPlayer(4);
#ifdef DEBUG
      Serial << "DYING" << endl;
#endif
    }
    else if ( battery == ps3_status_battery_shutdown ) {
#ifdef DEBUG
      Serial << "SHUTDOWN" << endl;
#endif
    } else {
#ifdef DEBUG
      Serial << "UNDEFINED" << endl;
#endif
    }
  }
}
