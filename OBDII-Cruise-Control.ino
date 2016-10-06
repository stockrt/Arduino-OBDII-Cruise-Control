/*
  Cruise control implementation.
  Uses PID controller in a closed-loop system.

  Hardware:
  - Arduino Uno R3
  - HC-05 BlueTooth module (zs-040)
  - ELM327 OBDII BlueTooth adapter (OBD to RS232 interpreter v1.5)
  - Servo motor connected to pull car's throtle (TowerPro MG956R 12kg)
  - Switch button on throtle
  - Switch button on brake
  - Automatic transmission car with standard OBDII connection (this is the most expensive hardware)

  Setup:
  - HC-05 config (AT mode). Should be run only once:
  AT+ORGL
  AT+ROLE=1
  AT+CMODE=0
  AT+BIND=0019,5D,24E54A

  Author: Rogério Carvalho Schneider <stockrt@gmail.com>

  Tested on a Hyundai HB20 (Oct, 2016)
*/

#include <SoftwareSerial.h>
#include <Timer.h>

#define RxD 7 // Arduino pin connected to Tx of HC-05
#define TxD 8 // Arduino pin connected to Rx of HC-05

SoftwareSerial btSerial(RxD, TxD);
Timer t;

void setup() {
  // Serial config
  Serial.begin(38400);
  btSerial.begin(38400);

  Serial.println("Initializing...");

  // HC-05 communication
  pinMode(RxD, INPUT);
  pinMode(TxD, OUTPUT);

  // BlueTooth connection should be estabilished automatically if we have configured HC-05 correctly
  waitBT();

  t.every(500, cruiseControl);

  Serial.println("System initialized");
}

// Wait until we can communicate with OBDII adapter via HC-05 BlueTooth module
void waitBT() {
  String recv;
  int count;

  while (true) {
    btSerial.flush();
    btSerial.println("ATZ");

    count = 0;
    while (count < 200) { // Wait response from OBDII for two seconds at most
      if (btSerial.available()) break;
      count++;
      delay(10);
    }

    if (btSerial.available()) {
      recv = btSerial.readString();

      if (recv.indexOf("ELM327") != -1) {
        btSerial.flush();
        Serial.println("OBDII ready");
        break;
      } else {
        Serial.println("OBDII not ready yet (waiting 1s to retry)");
      }
    } else {
      Serial.println("No response from OBDII (waiting 1s to retry)");
    }

    delay(1000);
  }
}

void cruiseControl() {
  Serial.println("Cruising");
}

void loop() {
  t.update();

  if (btSerial.available()) Serial.write(btSerial.read());
  if (Serial.available()) btSerial.write(Serial.read());
}
