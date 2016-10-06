/*
  Cruise control implementation. Uses PID controller in a closed-loop system.
  This version only controls throtle, no brakes.

  Hardware:
  - Arduino Uno R3
  - HC-05 BlueTooth module (zs-040) (connects to OBDII)
  - HC-06 BlueTooth module (JY-MCU) (connects to a serial terminal for monitoring and command)
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

#define OBD_RxD 2 // Arduino pin connected to Tx of HC-05 (OBD)
#define OBD_TxD 3 // Arduino pin connected to Rx of HC-05
//#define MONIT_RxD 4 // Arduino pin connected to Tx of HC-06 (MONIT)
//#define MONIT_TxD 5 // Arduino pin connected to Rx of HC-06

int targetVelocity = 0;
int targetRPM = 0;

SoftwareSerial btSerial(OBD_RxD, OBD_TxD);
//SoftwareSerial btMonitSerial(MONIT_RxD, MONIT_TxD);
Timer t;

void setup() {
  // Serial config
  Serial.begin(38400);
  btSerial.begin(38400);
  //btMonitSerial.begin(9600);

  serialPrintln("Initializing...");

  // HC-05 communication to OBDII
  pinMode(OBD_RxD, INPUT);
  pinMode(OBD_TxD, OUTPUT);

  // HC-06 communication to serial monitor
  //pinMode(MONIT_RxD, INPUT);
  //pinMode(MONIT_TxD, OUTPUT);

  // BlueTooth connection should be estabilished automatically if we have configured HC-05 correctly
  waitBT();

  t.every(500, cruiseControl);

  serialPrintln("System initialized");
}

// Wait until we can communicate with OBDII adapter via HC-05 BlueTooth module
void waitBT() {
  String btRecv;
  int count;

  //btSerial.listen();
  while (true) {
    btSerial.flush();
    btSerial.println("ATZ");

    count = 0;
    while (count < 200) { // Wait response from OBDII for two seconds at most
      if (btSerial.available()) {
        break;
      }
      count++;
      delay(10);
    }

    if (btSerial.available()) {
      btRecv = btSerial.readString();

      if (btRecv.indexOf("ELM327") != -1) {
        btSerial.flush();
        serialPrintln("OBDII ready");
        break;
      } else {
        serialPrintln("OBDII not ready yet (waiting 1s to retry)");
      }
    } else {
      serialPrintln("No response from OBDII (waiting 1s to retry)");
    }

    delay(1000);
  }
}

void serialWrite(char msg) {
  Serial.write(msg);
  //btMonitSerial.write(msg);
}

void serialPrint(String msg) {
  Serial.print(msg);
  //btMonitSerial.print(msg);
}

void serialPrintln(String msg) {
  Serial.println(msg);
  //btMonitSerial.println(msg);
}

void cruiseControl() {
  serialPrintln("Cruising");
}

void loop() {
  t.update();

  // Bridge mode (for debugging)
  //btSerial.listen();
  //if (btSerial.available()) serialWrite(btSerial.read());
  //if (Serial.available()) btSerial.write(Serial.read());
  //btMonitSerial.listen();
  //if (btMonitSerial.available()) btSerial.write(btMonitSerial.read());

  String serialRecv;
  serialRecv = "empty";

  // Accept commands from serial terminals
  //btMonitSerial.listen(); // HC-06 port can be read now
  if (Serial.available()) {
    serialRecv = Serial.readString();
    serialRecv.trim();
  }
  //} else
  //if (btMonitSerial.available()) {
  //  serialRecv = btMonitSerial.readString();
  //}

  // Commands received
  if (serialRecv == "empty") {
    serialPrint("No command received");
  } else if (serialRecv.startsWith("v=")) {
    targetVelocity = serialRecv.substring(2).toInt();
    serialPrint("ACK command (target velocity set): ");
    serialPrint(String(targetVelocity));
    serialPrint(" (");
    serialPrint(serialRecv);
    serialPrint(")");
  } else if (serialRecv.startsWith("r=")) {
    targetRPM = serialRecv.substring(2).toInt();
    serialPrint("ACK command (target RPM set): ");
    serialPrint(String(targetRPM));
    serialPrint(" (");
    serialPrint(serialRecv);
    serialPrint(")");
  } else {
    serialPrint("Unknown command: ");
    serialPrint(serialRecv);
  }

  //btSerial.listen(); // HC-05 port can be can read

  serialPrintln("");
  serialPrint("Target velocity: ");
  serialPrintln(String(targetVelocity));
  serialPrint("Target RPM: ");
  serialPrintln(String(targetRPM));
  serialPrintln("");

  delay(2000);
}
