/*
  Cruise control implementation. Uses PID controller in a closed-loop system.
  This version only controls throtle, not brakes.
  The objective of this program is to keep vehicle's speed as close as the value set by the driver.
  It can also keep RPM as set by the user via Serial, just for fun.
  Commands issued via Serial have precedence over speed set via RF control.

  Hardware:
  - Arduino Uno R3
  - HC-05 BlueTooth module (zs-040) (connects to OBDII)
  - HC-06 BlueTooth module (JY-MCU) (connects to a serial terminal for monitoring and command)
  - RF 315/433 MHz four button sender and receiver pair
  - ELM327 OBDII BlueTooth adapter (OBD to RS232 interpreter v1.5)
  - Servo motor connected to pull car's throtle (TowerPro MG956R 12kg)
  - Switch button on throtle pedal
  - Switch button on brake pedal
  - Automatic transmission car with standard OBDII connection (this is the most expensive hardware)

  Setup:
  - HC-05 config (AT mode). Should be run only once:
  AT+ORGL
  AT+ROLE=1
  AT+CMODE=0
  AT+BIND=0019,5D,24E54A

  Patches:
  - See OBD.h.patch (0001-Allow-use-of-SoftwareSerial-with-OBD.patch for Git)
    Allow use of SoftwareSerial port for BT communication with OBDII leaving Serial free
    to monitor and send commands.

  Author: Rogério Carvalho Schneider <stockrt@gmail.com>

  Tested on a Hyundai HB20 (Oct, 2016)
*/

#include <SoftwareSerial.h>
#include <Servo.h>
#include <Timer.h>
#include <OBD.h>

#define OBD_RxD 2 // Arduino pin connected to Tx of HC-05 (OBDII)
#define OBD_TxD 3 // Arduino pin connected to Rx of HC-05
//#define MONIT_RxD 4 // Arduino pin connected to Tx of HC-06 (MONIT)
//#define MONIT_TxD 5 // Arduino pin connected to Rx of HC-06
#define THROTLE_PIN 6
#define BRAKE_PIN 7
#define SERVO_PIN 8

int currentSPEED = 0;
int targetSPEED = 0;
int currentRPM = 0;
int targetRPM = 0;
int ethanol = -1;

int bridgeMode = 0;

// - Using INPUT_PULLUP semantics and wiring for pedal buttons:
// HIGH: driver is not using this pedal
// LOW: driver is pressing the pedal
int throtlePedalState = LOW;
int brakePedalState = LOW;

// Servo
int servoPosition = 0;

// - Control code
// 0: NO CONTROL
// 1: SPEED
// 2: RPM
int controlCode = 0;

// Objects
SoftwareSerial btSerial(OBD_RxD, OBD_TxD);
//SoftwareSerial btMonitSerial(MONIT_RxD, MONIT_TxD);
Servo servo;
Timer timer;
COBD obd;

void setup() {
  // Serial config
  Serial.begin(38400);

  // HC-05 communication to OBDII
  btSerial.begin(38400);
  pinMode(OBD_RxD, INPUT);
  pinMode(OBD_TxD, OUTPUT);

  // HC-06 communication to serial monitor
  //btMonitSerial.begin(9600);
  //pinMode(MONIT_RxD, INPUT);
  //pinMode(MONIT_TxD, OUTPUT);

  // Greetings
  serialPrintln("");
  serialPrintln("* Initializing...");

  // Pedal switches
  serialPrintln("* Initializing pedal switches...");
  pinMode(THROTLE_PIN, INPUT_PULLUP);
  pinMode(BRAKE_PIN, INPUT_PULLUP);

  // Servo for throtle control
  serialPrintln("* Initializing throtle servo...");
  pinMode(SERVO_PIN, OUTPUT);
  servo.attach(SERVO_PIN);
  servo.write(servoPosition);

  // BlueTooth connection should be estabilished automatically if we have configured HC-05 correctly
  serialPrintln("* Initializing OBDII BlueTooth connection...");
  waitBT();

  // OBDII
  serialPrintln("* Initializing OBDII library btSerial connection...");
  obd.begin();

  // Timers
  serialPrintln("* Initializing timers...");
  timer.every(100, readPedals);
  timer.every(300, readPIDs);
  timer.every(300, evaluateControl);
  timer.every(2000, showStatus);

  // Setup done
  serialPrintln("* System initialized!");
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
        serialPrintln("INFO: OBDII ready");
        break;
      } else {
        serialPrintln("WARN: OBDII not ready yet (waiting 1s to retry)");
      }
    } else {
      serialPrintln("WARN: No response from OBDII (waiting 1s to retry)");
    }

    delay(1000);
  }
}

void readPedals() {
  //serialPrintln("");
  //serialPrintln("*** Reading pedals ***");

  throtlePedalState = digitalRead(THROTLE_PIN);
  brakePedalState = digitalRead(BRAKE_PIN);

  if (throtlePedalState == LOW) {
    controlCode = 0;
    evaluateControl();
  }
  if (brakePedalState == LOW) {
    controlCode = 0;
    evaluateControl();
  }
}

void readPIDs() {
  if (bridgeMode) return;

  //serialPrintln("");
  //serialPrintln("*** Reading from ECU via OBDII ***");

  if (! obd.readPID(PID_SPEED, currentSPEED)) {
    serialPrintln("ERROR: Could not read SPEED from ECU");
    controlCode = 0;
    evaluateControl();
  }
  if (! obd.readPID(PID_RPM, currentRPM)) {
    serialPrintln("ERROR: Could not read RPM from ECU");
    controlCode = 0;
    evaluateControl();
  }
  if (! obd.readPID(PID_ETHANOL_FUEL, ethanol)) {
    serialPrintln("ERROR: Could not read ETHANOL_FUEL from ECU");
    controlCode = 0;
    evaluateControl();
  }
}

void evaluateControl() {
  //serialPrintln("");
  //serialPrintln("*** Evaluating control code ***");

  switch (controlCode) {
    case 0: // NO CONTROL
      servoPosition = 0;
      servo.write(servoPosition);
      break;
    case 1: // SPEED
      break;
    case 2: // RPM
      break;
    default: // NO CONTROL
      servoPosition = 0;
      servo.write(servoPosition);
      break;
  }
}

void showStatus() {
  if (bridgeMode) return;

  serialPrintln("");
  serialPrintln("*** STATUS ***");

  serialPrint("CONTROL code: ");
  serialPrint(String(controlCode));
  serialPrintln("");

  serialPrint("BRIDGE mode: ");
  serialPrint(String(bridgeMode));
  serialPrintln("");

  serialPrint("SERVO position: ");
  serialPrint(String(servoPosition));
  serialPrintln("");

  serialPrint("THROTLE pedal state: ");
  serialPrint(String(throtlePedalState));
  serialPrintln("");

  serialPrint("BRAKE pedal state: ");
  serialPrint(String(brakePedalState));
  serialPrintln("");

  serialPrint("SPEED (current/target): ");
  serialPrint(String(currentSPEED));
  serialPrint("/");
  serialPrint(String(targetSPEED));
  serialPrintln("");

  serialPrint("RPM (current/target): ");
  serialPrint(String(currentRPM));
  serialPrint("/");
  serialPrint(String(targetRPM));
  serialPrintln("");

  serialPrint("ETHANOL: ");
  serialPrint(String(ethanol));
  serialPrint("%");
  serialPrintln("");
}

void loop() {
  timer.update();

  String serialRecv;

  // Bridge / bridgeMode = 1 / b=1 (for debugging)
  if (bridgeMode) {
    //btSerial.listen();
    if (btSerial.available()) serialWrite(btSerial.read());
    if (Serial.available()) btSerial.write(Serial.read());
    //btMonitSerial.listen();
    //if (btMonitSerial.available()) btSerial.write(btMonitSerial.read());
  } // Bridge / bridgeMode = 1 / b=1

  // No bridge / bridgeMode = 0 / b=0 (default)
  else {
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
      //serialPrint("No command received");
    } else if (serialRecv.startsWith("s=")) {
      targetSPEED = serialRecv.substring(2).toInt();
      serialPrint("ACK command (target SPEED set): ");
      serialPrint(String(targetSPEED));
      serialPrint(" (");
      serialPrint(serialRecv);
      serialPrint(")");
      controlCode = 1;
      evaluateControl();
    } else if (serialRecv.startsWith("r=")) {
      targetRPM = serialRecv.substring(2).toInt();
      serialPrint("ACK command (target RPM set): ");
      serialPrint(String(targetRPM));
      serialPrint(" (");
      serialPrint(serialRecv);
      serialPrint(")");
      controlCode = 2;
      evaluateControl();
    } else if (serialRecv.startsWith("b=")) {
      bridgeMode = serialRecv.substring(2).toInt();
      serialPrint("ACK command (bridge mode set/unset): ");
      serialPrint(String(bridgeMode));
      serialPrint(" (");
      serialPrint(serialRecv);
      serialPrint(")");
      controlCode = 0;
      evaluateControl();
    } else if (serialRecv.startsWith("d=")) {
      serialPrint("ACK command (disable control): ");
      serialPrint("true");
      serialPrint(" (");
      serialPrint(serialRecv);
      serialPrint(")");
      controlCode = 0;
      evaluateControl();
    } else {
      serialPrint("ERROR: Unknown command: ");
      serialPrint(serialRecv);
      controlCode = 0;
      evaluateControl();
    }

    //btSerial.listen(); // HC-05 port can be can read
  } // No bridge / bridgeMode = 0 / b=0
}
