#include <HardwareSerial.h>
#include <HX711.h>
HardwareSerial SerialPort(2); // UART2
#define RXD2 16
#define TXD2 17

// A4988 (Motor 1)
const int stepPin1 = 26;
const int dirPin1 = 25;
const int stepDelay1 = 5000;  // µs

// SN754410 (Motor 2)
const int IN1 = 12;
const int IN2 = 13;
const int IN3 = 14;
const int IN4 = 15;
const int stepDelay2 = 3000;  // µs
const int stepSequence[8][4] = {
  {1, 0, 0, 0}, {1, 0, 1, 0}, {0, 0, 1, 0}, {0, 1, 1, 0},
  {0, 1, 0, 0}, {0, 1, 0, 1}, {0, 0, 0, 1}, {1, 0, 0, 1}
};


// ======== Load Cell Configuration ========
#define SCK_PIN     18
#define DT1_PIN     19
//#define DT2_PIN     21
//#define DT3_PIN     22
//#define DT4_PIN     23
//#define DT5_PIN     25
//#define DT6_PIN     26

HX711 scale1, scale2, scale3, scale4, scale5, scale6;
float sensorData[6];

void setup() {
  Serial.begin(115200);
  SerialPort.begin(115200, SERIAL_8N1, RXD2, TXD2);

  pinMode(stepPin1, OUTPUT);
  pinMode(dirPin1, OUTPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
    // Initialize load cells
  scale1.begin(DT1_PIN, SCK_PIN);
  //scale2.begin(DT2_PIN, SCK_PIN);
  //scale3.begin(DT3_PIN, SCK_PIN);
  //scale4.begin(DT4_PIN, SCK_PIN);
  //scale5.begin(DT5_PIN, SCK_PIN);
  //scale6.begin(DT6_PIN, SCK_PIN);
}

void loop() {
  readSensors();
  if (SerialPort.available()) {
    String cmd = SerialPort.readStringUntil('\n');
    cmd.trim();

    if (cmd == "REQ_HX711") {
      sendSensorData();
    }
    else if (cmd.startsWith("RUN_MOTOR1")) {
      int dir, steps;
      parseMotorCommand(cmd, dir, steps);
      runMotor1(dir, steps);
      SerialPort.println("ACK");
    }
    else if (cmd.startsWith("RUN_MOTOR2")) {
      int dir, steps;
      parseMotorCommand(cmd, dir, steps);
      runMotor2(dir, steps);
      SerialPort.println("ACK");
    }
  }
}

void parseMotorCommand(const String& cmd, int &dir, int &steps) {
  int firstComma = cmd.indexOf(',');
  int secondComma = cmd.indexOf(',', firstComma + 1);
  dir = cmd.substring(firstComma + 1, secondComma).toInt();
  steps = cmd.substring(secondComma + 1).toInt();
}

// === A4988 ===
void runMotor1(int direction, int steps) {
  digitalWrite(dirPin1, direction ? HIGH : LOW);
  for (int i = 0; i < steps; i++) {
    digitalWrite(stepPin1, HIGH);
    delayMicroseconds(stepDelay1);
    digitalWrite(stepPin1, LOW);
    delayMicroseconds(stepDelay1);
  }
}

// === SN754410 ===
void runMotor2(int direction, int steps) {
  int stepIndex = 0;
  for (int i = 0; i < steps; i++) {
    stepMotor(stepIndex);
    stepIndex = direction ? (stepIndex + 1) % 8 : (stepIndex + 7) % 8;
    delayMicroseconds(stepDelay2);
  }
}

void stepMotor(int index) {
  digitalWrite(IN1, stepSequence[index][0]);
  digitalWrite(IN2, stepSequence[index][1]);
  digitalWrite(IN3, stepSequence[index][2]);
  digitalWrite(IN4, stepSequence[index][3]);
}
void readSensors() {
  sensorData[0] = scale1.get_units(10);
  //sensorData[1] = scale2.get_units(10);
  //sensorData[2] = scale3.get_units(10);
  //sensorData[3] = scale4.get_units(10);
  //sensorData[4] = scale5.get_units(10);
  //sensorData[5] = scale6.get_units(10);
}
void sendSensorData() {
  SerialPort.print("DATA,");
  //for(int i = 0; i < 6; i++) {
    SerialPort.print(sensorData[0], 3);
    //if(i < 5) Serial2.print(",");
  //}
  SerialPort.println();
}