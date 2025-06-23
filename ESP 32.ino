#include <HardwareSerial.h>
#include <HX711.h>

// === UART2 Serial Communication ===
HardwareSerial SerialPort(2);
#define RXD2 16
#define TXD2 17

// === SN754410 Stepper Motor Driver (17HS4410 - Lead Screw) ===
const int IN1 = 12;  // SN754410 pin 2 
const int IN2 = 13;  // SN754410 pin 7 
const int IN3 = 14;  // SN754410 pin 10 
const int IN4 = 15;  // SN754410 pin 15 
const int stepDelay = 1000; // Using the working reference timing (microseconds)

// === Relay Control (Vacuum Pump) ===
const int RELAY_PIN = 2;

// === Stepper sequence for SN754410 (8-step half-step mode from working reference) ===
const int stepSequence[8][4] = {
  {1, 0, 0, 0}, {1, 0, 1, 0}, {0, 0, 1, 0}, {0, 1, 1, 0},
  {0, 1, 0, 0}, {0, 1, 0, 1}, {0, 0, 0, 1}, {1, 0, 0, 1}
};

// === Load Cell Configuration ===
#define SCK_PIN     18
#define DT1_PIN     19
#define DT2_PIN     21
#define DT3_PIN     22
#define DT4_PIN     23

// Scale calibration factors:
#define CALIBRATION_FACTOR_1 690.10
#define CALIBRATION_FACTOR_2 703.50
#define CALIBRATION_FACTOR_3 694.70
#define CALIBRATION_FACTOR_4 703.50

HX711 scale1, scale2, scale3, scale4;
float sensorData[4];

// === LEADSCREW SYSTEM (Updated for 8-step sequence) ===
// Motor: NEMA17 17HS4410 stepper motor (200 steps/rev, 1.8° per step)
// Driver: SN754410 in half-step mode (8-step sequence)
// 
// With 8-step half-step mode: 400 half-steps per revolution
// Leadscrew: T8 leadscrew (8mm pitch)
// - 8mm travel per revolution
// - Steps per mm = 400 half-steps/rev ÷ 8mm/rev = 50 half-steps/mm
const float STEPS_PER_MM = 50.0;  // 400 half-steps/rev ÷ 8mm pitch

// Travel parameters
const int MAX_TRAVEL_MM = 60;     // Maximum travel distance
int currentPositionMM = 0;        // Current position (0 = top/home)
int stepIndex = 0;                // Current step index (0-7 for 8-step sequence)

// === Function Prototypes ===
void initializeHardware();
void readAllSensors();
void sendSensorData();
void handleCommand(String command);
void moveToPosition(int targetMM);
void runMotorSteps(int steps, int direction);
void stepMotor(int index);
void enableVacuumPump();
void disableVacuumPump();
void returnToHome();
void dispensePills();

void setup() {
  Serial.begin(115200);
  SerialPort.begin(115200, SERIAL_8N1, RXD2, TXD2);
  
  Serial.println("ESP32 Fixed Leadscrew Controller Starting...");
  initializeHardware();
  Serial.println("System ready for commands");
}

void loop() {
  readAllSensors();

  if (SerialPort.available()) {
    String cmd = SerialPort.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() > 0) {
      handleCommand(cmd);
    }
  }

  delay(100);
}

void initializeHardware() {
  // SN754410 pins for leadscrew motor
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Initialize motor to OFF state
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  // Vacuum pump relay
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

  // Load cells
  scale1.begin(DT1_PIN, SCK_PIN);
  scale2.begin(DT2_PIN, SCK_PIN);
  scale3.begin(DT3_PIN, SCK_PIN);
  scale4.begin(DT4_PIN, SCK_PIN);

  scale1.set_scale(CALIBRATION_FACTOR_1);  scale1.tare(10);
  scale2.set_scale(CALIBRATION_FACTOR_2);  scale2.tare(10);
  scale3.set_scale(CALIBRATION_FACTOR_3);  scale3.tare(10);
  scale4.set_scale(CALIBRATION_FACTOR_4);  scale4.tare(10);
  
  Serial.println("Hardware initialized - Leadscrew using SN754410 8-step half-step mode");
}

void readAllSensors() {
  if (scale1.is_ready()) {
    sensorData[0] = scale1.get_units(10);
  }
  delay(10);

  if (scale2.is_ready()) {
    sensorData[1] = scale2.get_units(10);
  }
  delay(10);

  if (scale3.is_ready()) {
    sensorData[2] = scale3.get_units(10);
  }
  delay(10);

  if (scale4.is_ready()) {
    sensorData[3] = scale4.get_units(10);
  }
  delay(10);
  
  Serial.println(sensorData[0]);
  Serial.println(sensorData[1]);
  Serial.println(sensorData[2]);
  Serial.println(sensorData[3]);
}

void sendSensorData() {
  SerialPort.print("DATA,");
  for (int i = 0; i < 4; i++) {
    SerialPort.print(sensorData[i], 3);
    if (i < 3) SerialPort.print(",");
  }
  SerialPort.println();
}

void handleCommand(String command) {
  Serial.println("Received command: " + command);

  if (command == "REQ_HX711") {
    sendSensorData();
  } 
  else if (command == "PILLBOX_1") {
    dispensePills();
    SerialPort.println("DISPENSE_COMPLETE");
  }
  else if (command.startsWith("MOVE_")) {
    int targetMM = command.substring(5).toInt();
    if (targetMM >= 0 && targetMM <= MAX_TRAVEL_MM) {
      moveToPosition(targetMM);
      SerialPort.printf("MOVED_TO_%dmm\n", targetMM);
    } else {
      SerialPort.printf("ERROR: Position must be 0-%dmm\n", MAX_TRAVEL_MM);
    }
  }
  else if (command.startsWith("CALIBRATE_")) {
    int scaleNumber = command.substring(10).toInt();
    if (scaleNumber >= 1 && scaleNumber <= 4) {
      HX711* scale[] = {&scale1, &scale2, &scale3, &scale4};
      scale[scaleNumber - 1]->tare(10);
      SerialPort.println("CALIBRATED");
    }
  } 
  else if (command == "HOME") {
    returnToHome();
    SerialPort.println("HOME_COMPLETE");
  } 
  else if (command == "STATUS") {
    SerialPort.printf("STATUS,Position:%dmm,Vacuum:%s\n", 
                      currentPositionMM,
                      digitalRead(RELAY_PIN) ? "ON" : "OFF");
  }
  else if (command == "TEST_10MM") {
    Serial.println("Testing 10mm movement...");
    int testSteps = round(10 * STEPS_PER_MM);
    Serial.printf("10mm = %d half-steps\n", testSteps);
    
    Serial.println("Moving DOWN 10mm...");
    runMotorSteps(testSteps, 1);
    delay(2000);
    
    Serial.println("Moving UP 10mm...");
    runMotorSteps(testSteps, 0);
    SerialPort.println("TEST_COMPLETE");
  }
  else if (command == "TEST_FULL") {
    Serial.println("Testing full travel...");
    Serial.printf("Moving to %dmm...\n", MAX_TRAVEL_MM);
    moveToPosition(MAX_TRAVEL_MM);
    delay(2000);
    
    Serial.println("Returning to home...");
    returnToHome();
    SerialPort.println("FULL_TEST_COMPLETE");
  }
  else if (command == "VACUUM_ON") {
    enableVacuumPump();
    SerialPort.println("VACUUM_ON");
  }
  else if (command == "VACUUM_OFF") {
    disableVacuumPump();
    SerialPort.println("VACUUM_OFF");
  }
  else {
    /*
    SerialPort.println("ERROR: Unknown command");
    SerialPort.println("Available commands:");
    SerialPort.println("- DISPENSE: Full dispense cycle");
    SerialPort.println("- HOME: Return to top");
    SerialPort.println("- MOVE_<mm>: Move to position (0-300mm)");
    SerialPort.println("- TEST_10MM: Test 10mm movement");
    SerialPort.println("- TEST_FULL: Test full travel");
    SerialPort.println("- VACUUM_ON/OFF: Control vacuum");
    SerialPort.println("- STATUS: Get current status");
    */
  }
}

void dispensePills() {
  Serial.println("[DISPENSE] Starting pill dispensing sequence...");
  
  // Step 1: Enable vacuum pump
  enableVacuumPump();
  delay(500); // Allow vacuum to build up
  
  // Step 2: Move leadscrew down to pickup position
  Serial.printf("[DISPENSE] Moving down to %dmm pickup position...\n", MAX_TRAVEL_MM);
  moveToPosition(MAX_TRAVEL_MM);
  delay(1000); // Hold position for vacuum suction
  
  // Step 3: Move leadscrew back up to dispense position
  Serial.println("[DISPENSE] Moving up to dispense position...");
  returnToHome();
  delay(500);
  
  // Step 4: Disable vacuum pump to release pills
  disableVacuumPump();
  delay(1000); // Allow pills to fall
  
  Serial.println("[DISPENSE] Pill dispensing sequence complete");
}

void returnToHome() {
  Serial.println("[HOME] Returning to home position (0mm)...");
  moveToPosition(0);
  Serial.println("[HOME] Home position reached");
}

void moveToPosition(int targetMM) {
  // Clamp target to valid range
  if (targetMM < 0) targetMM = 0;
  if (targetMM > MAX_TRAVEL_MM) targetMM = MAX_TRAVEL_MM;
  
  int deltaMM = targetMM - currentPositionMM;
  if (deltaMM == 0) return;
  
  int direction = (deltaMM > 0) ? 1 : 0; // 1 = down, 0 = up
  int steps = round(abs(deltaMM) * STEPS_PER_MM);
  
  Serial.printf("[MOVE] Moving from %dmm to %dmm (delta: %dmm, %d half-steps)\n", 
                currentPositionMM, targetMM, deltaMM, steps);
  
  runMotorSteps(steps, direction);
  currentPositionMM = targetMM;
  
  Serial.printf("[MOVE] Reached position %dmm\n", currentPositionMM);
}

void runMotorSteps(int steps, int direction) {
  if (steps == 0) return;
  
  Serial.printf("[MOTOR] Moving %s for %d half-steps (~%.1fmm)\n",
                direction ? "DOWN" : "UP", steps, steps / STEPS_PER_MM);
  
  // Use the same logic as the working reference code
  for (int i = 0; i < steps; i++) {
    stepMotor(stepIndex);
    delayMicroseconds(stepDelay); // 1000us delay from working reference
    
    // Update step index using the same logic as working reference
    if (direction) {
      stepIndex = (stepIndex + 1) % 8;  // Forward through 8-step sequence
    } else {
      stepIndex = (stepIndex + 7) % 8;  // Backward (same as -1 mod 8)
    }
    
    // Add small delay every 50 steps to prevent missed steps
    if ((i + 1) % 50 == 0) {
      delay(1);
    }
  }
  
  // Turn off all coils to reduce heating
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  
  delay(10); // Allow motor to settle
  Serial.println("[MOTOR] Move complete");
}

void stepMotor(int index) {
  // Use the 8-step sequence from working reference
  digitalWrite(IN1, stepSequence[index][0]);
  digitalWrite(IN2, stepSequence[index][1]);
  digitalWrite(IN3, stepSequence[index][2]);
  digitalWrite(IN4, stepSequence[index][3]);
}

void enableVacuumPump() {
  digitalWrite(RELAY_PIN, HIGH);
  Serial.println("[VACUUM] Pump enabled");
}

void disableVacuumPump() {
  digitalWrite(RELAY_PIN, LOW);
  Serial.println("[VACUUM] Pump disabled");
}
