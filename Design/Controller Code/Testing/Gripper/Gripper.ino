#include <Servo.h>

// ===================== PIN DEFINITIONS =====================
// Arduino -> Robot (these connect to robot DI[0], DI[1], DI[2])
const int PIN_TO_ROBOT_START_CYCLE   = 22;  // -> Robot DI[0]
const int PIN_TO_ROBOT_GRIPPER_CLOSED = 23; // -> Robot DI[1]
const int PIN_TO_ROBOT_GRIPPER_OPENED = 24; // -> Robot DI[2]

// Robot -> Arduino (these connect from robot DO[0], DO[1], DO[2])
const int PIN_FROM_ROBOT_AT_PICKUP   = 26;  // <- Robot DO[0]
const int PIN_FROM_ROBOT_AT_PLACE    = 27;  // <- Robot DO[1]
const int PIN_FROM_ROBOT_CYCLE_DONE  = 28;  // <- Robot DO[2]

// Gripper servo (must be a PWM-capable pin)
const int PIN_GRIPPER_SERVO = 9;

// ===================== GRIPPER SETTINGS (TUNE THESE) =====================
const int GRIPPER_OPEN_ANGLE   = 90;   // degrees - TODO: tune
const int GRIPPER_CLOSED_ANGLE = 30;   // degrees - TODO: tune
const unsigned long GRIPPER_MOVE_TIME_MS = 800;  // time to let servo finish moving - TODO: tune

// ===================== TIMING SETTINGS =====================
const unsigned long SIGNAL_DEBOUNCE_MS = 50;  // signal must be stable this long to count
const unsigned long OUTPUT_PULSE_SETTLE_MS = 20;

Servo gripperServo;

// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);

  // Outputs to robot: start LOW so robot doesn't see a false trigger at boot
  pinMode(PIN_TO_ROBOT_START_CYCLE, OUTPUT);
  pinMode(PIN_TO_ROBOT_GRIPPER_CLOSED, OUTPUT);
  pinMode(PIN_TO_ROBOT_GRIPPER_OPENED, OUTPUT);
  clearAllSignalsToRobot();

  // Inputs from robot (use external pull-downs / your level shifter so they never float)
  pinMode(PIN_FROM_ROBOT_AT_PICKUP, INPUT);
  pinMode(PIN_FROM_ROBOT_AT_PLACE, INPUT);
  pinMode(PIN_FROM_ROBOT_CYCLE_DONE, INPUT);

  gripperServo.attach(PIN_GRIPPER_SERVO);
  openGripper();  // known starting state

  Serial.println("Arduino ready.");
}

// ===================== MAIN LOOP =====================
void loop() {
  // Make sure the robot has reset its outputs from the previous cycle
  waitForRobotSignalLow(PIN_FROM_ROBOT_AT_PICKUP);
  waitForRobotSignalLow(PIN_FROM_ROBOT_AT_PLACE);
  waitForRobotSignalLow(PIN_FROM_ROBOT_CYCLE_DONE);

  // 1. Wait for whatever condition says "start a cycle" (REED SWITCH)
  Serial.println("Waiting for start condition...");
  waitForStartCondition();

  // 2. Tell robot to begin (robot's Wait DI[0])
  Serial.println("Signaling robot: start cycle");
  sendSignalToRobot(PIN_TO_ROBOT_START_CYCLE);

  // 3. Robot reaches pickup point and sends DO[0]
  waitForRobotSignalHigh(PIN_FROM_ROBOT_AT_PICKUP);
  Serial.println("Robot at pickup -> closing gripper");
  closeGripper();
  sendSignalToRobot(PIN_TO_ROBOT_GRIPPER_CLOSED);  // robot's Wait DI[1]

  // 4. Robot reaches place point and sends DO[1]
  waitForRobotSignalHigh(PIN_FROM_ROBOT_AT_PLACE);
  Serial.println("Robot at place -> opening gripper");
  openGripper();
  sendSignalToRobot(PIN_TO_ROBOT_GRIPPER_OPENED);  // robot's Wait DI[2]

  // 5. Robot returns home and sends DO[2]
  waitForRobotSignalHigh(PIN_FROM_ROBOT_CYCLE_DONE);
  Serial.println("Cycle complete");

  // 6. Drop all outputs so robot's next "Wait DI[0]" doesn't fire early
  clearAllSignalsToRobot();
}

// ===================== PLACEHOLDER: START CONDITION =====================
// Replace the body with your real trigger (REED SWITCH, PLATFORM IN POSITION)
void waitForStartCondition() {
  // EXAMPLE PLACEHOLDER: wait 3 seconds
  delay(3000);
}

// ===================== GRIPPER FUNCTIONS =====================
void openGripper() {
  gripperServo.write(GRIPPER_OPEN_ANGLE);
  delay(GRIPPER_MOVE_TIME_MS);
}

void closeGripper() {
  gripperServo.write(GRIPPER_CLOSED_ANGLE);
  delay(GRIPPER_MOVE_TIME_MS);
}

// ===================== ROBOT COMMUNICATION HELPERS =====================
// Raise an output to the robot and leave it HIGH until clearAllSignalsToRobot()
void sendSignalToRobot(int pin) {
  digitalWrite(pin, HIGH);
  delay(OUTPUT_PULSE_SETTLE_MS);
}

void clearAllSignalsToRobot() {
  digitalWrite(PIN_TO_ROBOT_START_CYCLE, LOW);
  digitalWrite(PIN_TO_ROBOT_GRIPPER_CLOSED, LOW);
  digitalWrite(PIN_TO_ROBOT_GRIPPER_OPENED, LOW);
}

// Blocks until pin has been HIGH continuously for SIGNAL_DEBOUNCE_MS
void waitForRobotSignalHigh(int pin) {
  waitForStableState(pin, HIGH);
}

// Blocks until pin has been LOW continuously for SIGNAL_DEBOUNCE_MS
void waitForRobotSignalLow(int pin) {
  waitForStableState(pin, LOW);
}

void waitForStableState(int pin, int targetState) {
  unsigned long stableSince = 0;
  bool timing = false;

  while (true) {
    if (digitalRead(pin) == targetState) {
      if (!timing) {
        timing = true;
        stableSince = millis();
      } else if (millis() - stableSince >= SIGNAL_DEBOUNCE_MS) {
        return;
      }
    } else {
      timing = false;
    }
  }
}