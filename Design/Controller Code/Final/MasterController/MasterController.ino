/* ============= MAIN PROGRAM FOR THE MASTER CONTROLLER ============= */
#include <Servo.h>
/* 
KEY THINGS TO NOTE: //Put these into a variable
- ..._DIR = HIGH = CW rotation: conveyor moves forward, blade goes down, claws open, and rolling pin spins the right way
*/

/* ============= PIN ASSIGNMENTS ============= */ 
const int LS1 = 2; // Limit Switch 1 (start of the extrusion bar)
const int LS2 = 3; // Limit Switch 2 (end of the extrusion bar)
const int RS1 = 4; // Reed Switch 1 (after the rice roller)
const int RS2 = 5; // Reed Switch 2 (at the slicing station)
const int MOT_DR_CONVEYOR_EN = 6; // EN bit for conveyor belt motor
const int MOT_DR_CONVEYOR_STEP = 7; // STEP bit for conveyor belt motor
const int MOT_DR_CONVEYOR_DIR = 8; // DIR bit for conveyor belt motor
const int SER_MOT_GRIPPER = 9; // Servo Motor signal
const int REL_IN1= 22; // Relay Channel 1 (Arduino -> Robot)
const int REL_IN2 = 23; // Relay Channel 2 (Arduino -> Robot)
const int REL_IN3 = 24; // Relay Channel 3 (Arduino -> Robot)
const int OPTO_V1 = 33; // Optocoupler Channel 1 (Robot -> Arduino)
const int OPTO_V2 = 34; // Optocoupler Channel 2 (Robot -> Arduino)
const int OPTO_V3 = 35; // Optocoupler Channel 3 (Robot -> Arduino)
const int START = 36; // Start Button 
const int RESET = 37; // Reset Button

/* ============= TUNING PARAMETERS ============= */
const unsigned long CONVEYOR_STEP_INTERVAL_1 = 900; // speed as conveyor belt moves forward from LS1 and RS1
const unsigned long CONVEYOR_STEP_INTERVAL_2 = 900; // speed as conveyor belt moves forward from RS1 to RS2
const unsigned long CONVEYOR_STEP_INTERVAL_3 = 900; // speed as conveyor belt moves forward from RS2 to end
const unsigned long CONVEYOR_STEP_INTERVAL_BACK = 900; // speed as conveyor belt moves backwards from LS2 to LS1
const int SERIAL_TIMEOUT = 50; // how long [ms] a Serial blocking function will wait before giving up
const unsigned long RESEND_INTERVAL = 500; // how long [ms] to wait before resending Serial command
const int GRIPPER_CLOSED_ANGLE = 2; // degrees at the gripper's closed position
const int GRIPPER_OPEN_ANGLE = 30; // degrees at the gripper's open position
const unsigned long GRIPPER_MOVE_TIME = 1500; // delay to account for the time for servo to move

/* ============= INITIALIZE VARIABLES/OBJECTS ============= */
Servo gripper;

#define MOT_DR_EN_ON LOW // a digital LOW signal to a motor's EN bit turns it ON
#define MOT_DR_EN_OFF HIGH // a digital HIGH signal to a motor's EN bit turns it OFF
#define MOT_DR_CONVEYOR_FWD HIGH // a digital HIGH signal to the conveyor motor's DIR bit moves it forwards
#define MOT_DR_CONVEYOR_BACK LOW // a digital LOW signal to the conveyor motor's DIR bit moves it backwards

bool ACTIVE_LS1 = false; // flag that tell you if limit switch 1 was met (active HIGH)
bool ACTIVE_LS2 = false; // flag that tell you if limit switch 2 was hit (active HIGH)
bool ACTIVE_RS1 = false; // flag that tell you if reed switch 1 was met (active LOW)
bool ACTIVE_RS2 = false; // flag that tell you if reed switch 2 was met (active LOW)
bool ACTIVE_ACK_START_RICE = false; // flag that tells you if the "ACK START RICE" message was received
bool ACTIVE_ACK_STOP_RICE = false; // flag that tells you if the "ACK STOP RICE" message was received
bool ACTIVE_ACK_CLOSE_CLAWS = false; // flag that tells you if the "ACK CLOSE CLAWS" message was received
bool ACTIVE_DONE_CLOSE_CLAWS = false; // flag that tells you if the "DONE CLOSE CLAWS" message was received
bool ACTIVE_ACK_OPEN_CLAWS = false; // flag that tells you if the "ACK OPEN CLAWS" message was received
bool ACTIVE_DONE_OPEN_CLAWS = false; // flag that tells you if the "DONE OPEN CLAWS" message was received
bool ACTIVE_ACK_START_SLICER = false; // flag that tells you if the "ACK START SLICER" message was received
bool ACTIVE_DONE_SLICER = false; // flag that tells you if the "DONE SLICER" message was received
bool ACTIVE_OPTO_V1 = false; // flag that tells you if the OPTO_V1 signal from the robot was received (active LOW)
bool ACTIVE_OPTO_V2 = false; // flag that tells you if the OPTO_V2 signal from the robot was received (active LOW)
bool ACTIVE_OPTO_V3 = false; // flag that tells you if the OPTO_V3 signal from the robot was received (active LOW)
bool ACTIVE_START = false; // flag that tells you if the START button was pressed (active LOW)
bool ACTIVE_RESET = false; // flag that tells you if the RESET button was pressed (active LOW)

enum State { // gives a number to each state of the system
  ST_IDLE, 
  ST_START_RICE_ROLLING, 
  ST_MOVE_FWD, 
  ST_STOP_RICE_ROLLING, 
  ST_ROBOT_SAFETY_CHECK,
  ST_START_ROBOT_CYCLE,
  ST_CLOSE_GRIPPER,
  ST_OPEN_GRIPPER,
  ST_RESET_ROBOT_SIGNALS,
  ST_CLOSE_CLAWS,
  ST_WAIT_FOR_FINISH,
  ST_START_SLICING,
  ST_OPEN_CLAWS,
  ST_WAIT_FOR_RESET,
  ST_MOVE_BACK
};

State currentState; // the actual current state of the system 
bool stepLevel = false; // boolean that will be toggled and written to the STEP bit to create pulses
unsigned long conveyorStepInterval = 0; // integer that will be set in the state to determine how many microseconds between pulses
unsigned long timeLastStep = 0; // long that measures the time between the pulses
int conveyorLeg = 0; // integer that tells us the upcomiong leg of the MOVE_FWD state 
unsigned long timeLastSend = 0; // long that measures the time between Serial command retries
unsigned long timestampGripper = 0; // long that marks when the gripper starts closing or opening
bool signalSent = false; // boolean that will flag when a signal is sent once already

/* ============= SETUP ============= */
void setup() {
  // Initialize Serial monitor communication, and Master/Slave Arduino communication
  Serial.begin(115200);
  Serial3.begin(115200);
  Serial3.setTimeout(SERIAL_TIMEOUT);

  // Set input pins
  pinMode(LS1, INPUT_PULLUP);
  pinMode(LS2, INPUT_PULLUP);
  pinMode(RS1, INPUT_PULLUP);
  pinMode(RS2, INPUT_PULLUP);
  pinMode(OPTO_V1, INPUT_PULLUP);
  pinMode(OPTO_V2, INPUT_PULLUP);
  pinMode(OPTO_V3, INPUT_PULLUP);
  pinMode(START, INPUT_PULLUP);
  pinMode(RESET, INPUT_PULLUP);

  // Set output pins
  pinMode(MOT_DR_CONVEYOR_EN, OUTPUT);
  pinMode(MOT_DR_CONVEYOR_STEP, OUTPUT);
  pinMode(MOT_DR_CONVEYOR_DIR, OUTPUT);
  pinMode(REL_IN1, OUTPUT);
  pinMode(REL_IN2, OUTPUT);
  pinMode(REL_IN3, OUTPUT);

  // Set up servo motor pin
  gripper.write(GRIPPER_OPEN_ANGLE); // sends a signal for safe/home position
  gripper.attach(SER_MOT_GRIPPER);

  // Set initial startup outputs
  digitalWrite(MOT_DR_CONVEYOR_EN, MOT_DR_EN_OFF);
  digitalWrite(MOT_DR_CONVEYOR_STEP, LOW);
  digitalWrite(MOT_DR_CONVEYOR_DIR, MOT_DR_CONVEYOR_FWD);
  digitalWrite(REL_IN1, HIGH);
  digitalWrite(REL_IN2, HIGH);
  digitalWrite(REL_IN3, HIGH);

  // Clear Serial3 buffer just in case
  delay(100); // let Slave Arduino boot up
  while (Serial3.available() > 0) { // discard anything already received
    Serial3.read();
  }

  // System begins in the IDLE state
  changeState(ST_IDLE);
}

/* ============= MAIN LOOP ============= */
void loop() {
  // Reads every input into the Arduino and stores them into a variable
  updateVariables();

  // Puts the system into reset mode (ST_MOVE_BACK) and moves platform back to start; can be called from any state aside from ST_MOVE_BACK or ST_IDLE
  if (ACTIVE_RESET && currentState != ST_MOVE_BACK && (currentState != ST_IDLE || !ACTIVE_LS1)) {
    changeState(ST_MOVE_BACK);
    return;
  }

  // Runs every loop; meant for recurring outputs within a state
  switch (currentState) {
    case ST_IDLE:
      if (ACTIVE_START && ACTIVE_LS1) changeState(ST_START_RICE_ROLLING); // if the START button is pressed, move onto START_RICE_ROLLING state
      break;

    case ST_START_RICE_ROLLING:
      if (ACTIVE_ACK_START_RICE) { // if "ACK START RICE" message received, move onto ST_MOVE_FWD state (leg 1)
        conveyorLeg = 1; // after this state, conveyor belt will move through leg 1
        changeState(ST_MOVE_FWD); 
      } else if (millis() - timeLastSend >= RESEND_INTERVAL) { // if message still not received, try sending command again
        attemptResend("START RICE");
      } 
      break;

    case ST_MOVE_FWD:
      if (conveyorLeg == 1 && ACTIVE_RS1) changeState(ST_STOP_RICE_ROLLING); // if RS1 is met, move onto ST_STOP_RICE_ROLLING state
      else if (conveyorLeg == 2 && ACTIVE_RS2) changeState(ST_START_SLICING); // if RS2 is met, move onto ST_START_SLICING state
      else if (conveyorLeg == 3 && ACTIVE_LS2) changeState(ST_OPEN_CLAWS); // if LS2 is pressed, move onto ST_OPEN_CLAWS state
      else if (micros() - timeLastStep >= conveyorStepInterval) { // toggle the stepLevel bool being send to STEP bit to create pulses every conveyorStepInterval microseconds
        stepOnce();
      }
      break;

    case ST_STOP_RICE_ROLLING:
      if (ACTIVE_ACK_STOP_RICE) { // if "ACK STOP RICE" message received, move onto ST_ROBOT_SAFETY_CHECK state
        changeState(ST_ROBOT_SAFETY_CHECK);
      } else if (millis() - timeLastSend >= RESEND_INTERVAL) {
        attemptResend("STOP RICE");
      }
      break;

    case ST_ROBOT_SAFETY_CHECK:
      if (!ACTIVE_OPTO_V1 && !ACTIVE_OPTO_V2 && !ACTIVE_OPTO_V3) changeState(ST_START_ROBOT_CYCLE); // if all robot signals are HIGH (inactive), move onto ST_START_ROBOT_CYCLE state
      break;

    case ST_START_ROBOT_CYCLE:
      if (ACTIVE_OPTO_V1) changeState(ST_CLOSE_GRIPPER); // if LOW signal received from robot, move onto ST_CLOSE_GRIPPER state
      break;
    
    case ST_CLOSE_GRIPPER:
      if (ACTIVE_OPTO_V2) { //if LOW signal received from robot, move onto ST_OPEN_GRIPPER state
        changeState(ST_OPEN_GRIPPER);
      } else if (millis() - timestampGripper >= GRIPPER_MOVE_TIME && !signalSent) { // if the alloted time has passed since the gripper started to close, send the output signal
        digitalWrite(REL_IN2, LOW);
        signalSent = true;
      }
      break;

    case ST_OPEN_GRIPPER:
     if (ACTIVE_OPTO_V3) { //if LOW signal received from robot, move onto ST_RESET_ROBOT_SIGNALS state
        changeState(ST_RESET_ROBOT_SIGNALS);
      } else if (millis() - timestampGripper >= GRIPPER_MOVE_TIME && !signalSent) { // if the alloted time has passed since the gripper started to open, send the output signal
        digitalWrite(REL_IN3, LOW);
        signalSent = true;
      }
      break;

    case ST_RESET_ROBOT_SIGNALS:
      changeState(ST_CLOSE_CLAWS);
      break;
    
    case ST_CLOSE_CLAWS:
      if (ACTIVE_ACK_CLOSE_CLAWS) { // if "ACK CLOSE CLAWS" message received, move onto ST_WAIT_FOR_FINISH state
        changeState(ST_WAIT_FOR_FINISH); 
      } else if (ACTIVE_DONE_CLOSE_CLAWS) { // if "ACK CLOSE CLAWS" message missed but receives "DONE CLOSE CLAWS" message, ACK is implied so move directly to ST_MOVE_FWD state (leg 2)
        conveyorLeg = 2; // after this state, conveyor belt will move through leg 2
        changeState(ST_MOVE_FWD);
      } else if (millis() - timeLastSend >= RESEND_INTERVAL) { // if message still not received, try sending command again
        attemptResend("CLOSE CLAWS");
      } 
      break;

    case ST_WAIT_FOR_FINISH:
      if (ACTIVE_DONE_CLOSE_CLAWS) { // if "DONE CLOSE CLAWS" message received, move onto ST_MOVE_FWD state (leg 2)
        conveyorLeg = 2; // after this state, conveyor belt will move through leg 2
        changeState(ST_MOVE_FWD);
      } else if (ACTIVE_DONE_SLICER) { // if "DONE SLICER" message received, move onto ST_MOVE_FWD state (leg 3)
        conveyorLeg = 3; // after this state, conveyor belt will move through leg 3
        changeState(ST_MOVE_FWD);
      } else if (ACTIVE_DONE_OPEN_CLAWS) { // if "DONE OPEN CLAWS" message received, move onto ST_WAIT_FOR_RESET state
        changeState(ST_WAIT_FOR_RESET);
      }
      break;

    case ST_START_SLICING:
      if (ACTIVE_ACK_START_SLICER) { // if "ACK START SLICER" message received, move onto ST_WAIT_FOR_FINISH state
        changeState(ST_WAIT_FOR_FINISH);
      } else if (ACTIVE_DONE_SLICER) { // if "ACK START SLICER" message missed but receives "DONE SLICER" message, ACK is implied so move directly to ST_MOVE_FWD state (leg 3)
        conveyorLeg = 3; // after this state, conveyor belt will move through leg 3
        changeState(ST_MOVE_FWD);
      } else if (millis() - timeLastSend >= RESEND_INTERVAL) { // if message still not received, try sending command again
        attemptResend("START SLICER");
      }
      break;
    
    case ST_OPEN_CLAWS:
      if (ACTIVE_ACK_OPEN_CLAWS) { // if "ACK OPEN CLAWS" message received, move onto ST_WAIT_FOR_FINISH state
        changeState(ST_WAIT_FOR_FINISH);
      } else if (ACTIVE_DONE_OPEN_CLAWS) { // if "ACK OPEN CLAWS" message missed but receives "DONE OPEN CLAWS" message, ACK is implied so move directly to ST_WAIT_FOR_RESET state
        changeState(ST_WAIT_FOR_RESET);
      } else if (millis() - timeLastSend >= RESEND_INTERVAL) { // if message still not received, try sending command again
        attemptResend("OPEN CLAWS");
      }
      break;

    case ST_WAIT_FOR_RESET:
      if (ACTIVE_RESET) { // if RESET button pressed, move onto ST_MOVE_BACK state
        changeState(ST_MOVE_BACK);
      }
      break;

    case ST_MOVE_BACK:
      if (ACTIVE_LS1) changeState(ST_IDLE); // if LS1 is met, move onto ST_IDLE state
      else if (micros() - timeLastStep >= conveyorStepInterval) { // toggle the stepLevel bool being send to STEP bit to create pulses every conveyorStepInterval microseconds
        stepOnce();
      }
      break;
  }
}

/* ============= CHANGE STATE FUNCTION ============= */
void changeState(State nextState) {
  currentState = nextState; // update the current state to what we just changed it to

  // Runs once when the entry condition is true; meant for one-time actions that occur only upon state entry
  switch (currentState) {
    case ST_IDLE:
      Serial.println("ST_IDLE: Idling...");
      digitalWrite(MOT_DR_CONVEYOR_EN, MOT_DR_EN_ON);
      digitalWrite(MOT_DR_CONVEYOR_STEP, LOW);
      digitalWrite(MOT_DR_CONVEYOR_DIR, MOT_DR_CONVEYOR_FWD);
      digitalWrite(REL_IN1, HIGH);
      digitalWrite(REL_IN2, HIGH);
      digitalWrite(REL_IN3, HIGH);
      conveyorLeg = 0;
      break;

    case ST_START_RICE_ROLLING:
      Serial.println("ST_START_RICE_ROLLING: Sending START RICE command...");
      Serial3.println("START RICE"); // sends command to Slave
      timeLastSend = millis(); // timeLastSend is reset to current time
      break;

    case ST_MOVE_FWD:
      Serial.println("ST_MOVE_FWD: Moving forward...");
      // Sets the conveyor belt speed depending on the current leg 
      if (conveyorLeg == 1) conveyorStepInterval = CONVEYOR_STEP_INTERVAL_1;
      else if (conveyorLeg == 2) conveyorStepInterval = CONVEYOR_STEP_INTERVAL_2;
      else if (conveyorLeg == 3) conveyorStepInterval = CONVEYOR_STEP_INTERVAL_3;
      
      stepLevel = false; // stepLevel bool is reset to false
      digitalWrite(MOT_DR_CONVEYOR_EN, MOT_DR_EN_ON); // ensure EN bit is ON
      digitalWrite(MOT_DR_CONVEYOR_STEP, stepLevel); // STEP bit is reset to LOW to begin
      digitalWrite(MOT_DR_CONVEYOR_DIR, MOT_DR_CONVEYOR_FWD); // DIR bit is set to move forward
      timeLastStep = micros(); // timeLastStep is reset to current time
      break;

    case ST_STOP_RICE_ROLLING:
      Serial.println("ST_STOP_RICE_ROLLING: Sending STOP RICE command...");
      Serial3.println("STOP RICE"); // sends command to Slave
      timeLastSend = millis();
      break;
    
    case ST_ROBOT_SAFETY_CHECK:
      Serial.println("ST_ROBOT_SAFETY_CHECK: Waiting on all signals HIGH from robot...");
      digitalWrite(REL_IN1, HIGH); // resets relay outputs to HIGH (inactive)
      digitalWrite(REL_IN2, HIGH);
      digitalWrite(REL_IN3, HIGH);
      break;

    case ST_START_ROBOT_CYCLE:
      Serial.println("ST_START_ROBOT_CYCLE: Sending RELAY_IN1 signal to robot...");
      digitalWrite(REL_IN1, LOW); // sets relay output to LOW (active)
      break;
    
    case ST_CLOSE_GRIPPER:
      Serial.println("ST_CLOSE_GRIPPER: Closing gripper...");
      gripper.write(GRIPPER_CLOSED_ANGLE); // gripper moves to the close angle
      timestampGripper = millis(); // timestampGripper is reset to current time
      signalSent = false;
      break;

    case ST_OPEN_GRIPPER:
      Serial.println("ST_OPEN_GRIPPER: Opening gripper...");
      gripper.write(GRIPPER_OPEN_ANGLE);
      timestampGripper = millis(); // timestampGripper is reset to current time
      signalSent = false;
      break;

    case ST_RESET_ROBOT_SIGNALS:
      Serial.println("ST_RESET_ROBOT_SIGNALS: Resetting relay signals to HIGH...");
      digitalWrite(REL_IN1, HIGH); // resets relay outputs to HIGH (inactive)
      digitalWrite(REL_IN2, HIGH);
      digitalWrite(REL_IN3, HIGH);
      break;
    
    case ST_CLOSE_CLAWS:
      Serial.println("ST_CLOSE_CLAWS: Sending CLOSE CLAWS command...");
      Serial3.println("CLOSE CLAWS"); // sends command to Slave
      timeLastSend = millis(); // timeLastSend is reset to current time
      signalSent = false;
      break;

    case ST_WAIT_FOR_FINISH:
      Serial.println("ST_WAIT_FOR_FINISH: Waiting for DONE message..."); 
      break;

    case ST_START_SLICING:
      Serial.println("ST_START_SLICING: Sending START SLICER command...");
      Serial3.println("START SLICER"); // sends command to Slave
      timeLastSend = millis();
      break;
    
    case ST_OPEN_CLAWS:
      Serial.println("ST_OPEN_CLAWS: Sending OPEN CLAWS command...");
      Serial3.println("OPEN CLAWS"); // sends command to Slave
      timeLastSend = millis();
      break;

    case ST_WAIT_FOR_RESET:
      Serial.println("ST_WAIT_FOR_RESET: Waiting for reset button to be pressed...");
      break;

    case ST_MOVE_BACK:
      Serial.println("ST_MOVE_BACK: Moving back to start...");
      conveyorStepInterval = CONVEYOR_STEP_INTERVAL_BACK; // sets the conveyor belt speed
      stepLevel = false; // stepLevel bool is reset to false
      digitalWrite(MOT_DR_CONVEYOR_EN, MOT_DR_EN_ON); // ensure EN bit is ON
      digitalWrite(MOT_DR_CONVEYOR_STEP, stepLevel); // STEP bit is reset to LOW to begin
      digitalWrite(MOT_DR_CONVEYOR_DIR, MOT_DR_CONVEYOR_BACK); // DIR bit is set to move backwards
      timeLastStep = micros(); // timeLastStep is reset to current time
      break;

    default:
      Serial.println("Unknown state.");
  }

}

/* ============= UPDATE VARIABLES FUNCTION ============= */
void updateVariables() {
  ACTIVE_LS1 = (digitalRead(LS1) == HIGH); // flag that tell you if limit switch 1 was met (active HIGH)
  ACTIVE_LS2 = (digitalRead(LS2) == HIGH);  // flag that tell you if limit switch 2 was hit (active HIGH)
  ACTIVE_RS1 = (digitalRead(RS1) == LOW);  // flag that tell you if reed switch 1 was met (active LOW)
  ACTIVE_RS2 = (digitalRead(RS2) == LOW);// flag that tell you if reed switch 2 was met (active LOW)

  clearSerialFlags();
  if (Serial3.available() > 0) {
    String msg = Serial3.readStringUntil('\n');
    msg.trim();
    if (msg == "ACK START RICE") ACTIVE_ACK_START_RICE = true;
    else if (msg == "ACK STOP RICE") ACTIVE_ACK_STOP_RICE = true;
    else if (msg == "ACK CLOSE CLAWS") ACTIVE_ACK_CLOSE_CLAWS = true;
    else if (msg == "DONE CLOSE CLAWS") ACTIVE_DONE_CLOSE_CLAWS = true;
    else if (msg == "ACK OPEN CLAWS") ACTIVE_ACK_OPEN_CLAWS = true;
    else if (msg == "DONE OPEN CLAWS") ACTIVE_DONE_OPEN_CLAWS = true;
    else if (msg == "ACK START SLICER") ACTIVE_ACK_START_SLICER = true;
    else if (msg == "DONE SLICER") ACTIVE_DONE_SLICER = true;
  }
  
  ACTIVE_OPTO_V1 = (digitalRead(OPTO_V1) == LOW); // flag that tells you if the OPTO_V1 signal from the robot was received (active LOW)
  ACTIVE_OPTO_V2 = (digitalRead(OPTO_V2) == LOW); // flag that tells you if the OPTO_V2 signal from the robot was received (active LOW)
  ACTIVE_OPTO_V3 = (digitalRead(OPTO_V3) == LOW); // flag that tells you if the OPTO_V3 signal from the robot was received (active LOW)
  ACTIVE_START = (digitalRead(START) == LOW); // flag that tells you if the START button was pressed (active LOW)
  ACTIVE_RESET = (digitalRead(RESET) == LOW); // flag that tells you if the RESET button was pressed (active LOW)
}

/* ============= CLEAR SERIAL FLAGS FUNCTION ============= */
void clearSerialFlags() {
  ACTIVE_ACK_START_RICE = false; // flag that tells you if the "ACK START RICE" message was received
  ACTIVE_ACK_STOP_RICE = false; // flag that tells you if the "ACK STOP RICE" message was received
  ACTIVE_ACK_CLOSE_CLAWS = false; // flag that tells you if the "ACK CLOSE CLAWS" message was received
  ACTIVE_DONE_CLOSE_CLAWS = false; // flag that tells you if the "DONE CLOSE CLAWS" message was received
  ACTIVE_ACK_OPEN_CLAWS = false; // flag that tells you if the "ACK OPEN CLAWS" message was received
  ACTIVE_DONE_OPEN_CLAWS = false; // flag that tells you if the "DONE OPEN CLAWS" message was received
  ACTIVE_ACK_START_SLICER = false; // flag that tells you if the "ACK START SLICER" message was received
  ACTIVE_DONE_SLICER = false; // flag that tells you if the "DONE SLICER" message was received
}

/* ============= ATTEMPT RESEND FUNCTION ============= */
void attemptResend(String cmd) {
  Serial3.println(cmd);
  timeLastSend = millis();
}

/* ============= STEP ONCE FUNCTION ============= */
void stepOnce() {
  timeLastStep = timeLastStep + conveyorStepInterval;
  stepLevel = !stepLevel;
  digitalWrite(MOT_DR_CONVEYOR_STEP, stepLevel);
}