/*
 * APEX_PREDATOR_V3 - Pure Execution Firmware
 * Hardware: Mega 2560 + RAMPS 1.6 + 5x DRV8825
 * * MOTOR MAPPING:
 * Base      -> E0 Output (NEMA17 5.6Kgcm) 
 * Shoulder  -> X Output  (NEMA17 5.6Kgcm) - Limit Switch at X_MIN
 * Elbow     -> Y Output  (NEMA17 5.6Kgcm) - Limit Switch at Y_MIN
 * Roll      -> Z Output  (NEMA17 4.2Kgcm)
 * Gripper   -> E1 Header (2x MG90S Servos on D4, D11)
 *
 * COMMUNICATION PROTOCOL:
 * M,base,shoulder,elbow,roll\n  -> Move to absolute angles (e.g., M,45.0,90.0,-30.5,180.0)
 * G,angle\n                     -> Set gripper servos angle (0-180) (e.g., G,90)
 * E\n                           -> Enable motors
 * D\n                           -> Disable motors (Emergency Stop)
 */

#include <AccelStepper.h>
#include <Servo.h>

// ═══════════════════════════════════════════════════════════════
// PIN DEFINITIONS (RAMPS 1.6)
// ═══════════════════════════════════════════════════════════════
#define BASE_STEP_PIN     26  // E0
#define BASE_DIR_PIN      28
#define BASE_ENABLE_PIN   24

#define SHOULDER_STEP_PIN 54  // X
#define SHOULDER_DIR_PIN  55
#define SHOULDER_ENABLE_PIN 38

#define ELBOW_STEP_PIN    60  // Y
#define ELBOW_DIR_PIN     61
#define ELBOW_ENABLE_PIN  56

#define ROLL_STEP_PIN     46  // Z
#define ROLL_DIR_PIN      48
#define ROLL_ENABLE_PIN   62

#define LIMIT_SHOULDER    3   // X_MIN
#define LIMIT_ELBOW       14  // Y_MIN

#define SERVO_1_PIN       4   // E1 Header
#define SERVO_2_PIN       11  // E1 Header

#define FAN_PIN 9 // 12V Cooling Fan on D9

// ═══════════════════════════════════════════════════════════════
// MECHANICAL CONFIGURATION
// ═══════════════════════════════════════════════════════════════
const float MICROSTEPS = 8.0;         // 1/8 Microstepping (DRV8825)
const float STEPS_PER_REV = 200.0;    // 1.8 degree NEMA 17

// Gear Ratios
const float GEAR_BASE     = 19.0;      // 19:1
const float GEAR_SHOULDER = 20.0;     // Cycloidal 20:1
const float GEAR_ELBOW    = 20.0;     // Cycloidal 20:1
const float GEAR_ROLL     = 29.0;     // Worm gear 29:1

// Calculate Steps per Degree: (200 * 8 * GearRatio) / 360
const float SPD_BASE     = (STEPS_PER_REV * MICROSTEPS * GEAR_BASE) / 360.0;
const float SPD_SHOULDER = (STEPS_PER_REV * MICROSTEPS * GEAR_SHOULDER) / 360.0;
const float SPD_ELBOW    = (STEPS_PER_REV * MICROSTEPS * GEAR_ELBOW) / 360.0;
const float SPD_ROLL     = (STEPS_PER_REV * MICROSTEPS * GEAR_ROLL) / 360.0;

// ═══════════════════════════════════════════════════════════════
// OBJECT INSTANTIATION
// ═══════════════════════════════════════════════════════════════
AccelStepper base(AccelStepper::DRIVER, BASE_STEP_PIN, BASE_DIR_PIN);
AccelStepper shoulder(AccelStepper::DRIVER, SHOULDER_STEP_PIN, SHOULDER_DIR_PIN);
AccelStepper elbow(AccelStepper::DRIVER, ELBOW_STEP_PIN, ELBOW_DIR_PIN);
AccelStepper roll(AccelStepper::DRIVER, ROLL_STEP_PIN, ROLL_DIR_PIN);

Servo gripperLeft;
Servo gripperRight;

bool motorsEnabled = false;

// ═══════════════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(10); // Fast serial parsing
  
  pinMode(FAN_PIN, OUTPUT);
  digitalWrite(FAN_PIN, HIGH);

  // Setup Limits
  pinMode(LIMIT_SHOULDER, INPUT_PULLUP);
  pinMode(LIMIT_ELBOW, INPUT_PULLUP);

  // Setup Enables
  pinMode(BASE_ENABLE_PIN, OUTPUT);
  pinMode(SHOULDER_ENABLE_PIN, OUTPUT);
  pinMode(ELBOW_ENABLE_PIN, OUTPUT);
  pinMode(ROLL_ENABLE_PIN, OUTPUT);
  disableMotors(); // Start in safe, disabled state

  // Attach Servos
  gripperLeft.attach(SERVO_1_PIN);
  gripperRight.attach(SERVO_2_PIN);
  
  // Set Speeds (Adjust these later based on Python engine's trajectory planner)
  base.setMaxSpeed(2000);     base.setAcceleration(800);
  shoulder.setMaxSpeed(1500); shoulder.setAcceleration(300);
  elbow.setMaxSpeed(2000);    elbow.setAcceleration(800);
  roll.setMaxSpeed(2000);     roll.setAcceleration(800);

  Serial.println("SYSTEM:READY");
}

// ═══════════════════════════════════════════════════════════════
// MAIN LOOP
// ═══════════════════════════════════════════════════════════════
void loop() {
  // 1. Process incoming commands from Python Brain
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    processCommand(cmd);
  }

  // 2. Continually pulse the stepper motors if they haven't reached target
  if (motorsEnabled) {
    base.run();
    shoulder.run();
    elbow.run();
    roll.run();
  }
}

// ═══════════════════════════════════════════════════════════════
// COMMAND PARSER
// ═══════════════════════════════════════════════════════════════
void processCommand(String cmd) {
  if (cmd.startsWith("M,")) {
    // Example: M,45.0,90.0,-30.0,180.0
    int c1 = cmd.indexOf(',', 2);
    int c2 = cmd.indexOf(',', c1 + 1);
    int c3 = cmd.indexOf(',', c2 + 1);

    if (c1 > 0 && c2 > 0 && c3 > 0) {
      float tBase = cmd.substring(2, c1).toFloat();
      float tShoulder = cmd.substring(c1 + 1, c2).toFloat();
      float tElbow = cmd.substring(c2 + 1, c3).toFloat();
      float tRoll = cmd.substring(c3 + 1).toFloat();

      // Convert target angles to discrete steps
      base.moveTo(long(tBase * SPD_BASE));
      shoulder.moveTo(long(tShoulder * SPD_SHOULDER));
      elbow.moveTo(long(tElbow * SPD_ELBOW));
      roll.moveTo(long(tRoll * SPD_ROLL));
      
      Serial.println("ACK:MOVE");
    }
  } 
  else if (cmd.startsWith("G,")) {
    // Example: G,90
    int angle = cmd.substring(2).toInt();
    gripperLeft.write(angle);
    gripperRight.write(180 - angle); // Inverted for opposed gripping
    Serial.println("ACK:GRIPPER");
  }
  else if (cmd == "E") {
    enableMotors();
    Serial.println("ACK:ENABLED");
  }
  else if (cmd == "D") {
    disableMotors();
    Serial.println("ACK:DISABLED");
  }
}

// ═══════════════════════════════════════════════════════════════
// HARDWARE CONTROL FUNCTIONS
// ═══════════════════════════════════════════════════════════════
void enableMotors() {
  // RAMPS drivers enable on LOW signal
  digitalWrite(BASE_ENABLE_PIN, LOW);
  digitalWrite(SHOULDER_ENABLE_PIN, LOW);
  digitalWrite(ELBOW_ENABLE_PIN, LOW);
  digitalWrite(ROLL_ENABLE_PIN, LOW);
  motorsEnabled = true;
}

void disableMotors() {
  // RAMPS drivers disable on HIGH signal (Allows free-spinning and cools motors)
  digitalWrite(BASE_ENABLE_PIN, HIGH);
  digitalWrite(SHOULDER_ENABLE_PIN, HIGH);
  digitalWrite(ELBOW_ENABLE_PIN, HIGH);
  digitalWrite(ROLL_ENABLE_PIN, HIGH);
  motorsEnabled = false;
  
  // Clear any pending moves
  base.stop(); shoulder.stop(); elbow.stop(); roll.stop();
}
