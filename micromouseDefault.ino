#include <Wire.h>
#include <VL53L0X.h>

// ---------------- Pin Definitions ----------------
#define AIN1 6
#define AIN2 7
#define PWMA 9   // Left motor PWM

#define BIN1 10
#define BIN2 12
#define PWMB 11  // Right motor PWM

#define STBY 13
#define LED_PIN 8

// ToF XSHUT pins
#define XSHUT_FRONT A0
#define XSHUT_LEFT  A1
#define XSHUT_RIGHT A2

// ---------------- ToF Sensors ----------------
VL53L0X sensorFront;
VL53L0X sensorLeft;
VL53L0X sensorRight;

#define FRONT_ADDRESS 0x30
#define LEFT_ADDRESS  0x31
#define RIGHT_ADDRESS 0x32

// ---------------- Calibration ----------------
// Motor speeds
#define FORWARD_SPEED  120
#define TURN_SPEED     130

// Motor offsets (to fix curve drift)
#define LEFT_OFFSET     0
#define RIGHT_OFFSET    0

// Maze geometry (mm)
#define CORRIDOR_WIDTH   250   // avg maze corridor width
#define SENSOR_SPACING    80   // mm between L/R sensors
#define DESIRED_DISTANCE ((CORRIDOR_WIDTH - SENSOR_SPACING) / 2)  // ~85 mm

// Wall detection thresholds
#define WALL_THRESHOLD   180  // mm (≤20 cm = wall detected)
#define FRONT_OBSTACLE   120  // mm (≤12 cm = wall ahead)

// Turn timings (tuned experimentally)
#define TURN_90_TIME   265   // ms for 90°
#define TURN_180_TIME  530   // ms for 180°

int lastCorr = 0; 
int lastError = 0;  // for PD control

// ---------------- Setup ----------------
void setup() {
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  pinMode(LED_PIN, OUTPUT);

  Wire.begin();
  initSensors();

  // Blink LED to show ready
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH); delay(100);
    digitalWrite(LED_PIN, LOW); delay(100);
  }
  digitalWrite(LED_PIN, HIGH);
}

// ---------------- Main Loop ----------------
// ---------------- Main Loop ----------------
void loop() {
  uint16_t frontDist = sensorFront.readRangeContinuousMillimeters();
  uint16_t leftDist  = sensorLeft.readRangeContinuousMillimeters() - 30;
  uint16_t rightDist = sensorRight.readRangeContinuousMillimeters() - 70;

  bool frontWall = (frontDist < FRONT_OBSTACLE);
  bool leftWall  = (leftDist < WALL_THRESHOLD);
  bool rightWall = (rightDist < WALL_THRESHOLD);

  // --- Left-hand rule logic with dead-end recheck ---
  if (!leftWall) {
    turnLeft90();
    moveForwardUntilWall();
  } 
  else if (!frontWall) {
    moveForwardUntilWall();
  } 
  else if (!rightWall) {
    turnRight90();
    moveForwardUntilWall();
  } 
  else {
    // 🔴 Potential dead-end → confirm before U-turn
    delay(80);  // short pause to stabilize sensors
    uint16_t f = sensorFront.readRangeContinuousMillimeters();
    uint16_t l = sensorLeft.readRangeContinuousMillimeters() - 30;
    uint16_t r = sensorRight.readRangeContinuousMillimeters() - 70;

    bool fWall = (f < FRONT_OBSTACLE);
    bool lWall = (l < WALL_THRESHOLD);
    bool rWall = (r < WALL_THRESHOLD);

    if (fWall && lWall && rWall) {
      // ✅ confirmed dead-end
      turnAround();
      moveForwardUntilWall();
    } else {
      // ❌ false detection → just stop & retry
      stopMotors();
      delay(50);
    }
  }
}

// ---------------- Functions ----------------
void moveForwardUntilWall() {
  while (sensorFront.readRangeContinuousMillimeters() > FRONT_OBSTACLE) {
    uint16_t left  = sensorLeft.readRangeContinuousMillimeters() - 30;
    uint16_t right = sensorRight.readRangeContinuousMillimeters() - 70;
    int corr = 0;
    int error = 0;

    if (left > 400) left = 400;
    if (right > 400) right = 400;

    // --- Both walls available → center bot ---
    if (left < WALL_THRESHOLD && right < WALL_THRESHOLD) {
      error = (right - left);
    }
    // --- Left wall only ---
    else if (left < WALL_THRESHOLD) {
      error = (DESIRED_DISTANCE - left);
    }
    // --- Right wall only ---
    else if (right < WALL_THRESHOLD) {
      error = -(DESIRED_DISTANCE - right);
    }

    // --- PD correction ---
    int dError = error - lastError;
    corr = (error * 0.1) + (dError * 0.05);  // P + D
    lastError = error;

    // --- Deadband ---
    if (abs(error) < 5) corr = 0;

    // --- Straight open path → hold heading ---
    if (left >= WALL_THRESHOLD && right >= WALL_THRESHOLD) {
      corr = lastCorr * 0.9;
    }

    // Clamp correction
    corr = constrain(corr, -20, 20);
    lastCorr = corr;

    // Forward drive with correction
    setMotorSpeeds(FORWARD_SPEED - corr, FORWARD_SPEED + corr);
    delay(15);
  }

  stopMotors();
  delay(30);
}

// --- Movement ---
void turnLeft90() {
  stopMotors();
  setMotorSpeeds(-TURN_SPEED, TURN_SPEED);
  delay(TURN_90_TIME);
  stopMotors();
  setMotorSpeeds(FORWARD_SPEED, FORWARD_SPEED);
  delay(120);
  stopMotors();
}

void turnRight90() {
  stopMotors();
  setMotorSpeeds(TURN_SPEED, -TURN_SPEED);
  delay(TURN_90_TIME);
  stopMotors();
  setMotorSpeeds(FORWARD_SPEED, FORWARD_SPEED);
  delay(120);
  stopMotors();
}

void turnAround() {
  stopMotors();
  setMotorSpeeds(-TURN_SPEED, TURN_SPEED);
  delay(TURN_180_TIME);
  stopMotors();
}

// --- Motor Control ---
void stopMotors() {
  setMotorSpeeds(0, 0);
  delay(50);
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  leftSpeed  = constrain(leftSpeed + LEFT_OFFSET, -255, 255);
  rightSpeed = constrain(rightSpeed + RIGHT_OFFSET, -255, 255);

  if (leftSpeed >= 0) { digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW); }
  else { digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH); leftSpeed = -leftSpeed; }
  analogWrite(PWMA, leftSpeed);

  if (rightSpeed >= 0) { digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); }
  else { digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH); rightSpeed = -rightSpeed; }
  analogWrite(PWMB, rightSpeed);
}

// --- ToF Init ---
void initSensors() {
  pinMode(XSHUT_FRONT, OUTPUT);
  pinMode(XSHUT_LEFT, OUTPUT);
  pinMode(XSHUT_RIGHT, OUTPUT);

  digitalWrite(XSHUT_FRONT, LOW);
  digitalWrite(XSHUT_LEFT, LOW);
  digitalWrite(XSHUT_RIGHT, LOW);
  delay(50);

  digitalWrite(XSHUT_FRONT, HIGH); delay(50);
  sensorFront.init(true);
  sensorFront.setAddress(FRONT_ADDRESS);
  sensorFront.setTimeout(200);
  sensorFront.startContinuous();

  digitalWrite(XSHUT_LEFT, HIGH); delay(50);
  sensorLeft.init(true);
  sensorLeft.setAddress(LEFT_ADDRESS);
  sensorLeft.setTimeout(200);
  sensorLeft.startContinuous();

  digitalWrite(XSHUT_RIGHT, HIGH); delay(50);
  sensorRight.init(true);
  sensorRight.setAddress(RIGHT_ADDRESS);
  sensorRight.setTimeout(200);
  sensorRight.startContinuous();
}
