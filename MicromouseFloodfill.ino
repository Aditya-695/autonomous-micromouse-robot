#include <Wire.h>
#include <VL53L0X.h>
#include <Arduino.h>

// ---------------- Motor driver pins (from your existing sketch) ----------------
constexpr uint8_t AIN1 = 6;
constexpr uint8_t AIN2 = 7;
constexpr uint8_t PWMA = 9;   // left motor PWM
constexpr uint8_t BIN1 = 10;
constexpr uint8_t BIN2 = 12;
constexpr uint8_t PWMB = 11;  // right motor PWM
constexpr uint8_t STBY = 13;
constexpr uint8_t LED_PIN = 8;

// ---------------- ToF sensor XSHUT pins ----------------
constexpr uint8_t XSHUT_FRONT = A0;
constexpr uint8_t XSHUT_LEFT  = A1;
constexpr uint8_t XSHUT_RIGHT = A2;

VL53L0X sensorFront;
VL53L0X sensorLeft;
VL53L0X sensorRight;

constexpr uint8_t FRONT_ADDRESS = 0x30;
constexpr uint8_t LEFT_ADDRESS  = 0x31;
constexpr uint8_t RIGHT_ADDRESS = 0x32;

// ---------------- Encoder pins: VERIFY AGAINST YOUR WIRING ----------------
// Example allocation for UNO R4 Minima. Encoder A channels must use interrupt-capable
// pins supported by the selected board core. Change these to match your wiring.
constexpr uint8_t LEFT_ENC_A  = 2;
constexpr uint8_t LEFT_ENC_B  = 4;
constexpr uint8_t RIGHT_ENC_A = 3;
constexpr uint8_t RIGHT_ENC_B = 5;

volatile long leftTicks = 0;
volatile long rightTicks = 0;

// ---------------- Maze geometry ----------------
constexpr uint8_t MAZE_W = 8;
constexpr uint8_t MAZE_H = 8;
constexpr uint8_t START_X = 0;
constexpr uint8_t START_Y = 0;
constexpr uint8_t GOAL_X = 7;
constexpr uint8_t GOAL_Y = 7;
constexpr uint16_t CELL_MM = 250;

// ---------------- Tune these for your robot ----------------
// Counts are quadrature counts as seen by this ISR implementation.
// Measure ticks while rolling the robot exactly one 250 mm cell.
constexpr long TICKS_PER_CELL_LEFT = 900;   // CALIBRATE
constexpr long TICKS_PER_CELL_RIGHT = 900;  // CALIBRATE
// Measure ticks per wheel for an in-place 90-degree turn.
constexpr long TICKS_PER_90_LEFT = 225;     // CALIBRATE
constexpr long TICKS_PER_90_RIGHT = 225;    // CALIBRATE

constexpr int BASE_PWM = 100;
constexpr int TURN_PWM = 95;
constexpr int MAX_PWM = 170;
constexpr int MIN_PWM = 0;
constexpr uint16_t WALL_THRESHOLD_MM = 180;
constexpr uint16_t FRONT_STOP_MM = 105;
constexpr uint16_t SENSOR_TIMEOUT_MS = 200;
constexpr uint16_t CONTROL_PERIOD_MS = 10;

// PID gains are starting values only; tune on your chassis.
float Kp = 0.45f;
float Ki = 0.00f;
float Kd = 0.08f;

// Encoder balance PID state
float pidIntegral = 0.0f;
float previousError = 0.0f;
unsigned long previousControlMs = 0;

// ---------------- Maze representation ----------------
// Wall bits are stored for each cell. A wall bit means a wall is believed to exist.
constexpr uint8_t NORTH = 0x01;
constexpr uint8_t EAST  = 0x02;
constexpr uint8_t SOUTH = 0x04;
constexpr uint8_t WEST  = 0x08;
uint8_t walls[MAZE_H][MAZE_W];
uint8_t flood[MAZE_H][MAZE_W];

enum Heading : uint8_t { NORTH_DIR = 0, EAST_DIR = 1, SOUTH_DIR = 2, WEST_DIR = 3 };
Heading heading = NORTH_DIR;
uint8_t cellX = START_X;
uint8_t cellY = START_Y;

struct Distances {
  uint16_t front;
  uint16_t left;
  uint16_t right;
  bool valid;
};

uint16_t correctedDistance(VL53L0X &sensor, int16_t offsetMm) {
  uint16_t raw = sensor.readRangeContinuousMillimeters();
  if (sensor.timeoutOccurred()) return 8190;
  int32_t corrected = (int32_t)raw - offsetMm;
  if (corrected < 0) corrected = 0;
  if (corrected > 8190) corrected = 8190;
  return (uint16_t)corrected;
}

Distances readDistances() {
  Distances d;
  d.front = correctedDistance(sensorFront, 0);
  d.left  = correctedDistance(sensorLeft, 30);   // calibrate for your mounting
  d.right = correctedDistance(sensorRight, 70); // calibrate for your mounting
  d.valid = (d.front != 8190 && d.left != 8190 && d.right != 8190);
  return d;
}

void leftEncoderISR() {
  // Quadrature direction from A/B phase. Reverse sign if your motor wiring is opposite.
  leftTicks += (digitalRead(LEFT_ENC_A) == digitalRead(LEFT_ENC_B)) ? 1 : -1;
}
void rightEncoderISR() {
  rightTicks += (digitalRead(RIGHT_ENC_A) == digitalRead(RIGHT_ENC_B)) ? 1 : -1;
}

void getTicks(long &l, long &r) {
  noInterrupts();
  l = leftTicks;
  r = rightTicks;
  interrupts();
}

void resetTicks() {
  noInterrupts();
  leftTicks = 0;
  rightTicks = 0;
  interrupts();
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  digitalWrite(STBY, HIGH);

  if (leftSpeed >= 0) {
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
  } else {
    digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
    leftSpeed = -leftSpeed;
  }
  analogWrite(PWMA, constrain(leftSpeed, 0, 255));

  if (rightSpeed >= 0) {
    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
  } else {
    digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH);
    rightSpeed = -rightSpeed;
  }
  analogWrite(PWMB, constrain(rightSpeed, 0, 255));
}

void stopMotors() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW);
}

void initSensors() {
  pinMode(XSHUT_FRONT, OUTPUT);
  pinMode(XSHUT_LEFT, OUTPUT);
  pinMode(XSHUT_RIGHT, OUTPUT);

  digitalWrite(XSHUT_FRONT, LOW);
  digitalWrite(XSHUT_LEFT, LOW);
  digitalWrite(XSHUT_RIGHT, LOW);
  delay(50);

  digitalWrite(XSHUT_FRONT, HIGH); delay(50);
  if (!sensorFront.init()) { stopMotors(); while (true) { digitalWrite(LED_PIN, !digitalRead(LED_PIN)); delay(150); } }
  sensorFront.setAddress(FRONT_ADDRESS);
  sensorFront.setTimeout(SENSOR_TIMEOUT_MS);
  sensorFront.startContinuous(0);

  digitalWrite(XSHUT_LEFT, HIGH); delay(50);
  if (!sensorLeft.init()) { stopMotors(); while (true) { digitalWrite(LED_PIN, !digitalRead(LED_PIN)); delay(150); } }
  sensorLeft.setAddress(LEFT_ADDRESS);
  sensorLeft.setTimeout(SENSOR_TIMEOUT_MS);
  sensorLeft.startContinuous(0);

  digitalWrite(XSHUT_RIGHT, HIGH); delay(50);
  if (!sensorRight.init()) { stopMotors(); while (true) { digitalWrite(LED_PIN, !digitalRead(LED_PIN)); delay(150); } }
  sensorRight.setAddress(RIGHT_ADDRESS);
  sensorRight.setTimeout(SENSOR_TIMEOUT_MS);
  sensorRight.startContinuous(0);
}

uint8_t wallBitForDirection(Heading h) {
  switch (h) {
    case NORTH_DIR: return NORTH;
    case EAST_DIR: return EAST;
    case SOUTH_DIR: return SOUTH;
    default: return WEST;
  }
}
Heading rotateHeading(Heading h, int turns) {
  return (Heading)((((int)h + turns) % 4 + 4) % 4);
}

bool insideMaze(int x, int y) {
  return x >= 0 && x < MAZE_W && y >= 0 && y < MAZE_H;
}

void setWall(uint8_t x, uint8_t y, Heading dir, bool present) {
  if (!insideMaze(x, y)) return;
  int nx = x, ny = y;
  switch (dir) {
    case NORTH_DIR: ny--; break;
    case EAST_DIR: nx++; break;
    case SOUTH_DIR: ny++; break;
    case WEST_DIR: nx--; break;
  }
  uint8_t bit = wallBitForDirection(dir);
  if (present) walls[y][x] |= bit;
  else walls[y][x] &= (uint8_t)~bit;

  if (insideMaze(nx, ny)) {
    uint8_t opposite = wallBitForDirection(rotateHeading(dir, 2));
    if (present) walls[ny][nx] |= opposite;
    else walls[ny][nx] &= (uint8_t)~opposite;
  }
}

bool hasWall(uint8_t x, uint8_t y, Heading dir) {
  return (walls[y][x] & wallBitForDirection(dir)) != 0;
}

void initMaze() {
  memset(walls, 0, sizeof(walls));
  // Outer boundary walls
  for (uint8_t x = 0; x < MAZE_W; x++) {
    setWall(x, 0, NORTH_DIR, true);
    setWall(x, MAZE_H - 1, SOUTH_DIR, true);
  }
  for (uint8_t y = 0; y < MAZE_H; y++) {
    setWall(0, y, WEST_DIR, true);
    setWall(MAZE_W - 1, y, EAST_DIR, true);
  }
}

void updateWallsFromSensors(const Distances &d) {
  // Sensor readings are mapped to the current cell's front/left/right directions.
  // This assumes the robot is aligned with the cell center and sensors face cardinal directions.
  setWall(cellX, cellY, heading, d.front < WALL_THRESHOLD_MM);
  setWall(cellX, cellY, rotateHeading(heading, -1), d.left < WALL_THRESHOLD_MM);
  setWall(cellX, cellY, rotateHeading(heading, 1), d.right < WALL_THRESHOLD_MM);
}

bool isGoal(uint8_t x, uint8_t y) {
  return x == GOAL_X && y == GOAL_Y;
}

void computeFloodFill() {
  for (uint8_t y = 0; y < MAZE_H; y++)
    for (uint8_t x = 0; x < MAZE_W; x++)
      flood[y][x] = 255;

  // Queue sized for every cell.
  uint8_t qx[MAZE_W * MAZE_H], qy[MAZE_W * MAZE_H];
  uint8_t head = 0, tail = 0;
  flood[GOAL_Y][GOAL_X] = 0;
  qx[tail] = GOAL_X; qy[tail] = GOAL_Y; tail++;

  while (head != tail) {
    uint8_t x = qx[head], y = qy[head]; head++;
    const Heading dirs[4] = { NORTH_DIR, EAST_DIR, SOUTH_DIR, WEST_DIR };
    for (uint8_t i = 0; i < 4; i++) {
      Heading dir = dirs[i];
      int nx = x, ny = y;
      switch (dir) {
        case NORTH_DIR: ny--; break;
        case EAST_DIR: nx++; break;
        case SOUTH_DIR: ny++; break;
        case WEST_DIR: nx--; break;
      }
      if (!insideMaze(nx, ny) || hasWall(x, y, dir)) continue;
      if (flood[ny][nx] == 255) {
        flood[ny][nx] = flood[y][x] + 1;
        qx[tail] = nx; qy[tail] = ny; tail++;
      }
    }
  }
}

Heading chooseNextDirection() {
  computeFloodFill();
  Heading candidates[4] = {
    rotateHeading(heading, -1), heading,
    rotateHeading(heading, 1), rotateHeading(heading, 2)
  };
  uint8_t bestValue = 255;
  Heading bestDir = heading;
  for (uint8_t i = 0; i < 4; i++) {
    Heading dir = candidates[i];
    if (hasWall(cellX, cellY, dir)) continue;
    int nx = cellX, ny = cellY;
    switch (dir) {
      case NORTH_DIR: ny--; break;
      case EAST_DIR: nx++; break;
      case SOUTH_DIR: ny++; break;
      case WEST_DIR: nx--; break;
    }
    if (!insideMaze(nx, ny)) continue;
    if (flood[ny][nx] < bestValue) {
      bestValue = flood[ny][nx];
      bestDir = dir;
    }
  }
  return bestDir;
}

// PID-controlled straight cell movement. Uses average encoder ticks as progress,
// and wheel-count difference as heading/speed balance error.
bool moveOneCell() {
  resetTicks();
  pidIntegral = 0;
  previousError = 0;
  previousControlMs = millis();

  const long targetLeft = TICKS_PER_CELL_LEFT;
  const long targetRight = TICKS_PER_CELL_RIGHT;
  unsigned long startMs = millis();
  const unsigned long maxMoveMs = 5000;

  while (millis() - startMs < maxMoveMs) {
    Distances d = readDistances();
    if (!d.valid) {
      stopMotors();
      return false;
    }
    // Stop if an unexpected front obstacle is detected before reaching the cell target.
    long l, r;
    getTicks(l, r);
    if (d.front < FRONT_STOP_MM && (l < targetLeft * 0.75L || r < targetRight * 0.75L)) {
      stopMotors();
      return false;
    }

    if (l >= targetLeft && r >= targetRight) {
      stopMotors();
      return true;
    }

    unsigned long now = millis();
    float dt = (now - previousControlMs) / 1000.0f;
    if (dt <= 0) dt = CONTROL_PERIOD_MS / 1000.0f;
    previousControlMs = now;

    float error = (float)l - (float)r;
    pidIntegral += error * dt;
    pidIntegral = constrain(pidIntegral, -300.0f, 300.0f);
    float derivative = (error - previousError) / dt;
    float correction = Kp * error + Ki * pidIntegral + Kd * derivative;
    previousError = error;

    int leftPWM = constrain((int)(BASE_PWM - correction), MIN_PWM, MAX_PWM);
    int rightPWM = constrain((int)(BASE_PWM + correction), MIN_PWM, MAX_PWM);
    setMotorSpeeds(leftPWM, rightPWM);
    delay(CONTROL_PERIOD_MS);
  }

  stopMotors();
  return false; // timeout rather than running indefinitely
}

bool turnQuarter(int direction) {
  resetTicks();
  long targetL = TICKS_PER_90_LEFT;
  long targetR = TICKS_PER_90_RIGHT;
  unsigned long start = millis();
  const unsigned long maxTurnMs = 2500;

  // direction: -1 = left, +1 = right
  int leftSign = (direction < 0) ? -1 : 1;
  int rightSign = -leftSign;

  while (millis() - start < maxTurnMs) {
    long l, r;
    getTicks(l, r);
    long leftProgress = labs(l);
    long rightProgress = labs(r);
    if (leftProgress >= targetL && rightProgress >= targetR) {
      stopMotors();
      heading = rotateHeading(heading, direction);
      delay(80);
      return true;
    }
    int leftPWM = (leftProgress >= targetL) ? 0 : TURN_PWM;
    int rightPWM = (rightProgress >= targetR) ? 0 : TURN_PWM;
    setMotorSpeeds(leftSign * leftPWM, rightSign * rightPWM);
    delay(CONTROL_PERIOD_MS);
  }
  stopMotors();
  return false;
}

bool turnAround() {
  // Two encoder-controlled quarter turns.
  return turnQuarter(1) && turnQuarter(1);
}

void updateCellAfterMove() {
  switch (heading) {
    case NORTH_DIR: if (cellY > 0) cellY--; break;
    case EAST_DIR: if (cellX < MAZE_W - 1) cellX++; break;
    case SOUTH_DIR: if (cellY < MAZE_H - 1) cellY++; break;
    case WEST_DIR: if (cellX > 0) cellX--; break;
  }
}

void setup() {
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT); pinMode(LED_PIN, OUTPUT);
  digitalWrite(STBY, HIGH);
  digitalWrite(LED_PIN, LOW);

  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncoderISR, CHANGE);

  Wire.begin();
  initSensors();
  initMaze();
  digitalWrite(LED_PIN, HIGH);
  delay(300);
  digitalWrite(LED_PIN, LOW);
}

void loop() {
  if (isGoal(cellX, cellY)) {
    stopMotors();
    digitalWrite(LED_PIN, HIGH);
    while (true) delay(1000);
  }

  Distances d = readDistances();
  if (!d.valid) {
    stopMotors();
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(200);
    return;
  }

  updateWallsFromSensors(d);
  Heading next = chooseNextDirection();

  int turn = ((int)next - (int)heading + 4) % 4;
  if (turn == 3) {
    if (!turnQuarter(-1)) return;
  } else if (turn == 1) {
    if (!turnQuarter(1)) return;
  } else if (turn == 2) {
    if (!turnAround()) return;
  }

  // After aligning to the chosen direction, move one cell using encoder feedback.
  if (moveOneCell()) {
    updateCellAfterMove();
  } else {
    // A failed move is not treated as a successful cell transition.
    stopMotors();
    delay(150);
  }
}
