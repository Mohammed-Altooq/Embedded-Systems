// ================= SIMPLE MAZE SOLVER (NO LOOPS) ==================
// Algorithm:
// 1. If LEFT available  -> turn left
// 2. Else if FORWARD    -> go forward
// 3. Else if RIGHT      -> turn right
// 4. Else (dead end)    -> U-turn
//
// This works perfectly for tree mazes (no loops).
//
// ================================================================

// -------------------- SPEED SETTINGS --------------------
int SPEED_FWD  = 160;
int SPEED_TURN = 170;

// -------------------- SENSOR PINS ------------------------
const int IR_RIGHT      = 8;
const int IR_MID_RIGHT  = 9;
const int IR_MID_LEFT   = 10;
const int IR_LEFT       = 11;

// -------------------- MOTOR PINS -------------------------
const int motor1pin1 = 2;   // LEFT motor direction
const int motor1pin2 = 3;   // LEFT motor PWM
const int motor2pin1 = 4;   // RIGHT motor direction
const int motor2pin2 = 5;   // RIGHT motor PWM

int finishCounter = 0;

// ==================== MOVEMENT ===========================

void forward() {
  digitalWrite(motor1pin1, LOW);
  analogWrite(motor1pin2, SPEED_FWD);

  digitalWrite(motor2pin1, LOW);
  analogWrite(motor2pin2, SPEED_FWD);
}

void leftTurn() {
  // LEFT motor backward
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);

  // RIGHT motor forward
  digitalWrite(motor2pin1, LOW);
  analogWrite(motor2pin2, SPEED_TURN);
}

void rightTurn() {
  // LEFT motor forward
  digitalWrite(motor1pin1, LOW);
  analogWrite(motor1pin2, SPEED_TURN);

  // RIGHT motor backward
  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);
}

void uTurn() {
  // Spin robot 180 degrees
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);

  digitalWrite(motor2pin1, LOW);
  analogWrite(motor2pin2, SPEED_TURN);

  delay(450); // adjust for your robot
}

void stopMotors() {
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, LOW);
  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, LOW);
}

// ======================= SETUP ============================

void setup() {
  pinMode(IR_RIGHT, INPUT);
  pinMode(IR_MID_RIGHT, INPUT);
  pinMode(IR_MID_LEFT, INPUT);
  pinMode(IR_LEFT, INPUT);

  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);

  Serial.begin(9600);
  Serial.println("Simple Tree Maze Solver Ready");
}

// ======================= MAIN LOOP ========================

void loop() {
  int R  = digitalRead(IR_RIGHT);
  int MR = digitalRead(IR_MID_RIGHT);
  int ML = digitalRead(IR_MID_LEFT);
  int L  = digitalRead(IR_LEFT);

  int M = (MR || ML); // forward sensor

  // ---------------- FINISH LINE CHECK ----------------
  if (L == 1 && ML == 1 && MR == 1 && R == 1) {
    finishCounter++;
  } else {
    finishCounter = 0;
  }

  if (finishCounter > 10) {
    stopMotors();
    while (true);  // stop forever at finish
  }

  // ---------- SIMPLE MAZE-SOLVING LOGIC (NO LOOPS) ----------
  
  // 1) If LEFT branch available → turn left
  if (L == 1 && M == 0 && R == 0) {
    leftTurn();
  }
  // 2) Else if FORWARD available → go forward
  else if (M == 1) {
    forward();
  }
  // 3) Else if RIGHT branch → turn right
  else if (R == 1) {
    rightTurn();
  }
  // 4) Else (dead end) → U-turn
  else {
    uTurn();
  }

  delay(20);
}
