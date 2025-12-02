// ================= SIMPLE MAZE SOLVER (NO LOOPS) ==================
// Updated with:
// - Smooth turning (arc turns, not tank spins)
// - Adjustable speeds (SPEED_FWD & SPEED_TURN)
// - Correct motor direction after wire swap
// - Handles left/forward/right/dead-end
// - Designed for tree mazes (no loops)
// ================================================================

// -------------------- SPEED SETTINGS --------------------
int SPEED_FWD  = 160;     // forward speed  (increase = faster)
int SPEED_TURN = 170;     // turning speed  (increase = sharper turn)

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

// Forward movement
void forward() {
  digitalWrite(motor1pin1, LOW);
  analogWrite(motor1pin2, SPEED_FWD);

  digitalWrite(motor2pin1, LOW);
  analogWrite(motor2pin2, SPEED_FWD);
}

// Smooth LEFT turn (arc turn)
void leftTurn() {
  // LEFT motor = slower forward (inner wheel)
  digitalWrite(motor1pin1, LOW);
  analogWrite(motor1pin2, SPEED_FWD / 2);

  // RIGHT motor = faster forward (outer wheel)
  digitalWrite(motor2pin1, LOW);
  analogWrite(motor2pin2, SPEED_TURN);
}

// Smooth RIGHT turn (arc turn)
void rightTurn() {
  // LEFT motor = faster forward (outer wheel)
  digitalWrite(motor1pin1, LOW);
  analogWrite(motor1pin2, SPEED_TURN);

  // RIGHT motor = slower forward (inner wheel)
  digitalWrite(motor2pin1, LOW);
  analogWrite(motor2pin2, SPEED_FWD / 2);
}

// U-turn for dead-ends (spin-in-place)
void uTurn() {
  // LEFT motor backward
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);

  // RIGHT motor forward
  digitalWrite(motor2pin1, LOW);
  analogWrite(motor2pin2, SPEED_TURN);

  delay(450); // adjust for perfect 180°
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
  Serial.println("Smooth Maze Solver Ready");
}

// ======================= MAIN LOOP ========================

void loop() {
  int R  = digitalRead(IR_RIGHT);
  int MR = digitalRead(IR_MID_RIGHT);
  int ML = digitalRead(IR_MID_LEFT);
  int L  = digitalRead(IR_LEFT);

  int M = (MR || ML); // middle detection

  // ---------------- FINISH LINE CHECK ----------------
  if (L == 1 && ML == 1 && MR == 1 && R == 1) {
    finishCounter++;
  } else {
    finishCounter = 0;
  }

  if (finishCounter > 10) {
    stopMotors();
    while (true);  // stop at finish
  }

  // ---------------- SIMPLE TREE MAZE LOGIC --------------
  
  // 1) If LEFT turn available
  if (L == 1 && M == 0 && R == 0) {
    leftTurn();
  }
  // 2) Else go FORWARD if possible
  else if (M == 1) {
    forward();
  }
  // 3) Else RIGHT turn available
  else if (R == 1) {
    rightTurn();
  }
  // 4) Else dead end
  else {
    uTurn();
  }

  delay(20);
}
