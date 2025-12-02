// ================= BASIC LINE FOLLOWER WITH PWM ===================
// This version:
// - Uses PWM for smooth forward movement
// - Uses corrected direction logic AFTER motor wire swap
// - Ensures left/right turns rotate the correct direction
// - Keeps your original line-following logic

// -------------------- SPEED SETTINGS --------------------
int SPEED_FWD  = 160;   // forward speed (increase if slow)
int SPEED_TURN = 170;   // turning speed (slightly faster)

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

// ==================== MOVEMENT FUNCTIONS ==================

void forward() {
  // LEFT motor forward
  digitalWrite(motor1pin1, LOW);
  analogWrite(motor1pin2, SPEED_FWD);

  // RIGHT motor forward
  digitalWrite(motor2pin1, LOW);
  analogWrite(motor2pin2, SPEED_FWD);
}

void left() {
  // LEFT motor backward
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);

  // RIGHT motor forward (PWM)
  digitalWrite(motor2pin1, LOW);
  analogWrite(motor2pin2, SPEED_TURN);
}

void right() {
  // LEFT motor forward (PWM)
  digitalWrite(motor1pin1, LOW);
  analogWrite(motor1pin2, SPEED_TURN);

  // RIGHT motor backward
  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);
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
  Serial.println("PWM Line Follower Ready");
}

// ======================= MAIN LOOP ========================

void loop() {
  int R  = digitalRead(IR_RIGHT);
  int MR = digitalRead(IR_MID_RIGHT);
  int ML = digitalRead(IR_MID_LEFT);
  int L  = digitalRead(IR_LEFT);

  int M = (MR || ML); // center detection

  Serial.print("R="); Serial.print(R);
  Serial.print(" MR="); Serial.print(MR);
  Serial.print(" ML="); Serial.print(ML);
  Serial.print(" L="); Serial.println(L);

  // ---------------- FINISH LINE CHECK ----------------
  if (L == 1 && ML == 1 && MR == 1 && R == 1) {
    finishCounter++;
  } else {
    finishCounter = 0;
  }

  if (finishCounter > 10) {
    stopMotors();
    while (true);  // freeze at finish
  }

  // ---------------- LINE FOLLOWING LOGIC --------------
  if (M == 1) {
    forward();
  }
  else {
    if (L == 1 && R == 0) {
      left();
    }
    else if (R == 1 && L == 0) {
      right();
    }
    else if (L == 1 && R == 1) {
      left();  // your preferred rule
    }
    else {
      left();  // lost → search left
    }
  }

  delay(20);
}
