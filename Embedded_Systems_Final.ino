int finishCounter = 0;
int allWhiteCounter = 0;
 
// Sensor pins (RIGHT → LEFT)
const int IR_RIGHT      = 11;
const int IR_MID_RIGHT  = 10;
const int IR_MID_LEFT   = 9;
const int IR_LEFT       = 8;
 
// Motor pins (L298N)
const int motor1pin1 = 2;  
const int motor1pin2 = 3;  
const int motor2pin1 = 4;  
const int motor2pin2 = 5;
 
// ------------------ MOTOR FUNCTIONS ------------------
 
void forward() {
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, HIGH);
  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, HIGH);
}
 
void left() {
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, HIGH);
  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);
}
 
void right() {
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);
  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, HIGH);
}
 
void stopMotors() {
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, LOW);
  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, LOW);
}
 
// ------------------ U-TURN (LEFT ROTATION) ------------------
 
void turnAround() {
  while (true) {
    int MR = digitalRead(IR_MID_RIGHT);
    int ML = digitalRead(IR_MID_LEFT);
 
    if (MR == 1 || ML == 1) {
      stopMotors();
      delay(50);
      forward();
      delay(80);
      stopMotors();
      break;
    }
    left();
    delay(25);
  }
}
 
// ------------------ SETUP ------------------
 
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
  delay(500);
}
 
// ------------------ MAIN LOOP ------------------
 
void loop() {
  int R  = digitalRead(IR_RIGHT);
  int MR = digitalRead(IR_MID_RIGHT);
  int ML = digitalRead(IR_MID_LEFT);
  int L  = digitalRead(IR_LEFT);
 
  int M  = MR || ML;  // middle sees line?
 
  // ---------- FINISH LINE (ALL BLACK) ----------
  if (L == 1 && ML == 1 && MR == 1 && R == 1) finishCounter++;
  else finishCounter = 0;
 
  if (finishCounter > 10) {
    stopMotors();
    while (true) {}
  }
 
  // ---------- POSSIBLE DEAD-END (ALL WHITE) ----------
  if (L == 0 && ML == 0 && MR == 0 && R == 0) {
    allWhiteCounter++;
  } else {
    allWhiteCounter = 0;
  }
 
  if (allWhiteCounter > 5) {
    allWhiteCounter = 0;
 
    // ------------------ PEEK RIGHT CHECK ------------------
    stopMotors();
    delay(30);
    right();        // peek a little to check for a hidden right branch
    delay(120);     // L298N small angle (tune 80–130 ms)
    stopMotors();
    delay(30);
 
    int R2  = digitalRead(IR_RIGHT);
    int MR2 = digitalRead(IR_MID_RIGHT);
    int ML2 = digitalRead(IR_MID_LEFT);
 
    // If right found -> commit full right turn
    if (R2 == 1 || MR2 == 1 || ML2 == 1) {
      right();
      delay(250);   // Full 90° turn on L298N (tune 200–300 ms)
      stopMotors();
      return;
    }
 
    // If no right turn, return to center position
    left();
    delay(100);
    stopMotors();
    delay(20);
 
    // ------------------ FORWARD REVEAL TEST ------------------
    forward();      // small forward to reveal corners / overshoot fix
    delay(70);
    stopMotors();
    delay(20);
 
    // Read again
    R  = digitalRead(IR_RIGHT);
    MR = digitalRead(IR_MID_RIGHT);
    ML = digitalRead(IR_MID_LEFT);
    L  = digitalRead(IR_LEFT);
 
    // If line appeared -> continue
    if (R || MR || ML || L) return;
 
    // Real dead-end -> U-turn
    turnAround();
    return;
  }
 
  // ---------- NORMAL LEFT-HAND RULE (YOUR ORIGINAL PRIORITY) ----------
 
  // 1) Prefer LEFT TURN
  if (L == 1 && (ML == 1 || MR == 1)) {
    left();
    delay(70);
    return;
  }
 
  // 2) If straight available, take it
  if (M == 1) {
    forward();
    return;
  }
 
  // 3) If no straight, take RIGHT
  if (R == 1 && (ML == 1 || MR == 1)) {
    right();
    delay(70);
    return;
  }
 
  // 4) Small corrections if drifting
  if (L == 1) left();
  else if (R == 1) right();
  else stopMotors();
 
  delay(20);
}