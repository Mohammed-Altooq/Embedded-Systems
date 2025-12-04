int finishCounter = 0;
int allWhiteCounter = 0;   // debounce for dead-end / lost detection
 
// Sensor pins (RIGHT → LEFT)
const int IR_RIGHT      = 11;
const int IR_MID_RIGHT  = 10;
const int IR_MID_LEFT   = 9;
const int IR_LEFT       = 8;
 
// Motor pins
const int motor1pin1 = 2;  
const int motor1pin2 = 3;  
const int motor2pin1 = 4;  
const int motor2pin2 = 5;
 
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
  Serial.println("Left-hand maze solver with dead-end handling");
  delay(1000);
}
 
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
 
void backward() {
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);
  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);
}
 
void stopMotors() {
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, LOW);
  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, LOW);
}
 
// Turn around in place until middle sees line again
void turnAround() {
  Serial.println("U-turn");
  while (true) {
    int R  = digitalRead(IR_RIGHT);
    int MR = digitalRead(IR_MID_RIGHT);
    int ML = digitalRead(IR_MID_LEFT);
    int L  = digitalRead(IR_LEFT);
 
    // if middle sensors see black, stop turning
    if (MR == 1 || ML == 1) {
      stopMotors();
      break;
    }
 
    // rotate on spot to the left
    digitalWrite(motor1pin1, LOW);
    digitalWrite(motor1pin2, HIGH);
    digitalWrite(motor2pin1, HIGH);
    digitalWrite(motor2pin2, LOW);
    delay(25);
    stopMotors();
    delay(5);
  }
}
 
// Dead-end / "really lost" handler
void handleDeadEnd() {
  Serial.println("Dead-end / lost: trying to recover");
 
  // 1) Small bump backward
  backward();
  delay(150);
  stopMotors();
  delay(50);
 
  // 2) Check sensors again
  int R  = digitalRead(IR_RIGHT);
  int MR = digitalRead(IR_MID_RIGHT);
  int ML = digitalRead(IR_MID_LEFT);
  int L  = digitalRead(IR_LEFT);
 
  // If we found the line again after backing up → done
  if (L == 1 || ML == 1 || MR == 1 || R == 1) {
    Serial.println("Line found again after backing up");
    return;
  }
 
  // 3) Still all white → assume true dead end → U-turn
  Serial.println("Still white: true dead end, turning around");
  turnAround();
}
 
void loop() {
  int R  = digitalRead(IR_RIGHT);
  int MR = digitalRead(IR_MID_RIGHT);
  int ML = digitalRead(IR_MID_LEFT);
  int L  = digitalRead(IR_LEFT);
 
  int M  = MR || ML;  // any middle sensor sees black
 
  // ---------- FINISH LINE DETECTION ----------
  if (L == 1 && ML == 1 && MR == 1 && R == 1) {
    finishCounter++;
  } else {
    finishCounter = 0;
  }
 
  if (finishCounter > 10) {
    stopMotors();
    Serial.println("Finish line reached, stopping.");
    while (true) {}
  }
 
  // ---------- ALL-WHITE DEBOUNCE (possible dead-end / lost) ----------
  if (L == 0 && ML == 0 && MR == 0 && R == 0) {
    allWhiteCounter++;
  } else {
    allWhiteCounter = 0;
  }
 
  // Only handle dead-end if it's been white for a while (avoid glitches)
  if (allWhiteCounter > 5) {      // ~100ms of solid white
    allWhiteCounter = 0;
    handleDeadEnd();
    delay(20);
    return;
  }
 
  // ---------- LEFT-HAND EXPLORATION LOGIC (hug left but still walk straight) ----------
 
  // 1) LEFT-HAND RULE:
  // If there is a left branch while still on main line -> turn left
  // "Left branch" = left sensor + at least one middle sensor on black
  if (L == 1 && (ML == 1 || MR == 1)) {
    Serial.println("LEFT BRANCH → turn left (left-hand rule)");
    left();
    delay(80);   // commit into the turn a bit
    return;
  }
 
  // 2) Otherwise, go STRAIGHT if middle sees line
  if (M == 1) {
    forward();
    return;
  }
 
  // 3) If no left & no straight, but right + middle sees a branch → turn right
  if (R == 1 && (ML == 1 || MR == 1)) {
    Serial.println("RIGHT BRANCH → turn right");
    right();
    delay(80);
    return;
  }
 
  // 4) Small corrections when only one side sees the line
  if (L == 1 && R == 0) {
    left();
  } else if (R == 1 && L == 0) {
    right();
  } else {
    // Probably near lost, but not fully white yet → stop briefly
    stopMotors();
  }
 
  delay(20);
}