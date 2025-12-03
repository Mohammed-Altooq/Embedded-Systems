int finishCounter = 0;

// Sensor pins (RIGHT → LEFT)
const int IR_RIGHT = 11;
const int IR_MID_RIGHT = 10;
const int IR_MID_LEFT = 9;
const int IR_LEFT = 8;

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
  Serial.println("Line follower with backward correction");
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

void loop() {

  int R = digitalRead(IR_RIGHT);
  int MR = digitalRead(IR_MID_RIGHT);
  int ML = digitalRead(IR_MID_LEFT);
  int L = digitalRead(IR_LEFT);

  int M = MR || ML;  // any middle sensor sees black

  // ---------- FINISH LINE DETECTION ----------
  if (L == 1 && ML == 1 && MR == 1 && R == 1) {
    finishCounter++;
  } else {
    finishCounter = 0;
  }

  if (finishCounter > 10) {
    stopMotors();
    while (true) {}
  }

  // ---------- BACKWARD RECOVERY ----------
  if (L == 0 && ML == 0 && MR == 0 && R == 0) {
    backward();
    delay(120);       // small bump backward
    stopMotors();
    delay(50);
    return;           // after recovery attempt, restart loop
  }

  // ---------- NORMAL LINE FOLLOWING ----------
  if (M == 1) {
    forward();
  } else {
    if (L == 1 && R == 0) left();
    else if (R == 1 && L == 0) right();
    else if (L == 1 && R == 1) left();   // your rule
    else stopMotors();
  }

  delay(20);
}
