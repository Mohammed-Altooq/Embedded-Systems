// test if we can push
// test 2
//asdsadsada
int finishCounter = 0;
// Sensor pins (from RIGHT to LEFT: 8, 9, 10, 11)
const int IR_RIGHT = 8;      // right sensor
const int IR_MID_RIGHT = 9;  // middle-right sensor
const int IR_MID_LEFT = 10;  // middle-left sensor (closer to left)
const int IR_LEFT = 11;      // left sensor

// Motor pins
const int motor1pin1 = 2;  // left motor IN1
const int motor1pin2 = 3;  // left motor IN2
const int motor2pin1 = 4;  // right motor IN1
const int motor2pin2 = 5;  // right motor IN2

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
  Serial.println("Line follower with 4 sensors");
}

void forward() {
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);
  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);
}

void left() {
  // turn left
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, HIGH);
  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);
}

void right() {
  // turn right
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

void loop() {
  int R = digitalRead(IR_RIGHT);
  int MR = digitalRead(IR_MID_RIGHT);
  int ML = digitalRead(IR_MID_LEFT);
  int L = digitalRead(IR_LEFT);

  int M = (MR || ML);  // any middle sensor sees black

  Serial.print("R=");
  Serial.print(R);
  Serial.print(" MR=");
  Serial.print(MR);
  Serial.print(" ML=");
  Serial.print(ML);
  Serial.print(" L=");
  Serial.println(L);

  // ---------- FINISH LINE CHECK WITH COUNTER ----------
  if (L == 1 && ML == 1 && MR == 1 && R == 1) {
    finishCounter++;  // all sensors black → count
  } else {
    finishCounter = 0;  // not all black → reset
  }

  // If robot sees ALL black for 1 second → STOP FOREVER
  // delay = 20 ms → 1000ms / 20ms = 50 loops ≈ 1 second
  if (finishCounter > 10) {
    stopMotors();
    while (true)
      ;  // freeze robot forever at finish
  }

  // ---------- NORMAL MAZE LOGIC ----------

  // 1) If center sees the line → go straight
  if (M == 1) {
    forward();
  } else {
    // 2) Center lost → use sides

    if (L == 1 && R == 0) {
      left();
    } else if (R == 1 && L == 0) {
      right();
    } else if (L == 1 && R == 1) {
      left();  // your rule: choose LEFT when both see
    } else {
      // nothing sees → search left
      left();
    }
  }

  delay(20);
}