// ======================= FINAL CODE ==========================
//  Line Follower + PWM + Lightweight BFS Return to Start
// =============================================================

// ------------------- Sensor Pins -----------------------------
const int IR_RIGHT      = 8;
const int IR_MID_RIGHT  = 9;
const int IR_MID_LEFT   = 10;
const int IR_LEFT       = 11;

// ------------------- Motor Pins (your setup) -----------------
const int motor1pin1 = 2;   // LEFT motor IN1  (direction)
const int motor1pin2 = 3;   // LEFT motor IN2  (PWM)

const int motor2pin1 = 4;   // RIGHT motor IN1 (direction)
const int motor2pin2 = 5;   // RIGHT motor IN2 (PWM)

// ------------------- Speed Settings --------------------------
const int SPEED_FWD  = 80;     // Forward speed (PWM) 0–255
const int SPEED_TURN = 70;     // Turning PWM speed
const int SPEED_BACK = 100;    // Backward full-speed

// ------------------- Modes -----------------------------------
enum Mode { EXPLORE, RETURN, DONE };
Mode mode = EXPLORE;

// ------------------- Path Logging ----------------------------
const int MAX_STEPS = 60;
char forwardMoves[MAX_STEPS];   // Moves START -> FINISH
int stepCount = 0;

char returnMoves[MAX_STEPS];    // Moves FINISH -> START
int returnCount = 0;
int returnIndex = 0;

// ------------------- BFS Structures --------------------------
bool visited[MAX_STEPS + 1];
int parentNode[MAX_STEPS + 1];
int q[MAX_STEPS + 1];
int nodePath[MAX_STEPS + 1];

// ------------------- Globals ---------------------------------
int finishCounter = 0;

// ================== Motor Control (PWM Improved) =============

// Forward both motors using PWM on IN2 pins
void forwardMotors() {
  // LEFT motor forward
  digitalWrite(motor1pin1, LOW);
  analogWrite(motor1pin2, SPEED_FWD);

  // RIGHT motor forward
  digitalWrite(motor2pin1, LOW);
  analogWrite(motor2pin2, SPEED_FWD);
}

void leftMotors() {
  // LEFT motor backward (full speed)
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);

  // RIGHT motor forward (slower PWM)
  digitalWrite(motor2pin1, LOW);
  analogWrite(motor2pin2, SPEED_TURN);
}

void rightMotors() {
  // LEFT motor forward (PWM)
  digitalWrite(motor1pin1, LOW);
  analogWrite(motor1pin2, SPEED_TURN);

  // RIGHT motor backward (full speed)
  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);
}

void stopMotors() {
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, LOW);
  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, LOW);
}

void turnAround() {
  rightMotors();
  delay(600);   // Adjust for your robot's 180° turn
  stopMotors();
}


// ================== Logging =============================

void recordForwardMove(char m) {
  if (stepCount < MAX_STEPS) {
    forwardMoves[stepCount++] = m;
  }
}


// ================== BFS (Chain) ===========================
// Graph is implicitly: 0–1–2–3–...–stepCount

void runBFSAndBuildReturn() {
  int N = stepCount + 1;

  // Init BFS arrays
  for (int i = 0; i < N; i++) {
    visited[i] = false;
    parentNode[i] = -1;
  }

  int start = stepCount; // FINISH
  int goal  = 0;         // START

  int head = 0, tail = 0;

  visited[start] = true;
  q[tail++] = start;

  bool found = false;

  while (head < tail) {
    int u = q[head++];

    if (u == goal) { found = true; break; }

    // neighbor u-1
    if (u - 1 >= 0 && !visited[u - 1]) {
      visited[u - 1] = true;
      parentNode[u - 1] = u;
      q[tail++] = u - 1;
    }

    // neighbor u+1
    if (u + 1 < N && !visited[u + 1]) {
      visited[u + 1] = true;
      parentNode[u + 1] = u;
      q[tail++] = u + 1;
    }
  }

  // Reconstruct BFS path
  int len = 0;
  int cur = goal;
  while (cur != -1 && len < N) {
    nodePath[len++] = cur;
    cur = parentNode[cur];
  }

  // Build forward moves along BFS path
  char pathForwardMoves[MAX_STEPS];
  int pathMoveCount = 0;

  for (int i = 0; i < len - 1; i++) {
    int u = nodePath[i];
    int v = nodePath[i+1];
    int idx = min(u, v);

    if (idx >= 0 && idx < stepCount)
      pathForwardMoves[pathMoveCount++] = forwardMoves[idx];
  }

  // Build returnMoves = reverse + swap L<->R
  returnCount = 0;

  for (int i = pathMoveCount - 1; i >= 0; i--) {
    char fm = pathForwardMoves[i];
    char rm = 'F';
    if (fm == 'L') rm = 'R';
    if (fm == 'R') rm = 'L';

    returnMoves[returnCount++] = rm;
  }

  returnIndex = 0;
}


// ================== SETUP ===============================

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
  Serial.println("Final Robot Code: PWM + BFS Return");
}


// ================== MAIN LOOP ===========================

void loop() {

  int R  = digitalRead(IR_RIGHT);
  int MR = digitalRead(IR_MID_RIGHT);
  int ML = digitalRead(IR_MID_LEFT);
  int L  = digitalRead(IR_LEFT);

  int M = (MR || ML); // middle sees line

  // ------------ FINISH DETECTION --------------

  if (L == 1 && ML == 1 && MR == 1 && R == 1) finishCounter++;
  else finishCounter = 0;

  if (mode == EXPLORE && finishCounter > 10) {
    stopMotors();
    Serial.println("FINISH reached. Performing BFS...");

    runBFSAndBuildReturn();
    turnAround();

    mode = RETURN;
    return;
  }


  // ================= EXPLORE MODE =================

  if (mode == EXPLORE) {

    if (M == 1) {
      forwardMotors();
    } else {
      if (L == 1 && R == 0) {
        leftMotors();
        recordForwardMove('L');
      } else if (R == 1 && L == 0) {
        rightMotors();
        recordForwardMove('R');
      } else if (L == 1 && R == 1) {
        leftMotors();
        recordForwardMove('L');
      } else {
        leftMotors(); // correction
      }
    }

  }

  // ================= RETURN MODE =================

  else if (mode == RETURN) {

    if (returnIndex >= returnCount) {
      stopMotors();
      Serial.println("RETURN complete. Back at START.");
      mode = DONE;
      return;
    }

    char cmd = returnMoves[returnIndex];

    bool junction = (M == 0 && (L || R)) || (M == 1 && (L || R));

    if (junction) {
      if (cmd == 'L') {
        leftMotors();
      } else if (cmd == 'R') {
        rightMotors();
      } else {
        forwardMotors();
      }

      returnIndex++;
    } else {
      if (M == 1) forwardMotors();
      else if (L == 1) leftMotors();
      else if (R == 1) rightMotors();
      else leftMotors();
    }
  }

  delay(20);
}
