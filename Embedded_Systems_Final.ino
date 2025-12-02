// ========== LINE FOLLOWER + LIGHTWEIGHT BFS RETURN ==========
// Memory-optimized for Arduino Uno (2KB SRAM).
// - EXPLORE: same line-following logic as your original code.
// - Log L/R junction decisions into forwardMoves[].
// - At FINISH: run a tiny BFS on an implicit chain 0..stepCount,
//   compute returnMoves[] (FINISH -> START), turn around,
//   then RETURN using those moves while still following the line.
// ============================================================

int finishCounter = 0;

// Sensor pins (from RIGHT to LEFT: 8, 9, 10, 11)
const int IR_RIGHT      = 8;   // right sensor
const int IR_MID_RIGHT  = 9;   // middle-right sensor
const int IR_MID_LEFT   = 10;  // middle-left sensor (closer to left)
const int IR_LEFT       = 11;  // left sensor

// Motor pins
const int motor1pin1 = 2;   // left motor IN1
const int motor1pin2 = 3;   // left motor IN2
const int motor2pin1 = 4;   // right motor IN1
const int motor2pin2 = 5;   // right motor IN2

// ===================== MODES =====================

enum Mode { EXPLORE, RETURN, DONE };
Mode mode = EXPLORE;

// ===================== PATH LOGGING =====================

// Limit on number of junction decisions we log.
// Keep this modest to save RAM. Increase only if needed.
const int MAX_STEPS = 60;

char forwardMoves[MAX_STEPS];   // moves from START -> FINISH ('L'/'R')
int stepCount = 0;

char returnMoves[MAX_STEPS];    // moves from FINISH -> START
int returnCount = 0;
int returnIndex = 0;

// ===================== BFS STRUCTURES =====================

// Nodes are 0..stepCount (chain).
const int MAX_NODES = MAX_STEPS + 1;

bool visited[MAX_NODES];
int parentNode[MAX_NODES];

// Simple queue and path buffer
int q[MAX_NODES];
int nodePath[MAX_NODES];

// ===================== MOVEMENT =====================

void forwardMotors() {
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);
  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);
}

void leftMotors() {
  // turn left
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, HIGH);
  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);
}

void rightMotors() {
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

// Semantic helpers like your original
void forward()  { forwardMotors(); }
void left()     { leftMotors();    }
void right()    { rightMotors();   }

// 180° turn at finish (tune delay for your robot)
void turnAround() {
  rightMotors();
  delay(500);   // adjust for ~180 degrees
  stopMotors();
}

// ===================== LOGGING =====================

void recordForwardMove(char m) {
  if (stepCount < MAX_STEPS) {
    forwardMoves[stepCount++] = m;
  }
}

// ===================== BFS ON IMPLICIT CHAIN =====================
// Nodes are 0..stepCount.
// Neighbors of u are: u-1 (if >=0) and u+1 (if <=stepCount).
// We BFS from FINISH node (stepCount) to START node (0).

void runBFSAndBuildReturn() {
  int N = stepCount + 1;   // number of nodes

  if (N < 2) {
    // No real moves made
    returnCount = 0;
    returnIndex = 0;
    return;
  }

  // Init BFS arrays
  for (int i = 0; i < N; i++) {
    visited[i] = false;
    parentNode[i] = -1;
  }

  int start = stepCount;  // FINISH node
  int goal  = 0;          // START node

  int head = 0, tail = 0;

  visited[start] = true;
  parentNode[start] = -1;
  q[tail++] = start;

  bool found = false;

  while (head < tail) {
    int u = q[head++];

    if (u == goal) {
      found = true;
      break;
    }

    // Neighbor 1: u - 1
    if (u - 1 >= 0 && !visited[u - 1]) {
      visited[u - 1] = true;
      parentNode[u - 1] = u;
      q[tail++] = u - 1;
    }

    // Neighbor 2: u + 1
    if (u + 1 < N && !visited[u + 1]) {
      visited[u + 1] = true;
      parentNode[u + 1] = u;
      q[tail++] = u + 1;
    }
  }

  if (!found) {
    Serial.println("BFS: no path from FINISH to START (unexpected for chain).");
    returnCount = 0;
    returnIndex = 0;
    return;
  }

  // Reconstruct node path from START back to FINISH
  int len = 0;
  int cur = goal;
  while (cur != -1 && len < N) {
    nodePath[len++] = cur;
    cur = parentNode[cur];
  }

  // Debug: print node path
  Serial.print("BFS nodes (START->FINISH): ");
  for (int i = 0; i < len; i++) {
    Serial.print(nodePath[i]);
    if (i < len - 1) Serial.print(" -> ");
  }
  Serial.println();

  // Build forward path moves along this node sequence.
  // Edge between nodePath[i] and nodePath[i+1] corresponds to
  // forwardMoves[min(u,v)] (for chain).
  char pathForwardMoves[MAX_STEPS];
  int pathMoveCount = 0;

  for (int i = 0; i < len - 1 && pathMoveCount < MAX_STEPS; i++) {
    int u = nodePath[i];
    int v = nodePath[i + 1];
    int idx = (u < v) ? u : v;   // edge index in forwardMoves

    if (idx >= 0 && idx < stepCount) {
      pathForwardMoves[pathMoveCount++] = forwardMoves[idx];
    }
  }

  // Now derive returnMoves by reversing pathForwardMoves and swapping L <-> R
  returnCount = 0;

  for (int i = pathMoveCount - 1; i >= 0 && returnCount < MAX_STEPS; i--) {
    char fm = pathForwardMoves[i];
    char rm;
    if (fm == 'L')      rm = 'R';
    else if (fm == 'R') rm = 'L';
    else                rm = 'F';
    returnMoves[returnCount++] = rm;
  }

  Serial.print("Return moves (FINISH->START): ");
  for (int i = 0; i < returnCount; i++) {
    Serial.print(returnMoves[i]);
    Serial.print(' ');
  }
  Serial.println();

  returnIndex = 0;
}

// ===================== SETUP =====================

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
  Serial.println("Line follower with lightweight BFS return");
}

// ===================== MAIN LOOP =====================

void loop() {
  int R   = digitalRead(IR_RIGHT);
  int MR  = digitalRead(IR_MID_RIGHT);
  int ML  = digitalRead(IR_MID_LEFT);
  int L   = digitalRead(IR_LEFT);

  int M = (MR || ML); // any middle sensor sees black

  // Debug (you can comment these out to save a bit more time/serial spam)
  Serial.print("Mode=");
  Serial.print(mode == EXPLORE ? "EXP" : (mode == RETURN ? "RET" : "DONE"));
  Serial.print(" | R=");
  Serial.print(R);
  Serial.print(" MR=");
  Serial.print(MR);
  Serial.print(" ML=");
  Serial.print(ML);
  Serial.print(" L=");
  Serial.println(L);

  // ---------- FINISH LINE DETECTION ----------
  if (L == 1 && ML == 1 && MR == 1 && R == 1) {
    finishCounter++;      // all sensors black → count
  } else {
    finishCounter = 0;    // not all black → reset
  }

  if (mode == EXPLORE && finishCounter > 10) {
    // Reached FINISH in exploration
    stopMotors();
    Serial.println("FINISH reached in EXPLORE mode.");

    // Compute BFS-based return path
    runBFSAndBuildReturn();

    // Turn around to face back toward START
    turnAround();

    // Switch to RETURN mode
    mode = RETURN;
    return;
  }

  if (mode == EXPLORE) {
    // ---------- ORIGINAL MAZE LOGIC (EXPLORATION) ----------
    if (M == 1) {
      // center sees the line → go straight
      forward();
      // We do NOT log forward; only explicit L/R choices.
    } else {
      // Center lost → use sides

      if (L == 1 && R == 0) {
        left();
        recordForwardMove('L');
      } else if (R == 1 && L == 0) {
        right();
        recordForwardMove('R');
      } else if (L == 1 && R == 1) {
        left();    // your rule: choose LEFT when both see
        recordForwardMove('L');
      } else {
        // nothing sees → search left
        left();
        // not logged as a graph decision
      }
    }

  } else if (mode == RETURN) {
    if (returnIndex >= returnCount) {
      // We used all return moves → assume we are back at START.
      stopMotors();
      Serial.println("RETURN complete. Back at START (assumed). DONE.");
      mode = DONE;
      while (true); // freeze
    }

    char cmd = returnMoves[returnIndex];

    bool junctionLike = (M == 0 && (L == 1 || R == 1)) || (M == 1 && (L == 1 || R == 1));

    if (junctionLike) {
      // Apply planned return decision at junction
      if (cmd == 'L') {
        left();
        Serial.println("RETURN: LEFT at junction");
      } else if (cmd == 'R') {
        right();
        Serial.println("RETURN: RIGHT at junction");
      } else { // 'F' or unknown
        if (M == 1) {
          forward();
          Serial.println("RETURN: FORWARD at junction");
        } else if (L == 1) {
          left();
        } else if (R == 1) {
          right();
        } else {
          left();
        }
      }
      returnIndex++;
    } else {
      // Not a junction: just stay on the line using same logic
      if (M == 1) {
        forward();
      } else {
        if (L == 1 && R == 0) {
          left();
        } else if (R == 1 && L == 0) {
          right();
        } else if (L == 1 && R == 1) {
          left();
        } else {
          left();
        }
      }
    }

  } else {
    // DONE
    stopMotors();
  }

  delay(20);
}
