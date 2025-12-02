// ========== LINE FOLLOWER + LOGGING + BFS RETURN ==========
// Assumptions:
// - Maze is a line maze with junctions (no crazy loops).
// - First run: follow the line from START to FINISH using your original logic.
// - We log every 'L' and 'R' decision as we go.
// - At FINISH: we build a chain graph, run BFS from FINISH node back to START,
//   derive a sequence of return moves, then physically drive back to START,
//   using the line follower plus the stored moves.
//
// Pins and low-level movement are the same as your original code.
// ==========================================================

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

// We will log each high-level junction decision during EXPLORE.
// MAX_STEPS = max number of junction moves we expect.
const int MAX_STEPS = 200;
char forwardMoves[MAX_STEPS];   // moves taken from START -> FINISH ('L'/'R')
int stepCount = 0;

// Return moves from FINISH back to START (computed via BFS on chain graph)
char returnMoves[MAX_STEPS];
int returnCount = 0;
int returnIndex = 0;  // how many return commands we have already used

// ===================== BFS STRUCTURES =====================
// We model the path as a chain of nodes 0..stepCount, where:
//
// Node 0      = START
// Node i      = state after i-th decision
// Node stepCount = FINISH
//
// Edges: (0-1), (1-2), ..., (stepCount-1, stepCount)

const int MAX_NODES = MAX_STEPS + 1;
int adj[MAX_NODES][2];   // each node in a chain has at most 2 neighbors
int deg[MAX_NODES];
bool visited[MAX_NODES];
int parentNode[MAX_NODES];

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

// Your original semantic helpers
void forward()  { forwardMotors(); }
void left()     { leftMotors();    }
void right()    { rightMotors();   }

// 180° turn at finish
void turnAround() {
  rightMotors();
  delay(500);   // tune for ~180 degrees
  stopMotors();
}

// ===================== LOGGING =====================

// Log junction decision during EXPLORE.
void recordForwardMove(char m) {
  if (stepCount < MAX_STEPS) {
    forwardMoves[stepCount++] = m;
  }
}

// Build a simple chain graph 0..stepCount for BFS.
void buildChainGraph() {
  // Reset adjacency
  for (int i = 0; i <= stepCount; i++) {
    deg[i] = 0;
    visited[i] = false;
    parentNode[i] = -1;
    adj[i][0] = adj[i][1] = -1;
  }

  // Chain edges
  for (int i = 0; i < stepCount; i++) {
    int u = i;
    int v = i + 1;

    adj[u][deg[u]++] = v;
    adj[v][deg[v]++] = u;
  }
}

// Run BFS on the chain from FINISH node back to START node.
void runBFSAndBuildReturn() {
  buildChainGraph();

  int start = stepCount;  // FINISH node
  int goal  = 0;          // START node

  // BFS queue
  int q[MAX_NODES];
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

    for (int i = 0; i < deg[u]; i++) {
      int v = adj[u][i];
      if (v == -1) continue;
      if (!visited[v]) {
        visited[v] = true;
        parentNode[v] = u;
        q[tail++] = v;
      }
    }
  }

  if (!found) {
    Serial.println("BFS: no path from FINISH to START (shouldn't happen in chain)");
    returnCount = 0;
    return;
  }

  // Reconstruct node path from START back to FINISH
  int nodePath[MAX_NODES];
  int len = 0;
  int cur = goal;
  while (cur != -1) {
    nodePath[len++] = cur;
    cur = parentNode[cur];
  }

  // For debug: print path in terms of node indices
  Serial.print("BFS nodes from START to FINISH: ");
  for (int i = 0; i < len; i++) {
    Serial.print(nodePath[i]);
    if (i < len - 1) Serial.print(" -> ");
  }
  Serial.println();

  // Now derive returnMoves from forwardMoves.
  //
  // forwardMoves[i] is the move taken between node i and i+1
  // from START to FINISH. To go back, we walk the path from
  // START to FINISH (nodePath), but interpret edges in reverse.
  //
  // On a line, BFS path is basically 0..stepCount, so return
  // moves will be the forwardMoves reversed and with L<->R swapped.

  returnCount = 0;

  // 1) Extract the sequence of "edges" along the BFS path in forward direction.
  char pathForwardMoves[MAX_STEPS];
  int pathMoveCount = 0;

  // For path nodes [n0, n1, n2, ..., nk], edges are between (n0,n1), (n1,n2), ...
  for (int i = 0; i < len - 1; i++) {
    int u = nodePath[i];
    int v = nodePath[i + 1];

    // Edges correspond to forwardMoves[index] where index = min(u,v)
    int idx = (u < v) ? u : v;
    if (idx >= 0 && idx < stepCount) {
      pathForwardMoves[pathMoveCount++] = forwardMoves[idx];
    }
  }

  // 2) Reverse edges and swap L <-> R for return.
  for (int i = pathMoveCount - 1; i >= 0; i--) {
    char fm = pathForwardMoves[i];
    char rm;
    if (fm == 'L')      rm = 'R';
    else if (fm == 'R') rm = 'L';
    else                rm = 'F';   // just in case
    returnMoves[returnCount++] = rm;
  }

  Serial.print("Return moves (FINISH -> START): ");
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
  Serial.println("Line follower with BFS-based return");
}

// ===================== MAIN LOOP =====================

void loop() {
  int R   = digitalRead(IR_RIGHT);
  int MR  = digitalRead(IR_MID_RIGHT);
  int ML  = digitalRead(IR_MID_LEFT);
  int L   = digitalRead(IR_LEFT);

  int M = (MR || ML); // any middle sensor sees black

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

  // ========== FINISH LINE DETECTION (BOTH MODES) ==========

  if (L == 1 && ML == 1 && MR == 1 && R == 1) {
    finishCounter++;      // all sensors black → count
  } else {
    finishCounter = 0;    // not all black → reset
  }

  if (mode == EXPLORE && finishCounter > 10) {
    // We reached finish for the first time
    stopMotors();
    Serial.println("FINISH reached in EXPLORE mode.");

    // Build BFS graph + compute return moves
    runBFSAndBuildReturn();

    // Turn around to face back towards START
    turnAround();

    // Switch to RETURN mode
    mode = RETURN;
    return;
  }

  // In RETURN, optionally stop if we detect START line pattern (if you have it)
  // For now we will stop when we consume all returnMoves.

  // ========== BEHAVIOUR BY MODE ==========

  if (mode == EXPLORE) {
    // ---------- ORIGINAL MAZE LOGIC (EXPLORATION) ----------
    if (M == 1) {
      // center sees the line → go straight
      forward();
      // We do NOT record forward moves here, only explicit L/R decisions.
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
        // this is "search/correction", we won't log it as a graph decision
      }
    }

  } else if (mode == RETURN) {
    // ---------- RETURN USING RECORDED MOVES + LINE FOLLOWER ----------

    if (returnIndex >= returnCount) {
      // We've applied all return moves → we should be back at START.
      stopMotors();
      Serial.println("RETURN complete. Assuming we are back at START. DONE.");
      mode = DONE;
      while (true);  // freeze
    }

    // Basic line-following to stay on the path,
    // but at "decision points" we force the choice based on returnMoves[].

    char cmd = returnMoves[returnIndex];

    // We treat "junction-ish" situations as moments to apply cmd.
    bool junctionLike = (M == 0 && (L == 1 || R == 1)) || (M == 1 && (L == 1 || R == 1));

    if (junctionLike) {
      // We are likely at/near a junction → apply next command.
      if (cmd == 'L') {
        left();
        Serial.println("RETURN: taking LEFT at junction");
      } else if (cmd == 'R') {
        right();
        Serial.println("RETURN: taking RIGHT at junction");
      } else {
        // 'F' or unknown → prefer straight
        if (M == 1) {
          forward();
          Serial.println("RETURN: going FORWARD at junction");
        } else if (L == 1) {
          left();
        } else if (R == 1) {
          right();
        } else {
          // lost → light correction
          left();
        }
      }
      returnIndex++;   // we used this return command
    } else {
      // Not a junction → use normal follower logic to stay on line.
      if (M == 1) {
        forward();
      } else {
        if (L == 1 && R == 0) {
          left();
        } else if (R == 1 && L == 0) {
          right();
        } else if (L == 1 && R == 1) {
          // ambiguous, keep some left bias
          left();
        } else {
          left(); // search
        }
      }
    }

  } else {
    // DONE mode
    stopMotors();
  }

  delay(20);
}
