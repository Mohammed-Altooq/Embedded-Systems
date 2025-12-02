// ================== Line Maze with BFS Return ==================
// Uses 4 IR sensors and 2 motors (your pinout).
// Phase 1: Explore maze using simple line-following + logging.
// Phase 2: After detecting finish, run BFS to compute optimal path
//          back to start, then execute that path.
//
// NOTE: This is a simplified educational example.
// - Cell transitions are time-based (delay) approximations.
// - BFS is used on an abstract grid map built during exploration.
// - Tune delays (CELL_FORWARD_TIME, TURN_TIME) for your robot.
//
// ===============================================================

// ----------------- Original Pin Definitions -----------------
int finishCounter = 0;

// Sensor pins (from RIGHT to LEFT: 8, 9, 10, 11)
const int IR_RIGHT     = 8;   // right sensor
const int IR_MID_RIGHT = 9;   // middle-right sensor
const int IR_MID_LEFT  = 10;  // middle-left sensor
const int IR_LEFT      = 11;  // left sensor

// Motor pins
const int motor1pin1 = 2;  // left motor IN1
const int motor1pin2 = 3;  // left motor IN2
const int motor2pin1 = 4;  // right motor IN1
const int motor2pin2 = 5;  // right motor IN2

// ----------------- Movement Timing (TUNE THESE) -----------------
const int CELL_FORWARD_TIME = 400;  // ms: time to move approx one cell
const int TURN_TIME         = 300;  // ms: time for ~90-degree turn

// ----------------- Maze / BFS Structures -----------------
// Direction (absolute)
enum Dir { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3 };

// Mode of the robot
enum Mode { EXPLORE, RETURN };
Mode mode = EXPLORE;

// Maze grid size (adjust if your maze is larger)
const int ROWS = 5;
const int COLS = 5;

// Each cell: visited flag + walls in 4 directions
struct Cell {
  bool visited;
  bool wall[4];  // wall[NORTH], wall[EAST], wall[SOUTH], wall[WEST]
};

Cell maze[ROWS][COLS];

// Robot logical position and heading in the grid
int curR = ROWS - 1;  // start at bottom-left (you can change)
int curC = 0;
Dir curDir = NORTH;

int startR = curR;
int startC = curC;

int goalR = -1;
int goalC = -1;

// BFS data
bool seen[ROWS][COLS];
int parentCell[ROWS][COLS]; // parent as r*COLS + c

struct QueueItem {
  int r;
  int c;
};

QueueItem q[ROWS * COLS];
int qHead = 0, qTail = 0;

// Commands for shortest path back: 'F','L','R','U'
char pathCommands[ROWS * COLS * 2];
int pathLen = 0;

// To run return sequence once
bool bfsDone = false;
bool returnStarted = false;

// ----------------- Motor Helper Functions -----------------
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

// Move robot ~one cell forward (time-based)
void moveOneCellForward() {
  forwardMotors();
  delay(CELL_FORWARD_TIME);
  stopMotors();
}

// Turn left ~90° and update direction
void turnLeft90() {
  leftMotors();
  delay(TURN_TIME);
  stopMotors();
  curDir = (Dir)((curDir + 3) % 4); // rotate left
}

// Turn right ~90° and update direction
void turnRight90() {
  rightMotors();
  delay(TURN_TIME);
  stopMotors();
  curDir = (Dir)((curDir + 1) % 4); // rotate right
}

// 180° turn
void turnAround() {
  rightMotors();
  delay(2 * TURN_TIME);
  stopMotors();
  curDir = (Dir)((curDir + 2) % 4);
}

// Update grid coordinates after moving one cell forward
void updatePositionForward() {
  if (curDir == NORTH)      curR--;
  else if (curDir == SOUTH) curR++;
  else if (curDir == EAST)  curC++;
  else if (curDir == WEST)  curC--;

  // clamp to grid just in case
  if (curR < 0) curR = 0;
  if (curR >= ROWS) curR = ROWS - 1;
  if (curC < 0) curC = 0;
  if (curC >= COLS) curC = COLS - 1;
}

// Get opposite direction
Dir oppositeDir(Dir d) {
  return (Dir)((d + 2) % 4);
}

// ----------------- Sensor Reading -----------------
void readSensors(int &R, int &MR, int &ML, int &L) {
  R  = digitalRead(IR_RIGHT);
  MR = digitalRead(IR_MID_RIGHT);
  ML = digitalRead(IR_MID_LEFT);
  L  = digitalRead(IR_LEFT);
}

// ----------------- Maze Logging During Exploration -----------------
// Mark that from (curR,curC) we moved one cell in direction dAbs
// Also open the corresponding wall in neighbor cell.
void logMovement(Dir dAbs) {
  // current cell open in direction dAbs
  maze[curR][curC].wall[dAbs] = false;

  // neighbor coordinates
  int nr = curR;
  int nc = curC;

  if (dAbs == NORTH)      nr--;
  else if (dAbs == SOUTH) nr++;
  else if (dAbs == EAST)  nc++;
  else if (dAbs == WEST)  nc--;

  if (nr < 0 || nr >= ROWS || nc < 0 || nc >= COLS) {
    return; // out of grid, ignore
  }

  // neighbor cell also open in opposite direction
  Dir opp = oppositeDir(dAbs);
  maze[nr][nc].wall[opp] = false;
}

// Decide next move in EXPLORE mode based on sensor pattern
// Return: 'F' (forward), 'L' (left), 'R' (right), 'U' (U-turn)
char decideNextMoveExplore() {
  int R, MR, ML, L;
  readSensors(R, MR, ML, L);
  int M = (MR || ML);

  // Debug print
  Serial.print("R="); Serial.print(R);
  Serial.print(" MR="); Serial.print(MR);
  Serial.print(" ML="); Serial.print(ML);
  Serial.print(" L="); Serial.println(L);

  // Basic same logic as your original code:
  if (M == 1) {
    return 'F';
  } else {
    if (L == 1 && R == 0) {
      return 'L';
    } else if (R == 1 && L == 0) {
      return 'R';
    } else if (L == 1 && R == 1) {
      // your rule: choose LEFT when both see
      return 'L';
    } else {
      // nothing sees → search left (you can change to 'U' if needed)
      return 'L';
    }
  }
}

// Execute a single navigation command and update maze map
void executeExploreStep() {
  char cmd = decideNextMoveExplore();

  // Compute absolute move direction based on cmd and current direction
  Dir moveDir = curDir;

  if (cmd == 'L') {
    // we will turn left then go forward
    turnLeft90();
    moveDir = curDir;
    moveOneCellForward();
  } else if (cmd == 'R') {
    turnRight90();
    moveDir = curDir;
    moveOneCellForward();
  } else if (cmd == 'U') {
    turnAround();
    moveDir = curDir;
    moveOneCellForward();
  } else { // 'F'
    moveDir = curDir;
    moveOneCellForward();
  }

  // Log that we moved from (curR,curC) in direction moveDir
  logMovement(moveDir);

  // Now update our position in the grid
  updatePositionForward();

  // Mark visited
  maze[curR][curC].visited = true;
}

// ----------------- Finish Line Detection -----------------
bool checkFinishLine() {
  int R, MR, ML, L;
  readSensors(R, MR, ML, L);

  // all sensors black?
  if (L == 1 && ML == 1 && MR == 1 && R == 1) {
    finishCounter++;
  } else {
    finishCounter = 0;
  }

  // adjust threshold for how long finish must be seen
  if (finishCounter > 10) { // ~1 sec if loop delay ~20-50ms
    return true;
  }
  return false;
}

// ----------------- BFS and Path Construction -----------------
// BFS from (startR,startC) to (goalR,goalC) on known maze
void runBFS() {
  Serial.println("Running BFS for shortest path back...");

  // Init
  for (int r = 0; r < ROWS; r++) {
    for (int c = 0; c < COLS; c++) {
      seen[r][c] = false;
      parentCell[r][c] = -1;
    }
  }

  qHead = 0;
  qTail = 0;

  // Enqueue start cell (goal → start OR start → goal; we want path goal->start for return)
  // We'll do BFS from goal to start.
  q[qTail++] = {goalR, goalC};
  seen[goalR][goalC] = true;

  bool found = false;

  while (qHead < qTail) {
    QueueItem cur = q[qHead++];
    int r = cur.r;
    int c = cur.c;

    if (r == startR && c == startC) {
      found = true;
      break;
    }

    // Try 4 directions
    for (int d = 0; d < 4; d++) {
      if (maze[r][c].wall[d]) continue; // wall → no move

      int nr = r;
      int nc = c;

      if (d == NORTH)      nr--;
      else if (d == SOUTH) nr++;
      else if (d == EAST)  nc++;
      else if (d == WEST)  nc--;

      if (nr < 0 || nr >= ROWS || nc < 0 || nc >= COLS) continue;
      if (!maze[nr][nc].visited) continue;  // only use explored cells
      if (seen[nr][nc]) continue;

      seen[nr][nc] = true;
      parentCell[nr][nc] = r * COLS + c;
      q[qTail++] = {nr, nc};
    }
  }

  if (!found) {
    Serial.println("No path found by BFS!");
    pathLen = 0;
    return;
  }

  // Reconstruct path (cells) from start back to goal
  int cellPath[ROWS * COLS];
  int len = 0;

  int cr = startR;
  int cc = startC;

  while (!(cr == goalR && cc == goalC)) {
    cellPath[len++] = cr * COLS + cc;
    int p = parentCell[cr][cc];
    if (p == -1) {
      // Should not happen if found == true
      break;
    }
    cr = p / COLS;
    cc = p % COLS;
  }
  // include goal cell
  cellPath[len++] = goalR * COLS + goalC;

  // The cellPath is from start → goal (because we walked parents).
  // But robot is currently at goal and wants to go to start,
  // so we will traverse it backward to build movement commands.

  // Starting direction for RETURN phase is current direction at goal:
  Dir dCur = curDir;
  pathLen = 0;

  // Traverse: goal → ... → start
  for (int i = len - 1; i > 0; i--) {
    int cell1 = cellPath[i];     // current cell
    int cell2 = cellPath[i - 1]; // next cell

    int r1 = cell1 / COLS;
    int c1 = cell1 % COLS;
    int r2 = cell2 / COLS;
    int c2 = cell2 % COLS;

    Dir moveDir;

    if (r2 == r1 - 1 && c2 == c1)      moveDir = NORTH;
    else if (r2 == r1 + 1 && c2 == c1) moveDir = SOUTH;
    else if (r2 == r1 && c2 == c1 + 1) moveDir = EAST;
    else if (r2 == r1 && c2 == c1 - 1) moveDir = WEST;
    else                               continue; // not adjacent, skip

    // Convert from current heading dCur to moveDir => command
    char cmd;
    if (moveDir == dCur) {
      cmd = 'F';
    } else if (moveDir == (Dir)((dCur + 1) % 4)) {
      cmd = 'R';
    } else if (moveDir == (Dir)((dCur + 3) % 4)) {
      cmd = 'L';
    } else {
      cmd = 'U'; // 180 turn
      // after U-turn, heading changes by 180
    }

    pathCommands[pathLen++] = cmd;

    // update heading after executing this turn (for next segment)
    if (cmd == 'L') {
      dCur = (Dir)((dCur + 3) % 4);
    } else if (cmd == 'R') {
      dCur = (Dir)((dCur + 1) % 4);
    } else if (cmd == 'U') {
      dCur = (Dir)((dCur + 2) % 4);
    } // 'F' keeps heading
  }

  Serial.print("BFS path commands (goal->start): ");
  for (int i = 0; i < pathLen; i++) {
    Serial.print(pathCommands[i]);
    Serial.print(' ');
  }
  Serial.println();

  bfsDone = true;
}

// Execute a single command in RETURN mode (no logging needed)
void runReturnCommand(char cmd) {
  if (cmd == 'L') {
    turnLeft90();
    moveOneCellForward();
  } else if (cmd == 'R') {
    turnRight90();
    moveOneCellForward();
  } else if (cmd == 'U') {
    turnAround();
    moveOneCellForward();
  } else { // 'F'
    moveOneCellForward();
  }
}

// ----------------- Arduino Setup & Loop -----------------
void setup() {
  pinMode(IR_RIGHT,     INPUT);
  pinMode(IR_MID_RIGHT, INPUT);
  pinMode(IR_MID_LEFT,  INPUT);
  pinMode(IR_LEFT,      INPUT);

  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);

  Serial.begin(9600);
  Serial.println("Maze solver with BFS return");

  // Initialize maze
  for (int r = 0; r < ROWS; r++) {
    for (int c = 0; c < COLS; c++) {
      maze[r][c].visited = false;
      for (int d = 0; d < 4; d++) {
        maze[r][c].wall[d] = true; // unknown = wall initially
      }
    }
  }

  maze[curR][curC].visited = true;
}

void loop() {
  if (mode == EXPLORE) {
    // Check finish line first
    if (checkFinishLine()) {
      stopMotors();
      Serial.println("Finish detected! Setting goal and running BFS...");

      goalR = curR;
      goalC = curC;

      // Run BFS to compute optimal path back to start
      runBFS();

      // Now switch to RETURN mode (robot is already at goal)
      mode = RETURN;
      return;
    }

    // Normal exploration step
    executeExploreStep();

    delay(50); // small delay to avoid spamming
  }
  else if (mode == RETURN) {
    if (!bfsDone) {
      // Safety: if BFS somehow not done, just stop
      stopMotors();
      while (true);
    }

    // Execute the shortest path commands ONCE
    if (!returnStarted) {
      Serial.println("Starting RETURN along BFS path...");
      returnStarted = true;

      for (int i = 0; i < pathLen; i++) {
        runReturnCommand(pathCommands[i]);
      }

      stopMotors();
      Serial.println("RETURN complete. Stopping forever.");
      while (true); // freeze at start
    }
  }
}
