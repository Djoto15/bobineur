/*
  Coil winder - 2 stepper motors (A4988 / DRV8825 drivers, 1/16 microstepping)
  Motor 1 = spool, motor 2 = carriage on a TR8x8 lead screw (Prusa MK3/MK4 type).
  Controlled over serial (115200 baud) by the Python GUI, or by hand in the serial monitor.

  MANUAL MODE
    S;speed1;speed2         signed rpm: >0 forward, <0 reverse, 0 stop
                            e.g.  S;60;0   S;-30;45   S;0;0
  WINDING MODE
    W;wire;turns;layers;rpm;dir
        wire   = wire diameter in mm (carriage advance per spool turn)
        turns  = turns per layer
        layers = number of layers
        rpm    = spool speed
        dir    = carriage direction for the first layer (1 or -1)
        e.g.  W;0.5;22;10;60;1
    P = pause    R = resume    X = abort (or stop everything when idle)
    ?                        identify -> ID;WINDER;1

  Replies:
    READY;WINDER             at startup
    OK / ERR ...             command accepted / refused
    WINDING                  winding started
    T;turn;layer             progress, once per spool turn
    PAUSED / RESUMED
    DONE                     winding finished (motors keep holding until next S command)
    ABORTING, then STOPPED   after X
*/

// ===================== SETTINGS =====================
const float MAX_RPM = 300.0;            // safety limit (must match MAX_RPM in the Python GUI)
const float ACCELERATION_RPM_S = 240.0; // ramp: rpm gained per second (0 = no ramp)
const float CRAWL_RPM = 5.0;            // spool speed for the very end of a winding (precise stop)

const int   STEPS_PER_REV = 200;        // 1.8° motor = 200 steps/rev
const int   MICROSTEPS    = 16;         // driver setting (1/16)
const float LEAD_MM       = 8.0;        // TR8x8 lead screw: carriage moves 8 mm per screw turn

const bool INVERT_DIR1 = true;         // set true if motor 1 turns the wrong way
const bool INVERT_DIR2 = false;         // same for motor 2
const bool HOLD_TORQUE_WHEN_STOPPED = false; // true = drivers always enabled (motors heat up)

const bool DEBUG_SERIAL = false;        // true = echo received commands in the serial monitor
// ====================================================

const long  STEPS_PER_TURN = (long)STEPS_PER_REV * MICROSTEPS;   // 3200 microsteps per motor turn
const float CARRIAGE_STEPS_PER_MM = STEPS_PER_TURN / LEAD_MM;    // 400 microsteps per mm

// Pins
const int DIR1 = 2, STEP1 = 3, ENABLE1 = 4;    // spool motor
const int DIR2 = 8, STEP2 = 9, ENABLE2 = 10;   // carriage motor

struct Motor {
  byte pinDir, pinStep, pinEnable;
  bool invert;
  float targetSpeed;          // rpm requested (always >= 0)
  int  requestedDir;          // -1, 0, 1 (what is asked)
  int  currentDir;            // -1, 0, 1 (what is applied)
  float rpm;                  // instantaneous speed (ramp)
  unsigned long interval;     // µs between steps (0 = stopped)
  unsigned long lastStep;     // µs
};

Motor motors[2] = {
  { DIR1, STEP1, ENABLE1, INVERT_DIR1, 0.0, 0, 0, 0.0, 0, 0 },
  { DIR2, STEP2, ENABLE2, INVERT_DIR2, 0.0, 0, 0, 0.0, 0, 0 }
};

// ---------- Winding state ----------
enum WindState { W_IDLE, W_RUNNING, W_PAUSED, W_ABORTING };
WindState windState = W_IDLE;

float windRpm = 0;
long carriageStepsPerTurn = 0;   // carriage microsteps per spool turn (= wire pitch)
long stepsPerLayer = 0;          // spool microsteps per layer
long totalSteps = 0;             // spool microsteps for the whole coil
long stepsDone = 0;
long stepsInLayer = 0;
long stepsInTurn = 0;
long accumulator = 0;            // spool -> carriage step ratio (Bresenham)
long turnsDone = 0;
int  layer = 0;                  // current layer (0-based)
int  carriageDir = 1;
bool finalApproach = false;
bool holdAfterWinding = false;   // keep drivers enabled after winding (holds wire tension)

bool turnToReport = false;
bool doneToReport = false;
long reportTurn = 0;
int  reportLayer = 0;

// ---------- Timing / serial ----------
const unsigned long RAMP_PERIOD_MS = 10;
unsigned long lastRamp = 0;

const char FIRMWARE_ID[] = "WINDER";
const int  FIRMWARE_VERSION = 1;

char buffer[40];
byte length = 0;
unsigned long lastCharTime = 0;

// ---------------------------------------------------------------

void setup() {
  Serial.begin(115200);

  for (int i = 0; i < 2; i++) {
    pinMode(motors[i].pinDir, OUTPUT);
    pinMode(motors[i].pinStep, OUTPUT);
    pinMode(motors[i].pinEnable, OUTPUT);
    digitalWrite(motors[i].pinStep, LOW);
    digitalWrite(motors[i].pinEnable, HOLD_TORQUE_WHEN_STOPPED ? LOW : HIGH); // HIGH = driver disabled
  }

  Serial.print(F("READY;"));
  Serial.println(FIRMWARE_ID);
}

void loop() {
  readSerial();

  unsigned long nowMs = millis();
  if (nowMs - lastRamp >= RAMP_PERIOD_MS) {
    float dt = (nowMs - lastRamp) / 1000.0;
    lastRamp = nowMs;
    updateWinding();
    updateRamp(motors[0], dt);
    updateRamp(motors[1], dt);
  }

  unsigned long nowUs = micros();
  if (stepIfDue(motors[0], nowUs) && windState != W_IDLE) onSpoolStep();
  stepIfDue(motors[1], nowUs);

  reportProgress();
}

// ---------------------------------------------------------------
// Motion

void stepPulse(byte pin) {
  digitalWrite(pin, HIGH);
  delayMicroseconds(3);              // min pulse ~2 µs (DRV8825)
  digitalWrite(pin, LOW);
}

// Sends one step pulse when the interval has elapsed (non-blocking). Returns true if it stepped.
bool stepIfDue(Motor &m, unsigned long now) {
  if (m.interval == 0) return false;
  if (now - m.lastStep < m.interval) return false;

  m.lastStep += m.interval;
  if (now - m.lastStep > m.interval) m.lastStep = now; // avoid catch-up bursts
  stepPulse(m.pinStep);
  return true;
}

// Acceleration / deceleration, direction change only at zero speed, driver enable
void updateRamp(Motor &m, float dt) {
  float setpoint;
  if (m.requestedDir != m.currentDir) setpoint = 0;          // slow down first
  else setpoint = (m.currentDir == 0) ? 0 : m.targetSpeed;

  if (ACCELERATION_RPM_S <= 0) {
    m.rpm = setpoint;
  } else {
    float delta = ACCELERATION_RPM_S * dt;
    if (m.rpm < setpoint) {
      m.rpm += delta;
      if (m.rpm > setpoint) m.rpm = setpoint;
    } else if (m.rpm > setpoint) {
      m.rpm -= delta;
      if (m.rpm < setpoint) m.rpm = setpoint;
    }
  }

  // At standstill: apply the new direction
  if (m.rpm <= 0 && m.requestedDir != m.currentDir) {
    m.rpm = 0;
    m.currentDir = m.requestedDir;
    if (m.currentDir != 0) {
      bool forward = (m.currentDir > 0) != m.invert;
      digitalWrite(m.pinDir, forward ? HIGH : LOW);
      delayMicroseconds(5);           // DIR setup time
    }
  }

  if (m.rpm > 0) {
    m.interval = (unsigned long)(60000000.0 / (m.rpm * (float)STEPS_PER_TURN));
  } else {
    m.interval = 0;
  }

  bool enable = HOLD_TORQUE_WHEN_STOPPED || holdAfterWinding
                || windState != W_IDLE || m.currentDir != 0;
  digitalWrite(m.pinEnable, enable ? LOW : HIGH);   // LOW = driver enabled
}

bool motorStopped(const Motor &m) {
  return m.rpm <= 0 && m.currentDir == 0 && m.requestedDir == 0;
}

// Signed rpm -> direction + absolute speed
void setMotor(Motor &m, float signedRpm) {
  m.targetSpeed = fabs(signedRpm);
  if (signedRpm > 0)      m.requestedDir = 1;
  else if (signedRpm < 0) m.requestedDir = -1;
  else                    m.requestedDir = 0;
}

// ---------------------------------------------------------------
// Winding

void applyCarriageDir() {
  bool forward = (carriageDir > 0) != motors[1].invert;
  digitalWrite(motors[1].pinDir, forward ? HIGH : LOW);
  delayMicroseconds(5);
}

// Called after every spool step during a winding
void onSpoolStep() {
  // Carriage follows the spool: carriageStepsPerTurn carriage steps per STEPS_PER_TURN spool steps
  accumulator += carriageStepsPerTurn;
  if (accumulator >= STEPS_PER_TURN) {
    accumulator -= STEPS_PER_TURN;
    stepPulse(motors[1].pinStep);
  }

  stepsDone++;
  stepsInLayer++;
  stepsInTurn++;

  if (stepsInTurn >= STEPS_PER_TURN) {
    stepsInTurn = 0;
    turnsDone++;
    reportTurn = turnsDone;
    reportLayer = layer + 1;
    turnToReport = true;
  }

  if (stepsDone >= totalSteps) {      // last turn done: exact stop
    Motor &s = motors[0];
    s.rpm = 0;
    s.interval = 0;
    endWinding();
    doneToReport = true;
    return;
  }

  if (stepsInLayer >= stepsPerLayer) { // end of layer: reverse the carriage
    stepsInLayer = 0;
    layer++;
    carriageDir = -carriageDir;
    applyCarriageDir();
  }
}

// Spool speed control during a winding (called every RAMP_PERIOD_MS)
void updateWinding() {
  if (windState == W_IDLE) return;
  Motor &s = motors[0];

  if (windState == W_RUNNING) {
    // Start slowing down early enough to finish the last turn at CRAWL_RPM
    if (!finalApproach && ACCELERATION_RPM_S > 0) {
      float decelSteps = (s.rpm * s.rpm) / (120.0 * ACCELERATION_RPM_S) * STEPS_PER_TURN;
      if (totalSteps - stepsDone <= (long)decelSteps + STEPS_PER_TURN / 8) finalApproach = true;
    }
    s.targetSpeed = finalApproach ? min(CRAWL_RPM, windRpm) : windRpm;
  } else {
    s.targetSpeed = 0;                 // paused or aborting: ramp down, keep sync
    if (windState == W_ABORTING && s.rpm <= 0) {
      endWinding();
      Serial.println(F("STOPPED"));
    }
  }
}

void endWinding() {
  windState = W_IDLE;
  holdAfterWinding = false;             // drivers stay enabled to hold the wire
  motors[0].targetSpeed = 0;
  motors[0].requestedDir = 0;
}

void reportProgress() {
  if (turnToReport) {
    turnToReport = false;
    Serial.print(F("T;"));
    Serial.print(reportTurn);
    Serial.print(';');
    Serial.println(reportLayer);
  }
  if (doneToReport) {
    doneToReport = false;
    Serial.println(F("DONE"));
  }
}

// ---------------------------------------------------------------
// Serial

// A command is processed on a line ending,
// OR after 50 ms without a new character (works with "No line ending").
void readSerial() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    lastCharTime = millis();
    if (c == '\n' || c == '\r') {
      submitBuffer();
    } else if (c != ' ' && length < sizeof(buffer) - 1) {
      buffer[length++] = c;
    }
  }
  if (length > 0 && millis() - lastCharTime > 50) {
    submitBuffer();
  }
}

void submitBuffer() {
  if (length == 0) return;
  buffer[length] = '\0';
  debugPrint(buffer);
  processCommand(buffer);
  length = 0;
}

// Prints the received command (only when DEBUG_SERIAL is true)
void debugPrint(const char *cmd) {
  if (!DEBUG_SERIAL) return;
  Serial.print(F("Received: \""));
  Serial.print(cmd);
  Serial.println(F("\""));
}

void processCommand(char *cmd) {
  char *f[7];
  int n = 0;
  char *tok = strtok(cmd, ";");
  while (tok != NULL && n < 7) {
    f[n++] = tok;
    tok = strtok(NULL, ";");
  }
  if (n == 0) return;

  char c = (f[0][1] == '\0') ? toupper(f[0][0]) : 0;

  switch (c) {
    case 'S':
      if (n != 3) { Serial.println(F("ERR format: S;speed1;speed2")); return; }
      cmdManual(atof(f[1]), atof(f[2]));
      break;
    case 'W':
      if (n != 6) { Serial.println(F("ERR format: W;wire;turns;layers;rpm;dir")); return; }
      cmdWind(atof(f[1]), atol(f[2]), atol(f[3]), atof(f[4]), atoi(f[5]));
      break;
    case 'P': cmdPause();  break;
    case 'R': cmdResume(); break;
    case 'X': cmdAbort();  break;
    case '?':
      Serial.print(F("ID;"));
      Serial.print(FIRMWARE_ID);
      Serial.print(';');
      Serial.println(FIRMWARE_VERSION);
      break;
    default:  Serial.println(F("ERR unknown command"));
  }
}

void cmdManual(float v1, float v2) {
  if (windState != W_IDLE) { Serial.println(F("ERR busy: winding in progress")); return; }
  if (fabs(v1) > MAX_RPM || fabs(v2) > MAX_RPM) { Serial.println(F("ERR speed above MAX_RPM")); return; }

  holdAfterWinding = false;
  setMotor(motors[0], v1);
  setMotor(motors[1], v2);
  Serial.println(F("OK"));
}

void cmdWind(float wire, long turns, long layers, float rpm, int dir) {
  if (windState != W_IDLE)                 { Serial.println(F("ERR busy: winding in progress")); return; }
  if (!motorStopped(motors[0]) || !motorStopped(motors[1]))
                                           { Serial.println(F("ERR stop the motors first")); return; }
  if (wire < 0.05 || wire > 3.0)           { Serial.println(F("ERR wire diameter: 0.05 to 3 mm")); return; }
  if (turns < 1 || turns > 1000)           { Serial.println(F("ERR turns per layer: 1 to 1000")); return; }
  if (layers < 1 || layers > 200)          { Serial.println(F("ERR layers: 1 to 200")); return; }
  if (rpm <= 0 || rpm > MAX_RPM)           { Serial.println(F("ERR speed: 0 to MAX_RPM")); return; }
  if (dir != 1 && dir != -1)               { Serial.println(F("ERR dir: 1 or -1")); return; }

  carriageStepsPerTurn = (long)(wire * CARRIAGE_STEPS_PER_MM + 0.5);
  stepsPerLayer = turns * STEPS_PER_TURN;
  totalSteps    = stepsPerLayer * layers;
  stepsDone = 0; stepsInLayer = 0; stepsInTurn = 0;
  accumulator = 0; turnsDone = 0; layer = 0;
  windRpm = rpm;
  carriageDir = dir;
  finalApproach = false;
  holdAfterWinding = false;
  windState = W_RUNNING;

  digitalWrite(motors[1].pinEnable, LOW);  // carriage driver on
  applyCarriageDir();
  motors[0].targetSpeed = rpm;             // spool forward, the ramp does the rest
  motors[0].requestedDir = 1;

  Serial.println(F("WINDING"));
}

void cmdPause() {
  if (windState != W_RUNNING) { Serial.println(F("ERR not winding")); return; }
  windState = W_PAUSED;
  Serial.println(F("PAUSED"));
}

void cmdResume() {
  if (windState != W_PAUSED) { Serial.println(F("ERR not paused")); return; }
  windState = W_RUNNING;
  Serial.println(F("RESUMED"));
}

void cmdAbort() {
  if (windState == W_RUNNING || windState == W_PAUSED) {
    windState = W_ABORTING;
    Serial.println(F("ABORTING"));
  } else if (windState == W_IDLE) {      // acts as a manual stop
    setMotor(motors[0], 0);
    setMotor(motors[1], 0);
    Serial.println(F("OK"));
  }
}