/*
  Coil winder - 2 stepper motors (A4988 / DRV8825 drivers, 1/16 microstepping)

  Serial monitor commands (9600 baud, any line ending):
     M1;M2   with M1 and M2 = -1, 0 or 1
       1;0   -> motor 1 forward, motor 2 stopped
       1;1   -> both motors forward
      -1;0   -> motor 1 reverse
       0;-1  -> motor 2 reverse
       0;0   -> stop all
*/

// ===================== SETTINGS =====================
const float MOTOR1_SPEED_RPM = 60.0;    // motor 1 speed (rev/min)
const float MOTOR2_SPEED_RPM = 60.0;    // motor 2 speed (rev/min)
const float ACCELERATION_RPM_S = 240.0; // ramp: rpm gained per second (0 = no ramp)

const int STEPS_PER_REV = 200;          // 1.8° motor = 200 steps/rev
const int MICROSTEPS    = 16;           // driver setting (1/16)

const bool INVERT_DIR1 = false;         // set true if motor 1 turns the wrong way
const bool INVERT_DIR2 = false;         // same for motor 2
const bool HOLD_TORQUE_WHEN_STOPPED = false; // true = drivers stay enabled when stopped (motors heat up)

const bool DEBUG_SERIAL = false;        // true = echo received commands in the serial monitor
// ====================================================

// Pins
const int DIR1 = 2, STEP1 = 3, ENABLE1 = 4;    // spool motor
const int DIR2 = 8, STEP2 = 9, ENABLE2 = 10;   // carriage motor

struct Motor {
  byte pinDir, pinStep, pinEnable;
  float targetSpeed;          // rpm
  bool invert;
  int  requestedDir;          // -1, 0, 1 (what the user asked for)
  int  currentDir;            // -1, 0, 1 (what is applied)
  float rpm;                  // instantaneous speed (ramp)
  unsigned long interval;     // µs between steps (0 = stopped)
  unsigned long lastStep;     // µs
};

Motor motors[2] = {
  { DIR1, STEP1, ENABLE1, MOTOR1_SPEED_RPM, INVERT_DIR1, 0, 0, 0.0, 0, 0 },
  { DIR2, STEP2, ENABLE2, MOTOR2_SPEED_RPM, INVERT_DIR2, 0, 0, 0.0, 0, 0 }
};

const unsigned long RAMP_PERIOD_MS = 10;  // ramp update period
unsigned long lastRamp = 0;

char buffer[24];
byte length = 0;
unsigned long lastCharTime = 0;

// ---------------------------------------------------------------

void setup() {
  Serial.begin(9600);

  for (int i = 0; i < 2; i++) {
    pinMode(motors[i].pinDir, OUTPUT);
    pinMode(motors[i].pinStep, OUTPUT);
    pinMode(motors[i].pinEnable, OUTPUT);
    digitalWrite(motors[i].pinStep, LOW);
    digitalWrite(motors[i].pinEnable, HOLD_TORQUE_WHEN_STOPPED ? LOW : HIGH); // HIGH = driver disabled
  }
}

void loop() {
  readSerial();

  unsigned long nowMs = millis();
  if (nowMs - lastRamp >= RAMP_PERIOD_MS) {
    float dt = (nowMs - lastRamp) / 1000.0;
    lastRamp = nowMs;
    updateRamp(motors[0], dt);
    updateRamp(motors[1], dt);
  }

  unsigned long nowUs = micros();
  stepIfDue(motors[0], nowUs);
  stepIfDue(motors[1], nowUs);
}

// ---------------------------------------------------------------
// Sends one step pulse when the interval has elapsed (non-blocking)
void stepIfDue(Motor &m, unsigned long now) {
  if (m.interval == 0) return;

  if (now - m.lastStep >= m.interval) {
    m.lastStep += m.interval;
    if (now - m.lastStep > m.interval) m.lastStep = now; // avoid catch-up bursts

    digitalWrite(m.pinStep, HIGH);
    delayMicroseconds(3);            // min pulse ~2 µs (DRV8825)
    digitalWrite(m.pinStep, LOW);
  }
}

// ---------------------------------------------------------------
// Acceleration / deceleration, direction change only at zero speed
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

  // At standstill: apply the new direction (or disable the driver)
  if (m.rpm <= 0 && m.requestedDir != m.currentDir) {
    m.rpm = 0;
    m.currentDir = m.requestedDir;
    if (m.currentDir != 0) {
      bool forward = (m.currentDir > 0) != m.invert;
      digitalWrite(m.pinDir, forward ? HIGH : LOW);
      digitalWrite(m.pinEnable, LOW);   // enable driver
      delayMicroseconds(5);             // DIR setup time
    } else if (!HOLD_TORQUE_WHEN_STOPPED) {
      digitalWrite(m.pinEnable, HIGH);  // disable driver
    }
  }

  if (m.rpm > 0) {
    m.interval = (unsigned long)(60000000.0 / (m.rpm * (float)STEPS_PER_REV * MICROSTEPS));
  } else {
    m.interval = 0;
  }
}

// ---------------------------------------------------------------
// Non-blocking serial read.
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
  char *sep = strchr(cmd, ';');
  if (sep == NULL) {
    Serial.println(F("Error: expected format M1;M2 (e.g. 1;0)"));
    return;
  }
  *sep = '\0';
  int d1 = atoi(cmd);
  int d2 = atoi(sep + 1);

  if (d1 < -1 || d1 > 1 || d2 < -1 || d2 > 1) {
    Serial.println(F("Error: allowed values are -1, 0 or 1"));
    return;
  }

  motors[0].requestedDir = d1;
  motors[1].requestedDir = d2;
}