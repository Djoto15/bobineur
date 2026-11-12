/*
  Current source - PWM-driven MOSFET on D9 + INA219 current sensor (DFRobot SEN0291, 0.01 ohm shunt).
  Closed-loop PID regulation of the current, or open-loop PWM.
  Controlled over serial (115200 baud) by the Python GUI, or by hand in the serial monitor.
  Telemetry lines stay compatible with the Arduino Serial Plotter.

  COMMANDS (one per line, fields separated by ';', any line ending)
    <number>          shortcut for C;<number>   e.g. "40"
    C;mA              closed-loop regulation at mA (0 = output off)
    O;pwm             open loop: fixed PWM duty 0..255
    X                 output off
    K;kp;ki;kd        PID gains (output in PWM counts: counts/mA, counts/(mA.s), counts.s/mA)
    T;ms              control / telemetry period, 10..2000 ms
    A;alpha           low-pass filter weight of a new sample, 0.01..1 (1 = no filtering)
    L;mA              over-current trip limit on the raw measurement (0 = disabled)
    F;0|1             PWM frequency on D9: 0 = 490 Hz (Arduino default), 1 = 31.25 kHz
    B;0|1             at power-up: 0 = output off, 1 = regulate at the saved setpoint
    Z                 reset the integrator
    G                 get settings -> P;mode;setpoint;pwm;kp;ki;kd;period;alpha;limit;fast;boot
                                      (mode: 0 = off, 1 = regulation, 2 = open loop)
    E                 save settings to EEPROM
    D                 restore default settings (not saved until E)
    ?                 identify     -> ID;CURRENT;2

  REPLIES
    READY;CURRENT                 at startup
    OK / ERR ...                  command accepted / refused
    FAULT;OVERCURRENT;mA          output cut by the over-current protection
    FAULT;SENSOR                  INA219 not found (output kept off, retried every second)
    Consigne:..,Courant:..,Filtre:..,PWM:..,Tension:..    telemetry, once per period
*/

#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_INA219.h>

// ===================== SETTINGS =====================
const uint8_t INA219_ADDR      = 0x45;     // hardware I2C address of the INA219
const uint16_t INA219_CONFIG   = 0x39EF;   // 32 V range, gain /8 (320 mV), shunt ADC 12-bit x32 samples (~17 ms), continuous
/*
 * Shunt correction: the Adafruit library assumes a 0.1 ohm shunt (R100), the DFRobot SEN0291
 * module uses 0.01 ohm (R010). The library value is therefore 10x too low.
 */
const float SHUNT_CORRECTION   = 10.0;
const float MAX_SETPOINT_MA    = 1000.0;   // must match CURRENT_MAX_MA in the Python GUI
const int   PWM_PIN            = 9;        // Timer1 / OC1A
// ====================================================

const char FIRMWARE_ID[] = "CURRENT";
const int  FIRMWARE_VERSION = 2;

enum Mode : uint8_t { MODE_OFF = 0, MODE_REGULATION = 1, MODE_OPEN_LOOP = 2 };

struct Settings {
  uint16_t magic;
  float setpoint_mA;
  float kp, ki, kd;
  uint16_t period_ms;
  float alpha;
  float limit_mA;
  uint8_t fastPwm;
  uint8_t bootRegulation;
};

const uint16_t SETTINGS_MAGIC = 0xC0A2;    // change when the Settings layout changes

// Defaults. ki = 0.25 reproduces the previous incremental controller
// (duty += 0.05 * error every 200 ms  <=>  0.25 counts per mA.s).
const Settings DEFAULTS = { SETTINGS_MAGIC, 40.0, 0.0, 0.25, 0.0, 200, 0.10, 1000.0, 0, 0 };

Adafruit_INA219 ina219(INA219_ADDR);
Settings cfg;

Mode  mode = MODE_OFF;
float openLoopPwm = 0;        // duty used in open-loop mode
float duty = 0;               // duty currently applied (0..255)
float integral = 0;           // integral term, in PWM counts
float filtered_mA = 0;
float previousFiltered = 0;
bool  sensorOk = false;
bool  firstSample = true;

unsigned long lastControl = 0;
unsigned long lastSensorRetry = 0;

char buffer[48];
byte length = 0;
unsigned long lastCharTime = 0;

// ---------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  pinMode(PWM_PIN, OUTPUT);
  analogWrite(PWM_PIN, 0);

  loadSettings();
  applyPwmFrequency();
  Serial.print(F("READY;"));
  Serial.println(FIRMWARE_ID);

  initSensor();
  if (cfg.bootRegulation && cfg.setpoint_mA > 0) mode = MODE_REGULATION;
}

void loop() {
  readSerial();

  unsigned long now = millis();
  if (!sensorOk) {
    if (now - lastSensorRetry >= 1000) {
      lastSensorRetry = now;
      initSensor();
    }
    return;
  }

  if (now - lastControl >= cfg.period_ms) {
    float dt = (now - lastControl) / 1000.0;
    if (dt > 4.0 * cfg.period_ms / 1000.0) dt = cfg.period_ms / 1000.0;   // first run / long stall
    lastControl = now;
    controlStep(dt);
  }
}

// ---------------------------------------------------------------
// Sensor

void ina219WriteConfig(uint16_t value) {
  Wire.beginTransmission(INA219_ADDR);
  Wire.write(0x00);
  Wire.write(value >> 8);
  Wire.write(value & 0xFF);
  Wire.endTransmission();
}

void initSensor() {
  sensorOk = ina219.begin();
  if (sensorOk) {
    ina219WriteConfig(INA219_CONFIG);
    firstSample = true;
    lastControl = millis();
  } else {
    setOutputOff();
    Serial.println(F("FAULT;SENSOR"));
  }
}

// ---------------------------------------------------------------
// Control

void controlStep(float dt) {
  float raw_mA = ina219.getCurrent_mA() * SHUNT_CORRECTION;
  if (raw_mA < 0.0) raw_mA = 0.0;
  float bus_V = ina219.getBusVoltage_V();

  // Exponential low-pass filter against switching noise and aliasing
  if (firstSample) {
    filtered_mA = raw_mA;
    previousFiltered = raw_mA;
    firstSample = false;
  } else {
    filtered_mA += cfg.alpha * (raw_mA - filtered_mA);
  }

  if (cfg.limit_mA > 0 && raw_mA > cfg.limit_mA && mode != MODE_OFF) {
    setOutputOff();
    Serial.print(F("FAULT;OVERCURRENT;"));
    Serial.println(raw_mA);
  }

  if (mode == MODE_REGULATION) {
    float error = cfg.setpoint_mA - filtered_mA;
    float p = cfg.kp * error;
    // Derivative on the measurement: no kick when the setpoint changes
    float d = (dt > 0) ? -cfg.kd * (filtered_mA - previousFiltered) / dt : 0;

    // Integrate only when it does not push the output further into saturation (anti-windup)
    float candidate = integral + cfg.ki * error * dt;
    float u = p + candidate + d;
    if (!((u > 255.0 && error > 0) || (u < 0.0 && error < 0))) integral = candidate;
    integral = constrain(integral, 0.0, 255.0);

    duty = constrain(p + integral + d, 0.0, 255.0);
  } else if (mode == MODE_OPEN_LOOP) {
    duty = openLoopPwm;
  } else {
    duty = 0;
  }
  previousFiltered = filtered_mA;

  analogWrite(PWM_PIN, (int)(duty + 0.5));

  // Telemetry (Serial Plotter format)
  Serial.print(F("Consigne:"));
  Serial.print(mode == MODE_REGULATION ? cfg.setpoint_mA : 0.0);
  Serial.print(F(",Courant:"));
  Serial.print(raw_mA);
  Serial.print(F(",Filtre:"));
  Serial.print(filtered_mA);
  Serial.print(F(",PWM:"));
  Serial.print((int)(duty + 0.5));
  Serial.print(F(",Tension:"));
  Serial.println(bus_V, 3);
}

void setOutputOff() {
  mode = MODE_OFF;
  duty = 0;
  integral = 0;
  analogWrite(PWM_PIN, 0);
}

void startRegulation(float setpoint) {
  cfg.setpoint_mA = setpoint;
  if (setpoint <= 0) { setOutputOff(); return; }
  if (mode != MODE_REGULATION) {
    integral = duty;          // bumpless transfer from open loop / off
    mode = MODE_REGULATION;
  }
}

// Timer1 prescaler: 1 -> 31.25 kHz, 64 -> 490 Hz (8-bit phase-correct PWM set up by the core)
void applyPwmFrequency() {
  TCCR1B = (TCCR1B & 0xF8) | (cfg.fastPwm ? _BV(CS10) : (_BV(CS11) | _BV(CS10)));
}

// ---------------------------------------------------------------
// Settings

bool settingsValid(const Settings &s) {
  return s.magic == SETTINGS_MAGIC
      && s.setpoint_mA >= 0 && s.setpoint_mA <= MAX_SETPOINT_MA
      && s.period_ms >= 10 && s.period_ms <= 2000
      && s.alpha > 0 && s.alpha <= 1
      && s.kp >= 0 && s.ki >= 0 && s.kd >= 0;   // also false for NaN
}

void loadSettings() {
  EEPROM.get(0, cfg);
  if (!settingsValid(cfg)) cfg = DEFAULTS;
}

void printSettings() {
  Serial.print(F("P;"));
  Serial.print(mode);                 Serial.print(';');
  Serial.print(cfg.setpoint_mA, 2);   Serial.print(';');
  Serial.print((int)openLoopPwm);     Serial.print(';');
  Serial.print(cfg.kp, 5);            Serial.print(';');
  Serial.print(cfg.ki, 5);            Serial.print(';');
  Serial.print(cfg.kd, 5);            Serial.print(';');
  Serial.print(cfg.period_ms);        Serial.print(';');
  Serial.print(cfg.alpha, 3);         Serial.print(';');
  Serial.print(cfg.limit_mA, 1);      Serial.print(';');
  Serial.print(cfg.fastPwm);          Serial.print(';');
  Serial.println(cfg.bootRegulation);
}

// ---------------------------------------------------------------
// Serial

// A command is processed on a line ending, OR after 50 ms without a new character
// (works with "No line ending" in the serial monitor).
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
  if (length > 0 && millis() - lastCharTime > 50) submitBuffer();
}

void submitBuffer() {
  if (length == 0) return;
  buffer[length] = '\0';
  processCommand(buffer);
  length = 0;
}

void ok()                    { Serial.println(F("OK")); }
void err(const __FlashStringHelper *msg) { Serial.print(F("ERR ")); Serial.println(msg); }

void processCommand(char *cmd) {
  // Bare number = new setpoint (same behaviour as the original sketch)
  if (isdigit(cmd[0]) || cmd[0] == '.') {
    float sp = atof(cmd);
    if (sp < 0 || sp > MAX_SETPOINT_MA) { err(F("setpoint out of range")); return; }
    startRegulation(sp);
    ok();
    return;
  }

  char *f[5];
  int n = 0;
  char *tok = strtok(cmd, ";");
  while (tok != NULL && n < 5) {
    f[n++] = tok;
    tok = strtok(NULL, ";");
  }
  if (n == 0) return;
  char c = (f[0][1] == '\0') ? toupper(f[0][0]) : 0;

  switch (c) {
    case 'C': {
      if (n != 2) { err(F("format: C;mA")); return; }
      float sp = atof(f[1]);
      if (sp < 0 || sp > MAX_SETPOINT_MA) { err(F("setpoint out of range")); return; }
      if (!sensorOk) { err(F("sensor not found")); return; }
      startRegulation(sp);
      ok();
      break;
    }
    case 'O': {
      if (n != 2) { err(F("format: O;pwm")); return; }
      int pwm = atoi(f[1]);
      if (pwm < 0 || pwm > 255) { err(F("pwm: 0 to 255")); return; }
      if (!sensorOk) { err(F("sensor not found")); return; }
      openLoopPwm = pwm;
      mode = MODE_OPEN_LOOP;
      integral = 0;
      ok();
      break;
    }
    case 'X':
      setOutputOff();
      ok();
      break;
    case 'K': {
      if (n != 4) { err(F("format: K;kp;ki;kd")); return; }
      float kp = atof(f[1]), ki = atof(f[2]), kd = atof(f[3]);
      if (kp < 0 || ki < 0 || kd < 0 || kp > 1000 || ki > 1000 || kd > 1000) { err(F("gains: 0 to 1000")); return; }
      cfg.kp = kp; cfg.ki = ki; cfg.kd = kd;
      ok();
      break;
    }
    case 'T': {
      if (n != 2) { err(F("format: T;ms")); return; }
      long ms = atol(f[1]);
      if (ms < 10 || ms > 2000) { err(F("period: 10 to 2000 ms")); return; }
      cfg.period_ms = ms;
      ok();
      break;
    }
    case 'A': {
      if (n != 2) { err(F("format: A;alpha")); return; }
      float a = atof(f[1]);
      if (a < 0.01 || a > 1.0) { err(F("alpha: 0.01 to 1")); return; }
      cfg.alpha = a;
      ok();
      break;
    }
    case 'L': {
      if (n != 2) { err(F("format: L;mA")); return; }
      float l = atof(f[1]);
      if (l < 0) { err(F("limit must be >= 0")); return; }
      cfg.limit_mA = l;
      ok();
      break;
    }
    case 'F':
      if (n != 2) { err(F("format: F;0|1")); return; }
      cfg.fastPwm = atoi(f[1]) ? 1 : 0;
      applyPwmFrequency();
      ok();
      break;
    case 'B':
      if (n != 2) { err(F("format: B;0|1")); return; }
      cfg.bootRegulation = atoi(f[1]) ? 1 : 0;
      ok();
      break;
    case 'Z':
      integral = 0;
      ok();
      break;
    case 'G':
      printSettings();
      break;
    case 'E':
      cfg.magic = SETTINGS_MAGIC;
      EEPROM.put(0, cfg);
      ok();
      break;
    case 'D':
      cfg = DEFAULTS;
      applyPwmFrequency();
      ok();
      break;
    case '?':
      Serial.print(F("ID;"));
      Serial.print(FIRMWARE_ID);
      Serial.print(';');
      Serial.println(FIRMWARE_VERSION);
      break;
    default:
      err(F("unknown command"));
  }
}
