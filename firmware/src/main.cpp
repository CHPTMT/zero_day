// Arm controller firmware. Implements the serial protocol in docs/protocol.md.
//
// Each control tick moves every joint's setpoint toward its target, capped at
// the joint's max velocity. The host does the smooth trajectory planning and
// streams MOVE commands; this loop is the safety net that enforces limits.

#include <Arduino.h>

#ifdef ARM_ESP32
#include <ESP32Servo.h>
#else
#include <Servo.h>
#endif

#include "arm_config.h"

static int32_t target_cdeg[NUM_JOINTS];
static float setpoint_cdeg[NUM_JOINTS];
static int32_t step_pos[NUM_JOINTS];
static Servo servos[NUM_JOINTS];
static Servo gripper;
static bool enabled = false;
static bool estopped = false;

static char line[LINE_BUFFER];
static uint8_t line_len = 0;
static unsigned long last_tick = 0;

static int32_t clamp32(int32_t v, int32_t lo, int32_t hi) { return v < lo ? lo : (v > hi ? hi : v); }

static void write_servo(Servo &s, const JointDef &j, float cdeg) {
  float deg = cdeg / 100.0f;
  if (j.invert) deg = -deg;
  // Servo centre maps to joint zero.
  float frac = (deg + j.range_deg / 2) / j.range_deg;
  frac = constrain(frac, 0.0f, 1.0f);
  s.writeMicroseconds(j.pulse_min_us + (uint16_t)(frac * (j.pulse_max_us - j.pulse_min_us)));
}

static void set_enabled(bool on) {
  enabled = on && !estopped;
  digitalWrite(PIN_DRIVER_ENABLE, enabled ? LOW : HIGH);
}

static void stop_all() {
  for (uint8_t i = 0; i < NUM_JOINTS; i++) target_cdeg[i] = (int32_t)setpoint_cdeg[i];
}

static void reply_pos() {
  Serial.print(F("POS"));
  for (uint8_t i = 0; i < NUM_JOINTS; i++) {
    Serial.print(' ');
    Serial.print((int32_t)setpoint_cdeg[i]);
  }
  Serial.println();
}

static void handle_line(char *cmd) {
  char *tok = strtok(cmd, " ");
  if (!tok) return;

  if (!strcmp(tok, "STATUS")) { reply_pos(); return; }
  if (!strcmp(tok, "STOP")) { stop_all(); Serial.println(F("OK")); return; }
  if (!strcmp(tok, "DISABLE")) { stop_all(); set_enabled(false); Serial.println(F("OK")); return; }
  if (!strcmp(tok, "ENABLE")) {
    set_enabled(true);
    Serial.println(estopped ? F("ERR estop active") : F("OK"));
    return;
  }

  if (estopped) { Serial.println(F("ERR estop active")); return; }
  if (!enabled) { Serial.println(F("ERR disabled")); return; }

  if (!strcmp(tok, "HOME")) {
    // Placeholder: assumes the arm was powered on at its home pose.
    // Replace with a limit switch homing routine once switches are fitted.
    for (uint8_t i = 0; i < NUM_JOINTS; i++) target_cdeg[i] = 0;
    Serial.println(F("OK"));
    return;
  }

  if (!strcmp(tok, "MOVE")) {
    int32_t vals[NUM_JOINTS];
    for (uint8_t i = 0; i < NUM_JOINTS; i++) {
      char *v = strtok(nullptr, " ");
      if (!v) { Serial.println(F("ERR expected one angle per joint")); return; }
      vals[i] = atol(v);
      if (vals[i] < JOINTS[i].min_cdeg || vals[i] > JOINTS[i].max_cdeg) {
        Serial.print(F("ERR limit joint "));
        Serial.println(i);
        return;
      }
    }
    for (uint8_t i = 0; i < NUM_JOINTS; i++) target_cdeg[i] = vals[i];
    Serial.println(F("OK"));
    return;
  }

  if (!strcmp(tok, "GRIP")) {
    char *v = strtok(nullptr, " ");
    if (!v) { Serial.println(F("ERR expected angle")); return; }
    int32_t c = clamp32(atol(v), GRIPPER_MIN_CDEG, GRIPPER_MAX_CDEG);
    gripper.write(c / 100);
    Serial.println(F("OK"));
    return;
  }

  Serial.println(F("ERR unknown command"));
}

static void read_serial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      line[line_len] = '\0';
      handle_line(line);
      line_len = 0;
    } else if (line_len < LINE_BUFFER - 1) {
      line[line_len++] = c;
    } else {
      line_len = 0;  // overflow: drop the line
      Serial.println(F("ERR line too long"));
    }
  }
}

static void pulse_step(uint8_t pin) {
  digitalWrite(pin, HIGH);
  delayMicroseconds(2);
  digitalWrite(pin, LOW);
}

static void control_tick(float dt) {
  for (uint8_t i = 0; i < NUM_JOINTS; i++) {
    const JointDef &j = JOINTS[i];
    float err = target_cdeg[i] - setpoint_cdeg[i];
    float max_step = j.max_vel_cdeg_s * dt;
    setpoint_cdeg[i] += constrain(err, -max_step, max_step);

    if (j.type == DRIVE_SERVO) {
      write_servo(servos[i], j, setpoint_cdeg[i]);
      continue;
    }
    // Stepper: emit the steps needed to reach the new setpoint this tick.
    // Fine for moderate step rates. Move to a timer ISR if step rates exceed ~5 kHz.
    int32_t want = lroundf(setpoint_cdeg[i] / 100.0f * j.steps_per_deg);
    if (want == step_pos[i]) continue;
    bool fwd = want > step_pos[i];
    digitalWrite(j.dir_pin, (fwd ^ j.invert) ? HIGH : LOW);
    while (step_pos[i] != want) {
      pulse_step(j.step_pin);
      step_pos[i] += fwd ? 1 : -1;
    }
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  pinMode(PIN_DRIVER_ENABLE, OUTPUT);
  pinMode(PIN_ESTOP, INPUT_PULLUP);
  set_enabled(false);

  for (uint8_t i = 0; i < NUM_JOINTS; i++) {
    const JointDef &j = JOINTS[i];
    if (j.type == DRIVE_STEPPER) {
      pinMode(j.step_pin, OUTPUT);
      pinMode(j.dir_pin, OUTPUT);
    } else {
      servos[i].attach(j.servo_pin, j.pulse_min_us, j.pulse_max_us);
      write_servo(servos[i], j, 0);
    }
  }
  gripper.attach(GRIPPER_PIN);
  Serial.println(F("OK ready"));
  last_tick = micros();
}

void loop() {
  bool estop_now = digitalRead(PIN_ESTOP) == HIGH;
  if (estop_now && !estopped) {
    estopped = true;
    stop_all();
    set_enabled(false);
    Serial.println(F("ERR estop"));
  } else if (!estop_now && estopped) {
    estopped = false;  // stays disabled until the host sends ENABLE
  }

  read_serial();

  unsigned long now = micros();
  if (now - last_tick >= CONTROL_PERIOD_US) {
    float dt = (now - last_tick) / 1e6f;
    last_tick = now;
    if (enabled) control_tick(dt);
  }
}
