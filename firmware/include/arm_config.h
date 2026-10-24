// Joint table for the firmware. Keep in sync with config/arm.yaml.
// PLACEHOLDER pins and values: update once the wiring and specs are final.
#pragma once

#include <stdint.h>

#define SERIAL_BAUD 115200
#define CONTROL_PERIOD_US 10000  // 100 Hz
#define NUM_JOINTS 5
#define LINE_BUFFER 96

// Global stepper driver enable (active low on A4988 / DRV8825 / TMC2209).
#define PIN_DRIVER_ENABLE 8
// Normally closed e-stop to ground, input pullup: HIGH means pressed or wire broken.
#define PIN_ESTOP 2

enum DriveType : uint8_t { DRIVE_STEPPER, DRIVE_SERVO };

struct JointDef {
  const char *name;
  DriveType type;
  int32_t min_cdeg;   // limits in centidegrees
  int32_t max_cdeg;
  int32_t max_vel_cdeg_s;
  // stepper
  uint8_t step_pin, dir_pin;
  float steps_per_deg;  // steps_per_rev * microsteps * gear_ratio / 360
  // servo
  uint8_t servo_pin;
  uint16_t pulse_min_us, pulse_max_us;
  float range_deg;
  bool invert;
};

// Columns: name, type, min, max, max_vel, step, dir, steps/deg, servo pin, pulse min, pulse max, range, invert
static const JointDef JOINTS[NUM_JOINTS] = {
  {"base_yaw",       DRIVE_STEPPER, -17000, 17000, 6000, 54, 55, 200 * 16 * 5.0f / 360, 0, 0, 0, 0, false},
  {"shoulder_pitch", DRIVE_STEPPER,  -9000,  9000, 4500, 60, 61, 200 * 16 * 10.0f / 360, 0, 0, 0, 0, false},
  {"elbow_pitch",    DRIVE_STEPPER, -13500, 13500, 6000, 46, 48, 200 * 16 * 8.0f / 360, 0, 0, 0, 0, false},
  {"wrist_pitch",    DRIVE_SERVO,   -12000, 12000, 9000, 0, 0, 0, 11, 500, 2500, 270.0f, false},
  {"wrist_roll",     DRIVE_SERVO,   -18000, 18000, 12000, 0, 0, 0, 12, 500, 2500, 270.0f, false},
};

#define GRIPPER_PIN 13
#define GRIPPER_MIN_CDEG 0
#define GRIPPER_MAX_CDEG 9000
