#pragma once

// ---------- I2C bus (MPU9350 + LCD) ----------
#define PIN_SDA 21
#define PIN_SCL 22

// ---------- L298N motor driver ----------
// Left channel (motor A)
#define PIN_ENA 25   // PWM
#define PIN_IN1 26
#define PIN_IN2 27
// Right channel (motor B)
#define PIN_ENB 14   // PWM
#define PIN_IN3 12
#define PIN_IN4 13

#define PWM_FREQ_HZ 20000
#define PWM_RESOLUTION_BITS 8
#define PWM_CHANNEL_LEFT 0
#define PWM_CHANNEL_RIGHT 1

// ---------- Status LEDs ----------
#define PIN_LED_GREEN 32   // upright / balancing
#define PIN_LED_RED   33   // fallen / cutoff

// ---------- Calibration switch ----------
// Hold the robot upright and press this to re-zero the pitch angle.
#define PIN_CALIBRATE_SWITCH 4   // wired with INPUT_PULLUP, active LOW

// ---------- LCD (I2C) ----------
#define LCD_I2C_ADDRESS 0x27
#define LCD_COLS 16
#define LCD_ROWS 2

// ---------- IMU orientation ----------
// The complementary filter assumes the robot leans forward/back around
// one axis. Depending on how the MPU9350 is physically mounted, you may
// need to flip these signs after your first test run (if the robot drives
// the wrong way when it tips, flip IMU_INVERT_PITCH; if it reacts to
// side-tilt instead of forward/back tilt, the sensor is mounted rotated
// 90 degrees).
#define IMU_INVERT_PITCH 1   // +1 or -1

// ---------- Safety ----------
#define FALL_ANGLE_DEG 45.0f   // cut motors if |pitch| exceeds this

// ---------- PID defaults (overridden live from the dashboard) ----------
#define PID_KP_DEFAULT 18.0f
#define PID_KI_DEFAULT 140.0f
#define PID_KD_DEFAULT 0.6f
#define PID_OUTPUT_LIMIT 255.0f

// ---------- Timing ----------
#define CONTROL_LOOP_HZ 200
#define TELEMETRY_INTERVAL_MS 60
#define LCD_UPDATE_INTERVAL_MS 200
#define WS_RECONNECT_INTERVAL_MS 3000
