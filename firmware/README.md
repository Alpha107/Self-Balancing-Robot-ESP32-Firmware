# Self-Balancing Robot — ESP32 Firmware

PlatformIO project. Reads pitch from the MPU9350 (accel+gyro register map is
MPU6050/9250-compatible), runs a PID loop to drive the L298N/BO motors, and
streams telemetry to / receives live PID gains from a WebSocket backend
(Django Channels). See the root [`README.md`](../README.md) for the full
picture (architecture diagram, dashboard screenshots) — the backend and
dashboard aren't included in this repo, but the firmware here works
standalone (it'll just have nothing to talk to over WebSocket).

Gain updates can come from the dashboard's sliders *or* from an RL
auto-tuner — the firmware can't tell them apart and doesn't need to: both
just send the same `pid_update` message, and this firmware's own fall-angle
safety cutoff (below) is always the one deciding whether the motors are
allowed to run, regardless of who set the gains.

## Wiring

| Signal          | ESP32 pin |
|-----------------|-----------|
| MPU9350 SDA     | 21        |
| MPU9350 SCL     | 22        |
| LCD (I2C) SDA   | 21        |
| LCD (I2C) SCL   | 22        |
| L298N ENA (PWM) | 25        |
| L298N IN1       | 26        |
| L298N IN2       | 27        |
| L298N ENB (PWM) | 14        |
| L298N IN3       | 12        |
| L298N IN4       | 13        |
| Green LED       | 32        |
| Red LED         | 33        |
| Calibrate switch| 4 (to GND, uses internal pull-up) |

Power the BO motors from a separate battery pack through the L298N — do not
run them off the ESP32's 5V. Share ground between the battery pack, L298N,
and ESP32.

## Setup

1. Install [PlatformIO](https://platformio.org/) (VS Code extension or CLI).
2. `cp include/Secrets.h.example include/Secrets.h` and fill in your WiFi
   credentials and the LAN IP/port of the machine running the backend.
3. Build and flash:
   ```
   pio run -t upload
   pio device monitor
   ```

## First run / calibration

- Set the robot on a stand or hold it upright, then power on — it calibrates
  gyro offset and zeroes pitch automatically in `setup()`.
- Press the calibrate switch (GPIO 4 → GND) any time to re-zero while held
  upright; it also resets the PID integral term.
- If the robot drives away from vertical instead of correcting back to it,
  flip `IMU_INVERT_PITCH` in `include/Config.h` between `1` and `-1`.
- Default PID gains are placeholders (`PID_KP_DEFAULT` etc. in `Config.h`) —
  expect to retune once you can see it move. Once a dashboard is up you can
  tune `Kp`/`Ki`/`Kd` live without reflashing.

## Safety cutoff

If `|pitch| > FALL_ANGLE_DEG` (default 45°) the firmware cuts motor output
and lights the red LED. It auto-resumes once the robot is set back upright.

## Telemetry protocol (WebSocket, JSON)

Robot → backend, every `TELEMETRY_INTERVAL_MS` (default 60ms):
```json
{"type": "telemetry", "angle": 1.23, "output": -40.5, "kp": 18.0, "ki": 140.0, "kd": 0.6, "uptime_ms": 12345}
```

Backend → robot, whenever gains are updated:
```json
{"type": "pid_update", "kp": 20.0, "ki": 150.0, "kd": 0.8}
```

The `type` field matters: telemetry also carries `kp`/`ki`/`kd` (the gains
that produced that reading), so without an explicit type tag the firmware
can't tell an incoming gain update apart from an echo of its own telemetry.
