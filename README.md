# Self-Balancing Robot

A two-wheel self-balancing robot built on an ESP32 (MPU9350 IMU + L298N/BO
motors), part of a larger project that also includes a Django Channels
backend, a real-time RL auto-tuner that learns better PID gains from live
telemetry, and a Next.js dashboard with a real-world-scale interactive 2D/3D
model of the build, live-linked to the robot's actual tilt.

> **This public repo contains the ESP32 firmware and documentation/screenshots
> for the full project.** The backend and dashboard source live in a private
> repo — the screenshots below show what they do, even though the code
> itself isn't here.

![Dashboard overview](docs/screenshots/overview.png)

## What's in this repo

| Path | What it is |
|---|---|
| [`firmware/`](firmware/) | ESP32 firmware (PlatformIO): IMU sensor fusion, PID balance loop, WebSocket telemetry client |
| [`docs/screenshots/`](docs/screenshots/) | Screenshots of the dashboard, 3D/2D model, and RL training (see below) |

`firmware/README.md` has full setup/wiring/protocol detail for the piece
that's here.

## How the full project fits together

```mermaid
flowchart LR
    subgraph Robot["Robot (or a simulator)"]
        ESP32["ESP32 firmware<br/>IMU fusion + PID loop<br/>(this repo)"]
    end

    Backend["Django Channels backend<br/>ws://.../ws/telemetry/"]
    DB[("SQLite<br/>telemetry + RL episode history")]
    Dashboard["Next.js dashboard<br/>charts · 3D/2D model · controls"]
    RL["RL trainer<br/>Cross-Entropy Method"]

    ESP32 -- "telemetry (angle, output, gains)" --> Backend
    Backend -- "pid_update" --> ESP32
    Backend <-- "telemetry, pid_update, rl_episode" --> Dashboard
    Backend <--> RL
    Backend --> DB
```

Every participant — the robot, the dashboard, and the RL trainer — is just a
WebSocket client on the same `/ws/telemetry/` endpoint, all sending and
receiving the same three message types (`telemetry`, `pid_update`,
`rl_episode`). That's *why* the RL trainer can propose new gains: it sends
`pid_update` exactly the way the dashboard's sliders do, so it can never
bypass the firmware's own fall-angle safety cutoff — it's not a separate,
riskier control path, just another client turning the same knob. This repo
is the `ESP32 firmware` box above; the rest is documented here in screenshots
only.

## PID control, briefly

The robot balances by continuously measuring how far off vertical it is
(the MPU9350) and driving the wheels to correct it — proportional to the
error, plus an integral term for persistent lean, plus a derivative term
for damping:

```mermaid
flowchart LR
    SP["Setpoint 0&deg;"] --> ERR(("&minus;"))
    IMU["MPU9350<br/>measured angle"] --> ERR
    ERR --> P["P: Kp &times; error"]
    ERR --> I["I: Ki &times; &int;error"]
    ERR --> D["D: Kd &times; d(error)/dt"]
    P --> SUM(("+"))
    I --> SUM
    D --> SUM
    SUM -- "motor PWM" --> MOTORS["Wheels"]
    MOTORS -- tilts --> IMU
```

**[Read the full write-up →](docs/PID_CONTROL.md)** — what each term actually
does, the failure mode of getting them wrong, and a chart (generated from
this project's own simulated plant, not a generic textbook curve) comparing
five gain sets on the same 15° starting tilt, including a real result: the
RL auto-tuner found the hand-picked starting gains had far too much integral
action, and converged on something that settles both faster and tighter.

## Screenshots

### Live telemetry & control

The hero row: the 3D model and the live angle chart, visible together (no
navigating between pages to see both). PID gains are draggable sliders that
apply live over the WebSocket.

![3D model, assembled](docs/screenshots/3d-model.png)
![Pitch angle chart](docs/screenshots/angle-chart.png)
![PID gain sliders](docs/screenshots/pid-controls.png)
![Motor output chart](docs/screenshots/output-chart.png)
![Session stats and gain-change log](docs/screenshots/session-stats-gains-log.png)

### RL training

An episodic auto-tuner (Cross-Entropy Method) that searches for better
Kp/Ki/Kd by trying a candidate gain-set for a few seconds at a time and
scoring how stable the robot stayed. This chart is from an actual training
run against a simulated plant — reward climbing (and variance shrinking) as
the search converges, red dots marking any episode where it fell:

![RL training chart and status](docs/screenshots/rl-training.png)

### 3D model

Real-world-scale primitives sized from each component's actual bounding box,
colored by what the hardware actually looks like rather than by category —
black rubber tires with a separate hub, green PCB tones, brass standoffs.
The model's tilt is driven by live telemetry, and the wheels spin at a rate
driven by live motor output. A "component separation" control explodes the
whole assembly apart for a clearer look at what's stacked where:

![3D model exploded view](docs/screenshots/3d-model-exploded.png)

### 2D schematic

Four orthographic views (Top / Front / Left / Right), generated from the
same part data as the 3D model so the two can never drift out of sync.
Projection is geometrically real, not just a flat icon swap — a wheel
renders as a rectangle from the top or front (you're looking at its edge)
and only as a circle from the left/right (you're looking straight down its
axle), exactly like the real part would:

![2D schematic, top view](docs/screenshots/2d-schematic-top.png)
![2D schematic, front view](docs/screenshots/2d-schematic-front.png)
![2D schematic, left view](docs/screenshots/2d-schematic-left.png)

### Component reference & pin wiring

Clicking any component — in the 3D model, the 2D schematic, or the table —
shows its real spec and, for anything actually wired into the firmware
(source in this repo — see [`firmware/include/Config.h`](firmware/include/Config.h)),
its exact pin-to-pin connections. Reserved components (servos, Bluetooth,
the Arduino UNO) honestly show "not connected" rather than invented wiring:

![Component pin connections](docs/screenshots/part-info-pins.png)
![Full component table](docs/screenshots/parts-table.png)

## Hardware

Everything on the original component list, plus a few chassis/power items
the build needs but weren't on that list (clearly marked as such in the
dashboard, not silently presented as owned):

| Component | Role | Status |
|---|---|---|
| ESP32 devkit | Runs the PID loop + WebSocket link | Wired in |
| MPU9350 IMU | Pitch angle (accel + gyro fusion) | Wired in |
| L298N motor driver | Drives both BO motors | Wired in |
| 2x BO motor + wheel | Drive/balance | Wired in |
| 16x2 LCD (I2C) | Onboard status display | Wired in |
| Green / red LED | Balanced / fallen indicator | Wired in |
| Calibrate switch | Re-zero IMU + reset PID integral | Wired in |
| MG90S / SG90 servo | Reserved for a future pan-tilt mount | Owned, not wired |
| HC-05 Bluetooth | Reserved for a manual-control fallback | Owned, not wired |
| HC-SR04 ultrasonic | Reserved for future obstacle detection | Owned, not wired |
| Arduino UNO | Reserved as an optional watchdog MCU | Owned, not wired |
| Chassis plates, standoffs, battery pack | Structural/power — needed to build this | Not on the original list |

Full pin-level wiring for every "wired in" part is in `firmware/README.md`
and visible in the pin-connections screenshot above.

## Running the firmware

```
cd firmware
cp include/Secrets.h.example include/Secrets.h   # fill in WiFi + backend host
pio run -t upload
pio device monitor
```

It'll connect to WiFi and try to reach a backend over WebSocket at the host/
port in `Secrets.h` — see `firmware/README.md` for the wiring table, first-run
calibration, and the telemetry protocol. Without a backend to talk to, it'll
still balance locally (the WebSocket send just becomes a no-op); the backend,
dashboard, and RL trainer that give it a live UI aren't included in this repo.

## Design notes worth knowing

- **The dashboard's 2D/3D model is a schematic-scale reference, not
  fabrication-ready CAD.** Dimensions are typical/approximate figures for
  these common hobby parts, and each part is one primitive sized to its real
  bounding box, not a detailed mesh.
- **Wheel spin in the 3D view is illustrative.** There's no wheel encoder on
  this build, so the spin rate is derived from motor PWM, not a measured
  wheel speed.
- **The RL trainer never bypasses the firmware's safety cutoff.** It only
  ever proposes `(Kp, Ki, Kd)` over the same `pid_update` path the dashboard
  sliders use — see the architecture section above.

## The project story

### Motivation

This started after seeing a "Smart Parking Assistant" project shared
online — an ESP32, an ultrasonic sensor, and a Django dashboard. The idea of
taking ordinary hobby hardware and making it feel like a real product by
wrapping it in a proper dashboard/backend stuck. A self-balancing robot was
the choice for the hardware side specifically because it's a genuinely
harder control problem than a threshold check — it needs real-time sensor
fusion and continuous closed-loop feedback, not just a single reading.

### Objectives

- Get a two-wheel robot physically balancing using components already on
  hand (ESP32, MPU9350, L298N, BO motors, LCD, LEDs, a switch).
- Give it a live web dashboard — real-time telemetry, and PID tuning without
  reflashing the firmware for every gain change.
- Find out whether an automated search could tune the controller better
  than hand-picked gains, without turning the robot into something riskier
  to run.
- Build an interactive, real-world-scale 2D/3D visualization of the actual
  physical build — not just a chart of numbers, but something that shows
  what's actually stacked where and how it's wired.

### Real hardware first, simulation second

The firmware, the PID loop, the WebSocket telemetry protocol, and the
dashboard were all designed around a real physical robot from the start —
this was never a simulation-first project. A simulator
(`backend/simulate_robot.py`, not included in this repo) was added
*afterward*, once it became clear that developing and, especially,
*training* the RL auto-tuner needed many hours of continuous runtime and a
lot of trial-and-error iteration that isn't practical to do exclusively
against physical hardware — battery life, motor wear, and needing to be
physically present to reset a fallen robot, among other things. The
simulator runs the exact same PID formula and speaks the exact same
WebSocket protocol as the firmware, so anything developed or tuned against
it — the dashboard, the RL trainer — works unchanged against the real robot.

### Challenges, trials, and errors

A few of the real ones, not a generic list:

- **A port conflict on first run.** The intended default port turned out to
  already be in use by another local project on the same machine —
  diagnosed by checking which process actually held the socket before
  picking a different port, rather than guessing.
- **A real protocol bug, caught by testing.** The WebSocket hub originally
  broadcast every message to the whole group, including back to whoever
  sent it. Since both telemetry and gain-update messages carry `kp`/`ki`/`kd`,
  the robot could receive an echo of its *own* telemetry and misidentify it
  as a new gain update. Fixed with an explicit message `type` tag and by
  excluding the sender from its own broadcast — found by writing an actual
  round-trip test, not by inspection.
- **A simulator that "moved" but wasn't real.** The first version was a
  random-walk stand-in — convincing-looking motion, but `Ki` and `Kd` had
  zero effect on it, which would have made RL training meaningless. It was
  rebuilt as a genuine closed-loop simulated inverted pendulum running the
  *same* PID formula as the firmware before any training began. Getting its
  physics to feel right — visibly alive without looking fake — took three
  tuning passes: the first attempt decayed to a flat line, the second used
  disturbances too weak to see, the third finally applied them as real
  velocity impulses.
- **The first 3D model was just colored blocks.** Getting it to look like
  actual hardware — real materials, a proper tire-and-hub wheel instead of
  a flat cylinder, shadows, live rotation driven by motor output — took
  several rounds of visual feedback and iteration.
- **A geometry bug in the 2D schematic.** A wheel should only render as a
  circle in the view that looks straight down its own axle (from the
  side), and as a rectangle from the top or front — the first
  implementation had this backwards.
- **Trusting the RL results only after checking them.** Before letting the
  search run unsupervised for hundreds of episodes, its reward signal was
  validated by hand: confirming zero gains were clearly unstable, that the
  hand-tuned defaults behaved as expected, and that a hand-picked "better
  guess" actually scored better — so a converging reward curve could be
  trusted to mean something real. See [`docs/PID_CONTROL.md`](docs/PID_CONTROL.md)
  for what it actually found.

### Future improvements

- Wire up the components that are owned but still reserved: the HC-SR04 for
  obstacle avoidance, the HC-05 for a manual Bluetooth control fallback, the
  MG90S/SG90 servos for a pan-tilt sensor mount, and the Arduino UNO as an
  independent, hardware-isolated watchdog fall-cutoff.
- Add real wheel encoders — the 3D view's wheel spin is currently
  illustrative (derived from motor PWM), not a measured wheel speed.
- Run the RL auto-tuner against the real robot directly, once there's a
  safe way to supervise many unattended training episodes on physical
  hardware.
- Replace the schematic-scale 2D/3D primitives with real CAD/mesh models of
  the actual components.
- Persist tuned gains across power cycles — they currently reset to the
  firmware defaults on reboot until re-tuned.
