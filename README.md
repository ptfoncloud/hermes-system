# HERMES Avionics System

Hobby rocketry avionics stack — flight logger, mission control interface, and ground station.  
**Phase 0: sensor logger.** The first in a five-phase program ending with closed-loop TVC ascent and propulsive landing.

**Preston Foote · CS @ Austin Peay State University · Clarksville TN · 2026**

---

## What this is

A complete avionics system for a Phase 0 flight logger built on a Teensy 4.1. Every sensor driver is written from the datasheet — no Adafruit libraries. Every design decision is documented here.

The broader project goal is a rocket that ascends under thrust vector control, ejects its spent motor mid-flight, chambers a fresh one autonomously from an onboard magazine, and lands propulsively on its tail. Phase 0 is the foundation: get clean, reliable flight data.

---

## Hardware

| Part | Purpose | I²C addr |
|------|---------|----------|
| Teensy 4.1 | Flight computer — 600MHz Cortex-M7, hardware FPU, onboard µSD | — |
| BNO085 | 9-DOF orientation IMU — quaternion, gyro, linear accel | 0x4A |
| ADXL375 | High-g accelerometer ±200g — primary launch/burnout detect | 0x53 |
| BMP388 | Barometer — altitude and apogee detection | 0x76 |
| 1S 500mAh LiPo | Power — 5V boost → Teensy VIN → 3.3V rail to sensors | — |

All three sensors share one I²C bus (SDA=18, SCL=19, 400kHz). SD card uses the Teensy's onboard SDIO interface — no SPI wiring needed.

Battery voltage is monitored via a 100kΩ+100kΩ divider on A0 (Vbat/2 → Teensy ADC). The Teensy 4.1 ADC reference is 3.3V by default — no `analogReference()` call is made.

**Vehicle:** Estes Olympus #7293, 24mm mount, E12-6 motor (sim-determined for payload mass with 48" launch rod).

---

## Repository layout

```
hermes/
├── firmware/
│   ├── platformio.ini
│   ├── include/
│   │   ├── config.h          I²C addresses, FSM thresholds, pin assignments
│   │   ├── fsm.h / fsm.cpp   Flight state machine
│   │   ├── sensors.h / .cpp  Sensor aggregation layer
│   │   ├── logger.h / .cpp   Binary SD logger
│   │   ├── telemetry.h / .cpp  Binary USB serial telemetry
│   │   ├── mission_ctrl.h / .cpp  Pre-flight serial dashboard
│   │   └── drivers/
│   │       ├── bno085.h / .cpp   BNO085 SHTP/I²C driver
│   │       ├── adxl375.h / .cpp  ADXL375 I²C driver
│   │       └── bmp388.h / .cpp   BMP388 I²C driver
│   └── src/  (mirrors include/ with .cpp implementations)
├── ground_station/
│   ├── hermes_gs.py        Serial bridge + WebSocket server
│   ├── requirements.txt
│   └── ui/
│       └── index.html      Browser-based ground station UI
└── tools/
    └── parse_log.py        Post-flight binary log parser
```

---

## Firmware

### Boot sequence

```
Power on
  → I²C bus init (400kHz)
  → BNO085 init + enable reports (rotation vector, gyro, linear accel @ 100Hz)
  → ADXL375 init (800Hz ODR)
  → BMP388 init (pressure x8 OSR, 50Hz ODR)
  → Pad pressure reference (16-sample average, guarded against read failures)
  → SD card init → open FLIGHT_NNN.bin
  → Mission Control dashboard (serial) — blocks until ARM
  → FSM init
  → Flight loop @ 100Hz
```

### Flight state machine

```
PAD ──(30g for 50ms)──► BOOST ──(accel < 15m/s² for 100ms)──► COAST
                                                                    │
                                                          (vvel < 0.5m/s)
                                                                    ▼
LANDED ◄──(stable 2s)── DESCENT ◄──(5m drop confirmed)── APOGEE
```

Every state transition flushes the SD log and sends a telemetry event frame.  
FAULT latches if any sensor drops in a non-PAD, non-LANDED state.

### Sensor drivers

All three drivers are written directly from the manufacturer datasheet — no third-party sensor libraries.

**BNO085** uses SHTP (Sensor Hub Transport Protocol) over I²C. Enables three report types: Rotation Vector (Q14 quaternion), Gyroscope Calibrated (Q9 rad/s), Linear Acceleration (Q8 m/s²). `bno085_read()` returns true as soon as the device responds to polling, regardless of whether all three report types have been received yet — preventing a spurious FAULT in early BOOST. `bno085_data_complete()` is the stricter check used by the pre-flight ARM gate.

**ADXL375** is configured at 800Hz ODR, FULL_RES mode. Scale: 49 mg/LSB (per datasheet Table 1). Used as the primary launch and burnout detector because the BNO085's linear acceleration output clips during high-g boost.

**BMP388** uses the Bosch double-precision floating-point compensation formulas from datasheet §8.5 with Table 17 trim coefficient scaling. Pressure x8 oversampling gives ~15 Pa noise at 50Hz. The ICAO standard atmosphere equation converts pressure to altitude AGL relative to the pad reference.

### Vertical velocity

Derived from a finite-difference of barometric altitude, passed through a single-pole IIR filter (α = 0.1). The derivative history is reset every time the pad pressure reference is set or re-taken, preventing a velocity spike on the first post-reference tick. Used by the FSM for apogee detection only — not as a primary flight variable.

### SD logging

Binary records, 64 bytes each, written at 100Hz. Flushed every 50 records and forced on every state transition. Files auto-increment: `FLIGHT_000.bin`, `FLIGHT_001.bin`, etc.

Record layout: `magic(1) + version(1) + state(1) + flags(1) + ts_ms(4) + quaternion(16) + gyro(12) + hg_accel(12) + baro_alt(4) + pressure(4) + temp_c(4) + vert_vel(4) = 64 bytes`

Parse with `tools/parse_log.py`.

### Telemetry

Binary framed packets over USB serial at 115200 baud, sent at 20Hz during flight. Frame format:

```
[0xAA] [0x55] [TYPE] [LEN] [PAYLOAD...] [CRC-8]
```

CRC-8 is computed over `TYPE + LEN + PAYLOAD` (poly 0x07, init 0x00). Anything on serial that doesn't start with `0xAA 0x55` is treated as human-readable console text by the ground station.

| Type | Name | Bytes | Rate |
|------|------|-------|------|
| 0x01 | Sensor | 46 | 20Hz |
| 0x02 | Preflight | 6 | 2Hz |
| 0x03 | Config | 28 | on change |
| 0x04 | Event | 9 | per transition |

---

## Mission Control (on Teensy)

Runs over USB serial before flight. Works with PlatformIO Serial Monitor, `screen`, `minicom`, or PuTTY in VT100 mode.

Displays a live pre-flight dashboard using ANSI terminal codes:

```
══════════════════════════════════════════════════════════════
  HERMES AVIONICS  v0.1  ·  2026  ·  APSU AEROSPACE
  ──────────────────────────────────────────────────────────
  ROCKET  HERMES-001        MOTOR  E12-6
  EXP ALT 120m              PAD    195m ASL

  SENSORS ─────────────────────────────────────────────────
  ● BNO085  OK              ● ADXL375  OK
  ● BMP388  OK              ● SD CARD  FLIGHT_003.bin
  ● BATTERY 3.87V ████████░░ 86%

  CHECKS ──────────────────────────────────────────────────
  ✓ All sensors nominal
  ✓ SD card ready — FLIGHT_003.bin
  ✓ Battery 3.87V (OK)
  ✓ Pad reference set
  ✗ System not armed

  ┌────────────────────────────────────────────────────────┐
  │              READY — TYPE ARM TO PROCEED               │
  └────────────────────────────────────────────────────────┘

  Commands: [N]ame  [M]otor  [A]lt  [R]ef  ARM  [?]
══════════════════════════════════════════════════════════════
```

| Command | Effect |
|---------|--------|
| `N <name>` | Set rocket name (default: HERMES-001) |
| `M <motor>` | Set motor designation (default: E12-6) |
| `A <metres>` | Set expected apogee AGL |
| `R` | Re-take pad pressure reference |
| `ARM` | Arm — transitions to flight mode |
| `?` | Help |

ARM is refused unless all five checks pass: sensors nominal, SD mounted, battery ≥ 3.5V, pad reference set, BNO085 all three report types received.

---

## Ground Station

### Setup

```bash
cd ground_station
pip install -r requirements.txt      # pyserial, websockets

python3 hermes_gs.py                  # auto-detects Teensy
python3 hermes_gs.py --port COM7      # Windows — specify port
python3 hermes_gs.py --port /dev/ttyACM0   # Linux/Mac
```

Open `ground_station/ui/index.html` in Chrome or Firefox. It connects to `ws://localhost:8765` automatically.

### Post-flight replay

```bash
python3 hermes_gs.py --replay FLIGHT_003.bin
```

Streams the binary log through the same WebSocket at 10× speed. All charts, attitude indicator, and event log animate as though it were a live flight.

### Browser UI

- **Pre-flight panel** — live sensor status, battery, SD filename, GO/HOLD banner
- **State badge** — colour-coded: grey=PAD, amber=BOOST, cyan=COAST, green=APOGEE/LANDED, red=FAULT
- **Altitude chart** — 60-second scrolling window, current + peak
- **Acceleration chart** — scrolling, shows boost profile
- **Attitude indicator** — canvas-based artificial horizon, driven by quaternion
- **T+ timer** — starts on BOOST detection
- **Predicted apogee** — v²/2g from current altitude and velocity during ascent
- **Flight event log** — every state transition with T+ timestamp
- **Serial console** — live Mission Control output visible in browser

---

## FSM thresholds

All tunable in `firmware/include/config.h`. Current values are conservative starting points — adjust after Phase 0 flight data is analysed.

| Threshold | Value | Notes |
|-----------|-------|-------|
| Launch detect accel | 30 m/s² (~3g) | Sustained for 50ms |
| Burnout detect accel | 15 m/s² | Sustained for 100ms |
| Apogee vvel | 0.5 m/s | IIR-filtered baro derivative |
| Apogee alt drop | 5 m | Confirmed descent confirmation |
| Landed accel | < 12 m/s² (~1.2g) | Stable for 2s |

---

## Post-flight log analysis

```bash
python3 tools/parse_log.py FLIGHT_003.bin            # summary to stdout
python3 tools/parse_log.py FLIGHT_003.bin --csv      # export to .csv
python3 tools/parse_log.py FLIGHT_003.bin --plot     # matplotlib plots
```

The `--plot` flag produces altitude, vertical velocity, and acceleration charts with state transitions overlaid. Requires `matplotlib` (`pip install matplotlib`).

---

## Building and flashing

```bash
cd firmware
pio run                     # build only
pio run --target upload     # build + upload to Teensy 4.1
pio device monitor          # open serial monitor (Mission Control)
```

PlatformIO will not find a matching library for any sensor — that's by design. All drivers are in `src/drivers/`.

---

## Phase roadmap

| Phase | Goal | Gate | Status |
|-------|------|------|--------|
| 0 — Logger | Clean binary flight logs, sensor FSM | 3 flights: PAD→LANDED clean | **In progress** |
| 1 — E-Deploy | MOSFET pyro channels, continuity sensing | Electronic deploy fires live | Not started |
| 2 — Autoloader | Mid-flight solid motor reload mechanism | 50 unattended ground cycles | Not started |
| 3 — TVC | 2-axis gimbal, own EKF, PID at 200Hz | Full burn on stand, no oscillation | Not started |
| 4 — Sim/HIL | 6DOF RK4 sim, hardware-in-the-loop rig | 1000 HIL landings within dispersion | Not started |
| 5 — Integration | Custom PCB, LoRa telemetry, landing attempt | Upright, intact, near the pad | PCB designed |

---

## Open tasks (Phase 0)

- [ ] BNO085 SHTP driver — hardware verification on bench (I²C transactions are implemented; needs a power-on test to confirm advertisement packet handling and report streaming)
- [ ] Run `adxl375_self_test()` and confirm the ADXL375 passes the ±200g self-test per datasheet Table 3
- [ ] OpenRocket rerun with payload mass and 48" rod confirmed, order E12-6 motors
- [ ] Tripoli Junior membership + club contact
- [ ] GitHub repo initialised and first commit pushed
- [ ] BMP388 compensation verified: measure known altitude, compare to GPS or published elevation

---

## Rules

Every driver is written from the datasheet. Every sim result is interpreted personally. Every failure is documented. No Adafruit sensor libraries. No generated CAD. No shortcuts.

---

## Study references

Controls: Brian Douglas → Brunton Control Bootcamp → Åström & Murray  
Estimation: Labbe Kalman and Bayesian Filters in Python · Solà quaternion paper  
Embedded: K&R C · Elecia White *Making Embedded Systems*  
Protocol: Hillcrest SH-2 Reference Manual · SHTP Application Note  
Rocketry: Stine *Handbook of Model Rocketry* · Sutton *Rocket Propulsion Elements*  
PCB: Phil's Lab KiCad series  
CAD: Onshape
