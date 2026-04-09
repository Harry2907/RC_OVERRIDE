# Final_Int — Drone Trainer/Trainee GCS

A Raspberry Pi ground control station for dual-operator drone training. A **Trainer** (master RC) flies normally while a **Trainee** (slave TX12 USB joystick) can take control by holding a button. A small ST7735 TFT display shows live telemetry. Everything is connected over MAVLink (ArduPilot FC via USB).

---

## Hardware

| Component | Detail |
|---|---|
| Computer | Raspberry Pi (any model with USB + SPI) |
| Display | ST7735R 128×160 TFT (SPI, CE0, DC=GPIO25, RST=GPIO24) |
| Trainer RC | ArduPilot flight controller via `/dev/ttyACM*` at 57600 baud |
| Trainee RC | RadioMaster TX12 in USB joystick mode |

---

## File Overview

```
Final_Int/
├── main.py               # Core app — MAVLinkReader, DroneGCS state machine
├── boot.py               # Boot screen + reconnect screen (PIL drawing)
├── flags.py              # SharedFlags — thread-safe state shared across threads
├── mavlink_thread.py     # MAVLinkFlagThread — connects serial, watches for disconnect
├── joystick_thread.py    # JoystickFlagThread — watches TX12 USB plug/unplug
├── rc_override_thread.py # RCOverrideThread — sends RC_CHANNELS_OVERRIDE at 20 Hz
├── stfinal.py            # DroneDisplay — ST7735 render engine
└── errors.json           # PreArm error short-name lookup table
```

---

## How It Works

### Boot Sequence

1. `MAVLinkFlagThread` scans `/dev/ttyACM0–4`, connects, waits for heartbeat → sets `flags.mavlink_connected = True`
2. `JoystickFlagThread` detects TX12 USB joystick → sets `flags.slave_connected = True`
3. `boot_screen()` shows 3 blocks filling up: **PWR** (instant) → **LINK** (MAVLink) → **CTRL** (joystick)
4. Once all 3 blocks fill, boot is complete → state moves to **ACTIVE**

### ACTIVE State

- `MAVLinkReader` reuses the existing serial connection from `MAVLinkFlagThread`
- On the **first HEARTBEAT** received, stream requests are sent to FC (mode, GPS, altitude, RC channels at 4 Hz)
- `RCOverrideThread` idles until Trainer holds **RC channel 10** — then it reads TX12 axes and sends `RC_CHANNELS_OVERRIDE` to FC at 20 Hz
- Display updates on every telemetry change (dirty-flag driven, not polled)

### Control Handover

| CH10 | Who flies |
|---|---|
| LOW  | Trainer (master RC, FC handles it natively) |
| HIGH | Trainee (TX12 USB → `RC_CHANNELS_OVERRIDE` sent at 20 Hz) |

### Disconnect / Reconnect

If either device drops mid-session, a full-screen reconnect spinner is shown and the render loop pauses. Once the device is back, normal operation resumes automatically. No restart needed.

---

## Threading Model

```
Main thread        → _state_manager()  — BOOT / ACTIVE / RECONNECTING state machine
MAVLinkFlagThread  → serial connect + heartbeat watchdog
MAVLinkReader      → recv_match() loop, parses all telemetry, sets dirty flag
JoystickFlagThread → polls /dev/input for TX12 plug/unplug at 10 Hz
RCOverrideThread   → sends RC_CHANNELS_OVERRIDE at 20 Hz when CH10 active
Render thread      → wakes on dirty flag, calls DroneDisplay.render()
TTS thread         → drains pyttsx3 queue (pyttsx3 is not thread-safe)
```

`SharedFlags` is the only shared state between threads. All flag writes go through a lock and fire a threading.Event so waiters wake immediately.

---

## Install

```bash
pip install pymavlink pygame pillow adafruit-circuitpython-rgb-display pyttsx3 pyserial
```

Run:
```bash
cd Final_Int
python main.py
```

---



## Axis Mapping (TX12 USB)

Defined at the top of `rc_override_thread.py`. Defaults:

| Axis index | Stick | Channel |
|---|---|---|
| 0 | Right X | CH1 Roll |
| 1 | Right Y | CH2 Pitch |
| 2 | Left Y  | CH3 Throttle |
| 3 | Left X  | CH4 Yaw |

To find your TX12's axis order: `jstest /dev/input/js0`

---

## Display Layout

```
┌──────────────────┐
│  TRAINER / TRAINEE  (header — switches when CH10 active)
├──────────────────┤
│  MODE     ALT    │
│  STABILI  12.3m  │
├──────────────────┤
│  GPS   ARMED     │
│  FIX   NO        │
├──────────────────┤
│  STATUS MESSAGE  │
│  READY TO ARM    │
└──────────────────┘
```

Status cycles through: `INITIALIZING...` → `READY TO ARM` → PreArm errors (up to 2, shortened via `errors.json`) → `ARMED`