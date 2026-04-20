Drone Ground Control Station (GCS)

A lightweight, hardware-embedded Ground Control Station for **dual-operator (Trainer/Trainee) drone flight**, designed to run on a Raspberry Pi with a small SPI display. It bridges a MAVLink-connected flight controller with a USB joystick (slave transmitter), providing real-time telemetry display, voice feedback, and seamless RC handoff between operators.

---

## Features

- **Dual-operator RC handoff** — A dedicated RC channel (CH10) toggles control between the Trainer (master RC) and Trainee (slave USB joystick) in real time at 20 Hz.
- **Live telemetry display** on a 128×160 ST7735 TFT screen:
  - Flight mode, GPS fix status, altitude, armed/disarmed state, flight state (in-flight / on ground)
  - Pre-arm check results with human-readable error classification
  - Trainer / Trainee header that updates dynamically with control handoff
- **Boot sequence** with progress blocks for power, MAVLink link, and joystick presence
- **Reconnect screen** with animated spinner for dropped Trainer (MAVLink) or Trainee (joystick) connections
- **Voice announcements** via `pyttsx3` for mode changes, control handoffs, and connect/disconnect events
- **Thread-safe shared flag system** for clean inter-thread communication

---

## Hardware Requirements

| Component | Details |
|---|---|
| Single-board computer | Raspberry Pi (any model with SPI and GPIO) |
| Display | ST7735R 128×160 SPI TFT (connected via CE0, D25, D24) |
| Flight Controller | ArduPilot/PX4 FC connected via USB serial (`/dev/ttyACM0` or `/dev/ttyACM1`) |
| Slave transmitter | USB joystick (e.g. RadioMaster TX12 in USB HID mode) |
| Speaker / audio out | For `pyttsx3` voice feedback |

---

## Software Dependencies

Install via `pip`:

```bash
pip install pymavlink pygame pillow pyttsx3 adafruit-circuitpython-rgb-display
```

System packages (Raspberry Pi OS):

```bash
sudo apt install python3-serial espeak libespeak1 fonts-dejavu
```

---

## Project Structure

```
FF/
├── main.py                # Entry point — DroneGCS orchestrator
├── boot.py                # Boot screen and reconnect screen rendering
├── stfinal.py             # DroneDisplay — ST7735 telemetry UI renderer
├── flags.py               # SharedFlags — thread-safe inter-thread state
├── mavlink_thread.py      # MAVLinkFlagThread — connects to FC and sets flags
├── joystick_thread.py     # JoystickFlagThread — detects USB joystick presence
├── rc_override_thread.py  # RCOverrideThread — reads joystick, sends RC_CHANNELS_OVERRIDE
├── errors.json            # Pre-arm error classification rules
└── Y.png                  # Logo shown on boot screen
```

---

## How It Works

### Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│                        main.py (DroneGCS)                   │
│                                                             │
│  ┌─────────────┐  ┌──────────────┐  ┌───────────────────┐  │
│  │MAVLinkFlag  │  │JoystickFlag  │  │  RCOverride       │  │
│  │Thread       │  │Thread        │  │  Thread           │  │
│  │(connects FC)│  │(detects USB  │  │(reads joystick →  │  │
│  │             │  │ joystick)    │  │ RC_CHANNELS_      │  │
│  └──────┬──────┘  └──────┬───────┘  │ OVERRIDE @ 20Hz)  │  │
│         │                │          └─────────┬─────────┘  │
│         └────────────────┴───────────────┐    │            │
│                                    SharedFlags              │
│                                    (mavlink_connected,      │
│                                     slave_connected,        │
│                                     rc10_active, …)         │
│         ┌──────────────────────────────────────────────┐   │
│         │  MAVLinkReader (telemetry loop thread)        │   │
│         │  - HEARTBEAT, GPS, ALT, STATUSTEXT, RC_CH    │   │
│         └────────────────────┬─────────────────────────┘   │
│                              │ dirty event                  │
│         ┌────────────────────▼─────────────────────────┐   │
│         │  Render Loop (daemon thread)                  │   │
│         │  DroneDisplay.render() → ST7735               │   │
│         │  pyttsx3 voice announcements                  │   │
│         └──────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
```

### State Machine

`DroneGCS` runs a three-state machine on the main thread:

| State | Description |
|---|---|
| `BOOT` | Waits for MAVLink link and joystick to connect; shows progress blocks |
| `ACTIVE` | Streams telemetry to display; monitors for disconnections |
| `RECONNECTING` | Blocks with animated spinner until the dropped device reconnects |

### RC Handoff (Trainer ↔ Trainee)

- The flight controller streams RC channel data over MAVLink.
- When **CH10 > 1500 µs**, `rc10_active` is set to `True`.
- `RCOverrideThread` reads the slave joystick axes and sends `RC_CHANNELS_OVERRIDE` MAVLink messages at **20 Hz**, giving the Trainee full stick control.
- When CH10 is released, a zero-value override packet is sent to restore master RC authority.

### Pre-arm Checks

Every 5 seconds while disarmed, the GCS sends `MAV_CMD_RUN_PREARM_CHECKS` and collects `STATUSTEXT` messages for 1 second. Results are classified via `errors.json` and displayed on the status panel.

---

## Running

```bash
cd FF
python main.py
```

The MAVLink connection defaults to UDP (`udp:0.0.0.0:14550`). To use a serial connection, edit `DroneGCS.CONNECTION` in `main.py`:

```python
CONNECTION = "/dev/ttyACM0"   # serial at 57600 baud
```

---

## Configuration

### Joystick Axis Mapping (`rc_override_thread.py`)

```python
AXIS_ROLL     = 0   # Right stick X  → CH1
AXIS_PITCH    = 1   # Right stick Y  → CH2
AXIS_THROTTLE = 2   # Left  stick Y  → CH3
AXIS_YAW      = 3   # Left  stick X  → CH4
```

Adjust these indices to match your transmitter's USB HID axis order. Use `jstest /dev/input/js0` to identify them.

### MAVLink Serial Ports (`mavlink_thread.py`)

```python
PORTS = ["/dev/ttyACM0", "/dev/ttyACM1"]
BAUD  = 57600
```

### Pre-arm Error Rules (`errors.json`)

Add or edit rules to classify MAVLink `STATUSTEXT` messages into short display strings:

```json
{
  "battery": { "type": "PREARM", "short": "Battery low",   "severity": "HIGH" },
  "gps":     { "type": "PREARM", "short": "No GPS fix",    "severity": "HIGH" }
}
```

The key is a **lowercase substring** matched against the incoming status text.

---

## Display Layout

```
┌──────────────────┐
│     TRAINER      │  ← Header (switches to TRAINEE when CH10 active)
├─────────┬────────┤
│ ◉ LOCK  │ ✦ LOITER│  ← GPS fix | Flight mode
├─────────┼────────┤
│ ⌇IN     │ ↑12.4m │  ← Flight state | Altitude
│  FLIGHT │        │
├──────────────────┤
│      STATUS      │
│  ─────────────── │
│   READY TO ARM   │  ← Pre-arm status (green/yellow/red)
└──────────────────┘
```

---

## Known Limitations

- RSSI is received in the telemetry pipeline but not currently rendered on screen.
- Audio (`pyttsx3`) is synchronous and may briefly block the render loop on slow hardware. Consider moving to a dedicated TTS thread for latency-sensitive applications.
- The display driver assumes a BGR colour channel order (colours are swapped before sending to the ST7735).

---

