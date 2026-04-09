# Drone GCS — Master/Slave Ground Control Station

A multithreaded ground control station (GCS) running on a Raspberry Pi. Provides real-time telemetry display on a TFT LCD, dual-operator (master/slave) RC control switching via MAVLink, joystick input forwarding, voice announcements, and automatic reconnect handling for both the flight controller and slave joystick.

---

## Hardware Requirements

| Component | Details |
|---|---|
| Raspberry Pi | Any model with USB ports (tested on Ubuntu 24) |
| TFT LCD Display | SPI-connected, driven via `stfinal.py` |
| Flight Controller | ArduPilot/PX4 — connected via USB serial (`/dev/ttyACM0–4`) |
| Master RC (Trainer) | FC USB connection at 57600 baud |
| Slave RC (Trainee) | Radiomaster TX12 USB joystick (`/dev/input/jsX`) |
| Speaker | For voice announcements via pyttsx3 |

---

## Software Dependencies

```bash
pip install pymavlink pygame pillow pyttsx3 pyserial
```

---

## Project Structure

```
Final_Int/
├── main.py               # Entry point — DroneGCS app + MAVLinkReader
├── mavlink_thread.py     # MAVLinkFlagThread — serial port scanner & connector
├── joystick_thread.py    # JoystickFlagThread — slave joystick plug/unplug detection
├── rc_override_thread.py # RCOverrideThread — joystick axis → MAVLink RC override
├── boot.py               # Boot screen + reconnect screen (TFT LCD UI)
├── stfinal.py            # DroneDisplay — TFT LCD rendering engine
├── flags.py              # SharedFlags — thread-safe shared state object
└── errors.json           # PreArm error classification rules
```

---

## Architecture Overview

The entire application runs as a **single process** with **7 threads**.

### Process & Thread Map

```
python main.py  (one process)
│
├── [Main Thread]          _state_manager()       — BOOT → ACTIVE → RECONNECTING
├── [MAVLinkFlagThread]    mavlink_thread.py       — serial port scanner & connector
├── [MAVLinkReader]        main.py                 — MAVLink message parser (respawned on disconnect)
├── [JoystickFlagThread]   joystick_thread.py      — slave joystick plug/unplug watcher
├── [RCOverrideThread]     rc_override_thread.py   — joystick axes → RC_CHANNELS_OVERRIDE
├── [Render Thread]        main.py _render_loop()  — TFT LCD display updater (50ms tick)
└── [TTS Thread]           main.py _tts_loop()     — pyttsx3 voice announcements (queue-based)
```

### Thread Responsibilities

**Main Thread — `_state_manager()`**
The Python process entry point. Runs the top-level state machine and never returns. Owns the BOOT, ACTIVE, and RECONNECTING states. All other threads are spawned from here or from `__init__`.

**MAVLinkFlagThread**
Scans `/dev/ttyACM0` through `/dev/ttyACM4` on startup. Calls `wait_heartbeat()` to confirm a live FC, then stores the connection object in `SharedFlags.mavlink_master` and sets `mavlink_connected = True`. On disconnect it clears the stale connection and retries. Does not read any MAVLink data — connection duty only.

**MAVLinkReader thread**
Owns all MAVLink data reading. Uses the connection object established by `MAVLinkFlagThread`. Parses HEARTBEAT, GPS_RAW_INT, GLOBAL_POSITION_INT, STATUSTEXT, and RC_CHANNELS messages. Writes parsed values into the shared `state` dict under a lock and sets the `dirty` event to trigger a display redraw. Dies on `SerialException` (physical unplug) and is respawned fresh after reconnect. This is the only thread that gets killed and recreated.

**JoystickFlagThread**
Monitors the slave TX12 USB joystick. Sets `flags.slave_connected = True` when plugged in, `False` when removed.

**RCOverrideThread**
Reads joystick axes (roll, pitch, yaw, throttle) continuously and sends `RC_CHANNELS_OVERRIDE` MAVLink packets to the FC. When `rc10_active` is False (master in control), overrides are suppressed and the FC falls back to master RC input automatically.

**Render Thread**
Wakes every 50ms (or immediately when `dirty` event is set). Takes a snapshot of the `state` dict under the lock, calls `DroneDisplay.update_data()` then `DroneDisplay.render()`. Runs forever — never restarted. Handles mode change and RC10 switch voice triggers.

**TTS Thread**
The sole owner of the `pyttsx3` engine. All other threads call `_speak("text")` which is non-blocking — it just drops the text into a `queue.Queue`. The TTS thread drains this queue one message at a time with `engine.runAndWait()`. This prevents pyttsx3 deadlocks that would freeze the main thread and LCD.

---

## Classes & Objects

### `SharedFlags` (`flags.py`)
Thread-safe shared memory bus. One instance created at startup, passed to every thread. Uses an internal `threading.Lock` for property reads/writes and a `threading.Event` for change notifications.

| Flag | Type | Set by | Read by |
|---|---|---|---|
| `mavlink_master` | connection object | MAVLinkFlagThread | MAVLinkReader |
| `mavlink_connected` | bool | MAVLinkFlagThread / MAVLinkReader | Main thread, boot screen |
| `slave_connected` | bool | JoystickFlagThread | Main thread, boot screen |
| `rc10_active` | bool | MAVLinkReader (CH10) | RCOverrideThread, Render thread |
| `disconnected_device` | `"master"` / `"slave"` / `None` | Main thread | Main thread (state machine) |

### `MAVLinkReader` (`main.py`)
Parses MAVLink telemetry and maintains the `state` dict. Respawned on every master reconnect. Handles boot-complete detection, prearm check cycling, and GPS stream retry logic.

### `DroneGCS` (`main.py`)
Top-level application class. Owns the `state` dict, the `threading.Lock`, and the `dirty` event. Creates all threads at init. Runs `_state_manager()` on the main thread.

### `DroneDisplay` (`stfinal.py`)
TFT LCD rendering engine. Exposes `update_data()` to push new telemetry values, `render()` to redraw the screen (diff-based, skips unchanged frames), and `force_redraw()` to clear the diff cache after the reconnect screen overwrites the buffer.

---

## State Machine

```
         ┌──────────────────────────────────────────┐
         ▼                                          │
       BOOT                                         │
  (waits for MAVLink                                │
   + joystick ready)                                │
         │                                          │
         ▼                                          │
       ACTIVE  ──── master drops ──► RECONNECTING ──┘
         │
         └────── slave drops ────► RECONNECTING ────┘
```

**BOOT** — Displays progress blocks on TFT. Blocks until `mavlink_connected` and `slave_connected` are both True. Advances to ACTIVE.

**ACTIVE** — Spawns a fresh MAVLinkReader (unless returning from a slave-only disconnect). Starts the render loop. Watches `SharedFlags` for any disconnect event.

**RECONNECTING** — Displays animated reconnect screen on TFT. Blocks until the disconnected device comes back. On return, calls `force_redraw()` and transitions back to ACTIVE.

---

## Control Flow — Data Path

```
Flight Controller (USB serial)
        │
        ▼
MAVLinkFlagThread  ──connects──►  SharedFlags.mavlink_master
                                          │
                                          ▼
                                   MAVLinkReader
                                   (parses messages)
                                          │
                               updates state dict + sets dirty
                                          │
                                          ▼
                                   Render Thread
                                   (wakes on dirty)
                                          │
                                          ▼
                                   DroneDisplay
                                   (draws TFT LCD)

TX12 Joystick (USB)
        │
        ├──► JoystickFlagThread  ──► flags.slave_connected
        │
        └──► RCOverrideThread   ──► RC_CHANNELS_OVERRIDE ──► FC

Any Thread
        │
        └──► _speak("text")  ──► tts_queue  ──► TTS Thread  ──► Speaker
```

---

## Master / Slave Control Switching

Channel 10 on the master RC transmitter controls who has authority over the drone.

| CH10 Value | Mode | Behaviour |
|---|---|---|
| `> 1500` | Slave active | RCOverrideThread sends joystick values to FC |
| `≤ 1500` | Master active | RC overrides suppressed — FC uses master RC input |

The switch state is detected by `MAVLinkReader` on every `RC_CHANNELS` message and stored in `flags.rc10_active`. The render thread announces the change via TTS.

---

## Known Limitations

- `MAVLinkFlagThread` does not detect a frozen FC (USB connected but FC unresponsive) — only a physical `SerialException` triggers a reconnect cycle.
- TTS announcements queue up — if multiple events fire in quick succession, speech plays sequentially with a delay.
- Boot screen enforces MAVLink before joystick — if joystick connects first, the display appears frozen until MAVLink is ready.

---

## Running

```bash
cd Final_Int
python main.py
```

Expected startup output:
```
[STATE] BOOT
✓ MAVLink Connected (/dev/ttyACM0)
[MAVLINK THREAD] Connected
[JS THREAD] Connected: OpenTX Radiomaster TX12 Joystick
[STATE] Boot complete — 100%
[STATE] ACTIVE
Connecting to MAVLink...
Reusing existing MAVLink connection!
[MAVLINK] Stream rates requested
```