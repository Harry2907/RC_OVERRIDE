import time
import threading
import json
import pyttsx3
from serial import SerialException
from pymavlink import mavutil
from stfinal import DroneDisplay
from boot import boot_screen, reconnect_screen
from flags import SharedFlags
from mavlink_thread import MAVLinkFlagThread
from joystick_thread import JoystickFlagThread
from rc_override_thread import RCOverrideThread


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class MAVLinkReader:

    def __init__(self, connection_string, state, lock, dirty, flags):
        self._flags             = flags
        self._connection_string = connection_string
        self._state             = state
        self._lock              = lock
        self._dirty             = dirty
        self._master            = None
        self._thread            = threading.Thread(target=self._thread_loop, daemon=True)

        with open("errors.json", "r") as f:
            self._rules = json.load(f)

        self._got_heartbeat  = False
        self._boot_complete  = False

        self._last_prearm_check = 0
        self._prearm_interval   = 5
        self._armed_latched     = False
        self._collecting        = False
        self._collect_start     = 0
        self._collect_duration  = 1.0
        self._prearm_errors     = set()
        self._disarm_counter    = 0
        self._last_status       = None

    def _classify_error(self, msg):
        msg_lower = msg.lower()
        for key, val in self._rules.items():
            if key in msg_lower:
                return val["short"]
        return msg

    def _request_streams(self):
        """Ask the FC to stream all telemetry we need."""
        streams = [
            mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS,
            mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
            mavutil.mavlink.MAV_DATA_STREAM_POSITION,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
            mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS,
        ]
        for stream_id in streams:
            self._master.mav.request_data_stream_send(
                self._master.target_system,
                self._master.target_component,
                stream_id,
                4,    # 4 Hz
                1     # start streaming
            )
        print("[MAVLINK] Stream rates requested")

    def connect(self):
        print("Connecting to MAVLink...")

        if self._flags.mavlink_master is not None:
            self._master = self._flags.mavlink_master
            print("Reusing existing MAVLink connection!")
        else:
            print("Opening NEW connection (fallback)")
            self._master = mavutil.mavlink_connection(self._connection_string)
            print("Waiting for heartbeat...")
            self._master.wait_heartbeat()
            self._flags.mavlink_connected = True
            print("Connected!")

        self._request_streams()

    def start(self):
        self._thread.start()

    def _thread_loop(self):
        while True:
            # ── SerialException = master physically unplugged ─────────────
            try:
                msg = self._master.recv_match(blocking=True, timeout=0.5)
            except SerialException as e:
                print(f"[MAVLINK] Serial disconnected: {e}")
                self._flags.mavlink_connected   = False
                self._flags.mavlink_master      = None
                self._flags.disconnected_device = "master"
                self._dirty.set()
                return   # kill this thread — DroneGCS will spawn a fresh one

            if msg is None:
                continue

            mtype   = msg.get_type()
            changed = False
            now     = time.time()

            with self._lock:

                # ── HEARTBEAT ────────────────────────────────────────────
                if mtype == "HEARTBEAT":
                    self._got_heartbeat = True
                    mode = mavutil.mode_string_v10(msg)

                    if "Mode(" not in mode and mode != self._state["mode"]:
                        self._state["mode"] = mode
                        changed = True

                    armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0

                    if armed:
                        self._armed_latched  = True
                        self._disarm_counter = 0
                    else:
                        self._disarm_counter += 1
                        if self._disarm_counter > 3:
                            self._armed_latched = False

                    if self._armed_latched:
                        if self._state["status_msg"] != "ARMED":
                            self._state["status_msg"] = "ARMED"
                            self._last_status = "ARMED"
                            changed = True

                    if self._state.get("armed") != self._armed_latched:
                        self._state["armed"] = self._armed_latched
                        changed = True

                    fc_in_flight  = (msg.system_status == mavutil.mavlink.MAV_STATE_ACTIVE)
                    alt_in_flight = self._state["altitude"] > 0.3
                    new_in_flight = fc_in_flight or alt_in_flight

                    if new_in_flight != self._state["in_flight"]:
                        self._state["in_flight"] = new_in_flight
                        changed = True

                elif mtype == "GPS_RAW_INT":
                    fix = msg.fix_type >= 3
                    if fix != self._state["gps_fix"]:
                        self._state["gps_fix"] = fix
                        changed = True

                elif mtype == "GLOBAL_POSITION_INT":
                    alt = round(msg.relative_alt / 1000.0, 1)
                    if abs(alt - self._state["altitude"]) >= 0.1:
                        self._state["altitude"] = alt
                        changed = True

                elif mtype == "STATUSTEXT":
                    text = msg.text.decode() if isinstance(msg.text, bytes) else msg.text
                    if self._collecting and "PreArm" in text:
                        self._prearm_errors.add(text)

                elif mtype == "RC_CHANNELS":
                    ch10 = msg.chan10_raw
                    new_rc10 = ch10 > 1500
                    if new_rc10 != self._flags.rc10_active:
                        self._flags.rc10_active = new_rc10
                        print(f"[RC10] {'SLAVE active' if new_rc10 else 'MASTER restored'} (CH10={ch10})")

                # ── BOOT COMPLETION ──────────────────────────────────────
                if not self._boot_complete:
                    if self._got_heartbeat:
                        self._boot_complete = True
                        self._dirty.set()

                    if self._state["status_msg"] != "INITIALIZING...":
                        self._state["status_msg"] = "INITIALIZING..."
                        changed = True

                else:
                    # ── PREARM CYCLE (every 5 s while disarmed) ──────────
                    if (not self._state.get("armed", False)) and \
                       (now - self._last_prearm_check > self._prearm_interval):

                        self._master.mav.command_long_send(
                            self._master.target_system,
                            self._master.target_component,
                            mavutil.mavlink.MAV_CMD_RUN_PREARM_CHECKS,
                            0, 0, 0, 0, 0, 0, 0, 0
                        )
                        self._prearm_errors.clear()
                        self._collecting        = True
                        self._collect_start     = now
                        self._last_prearm_check = now

                    # ── COLLECT WINDOW CLOSED — write status ─────────────
                    if self._collecting and (now - self._collect_start > self._collect_duration):
                        self._collecting = False

                        if not self._state.get("armed", False):
                            if self._prearm_errors:
                                errors       = list(self._prearm_errors)[:2]
                                short_errors = [self._classify_error(e) for e in errors]
                                new_status   = " | ".join(short_errors)
                            else:
                                new_status = "READY TO ARM"

                            if new_status != self._last_status:
                                self._state["status_msg"] = new_status
                                self._last_status = new_status
                                changed = True

            if changed:
                self._dirty.set()


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class DroneGCS:

    RENDER_INTERVAL = 0.05
    CONNECTION      = "udp:0.0.0.0:14550"

    def __init__(self):

        self._state = {
            "mode":       "N/A",
            "gps_fix":    False,
            "altitude":   0.0,
            "rssi":       -1,
            "in_flight":  False,
            "status_msg": "INITIALIZING...",
            "armed":      False,
        }

        self._lock  = threading.Lock()
        self._dirty = threading.Event()

        self._flags = SharedFlags()
        MAVLinkFlagThread(self._flags).start()
        JoystickFlagThread(self._flags).start()
        RCOverrideThread(self._flags).start()

        self._display = DroneDisplay()

        self._engine = pyttsx3.init()
        self._engine.setProperty('rate', 140)
        self._engine.setProperty('volume', 0.1)

        self._render_thread = None   # tracked so we don't double-start

    # ── helpers ───────────────────────────────────────────────────────────
    def _reset_state(self):
        """Clear telemetry back to defaults before a reconnect cycle."""
        with self._lock:
            self._state.update({
                "mode":       "N/A",
                "gps_fix":    False,
                "altitude":   0.0,
                "rssi":       -1,
                "in_flight":  False,
                "status_msg": "INITIALIZING...",
                "armed":      False,
            })

    def _start_mavlink(self):
        """Create a fresh MAVLinkReader, connect, and start its thread."""
        mavlink = MAVLinkReader(
            self.CONNECTION,
            self._state,
            self._lock,
            self._dirty,
            self._flags,
        )
        mavlink.connect()
        mavlink.start()
        return mavlink

    def _start_render_loop(self):
        """Spin up the render thread (only once — it runs forever)."""
        if self._render_thread and self._render_thread.is_alive():
            return
        self._render_thread = threading.Thread(
            target=self._render_loop, daemon=True
        )
        self._render_thread.start()

    # ── STATE MANAGER (main thread) ───────────────────────────────────────
    def _state_manager(self):
        d          = self._display
        disp       = d.display
        img        = d.image
        draw       = d.draw
        W, H       = d.W, d.H
        font_mid   = d.font_status
        font_small = d.font_mode

        STATE = "BOOT"
        print(f"[STATE] {STATE}")

        while True:

            # ── BOOT ──────────────────────────────────────────────────────
            if STATE == "BOOT":
                boot_screen(disp, img, draw, W, H,
                            font_mid, font_small, self._flags)
                print("[STATE] Boot complete — 100%")
                STATE = "ACTIVE"

            # ── ACTIVE ────────────────────────────────────────────────────
            elif STATE == "ACTIVE":
                print("[STATE] ACTIVE")

                came_from = self._flags.disconnected_device  # "master"|"slave"|None
                self._flags.disconnected_device = None

                # Only (re)start MAVLink on first boot or after a master disconnect.
                # After a slave reconnect the MAVLink thread is still alive —
                # spawning a second one causes double reads on the serial port.
                if came_from != "slave":
                    self._reset_state()
                    mavlink = self._start_mavlink()

                self._dirty.set()
                self._start_render_loop()

                # watch for either device dropping
                while True:
                    self._flags.wait(timeout=0.3)

                    # master serial dropped (MAVLinkReader thread set the flag and died)
                    if not self._flags.mavlink_connected:
                        print("[STATE] Master disconnected → RECONNECTING")
                        self._flags.disconnected_device = "master"
                        self._engine.say("Trainer disconnected")
                        self._engine.runAndWait()
                        STATE = "RECONNECTING"
                        break

                    # slave joystick unplugged
                    if not self._flags.slave_connected:
                        print("[STATE] Slave disconnected → RECONNECTING")
                        self._flags.disconnected_device = "slave"
                        self._engine.say("Trainee disconnected")
                        self._engine.runAndWait()
                        STATE = "RECONNECTING"
                        break
            # ── RECONNECTING ──────────────────────────────────────────────
            elif STATE == "RECONNECTING":
                device = self._flags.disconnected_device   # "master" | "slave"
                print(f"[STATE] RECONNECTING — waiting for {device}")

                # blocks here, spinning on screen, until device is back
                reconnect_screen(disp, img, draw, W, H, self._flags, device)

                # reconnect_screen drew all over the shared image buffer.
                # Clear DroneDisplay's state cache so render() forces a full
                # redraw on the very next cycle instead of skipping (white screen).
                self._display.force_redraw()
                self._dirty.set()

                label = "Trainer" if device == "master" else "Trainee"
                self._engine.say(f"{label} reconnected")
                self._engine.runAndWait()
                print(f"[STATE] {device} reconnected → ACTIVE")
                STATE = "ACTIVE"

    def start(self):
        self._state_manager()

    # ── RENDER LOOP (daemon thread, runs forever) ─────────────────────────
    def _render_loop(self):
        last_mode_spoken = None
        last_rc10_spoken = None

        while True:
            self._dirty.wait(timeout=self.RENDER_INTERVAL)
            self._dirty.clear()

            with self._lock:
                s = dict(self._state)

            self._display.update_data(
                s["mode"],
                s["gps_fix"],
                s["altitude"],
                s["rssi"],
                s["in_flight"],
                s["status_msg"],
                slave_mode=self._flags.rc10_active,
            )

            self._display.render()

            if s["mode"] != last_mode_spoken and s["mode"] != "N/A":
                self._engine.say(f"Switched to {s['mode']} MODE")
                self._engine.runAndWait()
                last_mode_spoken = s["mode"]

            rc10 = self._flags.rc10_active
            if rc10 != last_rc10_spoken:
                speech = "Control to slave" if rc10 else "Control to master"
                self._engine.say(speech)
                self._engine.runAndWait()
                last_rc10_spoken = rc10


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
if __name__ == "__main__":
    DroneGCS().start()