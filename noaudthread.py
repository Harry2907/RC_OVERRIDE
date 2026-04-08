import time
import threading
import json
import pyttsx3
from pymavlink import mavutil
from stfinal import DroneDisplay
from boot import boot_screen
from flags import SharedFlags
from mavlink_thread import MAVLinkFlagThread
from joystick_thread import JoystickFlagThread


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class MAVLinkReader:

    def __init__(self, connection_string, state, lock, dirty):
        self._connection_string = connection_string
        self._state = state
        self._lock = lock
        self._dirty = dirty
        self._master = None
        self._thread = threading.Thread(target=self._thread_loop, daemon=True)

        # ---------------- LOAD JSON ----------------
        with open("errors.json", "r") as f:
            self._rules = json.load(f)

        # ---------------- BOOT FLAGS ----------------
        self._got_heartbeat = False
        self._got_gps = False
        self._got_sys = False
        self._boot_complete = False

        # ---------------- PREARM SYSTEM ----------------
        self._last_prearm_check = 0
        self._prearm_interval = 5
        self._armed_latched = False
        self._collecting = False
        self._collect_start = 0
        self._collect_duration = 1.0
        self._prearm_errors = set()
        self._disarm_counter = 0
        self._last_status = None

    def _classify_error(self, msg):
        msg_lower = msg.lower()

        for key, val in self._rules.items():
            if key in msg_lower:
                return val["short"]

        return msg

    def connect(self):
        print("Connecting to MAVLink…")
        self._master = mavutil.mavlink_connection(self._connection_string)

        print("Waiting for heartbeat…")
        self._master.wait_heartbeat()
        print("Connected!")

    def start(self):
        self._thread.start()

    def _thread_loop(self):
        while True:
            msg = self._master.recv_match(blocking=True, timeout=0.5)
            if msg is None:
                continue

            mtype = msg.get_type()
            changed = False
            now = time.time()

            with self._lock:

# ---------------- HEARTBEAT ----------------
                if mtype == "HEARTBEAT":
                    self._got_heartbeat = True
                    mode = mavutil.mode_string_v10(msg)

                    if "Mode(" not in mode and mode != self._state["mode"]:
                        self._state["mode"] = mode
                        changed = True

                    armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0

                    # --- LATCH LOGIC WITH DEBOUNCE ---
                    if armed:
                        self._armed_latched = True
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

                    fc_in_flight = (msg.system_status == mavutil.mavlink.MAV_STATE_ACTIVE)
                    alt_in_flight = self._state["altitude"] > 0.3
                    new_in_flight = fc_in_flight or alt_in_flight

                    if new_in_flight != self._state["in_flight"]:
                        self._state["in_flight"] = new_in_flight
                        changed = True

                elif mtype == "GPS_RAW_INT":
                    self._got_gps = True
                    fix = msg.fix_type >= 3

                    if fix != self._state["gps_fix"]:
                        self._state["gps_fix"] = fix
                        changed = True

                elif mtype == "SYS_STATUS":
                    self._got_sys = True

                elif mtype == "GLOBAL_POSITION_INT":
                    alt = round(msg.relative_alt / 1000.0, 1)

                    if abs(alt - self._state["altitude"]) >= 0.1:
                        self._state["altitude"] = alt
                        changed = True

                elif mtype == "STATUSTEXT":
                    text = msg.text.decode() if isinstance(msg.text, bytes) else msg.text

                    if self._collecting and "PreArm" in text:
                        self._prearm_errors.add(text)

                if not self._boot_complete:

                    if self._got_heartbeat and self._got_gps and self._got_sys:
                        self._boot_complete = True

                    if self._state["status_msg"] != "INITIALIZING...":
                        self._state["status_msg"] = "INITIALIZING..."
                        changed = True

                elif (not self._state.get("armed", False)) and (now - self._last_prearm_check > self._prearm_interval):

                    self._master.mav.command_long_send(
                        self._master.target_system,
                        self._master.target_component,
                        mavutil.mavlink.MAV_CMD_RUN_PREARM_CHECKS,
                        0,
                        0, 0, 0, 0, 0, 0, 0
                    )

                    self._prearm_errors.clear()
                    self._collecting = True
                    self._collect_start = now
                    self._last_prearm_check = now

                if self._collecting and (now - self._collect_start > self._collect_duration):

                    self._collecting = False
                    armed = self._state.get("armed", False)

                    if not armed:
                        if self._prearm_errors:
                            errors = list(self._prearm_errors)[:2]
                            short_errors = [self._classify_error(e) for e in errors]
                            new_status = " | ".join(short_errors)
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
    CONNECTION = "udp:0.0.0.0:14550"

    def __init__(self):
        self._state = {
            "mode": "N/A",
            "gps_fix": False,
            "altitude": 0.0,
            "rssi": -1,
            "in_flight": False,
            "status_msg": "INITIALIZING...",
            "armed": False,
        }
        self._lock  = threading.Lock()
        self._dirty = threading.Event()

        self._mavlink = MAVLinkReader(
            self.CONNECTION,
            self._state,
            self._lock,
            self._dirty,
        )

        self._display = DroneDisplay()

        # ---------------- AUDIO ENGINE ----------------
        self._engine = pyttsx3.init()
        self._engine.setProperty('rate', 140)
        self._engine.setProperty('volume', 0.1)

        self._last_mode_spoken = None

        # ── NEW: shared flags + background threads ────────────────────────
        self._flags = SharedFlags()
        MAVLinkFlagThread(self._flags).start()
        JoystickFlagThread(self._flags).start()

    # ── NEW: state manager ────────────────────────────────────────────────
    def _state_manager(self):
        STATE = "BOOT"

        d = self._display
        # grab the display hardware objects boot_screen needs
        disp       = d.display
        img        = d.image
        draw       = d.draw
        W, H       = d.W, d.H
        font_mid   = d.font_status
        font_small = d.font_mode

        print(f"[STATE] {STATE}")

        while True:

            # ── BOOT ─────────────────────────────────────────────────────
            if STATE == "BOOT":
                boot_screen(disp, img, draw, W, H,
                            font_mid, font_small, self._flags)
                # boot_screen returns only when all 3 blocks filled
                STATE = "ACTIVE_READY"
                print("[STATE] Boot complete — 100%")

            # ── ACTIVE_READY ─────────────────────────────────────────────
            elif STATE == "ACTIVE_READY":
                # placeholder — ACTIVE handoff implemented later
                print("[STATE] System ready. ACTIVE state not yet implemented.")
                break

    def start(self):
        self._state_manager()

        # _render_loop left intact, called later when ACTIVE is implemented
        # self._mavlink.connect()
        # self._mavlink.start()
        # self._render_loop()

    def _render_loop(self):
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
            )

            self._display.render()

            if s["mode"] != self._last_mode_spoken and s["mode"] != "N/A":
                speech = f"Switched to {s['mode']} MODE"
                self._engine.say(speech)
                self._engine.runAndWait()
                self._last_mode_spoken = s["mode"]


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
if __name__ == "__main__":
    DroneGCS().start()
