import time
import threading
from pymavlink import mavutil


class MAVLinkFlagThread:
    """
    Lightweight MAVLink heartbeat watcher.
    Only job: set flags.mavlink_connected True/False.
    Does NOT own telemetry — that stays in MAVLinkReader (noaudthread).
    """

    PORTS    = ["/dev/ttyACM0", "/dev/ttyACM1"]
    BAUD     = 57600
    TIMEOUT  = 5       # heartbeat wait seconds
    RETRY    = 2       # seconds between retries

    def __init__(self, flags):
        self._flags  = flags
        self._master = None
        self._thread = threading.Thread(target=self._loop, daemon=True)

    def start(self):
        self._thread.start()

    def _connect(self):
        for port in self.PORTS:
            try:
                m = mavutil.mavlink_connection(port, baud=self.BAUD)
                m.wait_heartbeat(timeout=self.TIMEOUT)
                            
                print(f"✓ MAVLink Connected ({port})")
                msg = m.recv_match(timeout=3)
                
                print(msg)
                self._master = m
                return True
            except Exception:
                pass
        return False

    def _loop(self):
        while self._flags.running:

            # ── try to connect ────────────────────────────────────────────
            if not self._flags.mavlink_connected:
                if self._connect():
                    print("[MAVLINK THREAD] Connected")
                    self._flags.mavlink_connected = True
                else:
                    time.sleep(self.RETRY)
                    continue

            # ── watch for heartbeat loss ──────────────────────────────────
            # msg = self._master.recv_match(blocking=True, timeout=3)
            # print(msg)
            if msg is None:
                print("[MAVLINK THREAD] Heartbeat lost")
                self._flags.mavlink_connected = False
                self._master = None
