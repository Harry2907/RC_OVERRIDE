import time
import threading
from pymavlink import mavutil


class MAVLinkFlagThread:

    PORTS    = ["/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyACM2", "/dev/ttyACM3", "/dev/ttyACM4"]
    BAUD     = 57600
    TIMEOUT  = 5
    RETRY    = 2

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

                self._flags.mavlink_master    = m
                self._flags.mavlink_connected = True

                self._master = m
                return True

            except Exception:
                continue

        return False

    def _loop(self):
        while self._flags.running:

            if not self._flags.mavlink_connected:
                # Clear stale master reference before attempting reconnect
                # so MAVLinkReader doesn't try to reuse a dead connection.
                self._flags.mavlink_master = None

                if self._connect():
                    print("[MAVLINK THREAD] Connected")
                else:
                    time.sleep(self.RETRY)
                    continue

            # DO NOTHING ELSE — MAVLinkReader owns all reading.
            time.sleep(1)
