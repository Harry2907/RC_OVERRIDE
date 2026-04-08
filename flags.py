import threading


class SharedFlags:
    def __init__(self):
        self._lock  = threading.Lock()
        self._event = threading.Event()

        self._mavlink_connected = False
        self._slave_connected   = False
        self.running            = True

    # ── internal setter ───────────────────────────────────────────────────
    def _set(self, attr, value):
        with self._lock:
            if getattr(self, attr) != value:
                setattr(self, attr, value)
                self._event.set()

    # ── properties ────────────────────────────────────────────────────────
    @property
    def mavlink_connected(self):
        return self._mavlink_connected

    @mavlink_connected.setter
    def mavlink_connected(self, v):
        self._set("_mavlink_connected", v)

    @property
    def slave_connected(self):
        return self._slave_connected

    @slave_connected.setter
    def slave_connected(self, v):
        self._set("_slave_connected", v)

    # ── wait for any flag change ──────────────────────────────────────────
    def wait(self, timeout=1.0):
        self._event.wait(timeout=timeout)
        self._event.clear()