import time
import threading
import os

os.environ["SDL_VIDEODRIVER"] = "dummy"
import pygame
pygame.init()
pygame.joystick.init()


class JoystickFlagThread:
    """
    Watches for USB joystick (slave TX12) presence only.
    Sets:
        flags.slave_connected  -> True when joystick is plugged in, False when removed

    Disconnect detection strategy:
      - Connect:    poll quit()+init()+get_count() every 1s while no joystick present.
                    Safe because RCOverrideThread only reads axes when rc10_active AND
                    slave_connected are both True — neither is true while searching.
      - Disconnect: check /dev/input/js* at 10 Hz. This is OS-level and never
                    touches the pygame joystick subsystem, so it cannot race with
                    RCOverrideThread reading axes at 20 Hz. Works on pygame 1.9.x
                    and 2.x alike.
    """

    POLL = 1.0   # seconds between connect-attempt polls (only while disconnected)

    def __init__(self, flags):
        self._flags = flags
        self._js    = None
        self._thread = threading.Thread(target=self._loop, daemon=True)

    def start(self):
        self._thread.start()

    # ── joystick init (only called when self._js is None) ─────────────────
    def _try_init(self):
        # Full reinit is safe here: slave_connected=False so RCOverrideThread
        # is idle and not touching the pygame joystick subsystem.
        pygame.joystick.quit()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            return None

        js = pygame.joystick.Joystick(0)
        js.init()
        print(f"[JS THREAD] Connected: {js.get_name()}")
        return js

    # ── main loop ─────────────────────────────────────────────────────────
    def _loop(self):
        while self._flags.running:

            # ── no joystick — poll until one appears ──────────────────────
            if self._js is None:
                self._flags.slave_connected = False
                self._js = self._try_init()
                if self._js is None:
                    time.sleep(self.POLL)
                    continue
                self._flags.slave_connected = True

            # ── joystick present — detect unplug via /dev/input ───────────
            # We never call pygame.joystick.quit()/init() here, so there is
            # zero risk of racing with RCOverrideThread's axis reads.
            # /dev/input/js0 (and js1, etc.) disappears the moment the USB
            # device is removed — reliable on all Linux kernels.
            joy_present = any(f.startswith("js") for f in os.listdir("/dev/input"))
            if not joy_present:
                print("[JS THREAD] Disconnected (/dev/input/js* gone)")
                self._js = None
                self._flags.slave_connected = False

            time.sleep(0.1)   # 10 Hz — fast enough, low CPU