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
        flags.slave_connected  → True when joystick is plugged in, False when removed
    No axis reading — stick movement is not required at boot.
    """

    POLL = 1.0   # seconds between plug/unplug checks

    def __init__(self, flags):
        self._flags = flags
        self._js    = None
        self._thread = threading.Thread(target=self._loop, daemon=True)

    def start(self):
        self._thread.start()

    # ── joystick init ─────────────────────────────────────────────────────
    def _try_init(self):
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

            # ── no joystick — try to find one ─────────────────────────────
            if self._js is None:
                self._flags.slave_connected = False

                self._js = self._try_init()
                if self._js is None:
                    time.sleep(self.POLL)
                    continue

                self._flags.slave_connected = True

            # ── joystick present — just check it's still there ────────────
            try:
                pygame.event.pump()
                time.sleep(self.POLL)

            except Exception:
                print("[JS THREAD] Disconnected")
                self._js = None
                self._flags.slave_connected = False