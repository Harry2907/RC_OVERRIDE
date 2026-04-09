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

    Disconnect detection: pygame.event.pump() does NOT raise when a joystick
    is unplugged — it silently continues. The only reliable method is to call
    pygame.joystick.quit() + pygame.joystick.init() and recheck get_count()
    each poll cycle.

    FIX: We skip the joystick subsystem reinit while rc10_active is True.
    RCOverrideThread is actively reading axes at that moment — calling
    pygame.joystick.quit() mid-read is what caused the "Joystick not
    initialized" race condition and the resulting channel snap-back.
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

            # ── joystick present — re-init subsystem and recount ──────────
            # Skip reinit while RC override is actively reading axes.
            # pygame.joystick.quit() during an active read causes the
            # "Joystick not initialized" exception in RCOverrideThread.
            if self._flags.rc10_active:
                time.sleep(self.POLL)
                continue

            pygame.event.pump()
            pygame.joystick.quit()
            pygame.joystick.init()

            if pygame.joystick.get_count() == 0:
                print("[JS THREAD] Disconnected")
                self._js = None
                self._flags.slave_connected = False

            time.sleep(self.POLL)