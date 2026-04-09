import time
import threading
import os

os.environ["SDL_VIDEODRIVER"] = "dummy"
import pygame

# ── Axis mapping for TX12 USB joystick ────────────────────────────────────────
# Adjust these indices if your TX12 reports axes in a different order.
# To find yours: run `jstest /dev/input/js0` or print axis values in debug mode.
AXIS_ROLL     = 0   # Right stick X  → CH1
AXIS_PITCH    = 1   # Right stick Y  → CH2
AXIS_THROTTLE = 2   # Left  stick Y  → CH3
AXIS_YAW      = 3   # Left  stick X  → CH4

# How often to send override packets (seconds)
SEND_RATE = 0.05   # 20 Hz

# PWM range
PWM_MIN  = 1000
PWM_MID  = 1500
PWM_MAX  = 2000

# Channels we do NOT override — FC keeps its own value
PASSTHROUGH = 65535


def axis_to_pwm(value, invert=False):
    """
    Convert pygame axis (-1.0 … +1.0) → PWM (1000 … 2000).
    Throttle: -1.0 (stick down) → 1000, +1.0 (stick up) → 2000
    Others:   centred = 1500
    """
    if invert:
        value = -value
    return int(PWM_MID + value * 500)


class RCOverrideThread:
    """
    When flags.rc10_active is True:
        - Reads slave TX12 joystick axes via pygame
        - Sends RC_CHANNELS_OVERRIDE to the flight controller at 20 Hz

    When flags.rc10_active is False:
        - Stops sending overrides immediately
        - FC reverts to master RC input automatically
    """

    def __init__(self, flags):
        self._flags  = flags
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._js     = None

    def start(self):
        self._thread.start()

    # ── get joystick handle (already initialised by JoystickFlagThread) ──────
    def _get_joystick(self):
        pygame.joystick.quit()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            return None
        js = pygame.joystick.Joystick(0)
        js.init()
        return js

    # ── read axes and build 8-channel PWM list ────────────────────────────────
    def _read_channels(self):
        pygame.event.pump()

        num_axes = self._js.get_numaxes()

        def safe_axis(idx):
            return self._js.get_axis(idx) if idx < num_axes else 0.0

        roll     = axis_to_pwm(safe_axis(AXIS_ROLL))
        pitch    = axis_to_pwm(safe_axis(AXIS_PITCH),    invert=True)  # pygame Y is inverted
        throttle = axis_to_pwm(safe_axis(AXIS_THROTTLE), invert=True)  # stick up = more throttle
        yaw      = axis_to_pwm(safe_axis(AXIS_YAW))

        # CH1=Roll, CH2=Pitch, CH3=Throttle, CH4=Yaw, CH5-CH8=passthrough
        return [roll, pitch, throttle, yaw,
                PASSTHROUGH, PASSTHROUGH, PASSTHROUGH, PASSTHROUGH]

    # ── send override packet ──────────────────────────────────────────────────
    def _send_override(self, channels):
        master = self._flags.mavlink_master
        if master is None:
            return
        master.mav.rc_channels_override_send(
            master.target_system,
            master.target_component,
            *channels        # CH1 … CH8
        )

    # ── clear override — lets FC fall back to master RC ───────────────────────
    def _clear_override(self):
        master = self._flags.mavlink_master
        if master is None:
            return
        # Sending 0 on all channels releases the override
        master.mav.rc_channels_override_send(
            master.target_system,
            master.target_component,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        print("[RC OVERRIDE] Released — master RC restored")

    # ── main loop ─────────────────────────────────────────────────────────────
    def _loop(self):
        was_active = False

        while self._flags.running:

            active = self._flags.rc10_active

            # ── RC10 just released → clear override once ──────────────────
            if was_active and not active:
                self._clear_override()
                self._js = None
                was_active = False
                continue

            # ── RC10 not active → idle ─────────────────────────────────────
            if not active:
                time.sleep(0.05)
                continue

            # ── RC10 active → acquire joystick if needed ───────────────────
            if self._js is None:
                if not self._flags.slave_connected:
                    print("[RC OVERRIDE] RC10 active but slave not connected — waiting")
                    time.sleep(0.1)
                    continue
                self._js = self._get_joystick()
                if self._js is None:
                    time.sleep(0.1)
                    continue
                print("[RC OVERRIDE] Slave joystick acquired — sending overrides")

            # ── send override packet ───────────────────────────────────────
            try:
                channels = self._read_channels()
                self._send_override(channels)
                was_active = True
            except Exception as e:
                print(f"[RC OVERRIDE] Error reading joystick: {e}")
                self._js = None

            time.sleep(SEND_RATE)
