import time
import board
import digitalio
from PIL import Image, ImageDraw, ImageFont
from adafruit_rgb_display import st7735


class DroneDisplay:
    def __init__(self):
        # ── Fonts ──────────────────────────────────────────────────────────
        BASE = "/usr/share/fonts/truetype/dejavu/"
        self.font_title  = ImageFont.truetype(BASE + "DejaVuSans-Bold.ttf",  16)
        self.font_label  = ImageFont.truetype(BASE + "DejaVuSans-Bold.ttf",  10)
        self.font_value  = ImageFont.truetype(BASE + "DejaVuSans.ttf",       11)
        self.Statefont_value  = ImageFont.truetype(BASE + "DejaVuSans.ttf", 9)
        self.font_status = ImageFont.truetype(BASE + "DejaVuSans.ttf",       12)
        self.font_mode   = ImageFont.truetype(BASE + "DejaVuSans-Bold.ttf",  8)

        # ── SPI / Display ──────────────────────────────────────────────────
        spi = board.SPI()
        cs  = digitalio.DigitalInOut(board.CE0)
        dc  = digitalio.DigitalInOut(board.D25)
        rst = digitalio.DigitalInOut(board.D24)

        self.display = st7735.ST7735R(
            spi, cs=cs, dc=dc, rst=rst,
            width=128, height=160,
            rotation=0, baudrate=24_000_000,
        )

        self.W, self.H = 128, 160

        # ── Palette ───────────────────────────────────────────────────────
        self.BG     = (25, 5, 15)
        self.BORDER = (100, 70, 30)
        self.TEXT   = (220, 230, 240)
        self.YELLOW = (0, 255, 255)
        self.CYAN   = (255, 200, 0)
        self.GREEN  = (0, 255, 120)
        self.RED    = (0, 0, 255)
        self.ORANGE = (65, 100, 255)
        self.WHITE  = (255, 255, 255)
        self.DIM    = (80, 80, 80)

        # ── Persistent image buffer (never re-allocate) ────────────────────
        self.image = Image.new("RGB", (self.W, self.H))
        self.draw  = ImageDraw.Draw(self.image)

        # ── State cache – only redraw when something changed ───────────────
        self._prev = {}

        # ── Live values ───────────────────────────────────────────────────
        self.rssi       = -1          # kept in data pipeline, not displayed
        self.mode       = "N/A"
        self.gps_fix    = False
        self.altitude   = 0.0
        self.in_flight  = False
        self.status_msg = "N/A"
        self.slave_mode = False       # True when RC10 active → header shows TRAINEE

    # ── Public update ──────────────────────────────────────────────────────
    def update_data(self, mode, gps_fix, altitude, rssi, in_flight, status_msg, slave_mode=False):
        self.mode       = mode
        self.gps_fix    = gps_fix
        self.altitude   = altitude
        self.rssi       = rssi          # kept, not displayed
        self.in_flight  = in_flight
        self.status_msg = status_msg
        self.slave_mode = slave_mode

    def _state_changed(self):
        """Return True when any displayed value has changed since last render."""
        cur = (self.mode, self.gps_fix,
               round(self.altitude, 1), self.in_flight, self.status_msg, self.slave_mode)
        if cur != self._prev.get("key"):
            self._prev["key"] = cur
            return True
        return False

    # ── Icon helpers ───────────────────────────────────────────────────────
    def _gps_icon(self, cx, cy, color, size=10):
        """Satellite-pin style GPS icon, centred on (cx, cy)."""
        d = self.draw
        r = size // 2.5
        d.ellipse((cx - r, cy - r, cx + r, cy + r), outline=color, width=2)
        d.ellipse((cx - 2, cy - 2, cx + 2, cy + 2), fill=color)
        d.line((cx, cy - r - 3, cx, cy - r + 1), fill=color, width=1)
        d.line((cx, cy + r - 1, cx, cy + r + 3), fill=color, width=1)
        d.line((cx - r - 3, cy, cx - r + 1, cy), fill=color, width=1)
        d.line((cx + r - 1, cy, cx + r + 3, cy), fill=color, width=1)

    def _mode_icon(self, x, y, color, size=2):
        """Drone top-view icon — 4 arms at 45° with rotor circles."""
        import math
        d      = self.draw
        cx     = x + size // 2
        cy     = y + size // 2
        arm    = size // 1.5
        rotor_r = max(2, size // 4)
        for angle in [45, 135, 225, 315]:
            rad = math.radians(angle)
            ex  = int(cx + arm * math.cos(rad))
            ey  = int(cy + arm * math.sin(rad))
            d.line((cx, cy, ex, ey), fill=color, width=1)
            d.ellipse((ex - rotor_r, ey - rotor_r,
                       ex + rotor_r, ey + rotor_r),
                      outline=color, width=1)
        d.ellipse((cx - 2, cy - 2, cx + 2, cy + 2), fill=color)

    def _horizon_icon(self, cx, cy, color, size=12):
        """Horizon icon: circle above a horizontal baseline with end ticks."""
        d = self.draw
        r      = size // 4
        line_y = cy + r + 2
        half   = size // 2
        # horizon line
        d.line((cx - half, line_y, cx + half, line_y), fill=color, width=2)
        # end ticks
        d.line((cx - half, line_y, cx - half, line_y + 3), fill=color, width=2)
        d.line((cx + half, line_y, cx + half, line_y + 3), fill=color, width=2)
        # circle — above line when in flight, filled on line when on ground
        circle_y = cy - r - 1
        d.ellipse((cx - r, circle_y - r,
                   cx + r, circle_y + r),
                  outline=color, width=2)

    def _alt_icon(self, x, y, color, size=10):
        """Altitude icon — vertical arrow with ground baseline."""
        d      = self.draw
        cx     = x + size // 2
        tip    = y
        mid    = y + size // 2
        bot    = y + size
        head_w = size // 2
        d.polygon([
            (cx,          tip),
            (cx - head_w, mid),
            (cx + head_w, mid),
        ], fill=color)
        shaft_w = max(1, size // 6)
        d.rectangle([
            (cx - shaft_w, mid),
            (cx + shaft_w, bot - 2),
        ], fill=color)
        d.line((x, bot, x + size, bot), fill=color, width=2)

    # ── Main render ────────────────────────────────────────────────────────
    def render(self):
        if not self._state_changed():
            return
        self._render_frame()

    def _render_frame(self):
        d = self.draw
        W, H = self.W, self.H

        TOP_H    = 20
        ROW1_H   = 36
        ROW2_H   = 30
        STATUS_Y = TOP_H + ROW1_H + ROW2_H

        d.rectangle((0, 0, W, H), fill=self.BG)

        # ── TOP BAR ──────────────────────────────────────────────────────
        d.rectangle((0, 0, W - 1, TOP_H - 1), fill=(10, 18, 35))
        title = "TRAINEE" if self.slave_mode else "TRAINER"
        title_color = self.GREEN if self.slave_mode else self.YELLOW
        tw = d.textlength(title, font=self.font_title)
        d.text(((W - tw) // 2, 2), title, font=self.font_title, fill=title_color)
        d.line((0, TOP_H - 1, W, TOP_H - 1), fill=self.BORDER)

        # ── ROW 1 — GPS | MODE ───────────────────────────────────────────
        r1_top = TOP_H
        r1_bot = TOP_H + ROW1_H - 1
        mid_x  = W // 2

        d.rectangle((0, r1_top, mid_x - 2, r1_bot), outline=self.BORDER)
        d.rectangle((mid_x + 1, r1_top, W - 1, r1_bot), outline=self.BORDER)

        gps_color = self.GREEN if self.gps_fix else self.RED
        gps_label = "LOCK" if self.gps_fix else " NO FIX"

        icon_cx = 11
        icon_cy = r1_top + ROW1_H // 2
        self._gps_icon(icon_cx, icon_cy, gps_color, size=12)

        lw = d.textlength(gps_label, font=self.font_label)
        d.text(((mid_x - 2 - lw) // 2 + 7, icon_cy - 5),
               gps_label, font=self.font_label, fill=gps_color)

        mx = mid_x + 6
        my = r1_top + 13
        self._mode_icon(mx, my + 2, self.CYAN, size=7)

        mode_text = self.mode if len(self.mode) <= 9 else self.mode[:9]
        d.text((mx + 11, my +1), mode_text,
               font=self.font_mode, fill=self.YELLOW)

        # ── ROW 2 — STATE | ALT ───────────────────────────────────────────
        r2_top = TOP_H + ROW1_H
        r2_bot = r2_top + ROW2_H - 1

        d.rectangle((0, r2_top, mid_x - 2, r2_bot), outline=self.BORDER)
        d.rectangle((mid_x + 1, r2_top, W - 1, r2_bot), outline=self.BORDER)

        if self.in_flight:
            state_color = self.GREEN
            state_str   = "IN FLIGHT"
        else:
            state_color = self.DIM
            state_str   = "ON GROUND"

        hi_cx = 10
        hi_cy = r2_top + ROW2_H // 2 
        self._horizon_icon(hi_cx, hi_cy, state_color, size=9)
        line_a, line_b = state_str.split()
        tx = hi_cx + 9
        ty = r2_top + ROW2_H // 2 - 8
        d.text((tx, ty),     line_a, font=self.Statefont_value, fill=state_color)
        d.text((tx, ty + 8), line_b, font=self.Statefont_value, fill=state_color)

        ai_x = mid_x + 6
        ai_y = r2_top + ROW2_H // 2 - 6
        self._alt_icon(ai_x, ai_y, self.ORANGE, size=10)
        alt_str = f"{self.altitude:.1f}m"
        d.text((ai_x + 15, ai_y), alt_str,
               font=self.font_value, fill=self.YELLOW)

        # ── STATUS PANEL ─────────────────────────────────────────────────
        d.rectangle((0, STATUS_Y, W - 1, H - 1), outline=self.BORDER)
        d.line((0, STATUS_Y, W, STATUS_Y), fill=self.BORDER, width=1)

        hdr = "STATUS"
        hw  = d.textlength(hdr, font=self.font_label)
        d.text(((W - hw) // 2, STATUS_Y + 3),
               hdr, font=self.font_label, fill=self.TEXT)
        d.line((4, STATUS_Y + 15, W - 4, STATUS_Y + 15),
               fill=self.BORDER, width=1)

        # ---------------- STATUS COLOR LOGIC ----------------
        msg = self.status_msg.strip()

        if msg == "READY TO ARM" or msg == "ARMED":
            msg_color = self.GREEN

        elif msg == "INITIALIZING...":
            msg_color = self.YELLOW
        
        else:
            msg_color = self.RED   

        words = msg.split()
        line1 = ""
        line2 = ""
        for w in words:
            if len(line1) + len(w) + 1 <= 19:
                line1 = (line1 + " " + w).strip()
            else:
                line2 = (line2 + " " + w).strip()

        d.text((6, STATUS_Y + 19), line1,
               font=self.font_status, fill=msg_color)
        if line2:
            d.text((6, STATUS_Y + 33), line2,
                   font=self.font_status, fill=msg_color)

        self.display.image(self.image)


# ── Standalone test (no MAVLink) ──────────────────────────────────────────────
# if __name__ == "__main__":
#     disp = DroneDisplay()
#     scenarios = [
#         ("LOITER",    True,  12.4, 72, "IN_FLIGHT", "READY TO ARM"),
#         ("STABILIZE", False,  0.0, 35, "ON_GROUND", "PreArm: RC not calibrated"),
#         ("AUTO",      False,  45.0, 25, "ON_GROUND", "OK"),
#         ("LAND",      True,   3.1, 15, "IN_FLIGHT", "LOW BATTERY – landing now"),
#     ]
#     for args in scenarios:
#         disp.update_data(*args)
#         disp.render()
#         time.sleep(1)