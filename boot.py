from PIL import Image
import time


def boot_screen(disp, img, draw, WIDTH, HEIGHT, font_mid, font_small, flags):
    """
    Shows 3 progress blocks that fill as real flags become True.
    Block 1 (PWR)  → always on
    Block 2 (LINK) → flags.mavlink_connected  (master RC via MAVLink)
    Block 3 (CTRL) → flags.slave_connected    (TX12 USB joystick plugged in)
    Returns when all 3 are filled.
    """

    def fix_color(image):
        r, g, b = image.split()
        return Image.merge("RGB", (b, g, r))

    # ── clear screen ──────────────────────────────────────────────────────
    draw.rectangle((0, 0, WIDTH, HEIGHT), fill=(0, 0, 0))

    # ── logo ──────────────────────────────────────────────────────────────
    try:
        logo = Image.open("Y.png")
        logo = logo.resize((70, 70))
        x = (WIDTH - logo.width) // 2
        img.paste(logo, (x, 30))
    except Exception:
        pass

    disp.image(fix_color(img))

    # ── block layout ──────────────────────────────────────────────────────
    BLOCK_W = 10
    BLOCK_H = 10       # square blocks
    GAP     = 8

    total_w = BLOCK_W * 3 + GAP * 2
    start_x = (WIDTH - total_w) // 2
    block_y = 120

    COLOR_FILL    = (0, 255, 120)
    COLOR_OUTLINE = (255, 255, 255)

    # ── draw all 3 empty square blocks (no labels) ────────────────────────
    for i in range(3):
        x = start_x + i * (BLOCK_W + GAP)
        draw.rectangle((x, block_y, x + BLOCK_W, block_y + BLOCK_H),
                        outline=COLOR_OUTLINE)

    disp.image(fix_color(img))

    # ── block fill state ─────────────────────────────────────────────────
    filled = [False, False, False]

    def fill_block(i):
        if filled[i]:
            return
        filled[i] = True
        x = start_x + i * (BLOCK_W + GAP)
        draw.rectangle((x, block_y, x + BLOCK_W, block_y + BLOCK_H),
                        fill=COLOR_FILL)
        disp.image(fix_color(img))

    # Block 1 — power always on
    fill_block(0)

    while not all(filled):
        flags.wait(timeout=0.2)

        if filled[0] and flags.mavlink_connected:
            fill_block(1)

        if filled[1] and flags.slave_connected:
            fill_block(2)

    time.sleep(0.5)   # brief pause so user sees 100%

def reconnect_screen(disp, img, draw, WIDTH, HEIGHT, flags, device_name):
    """
    Full-screen reconnect display. Blocks until the disconnected device
    reconnects, then returns.

    device_name : "master" | "slave"
      "master" -> waits for flags.mavlink_connected
      "slave"  -> waits for flags.slave_connected
    """
    import math
    from PIL import ImageFont

    def fix_color(image):
        r, g, b = image.split()
        return Image.merge("RGB", (b, g, r))

    # -- colour scheme (BGR display) ---------------------------------------
    BG        = (0,   0,   0)
    RED_BGR   = (0,   0,   255)
    ORANGE    = (255, 100, 65)
    WHITE     = (255, 255, 255)
    DIM       = (80,  80,  80)

    # -- layout -----------------------------------------------------------
    CX         = WIDTH  // 2
    ICON_Y     = 38
    LABEL_Y    = 70
    DEVICE_Y   = 83
    STATUS_Y   = 100
    SPINNER_CY = 135
    SPINNER_R  = 16
    ARC_SPAN   = 270

    device_label = "TRAINER" if device_name == "master" else "TRAINEE"

    # -- fonts ------------------------------------------------------------
    BASE        = "/usr/share/fonts/truetype/dejavu/"
    font_bold   = ImageFont.truetype(BASE + "DejaVuSans-Bold.ttf", 11)
    font_small  = ImageFont.truetype(BASE + "DejaVuSans.ttf",       9)
    font_exclam = ImageFont.truetype(BASE + "DejaVuSans-Bold.ttf",  14)

    # -- static background (drawn once) -----------------------------------
    draw.rectangle((0, 0, WIDTH, HEIGHT), fill=BG)

    # warning triangle
    draw.polygon([(CX, ICON_Y - 14), (CX - 14, ICON_Y + 10),
                  (CX + 14, ICON_Y + 10)], outline=ORANGE)
    ew = draw.textlength("!", font=font_exclam)
    draw.text((CX - ew // 2, ICON_Y - 8), "!", font=font_exclam, fill=ORANGE)

    # "TRAINER / TRAINEE"
    lw = draw.textlength(device_label, font=font_bold)
    draw.text((CX - lw // 2, LABEL_Y), device_label,
              font=font_bold, fill=RED_BGR)

    # "DISCONNECTED"
    dw = draw.textlength("DISCONNECTED", font=font_bold)
    draw.text((CX - dw // 2, DEVICE_Y), "DISCONNECTED",
              font=font_bold, fill=WHITE)

    # "RECONNECTING..."
    rw = draw.textlength("RECONNECTING...", font=font_small)
    draw.text((CX - rw // 2, STATUS_Y), "RECONNECTING...",
              font=font_small, fill= WHITE)

    # spinner track (dim ring)
    sx0 = CX - SPINNER_R
    sy0 = SPINNER_CY - SPINNER_R
    sx1 = CX + SPINNER_R
    sy1 = SPINNER_CY + SPINNER_R
    draw.ellipse((sx0, sy0, sx1, sy1), outline=(40, 40, 40))

    disp.image(fix_color(img))

    # -- spinner loop -----------------------------------------------------
    angle = 0
    STEP  = 30     # degrees per frame
    DELAY = 0.08   # ~12 fps

    while True:
        if device_name == "master" and flags.mavlink_connected:
            break
        if device_name == "slave"  and flags.slave_connected:
            break

        # erase old arc
        draw.ellipse((sx0, sy0, sx1, sy1), outline=(40, 40, 40))

        # draw spinning arc (handle 360 wrap)
        start_a = angle % 360
        end_a   = (angle + ARC_SPAN) % 360
        if start_a < end_a:
            draw.arc((sx0, sy0, sx1, sy1),
                     start=start_a, end=end_a, fill=ORANGE, width=3)
        else:
            draw.arc((sx0, sy0, sx1, sy1),
                     start=start_a, end=360, fill=ORANGE, width=3)
            draw.arc((sx0, sy0, sx1, sy1),
                     start=0, end=end_a, fill=ORANGE, width=3)

        disp.image(fix_color(img))
        angle = (angle + STEP) % 360
        time.sleep(DELAY)