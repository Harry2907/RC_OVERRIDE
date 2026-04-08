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

    # ── poll until all 3 filled ───────────────────────────────────────────
    while not all(filled):
        flags.wait(timeout=0.2)

        if flags.mavlink_connected:
            fill_block(1)

        if flags.slave_connected:          # plug-in only, no stick movement needed
            fill_block(2)

    time.sleep(0.5)   # brief pause so user sees 100%