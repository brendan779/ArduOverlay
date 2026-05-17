# ─────────────────────────────────────────────
#  ArduPilot OSD Config  — edit this file only
# ─────────────────────────────────────────────

# ── Output video ─────────────────────────────
OUTPUT_WIDTH   = 1920          # pixels (match your source footage)
OUTPUT_HEIGHT  = 1080
OUTPUT_FPS     = 30
OUTPUT_FILE    = "osd_overlay.mov"   # ProRes 4444 (transparent) .mov

# ── Timing ───────────────────────────────────
# Offset in seconds to shift telemetry vs video start
# Positive = telemetry starts this many seconds AFTER video
# Negative = telemetry starts this many seconds BEFORE video
TIME_OFFSET_SECONDS = 0.0

# ── Fields to display ────────────────────────
# Comment out any line to hide that field
ENABLED_FIELDS = [
    "speed",        # ground speed (m/s → converted to km/h)
    "altitude",     # relative altitude (m)
    "attitude",     # pitch / roll / yaw strip
    "flight_mode",  # ArduPilot flight mode name
    "messages",     # STATUSTEXT messages
]

# ── Lower-third layout ───────────────────────
LOWER_THIRD_HEIGHT_FRACTION = 0.18   # fraction of frame height
LOWER_THIRD_MARGIN_PX       = 48     # left/right/bottom margin

# ── Colours (R, G, B, A) — all 0.0–1.0 ──────
COLOR_BG          = (0.05, 0.05, 0.05, 0.70)   # panel background
COLOR_LABEL       = (0.65, 0.65, 0.65, 1.0)    # dim label text
COLOR_VALUE       = (1.0,  1.0,  1.0,  1.0)    # bright value text
COLOR_ACCENT      = (0.25, 0.72, 0.55, 1.0)    # teal accent line / highlights
COLOR_MODE_BG     = (0.25, 0.72, 0.55, 0.85)   # flight mode pill background
COLOR_MODE_TEXT   = (0.05, 0.05, 0.05, 1.0)    # flight mode pill text
COLOR_MESSAGE     = (1.0,  0.85, 0.35, 1.0)    # status message text (amber)
COLOR_WARN        = (1.0,  0.4,  0.1,  1.0)    # warning / error message

# ── Typography ───────────────────────────────
FONT_LABEL_SIZE  = 11    # pt
FONT_VALUE_SIZE  = 22    # pt
FONT_MODE_SIZE   = 13    # pt
FONT_MSG_SIZE    = 12    # pt

# ── Attitude bar ─────────────────────────────
ATTITUDE_BAR_THICKNESS = 3      # px
PITCH_RANGE_DEG        = 45     # full-scale deflection each way
ROLL_RANGE_DEG         = 60

# ── Messages ─────────────────────────────────
MESSAGE_DISPLAY_SECONDS = 4.0   # how long each message stays visible
MESSAGE_MAX_CHARS       = 72    # truncate longer messages

# ── Speed units ──────────────────────────────
# "kmh" | "mph" | "ms"
SPEED_UNITS = "kmh"

# ── Altitude datum ───────────────────────────
# "relative"  = above home (ArduPilot default)
# "absolute"  = AMSL (uses GPS altitude)
ALTITUDE_DATUM = "relative"
