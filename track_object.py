from lib.XRPLib.defaults import *

#-------------------------------------------------------------------------------
# HuskyLens Object Tracking (Yaw + Distance, Multi-Color Ensemble, Smoothed)
#
# Behavior:
#   - Uses HuskyLens over Qwiic (I2C) to find objects with IDs 1, 2, or 3.
#   - Treats these three IDs as the same "color class".
#   - Yaw centering uses the *smoothed* x-center of the largest block.
#   - Distance control uses the *smoothed* sum of areas of all blocks with
#     IDs 1–3 to keep distance roughly constant.
#   - Smoothing: simple moving average over the last 3 frames.
#-------------------------------------------------------------------------------

import sys
import time
import lib.qwiic_i2c as qwiic_i2c
import lib.qwiic_huskylens as qwiic_huskylens

from lib.XRPLib.differential_drive import DifferentialDrive

print("\nXRP HuskyLens Tracking (Yaw + Distance, Multi-Color, Smoothed) Script\n")

# ---------------------------------------------------------------------------
# XRPLib drivetrain setup
# ---------------------------------------------------------------------------

drivetrain = DifferentialDrive.get_default_differential_drive()

# ---------------------------------------------------------------------------
# HuskyLens setup (I2C / Qwiic)
# ---------------------------------------------------------------------------

# XRP Final board: Qwiic0 uses I2C0 with SDA=4, SCL=5
i2c = qwiic_i2c.getI2CDriver(sda=4, scl=5)

my_husky = qwiic_huskylens.QwiicHuskyLens(i2c_driver=i2c)

if not my_husky.connected:
    print("HuskyLens not connected, please check your connection")
    sys.exit()

print("HuskyLens connected!")

# Optionally set algorithm (e.g. Color Recognition) if you want:
# my_husky.set_algorithm(qwiic_huskylens.QwiicHuskyLens.kAlgorithmColorRecognition)

# ---------------------------------------------------------------------------
# Tracking settings
# ---------------------------------------------------------------------------

# HuskyLens image resolution (pixels). Adjust if needed.
HUSKY_WIDTH = 320.0
HUSKY_HEIGHT = 240.0

# IDs of learned colors to track (treated as one "meta-color")
TRACK_IDS = [1, 2, 3]

# Where we want the object to appear in the frame (normalized 0-1)
X_TARGET = 0.5

# Horizontal deadzone around center
X_DEADZONE = 0.05   # +/- around center where we consider it "good"

# Robot yaw control 
YAW_TURN_SPEED    = 5.0  # speed used to rotate left/right
SEARCH_TURN_SPEED = 5.0  # slower spin when searching

# Distance control (based on *total* area of all tracked colors)
AREA_TOLERANCE  = 0.15   # +/-15 percent size band considered "good" distance
FORWARD_SPEED   = 6.0    # creep forward speed
BACKWARD_SPEED  = 6.0    # creep backward speed

# Loop timing
LOOP_DT = 0.1            # seconds between control updates

# Reference area (set when we first see the object ensemble)
desired_area = None

# Smoothing buffers (last up-to-3 frames)
x_history = []
area_history = []
SMOOTH_WINDOW = 3

# ---------------------------------------------------------------------------
# Helper: get tracked blocks (IDs 1, 2, 3) from HuskyLens
# ---------------------------------------------------------------------------

def get_tracked_color_ensemble():
    """
    Query HuskyLens and return:
      - largest_block: block with id in TRACK_IDS with largest area, or None
      - total_area: sum of areas of all blocks with id in TRACK_IDS
    """
    objects = my_husky.get_objects_of_interest()

    largest_block = None
    total_area = 0.0

    for b in objects:
        if b.id in TRACK_IDS:
            area = b.width * b.height
            total_area += area

            if (largest_block is None) or (area > largest_block.width * largest_block.height):
                largest_block = b

    return largest_block, total_area

# ---------------------------------------------------------------------------
# Main tracking loop (yaw + distance, multi-color, smoothed)
# ---------------------------------------------------------------------------

def track_object():
    global desired_area, x_history, area_history

    print("Starting tracking. Waiting a moment...")
    time.sleep(1.0)

    while True:
        block, total_area = get_tracked_color_ensemble()

        if block is None or total_area <= 0:
            # No relevant colors: slowly spin to search
            print("No tracked colors (IDs 1–3) - searching...")
            drivetrain.set_speed(
                left_speed=SEARCH_TURN_SPEED,
                right_speed=-SEARCH_TURN_SPEED,
            )
            # Reset smoothing history so we don't use stale values
            x_history = []
            area_history = []
            time.sleep(LOOP_DT)
            continue

        # Raw measurements for this frame
        x_center_raw = block.xCenter / HUSKY_WIDTH
        area_raw = total_area

        # Update smoothing buffers
        x_history.append(x_center_raw)
        area_history.append(area_raw)

        if len(x_history) > SMOOTH_WINDOW:
            x_history.pop(0)
        if len(area_history) > SMOOTH_WINDOW:
            area_history.pop(0)

        # Smoothed values (moving average)
        x_center = sum(x_history) / len(x_history)
        total_area_smoothed = sum(area_history) / len(area_history)

        # Initialize desired total area on first detection (average over several frames)
        if desired_area is None:
            print("Calibrating desired distance from color ensemble...")
            area_sum = 0.0
            valid_samples = 0

            for _ in range(10):
                b2, tot2 = get_tracked_color_ensemble()
                if b2 is not None and tot2 > 0:
                    area_sum += tot2
                    valid_samples += 1
                time.sleep(0.05)

            if valid_samples > 0:
                desired_area = float(area_sum / valid_samples)
            else:
                desired_area = float(total_area_smoothed)

            print("Set desired total area to {:.1f}".format(desired_area))

        print(
            "Tracking colors {}: x={:.3f}, total_area={:.1f} (desired={:.1f})".format(
                TRACK_IDS, x_center, total_area_smoothed, desired_area
            )
        )

        # ---------------- YAW CONTROL (robot rotation) -----------------
        centered_horizontally = False

        if x_center < (X_TARGET - X_DEADZONE):
            # Ensemble left of center -> rotate left
            drivetrain.set_speed(
                left_speed=-YAW_TURN_SPEED,
                right_speed=YAW_TURN_SPEED,
            )
        elif x_center > (X_TARGET + X_DEADZONE):
            # Ensemble right of center -> rotate right
            drivetrain.set_speed(
                left_speed=YAW_TURN_SPEED,
                right_speed=-YAW_TURN_SPEED,
            )
        else:
            # Horizontally centered
            centered_horizontally = True

        # ---------------- DISTANCE CONTROL (creep fwd/back) ----------- 
        if centered_horizontally:
            ratio = total_area_smoothed / desired_area if desired_area > 0 else 1.0

            # Too far (ensemble smaller than desired): drive forward
            if ratio < (1.0 - AREA_TOLERANCE):
                print("Too far (ratio {:.2f}) -> creeping forward".format(ratio))
                drivetrain.set_speed(
                    left_speed=FORWARD_SPEED,
                    right_speed=FORWARD_SPEED,
                )

            # Too close (ensemble larger than desired): drive backward
            elif ratio > (1.0 + AREA_TOLERANCE):
                print("Too close (ratio {:.2f}) -> backing away".format(ratio))
                drivetrain.set_speed(
                    left_speed=-BACKWARD_SPEED,
                    right_speed=-BACKWARD_SPEED,
                )

            # Within tolerance: stop moving
            else:
                print("Distance OK (ratio {:.2f}) -> stop".format(ratio))
                drivetrain.set_speed(left_speed=0.0, right_speed=0.0)

        time.sleep(LOOP_DT)

# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    try:
        board.wait_for_button()
        track_object()
    except (KeyboardInterrupt, SystemExit):
        print("\nEnding program. Stopping drivetrain.")
        drivetrain.set_speed(left_speed=0.0, right_speed=0.0)
        sys.exit(0)
