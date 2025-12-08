from lib.XRPLib.defaults import *

#-------------------------------------------------------------------------------
# HuskyLens Object Tracking (Yaw + Distance)
#
# Behavior:
#   - Uses HuskyLens over Qwiic (I2C) to find a specific object (TRACK_ID).
#   - Rotates the robot (yaw) to keep the object centered left/right.
#   - Uses the apparent size (area) of the object to keep distance roughly
#     constant: drives forward if object shrinks, backwards if it grows.
#-------------------------------------------------------------------------------

import sys
import time
import lib.qwiic_i2c as qwiic_i2c
import lib.qwiic_huskylens as qwiic_huskylens

from lib.XRPLib.differential_drive import DifferentialDrive

print("\nXRP HuskyLens Tracking (Yaw + Distance) Script\n")

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

# Optionally set algorithm (e.g. Tag Recognition) if you want:
# my_husky.set_algorithm(qwiic_huskylens.QwiicHuskyLens.kAlgorithmTagRecognition)

# ---------------------------------------------------------------------------
# Tracking settings
# ---------------------------------------------------------------------------

# HuskyLens image resolution (pixels). Adjust if needed.
HUSKY_WIDTH = 320.0
HUSKY_HEIGHT = 240.0

# ID of learned object/tag to track
TRACK_ID = 1

# Where we want the object to appear in the frame (normalized 0-1)
X_TARGET = 0.5

# Horizontal deadzone around center
X_DEADZONE = 0.05   # +/- around center where we consider it "good"

# Robot yaw control (open-loop speeds)
YAW_TURN_SPEED = 8.0        # speed used to rotate left/right
SEARCH_TURN_SPEED = 5.0     # slower spin when searching

# Distance control (based on area)
AREA_TOLERANCE = 0.15       # +/-15 percent size band considered "good" distance
FORWARD_SPEED = 6.0         # creep forward speed
BACKWARD_SPEED = 6.0        # creep backward speed

# Loop timing
LOOP_DT = 0.1               # seconds between control updates

# Reference area (set when we first see the object)
desired_area = None

# ---------------------------------------------------------------------------
# Helper: get tracked block from HuskyLens
# ---------------------------------------------------------------------------

def get_tracked_block():
    """
    Query HuskyLens, return:
      - tracked_block: block with id == TRACK_ID (or None)
    """
    objects = my_husky.get_objects_of_interest()

    tracked_block = None

    for b in objects:
        if b.id == TRACK_ID:
            # If multiple, choose the one with largest area
            if tracked_block is None or (b.width * b.height >
                                         tracked_block.width * tracked_block.height):
                tracked_block = b

    return tracked_block

# ---------------------------------------------------------------------------
# Main tracking loop (yaw + distance)
# ---------------------------------------------------------------------------

def track_object():
    global desired_area

    print("Starting tracking. Waiting a moment...")
    time.sleep(1.0)

    while True:
        block = get_tracked_block()

        if block is None:
            # No object: slowly spin to search
            print("No tracked object - searching...")
            drivetrain.set_speed(
                left_speed=SEARCH_TURN_SPEED,
                right_speed=-SEARCH_TURN_SPEED,
            )
            time.sleep(LOOP_DT)
            continue

        # Compute center and area
        x_center = block.xCenter / HUSKY_WIDTH
        area = block.width * block.height

        # Initialize desired area on first detection
        if desired_area is None:
            desired_area = float(area)
            print("Set desired area to {:.1f}".format(desired_area))

        print(
            "Tracking object {} at x={:.3f}, area={:.1f} (desired={:.1f})".format(
                TRACK_ID, x_center, area, desired_area
            )
        )

        # ---------------- YAW CONTROL (robot rotation) -----------------
        centered_horizontally = False

        if x_center < (X_TARGET - X_DEADZONE):
            # Object left of center -> rotate left
            drivetrain.set_speed(
                left_speed=-YAW_TURN_SPEED,
                right_speed=YAW_TURN_SPEED,
            )
        elif x_center > (X_TARGET + X_DEADZONE):
            # Object right of center -> rotate right
            drivetrain.set_speed(
                left_speed=YAW_TURN_SPEED,
                right_speed=-YAW_TURN_SPEED,
            )
        else:
            # Horizontally centered
            centered_horizontally = True

        # ---------------- DISTANCE CONTROL (creep fwd/back) -----------
        # Only adjust distance once we're horizontally centered, so yaw and
        # distance commands do not fight each other.
        if centered_horizontally:
            # Compare current area to desired area
            ratio = area / desired_area if desired_area > 0 else 1.0

            # Too far (object smaller than desired): drive forward
            if ratio < (1.0 - AREA_TOLERANCE):
                print("Too far (ratio {:.2f}) -> creeping forward".format(ratio))
                drivetrain.set_speed(
                    left_speed=FORWARD_SPEED,
                    right_speed=FORWARD_SPEED,
                )

            # Too close (object larger than desired): drive backward
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
