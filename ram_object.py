from lib.XRPLib.defaults import *

#-------------------------------------------------------------------------------
# Combined HuskyLens Tag Navigation + "Knock Over Object" (Qwiic-only)
#
# Behavior:
#   - Uses HuskyLens over Qwiic (I2C) for everything.
#   - Finds, centers, and then rams the object of interest, with the goal of 
#     knocking it over
#
# Basket stage:
#   - Uses HuskyLens again to look for:
#       BASKET_ID  -> object to run into and knock over
#-------------------------------------------------------------------------------

import sys
import time
import lib.qwiic_i2c as qwiic_i2c
import lib.qwiic_huskylens as qwiic_huskylens

from lib.XRPLib.defaults import *
from lib.XRPLib.differential_drive import DifferentialDrive

print("\nXRP HuskyLens + Knock-Over Behavior (Qwiic-only) Script\n")

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

actions = [None, 85, -90, 45, -45, 5, 10, 100, 200]

# ---------------------------------------------------------------------------
# Basket "ram" settings
# ---------------------------------------------------------------------------

# HuskyLens image resolution (pixels). Adjust if needed.
HUSKY_WIDTH = 320.0
HUSKY_HEIGHT = 240.0

# ID of learned object/tag to knock over
# Train HuskyLens so that the object you want to hit is BASKET_ID
BASKET_ID = 1

RESTART_WAIT = 1.0              # Small delay before basket logic starts
SEARCH_TURN_SPEED = 10.0        # Drive speed when searching
LINEUP_TURN_SPEED = 13.0        # Drive speed when lining up object
DRIVE_SPEED = 20.0              # Drive speed when not bound by distance
MAX_EFFORT = 0.9                # Drive speed when driving by distance

# Basket alignment (normalized 0-1 coordinates)
BASKET_X_TARGET = 0.4
BASKET_X_DEADZONE = 0.02
BASKET_Y_TARGET = 0.82
BASKET_Y_DEADZONE = 0.02

# How far to "ram" the basket once lined up (cm)
RAM_DISTANCE = 65.0

VICTORY_TURN_DEGREES = 90.0
NUM_VICTORY_TURNS = 2

# Magic constant from original GitHub example
WHEEL_ROT_PER_ROBOT_ROT = 2.42

# State machine states
# 0: Look for the basket
# 1: Drive toward / line up with the basket
# 2: Ram the basket
# 3: Victory dance
# 4: Do nothing
current_state = 0

# ---------------------------------------------------------------------------
# Helper: encoder-based turn (from original basket script)
# ---------------------------------------------------------------------------

def encoder_turn(dt, degrees):
    """
    Hacky encoder-based turn.
    WARNING: not super accurate; uses encoder positions.
    """

    left_start = dt.left_motor.get_position()
    right_start = dt.right_motor.get_position()

    if degrees == 0.0:
        return
    elif degrees > 0.0:
        dt.set_speed(
            left_speed=SEARCH_TURN_SPEED,
            right_speed=-SEARCH_TURN_SPEED
        )
        target_pos = (degrees / 360.0) * WHEEL_ROT_PER_ROBOT_ROT
    else:  # degrees < 0
        dt.set_speed(
            left_speed=-SEARCH_TURN_SPEED,
            right_speed=SEARCH_TURN_SPEED
        )
        target_pos = -(degrees / 360.0) * WHEEL_ROT_PER_ROBOT_ROT

    while True:
        left_pos = dt.left_motor.get_position()
        right_pos = dt.right_motor.get_position()
        if (abs(left_pos - left_start) >= target_pos) or \
           (abs(right_pos - right_start) >= target_pos):
            break

    dt.set_speed(left_speed=0.0, right_speed=0.0)

# ---------------------------------------------------------------------------
# Helper: get basket block from HuskyLens
# ---------------------------------------------------------------------------

def get_basket_block():
    """
    Query HuskyLens, return:
      - basket_block: block with id == BASKET_ID (or None)
    """

    objects = my_husky.get_objects_of_interest()

    basket_block = None

    for b in objects:
        if b.id == BASKET_ID:
            # If multiple, choose the one with largest area
            if basket_block is None or (b.width * b.height >
                                        basket_block.width * basket_block.height):
                basket_block = b

    return basket_block

# ---------------------------------------------------------------------------
# "Knock over the basket" state machine (HuskyLens-based)
# ---------------------------------------------------------------------------

def ram_object():
    """
    Behavior:
      - Look for object with ID BASKET_ID.
      - Rotate in place to horizontally center it (no forward motion).
      - Once centered, drive straight forward at max effort to ram / knock it over.
    Triggered when a tag with id > 8 is seen in the main navigation loop.
    """

    global current_state
    current_state = 1
    align_counter = 0  # counts how many frames in a row we've been centered

    # Initial setup
    drivetrain.set_speed(left_speed=0.0, right_speed=0.0)

    print("Starting basket knock-over sequence.")
    time.sleep(RESTART_WAIT)
    print("Basket logic running...")

    while True:
        basket_block = get_basket_block()

        # ---------------- State 0: look for the basket ----------------
        if current_state == 0:
            if basket_block is not None:
                print("Basket found")
                drivetrain.set_speed(left_speed=0.0, right_speed=0.0)
                current_state = 1
                continue

            # If no basket, rotate in place
            drivetrain.set_speed(
                left_speed=SEARCH_TURN_SPEED,
                right_speed=-SEARCH_TURN_SPEED,
            )

        # ---------------- State 1: line up (rotate only) ---------------
        elif current_state == 1:

            if basket_block is None:
                print("Basket lost. Searching...")
                align_counter = 0
                current_state = 0
                continue

            # Convert to normalized coordinates
            x_center = basket_block.xCenter / HUSKY_WIDTH
            y_center = basket_block.yCenter / HUSKY_HEIGHT  # not used for motion, just debug

            print("Lining up basket: ({:.3f}, {:.3f})".format(x_center, y_center))

            # Line up basket horizontally ONLY (no forward/back movement here)
            if x_center < (BASKET_X_TARGET - BASKET_X_DEADZONE):
                # Basket is to the left -> rotate left
                drivetrain.set_speed(
                    left_speed=-LINEUP_TURN_SPEED,
                    right_speed=LINEUP_TURN_SPEED,
                )
                align_counter = 0  # reset since we're not centered
            elif x_center > (BASKET_X_TARGET + BASKET_X_DEADZONE):
                # Basket is to the right -> rotate right
                drivetrain.set_speed(
                    left_speed=LINEUP_TURN_SPEED,
                    right_speed=-LINEUP_TURN_SPEED,
                )
                align_counter = 0
            else:
                # Horizontally centered: stop and count how stable this is
                drivetrain.set_speed(left_speed=0.0, right_speed=0.0)
                align_counter += 1
                print("Basket horizontally centered ({} / 5)".format(align_counter))

                # Once we've seen it centered for a few frames, move on to ramming
                if align_counter >= 6:
                    print("Basket centered. Preparing to ram...")
                    time.sleep(0.6)
                    current_state = 2
                    continue

        # ---------------- State 2: ram the basket ---------------------
        elif current_state == 2:
            print("Ramming the basket...")

            # Drive straight forward at max effort for RAM_DISTANCE
            drivetrain.straight(
                distance=RAM_DISTANCE,
                max_effort=MAX_EFFORT,
            )

            print("Impact complete (hopefully knocked over).")
            current_state = 3
            continue
        else:
            drivetrain.set_speed(left_speed=0.0, right_speed=0.0)
            print("All done, please reset me")
            while True:
                time.sleep(5.0)

# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    try:
        board.wait_for_button()
        ram_object()
    except (KeyboardInterrupt, SystemExit):
        print("\nEnding program. Stopping drivetrain.")
        drivetrain.set_speed(left_speed=0.0, right_speed=0.0)
        sys.exit(0)
