# ---------------------------------------------------------------------------
# HuskyLens setup (I2C / Qwiic)
# ---------------------------------------------------------------------------
import sys
import time
import lib.qwiic_i2c as qwiic_i2c
import lib.qwiic_huskylens as qwiic_huskylens

from lib.XRPLib.defaults import *
from lib.XRPLib.differential_drive import DifferentialDrive

drivetrain = DifferentialDrive.get_default_differential_drive()

# XRP Final board: Qwiic0 uses I2C0 with SDA=4, SCL=5
i2c = qwiic_i2c.getI2CDriver(sda=4, scl=5)

my_husky = qwiic_huskylens.QwiicHuskyLens(i2c_driver=i2c)

if not my_husky.connected:
    print("HuskyLens not connected, please check your connection")
    sys.exit()

print("HuskyLens connected!")

print("\nXRP HuskyLens + Knock-Over Behavior (Qwiic-only) Script\n")

# ---------------------------------------------------------------------------
# Tag-based navigation actions (your original behavior)
#   1-4: angles to turn
#   5-8: distances to drive (cm)
# ---------------------------------------------------------------------------
actions = [None, 82, -82, 45, -45, 5, 10, 100, 200]

# available variables from defaults: left_motor, right_motor, drivetrain,
#      imu, rangefinder, reflectance, servo_one, board, webserver
# Write your code Here
def run():
    print("\nStarting HuskyLens tag navigation...\n")

    if not my_husky.connected:
        print("Device not connected, please check your connection")
        sys.exit()

    # Initialize HuskyLens
    if my_husky.begin() == False:
        print("Failed to initialize HuskyLens. Check connection.", file=sys.stderr)
        return

    nScans = 0

    while True:
        time.sleep(0.2)

        objects = my_husky.get_objects_of_interest()

        if len(objects) == 0:
            print("Nothing found, point HuskyLens at something it recognizes")
            continue

        nScans += 1
        print("-------------------- Scan #{} --------------------".format(nScans))

        for obj in objects:
            print("Object ID:", obj.id)
            print()

            if obj.id > 0:
                act(obj)
                # If act() starts basket_delivery(), it never returns.
                break
            
# ---------------------------------------------------------------------------
# Tag-based action function (your logic + basket trigger)
# ---------------------------------------------------------------------------

def act(obj):
    obj_id = obj.id

    if obj_id == 0:
        return

    # 1-4: turn in place
    if 1 <= obj_id <= 4:
        angle = actions[obj_id]
        print("Tag {} -> turn {} degrees".format(obj_id, angle))
        drivetrain.turn(
            turn_degrees=angle,
            max_effort=0.5,
            timeout=None
        )

    # 5-8: drive straight
    elif 5 <= obj_id <= 8:
        cm = actions[obj_id]
        print("Tag {} -> drive {} cm".format(obj_id, cm))
        drivetrain.straight(
            distance=cm,
            max_effort=0.25,
            timeout=None
        )

# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    try:
        board.wait_for_button()
        run()
    except (KeyboardInterrupt, SystemExit):
        print("\nEnding program. Stopping drivetrain.")
        drivetrain.set_speed(left_speed=0.0, right_speed=0.0)
        sys.exit(0)
