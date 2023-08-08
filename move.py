from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import sys

print("Try Connect")

vehicle = connect('tcp:127.0.0.1:5763', wait_ready=True)

def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode    = VehicleMode("GUIDED")
    vehicle.armed   = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        #Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

def fly_around(speed):
    vehicle.airspeed = speed
    cur_latitude = vehicle.location.global_relative_frame.lat
    cur_longtitude = vehicle.location.global_relative_frame.lon

    point1 = LocationGlobalRelative(cur_latitude + 0.001, cur_longtitude + 0.001, alt)
    vehicle.simple_goto(point1)
    # sleep so we can see the change in map
    time.sleep(30)

    point1 = LocationGlobalRelative(cur_latitude - 0.001, cur_longtitude + 0.001, alt)
    vehicle.simple_goto(point1)
    # sleep so we can see the change in map
    time.sleep(30)

    print("Returning to Launch")
    vehicle.mode = VehicleMode("RTL")

    # Close vehicle object before exiting script
    print("Close vehicle object")
    vehicle.close()

sys.stdout.write("Please enter an altitude:")
sys.stdout.flush()
alt = int(sys.stdin.readline().strip())

arm_and_takeoff(alt)

sys.stdout.write("Please enter an airspeed:")
sys.stdout.flush()
speed = int(sys.stdin.readline().strip())

fly_around(speed)