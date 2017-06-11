#!/usr/bin/env python

import rospy
import mavros
import time
from mrs import *
from mrs.setpoint import *
import RPi.GPIO as GPIO

def main():
    rospy.init_node('combined_test')
    mavros.set_namespace()

    # Setup GPIO
    IO = GPIO_Handler()

    # Connect to vehicle
    vehicle = Vehicle()

    # Create GPS setpoint
    gps_sp = GPS_Setpoint(vehicle)
    vel_sp = Velocity_Setpoint(vehicle)

    print 'Waiting for arms to lock...'

    while not IO.arms_locked():
        pass

    print 'Arms locked'

    print 'Setting takeoff mode...'  ####### Remove for Rocket Launch

    vehicle.set_mode('AUTO.TAKEOFF') ####### Remove for Rocket Launch

    time.sleep(1)                    ####### Remove for Rocket Launch

    print 'Arming...'

    # Arm
    vehicle.arm()

    # Wait for takeoff to finish
    time.sleep(15)                   ####### Remove for Rocket Launch

    # Set target GPS location
    print 'Setting GPS target...'         ####### Remove for Rocket Launch
    gps_sp.set(38.287162, -85.559213, 60) ####### Remove for Rocket Launch

    # Set to offboard
    print 'Setting to offboard...'        ####### MOVE FOR ROCKET LAUNCH
    vehicle.set_mode('OFFBOARD')          ####### MOVE FOR ROCKET LAUNCH

    # Wait for movement to point
    gps_sp.wait_until_reached()           ####### MOVE FOR ROCKET LAUNCH

    print 'Waiting for start MRS signal...'
    while not IO.get_Start_MRS_state():
        pass

    print 'Performing breakaway...'
    Utilities.breakaway(vel_sp, IO) #GET CURRENT VELOCITY#################

    print 'Breakaway complete maneuvering to targets'
    #             #################################
    gps_sp.set()  ###### Input Target Coords ######
    #             #################################
    gps_sp.wait_until_reached()

    print 'Targets reached setting to hold and waiting for detection'
    # start_time = time.time()
    # while time.time() <= start_time + 60:
    #     if TDS.objectsDetected():
    #         break

    time.sleep(30)

    print 'Targets detected maneuvering to landing area'
    #             ##################################
    gps_sp.set()  ###### Input Landing Coords ######
    #             ##################################
    gps_sp.wait_until_reached()

    # Set to hold and wait for clearance to land
    print 'Landing area reached'
    print 'Waiting for clearance to land...'
    vehicle.set_mode('AUTO.LOITER')

    start_time = time.time()
    while time.time() <= start_time + 60:
        if vehicle.get_mode() == 'AUTO.LAND':

            # Finished
            print 'Done!'

            return

    print 'No clearance recieved Firing RRS...'
    IO.fire_RRS()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
