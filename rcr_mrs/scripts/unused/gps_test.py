#!/usr/bin/env python

import rospy
import mavros
import time
from mrs import *
from mrs.setpoint import *

def main():
    rospy.init_node('gps_test')
    mavros.set_namespace()

    raw_input('Waiting for enter...')

    # Connect to vehicle
    vehicle = Vehicle()

    # Create GPS setpoint
    setpoint = GPS_Setpoint(vehicle)

    print 'Setting takeoff mode...'

    # Change mode
    vehicle.set_mode('AUTO.TAKEOFF')

    time.sleep(1)

    print 'Arming...'

    # Arm
    vehicle.arm()

    # Wait
    #raw_input('Waiting for enter...')
    time.sleep(15)

    # Set target GPS location
    print 'Setting GPS of Targets...'
    ########Target Coords#####################
    setpoint.set(37.0365766, -86.3526865, 30)

    # Set to offboard
    print 'Setting to offboard...'
    vehicle.set_mode('OFFBOARD')

    # Wait
    setpoint.wait_until_reached()

    print 'Waiting for 15 seconds...'

    vehicle.set_mode('AUTO.LOITER')

    time.sleep(15)

    print 'Travel to GPS location'
    setpoint.set(37.0356290, -86.3537156, 30)

    vehicle.set_mode('OFFBOARD')

    setpoint.wait_until_reached()

    # Wait
    print 'Done! Holding for 10 seconds...'

    vehicle.set_mode('AUTO.LOITER')

    time.sleep(10)

    print 'Returning home...'

    # Change to RTL mode
    vehicle.set_mode('AUTO.RTL')

    # Finished
    print 'Done!'


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
