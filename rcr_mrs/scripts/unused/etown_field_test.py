#!/usr/bin/env python

import rospy
import mavros
import time
from mrs import *
from mrs.setpoint import *

def main():
    rospy.init_node('combined_test')
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
    time.sleep(15)

    # Set target GPS location
    print 'Setting GPS Target...'
    setpoint.set(37.7368941, -85.9163571, 30)

    # Set to offboard
    print 'Setting to offboard...'
    vehicle.set_mode('OFFBOARD')

    # Wait
    setpoint.wait_until_reached()

    print 'Travel to targets'
    setpoint.set(37.7368941, -85.9163571, 160)
    setpoint.wait_until_reached()

    # Wait
    print 'Done! Holding...'

    vehicle.set_mode('AUTO.LOITER')

    # Finished
    print 'Done!'


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
