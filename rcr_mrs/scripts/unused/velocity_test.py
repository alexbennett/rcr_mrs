#!/usr/bin/env python

import rospy
import mavros
import time
from mrs import *
from mrs.setpoint import *

def main():
    rospy.init_node('velocity_test')
    mavros.set_namespace()

    raw_input('Waiting for enter...')

    # Connect to vehicle
    vehicle = Vehicle()

    Utilities.breakaway(vehicle)

    # Create velocity setpoint
    setpoint = Velocity_Setpoint(vehicle)

    # print 'Setting takeoff mode...'
    # # Change mode
    # vehicle.set_mode('AUTO.TAKEOFF')
    # time.sleep(1)

    setpoint.set(0, 0, 2)

    # Set to offboard
    print 'Setting to offboard...'
    vehicle.set_mode('OFFBOARD')

    # Wait
    raw_input('Waiting for enter to arm...')

    # Arm
    print 'Arming...'
    vehicle.arm()

    print 'Ascending for 3 seconds...'
    time.sleep(3)

    # Set target velocity
    print 'Moving at 10 m/s for 3 seconds...'
    setpoint.set(10.0, 0.0, 0.0, 3)

    # Hold
    print 'Setting to hold for 5 seconds...'
    setpoint.hold(5)

    # Print some shit brugh
    print 'Velocity:', vehicle.get_velocity()

    # Finished
    print 'Done!'


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
