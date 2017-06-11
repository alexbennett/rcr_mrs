#!/usr/bin/env python

import rospy
import mavros
import time
from mrs import *
from mrs.setpoint import *

def main():
    rospy.init_node('combined_test')
    mavros.set_namespace()

    # Connect to vehicle
    vehicle = Vehicle()

    # Set to takeoff mode
    vehicle.set_mode('AUTO.TAKEOFF')

    # Wait for input to arm
    raw_input('Waiting for enter to arm and begin...')
    vehicle.arm()

    time.sleep(10)

    # Create setpoint utilities
    gps_sp = GPS_Setpoint(vehicle)
    vel_sp = Velocity_Setpoint(vehicle)

    print 'Rising to 100 meters'
    gps_sp.set(vehicle.get_home_lat(), vehicle.get_home_lon(), 100)

    vehicle.set_mode('OFFBOARD')

    gps_sp.wait_until_reached()

    print 'Descend 3 seconds'
    vel_sp.set(0.0, 0.0, -6, 3)

    print 'Maneuver away'
    vel_sp.set(2.5, 0.0, -2.0, 5.0)
    vel_sp.set(2.5, 0.0, 0.0, 5.0)
    vel_sp.set(0.0, 0.0, 4.0, 5.0)

    print "Hold for 5 seconds"
    vel_sp.hold(5)

    print 'Return home'
    gps_sp.set(vehicle.get_home_lat(), vehicle.get_home_lon(), 4)
    gps_sp.wait_until_reached()

    vehicle.set_mode('AUTO.LOITER')
    time.sleep(5)
    vehicle.set_mode('AUTO.LAND')


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

