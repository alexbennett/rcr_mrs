#!/usr/bin/env python

import rospy
import mavros
import time
from mrs import *
from mrs.setpoint import *

def main():
    rospy.init_node('breck_field_gps_test1')
    mavros.set_namespace()

    # Create logger
    logger = Logger('logs/breck_field_gps_test1')

    # Connect to vehicle
    logger.log_info('Initializing vehicle...')
    vehicle = Vehicle(logger)

    # Initialize setpoints
    logger.log_info('Initializing setpoints...')
    gps_sp = GPS_Setpoint(vehicle)
    vel_sp = Velocity_Setpoint(vehicle)

    # Set to takeoff
    logger.log_info('Taking off!')

    if not vehicle.set_mode('AUTO.TAKEOFF'):
        logger.log_warning('Mode change rejected.')

    vehicle.arm()
    time.sleep(15)

    # Navigate to target detection location
    logger.log_info('Setting GPS location...')
    gps_sp.set(37.897002, -86.581023, 10)

    # Set back to OFFBOARD mode
    logger.log_info('Entering OFFBOARD mode and navigating to GPS location...')
    if not vehicle.set_mode('OFFBOARD'):
        logger.log_warning('Mode change rejected.')
        return

    # Wait until reached
    gps_sp.wait_until_reached()

    logger.log_info('Location reached! Loitering for 15 seconds...')
    logger.log_info('Entering LOITER mode...')

    if not vehicle.set_mode('AUTO.LOITER'):
        logger.log_warning('Mode change rejected.')
        return

    time.sleep(5)

    if not vehicle.set_mode('AUTO.LAND'):
        logger.log_warning('Mode change rejected.')
        return

    gps_sp.terminate()
    vel_sp.terminate()

    # Done
    logger.log_info('Done!')


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
