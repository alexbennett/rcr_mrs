#!/usr/bin/env python

import rospy
import mavros
import time
from mrs import *
from mrs.setpoint import *

def main():
    rospy.init_node('breck_field_gps_test3')
    mavros.set_namespace()

    # Create logger
    logger = Logger('logs/breck_field_gps_test3')

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
        return

    vehicle.arm()
    time.sleep(15)

    # Navigate to first coordinate
    logger.log_info('Setting first GPS location...')
    gps_sp.set(37.897002, -86.581023, 30)

    # Set back to OFFBOARD mode
    logger.log_info('Entering OFFBOARD mode and navigating to GPS location...')
    if not vehicle.set_mode('OFFBOARD'):
        logger.log_warning('Mode change rejected.')
        return

    # Wait until reached
    gps_sp.wait_until_reached()

    # Set to AUTO.LOITER
    if not vehicle.set_mode('AUTO.LOITER'):
        logger.log_warning('Mode change rejected.')
        return
    time.sleep(5)

    # Navigate to second coordinate
    logger.log_info('Setting second GPS location...')
    gps_sp.set(37.896323, -86.581003, 30)

    # Set back to OFFBOARD mode
    logger.log_info('Entering OFFBOARD mode and navigating to GPS location...')
    if not vehicle.set_mode('OFFBOARD'):
        logger.log_warning('Mode change rejected.')
        return

    # Wait until reached
    gps_sp.wait_until_reached()

    # Set to AUTO.LOITER
    if not vehicle.set_mode('AUTO.LOITER'):
        logger.log_warning('Mode change rejected.')
        return
    time.sleep(5)

    # Navigate to second coordinate
    logger.log_info('Setting third GPS location...')
    gps_sp.set(38.287374, -85.558511, 30)

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
