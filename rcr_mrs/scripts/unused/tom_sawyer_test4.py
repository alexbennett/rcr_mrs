#!/usr/bin/env python

import rospy
import mavros
import time
from mrs import *
from mrs.setpoint import *

def main():
    rospy.init_node('tom_sawyer_test4')
    mavros.set_namespace()

    # Create logger
    logger = Logger('logs/tom_sawyer_test4')

    # Load configuration file
    logger.log_info('Loading configuration...')
    config = Configuration('config.yaml')

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
    time.sleep(10)

    #########################################

    # Navigate to first coordinate
    logger.log_info('Setting GPS location...')
    gps_sp.set(38.287298, -85.559043, 75)

    # Set back to OFFBOARD mode
    logger.log_info('Entering OFFBOARD mode and navigating to GPS location...')
    if not vehicle.set_mode('OFFBOARD'):
        logger.log_warning('Mode change rejected.')
        return

    # Wait until reached
    gps_sp.wait_until_reached()

    # # Set to AUTO.LOITER
    # if not vehicle.set_mode('AUTO.LOITER'):
    #     logger.log_warning('Mode change rejected.')
    #     return
    # time.sleep(5)

    # Set velocity setpoint
    logger.log_info('Performing breakaway...')
    Utilities.perform_breakaway(logger, config, vel_sp, vehicle.get_velocity())

    #########################################

    logger.log_info('Done!')
    logger.log_info('Entering LOITER mode...')

    if not vehicle.set_mode('AUTO.LOITER'):
        logger.log_warning('Mode change rejected.')
        return

    time.sleep(10)

    if not vehicle.set_mode('AUTO.RTL'):
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
