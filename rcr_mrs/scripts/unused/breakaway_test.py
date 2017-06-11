#!/usr/bin/env python

import rospy
import mavros
import time
from mrs import *
from mrs.setpoint import *

def main():
    rospy.init_node('main')
    mavros.set_namespace()

    # Create logger
    logger = Logger('logs/breakaway_test')

    # Connect to vehicle
    logger.log_info('Initializing vehicle...')
    vehicle = Vehicle(logger)

    # Load configuration file
    logger.log_info('Loading configuration...')
    config = Configuration('config.yaml')

    # Print configuration information
    logger.log_warning('Target location: lat %f, lon %f, alt %f' % config.get_target_location())
    logger.log_warning('Landing location: lat %f, lon %f, alt %f' % config.get_landing_location())
    logger.log_warning('Ground speed multiplier: %f' % config.get_ground_speed_mult())
    logger.log_warning('TDS loiter time: %d' % config.get_tds_loiter_time())
    logger.log_warning('Landing loiter time: %d' % config.get_landing_loiter_time())

    # Initialize setpoints
    logger.log_info('Initializing setpoints...')
    gps_sp = GPS_Setpoint(vehicle)
    vel_sp = Velocity_Setpoint(vehicle)

    # Set to takeoff
    logger.log_info('Taking off!')
    vehicle.set_mode('AUTO.TAKEOFF')
    vehicle.arm()
    time.sleep(30)

    # Navigate to target detection location
    logger.log_info('Setting GPS location...')
    gps_sp.set(37.7371598, -85.9114385, 160)

    # Set back to OFFBOARD mode
    logger.log_info('Entering OFFBOARD mode and navigating to GPS location...')
    vehicle.set_mode('OFFBOARD')

    # Wait until reached
    gps_sp.wait_until_reached()

    logger.log_info('Location reached! Loitering for 30 seconds...')

    logger.log_info('Entering LOITER mode...')
    vehicle.set_mode('AUTO.LOITER')

    time.sleep(30)

    logger.log_info('Sending (0, 0, 0) velocity setpoint...')
    vel_sp.set(0, 0, 0)

    # Set back to OFFBOARD mode
    logger.log_info('Entering OFFBOARD mode...')
    vehicle.set_mode('OFFBOARD')

    # Perform breakaway
    logger.log_info('Performing breakaway...')
    Utilities.perform_breakaway(logger, config, vel_sp)

    logger.log_info('Entering LOITER mode...')
    vehicle.set_mode('AUTO.LOITER')

    vel_sp.terminate()

    # Done
    logger.log_info('Done!')


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
