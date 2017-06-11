#!/usr/bin/env python

import rospy
import mavros
import time
from mrs import *
from mrs.setpoint import *
from TDS import TDS

def main():
    rospy.init_node('main')
    mavros.set_namespace()

    # Setup global variables
    global logger
    global config
    global vehicle

    # Create logger
    logger = Logger('logs/main')

    # Load configuration file
    logger.log_info('Loading configuration...')
    config = Configuration('config.yaml')

    # Print configuration information
    logger.log_warning('Target location: lat %f, lon %f, alt %f' % config.get_target_location())
    logger.log_warning('Landing location: lat %f, lon %f, alt %f' % config.get_landing_location())
    logger.log_warning('Ground speed multiplier: %f' % config.get_ground_speed_mult())
    logger.log_warning('TDS loiter time: %d' % config.get_tds_loiter_time())
    logger.log_warning('Landing loiter time: %d' % config.get_landing_loiter_time())

    # Wait for enter before continuing
    raw_input('Press enter to continue initialization...')

    # Initialize TDS
    tds = TDS()

    # Initialize GPIO
    logger.log_info('Initializing GPIO...')
    Utilities.setup_gpio()

    # Connect to vehicle
    logger.log_info('Initializing vehicle...')
    vehicle = Vehicle(logger)

    # Initialize setpoints
    logger.log_info('Initializing setpoints...')
    gps_sp = GPS_Setpoint(vehicle)
    vel_sp = Velocity_Setpoint(vehicle)

    # Initialize throttle controller
    logger.log_info('Initializing throttle controller...')
    thr_control = Throttle_Controller()

    # Set throttle to 0
    logger.log_info('Setting throttle to 0%...')
    thr_control.set(0.0)

    # Set to offboard
    if not set_mode('OFFBOARD'):
        return

    # Start TDS
    tds.start()

    #################################################################################
    ## CAUTION: EVERYTHING BEYOND THIS POINT CAN CAUSE INJURY OR DAMAGE TO VEHICLE ##
    #################################################################################

    # Arm
    vehicle.arm()
    logger.log_info('Vehicle armed.')

    # Wait or all limit switches
    logger.log_info('Waiting for arm deployment...')
    while not Utilities.read_ls1_pin() or not Utilities.read_ls2_pin() or not Utilities.read_ls3_pin() or not Utilities.read_ls4_pin():
        pass

    # Terminate throttle controller
    thr_control.terminate()

    # Set to STABILIZED mode
    if not set_mode('STABILIZED'):
        return

    # Wait for RRS trigger to go LOW
    logger.log_info('Waiting for start signal from RRS...')
    while not Utilities.read_start_mrs_pin():
        pass

    # Send information to get into offboard
    vel_sp.set(0, 0, -4)

    # Set back to OFFBOARD mode
    if not set_mode('OFFBOARD'):
        return

    # Perform breakaway
    logger.log_info('Performing breakaway...')
    Utilities.perform_breakaway(logger, config, vel_sp, vehicle.get_velocity())
    vel_sp.terminate()

    # Navigate to target detection location
    logger.log_info('Navigating to target detection location...')
    gps_sp.set(config.get_target_location()[0], config.get_target_location()[1], config.get_target_location()[2])

    # Wait
    gps_sp.wait_until_reached()

    # Wait for TDS to complete target detection
    if not set_mode('AUTO.LOITER'):
        return

    logger.log_info('Allowing %d seconds for TDS...' % config.get_tds_loiter_time())

    # Wait
    time.sleep(config.get_tds_loiter_time())

    # Navigate to landing location
    logger.log_info('Navigating to landing location...')
    gps_sp.set(config.get_landing_location()[0], config.get_landing_location()[1], config.get_landing_location()[2])

    # Switch back to OFFBOARD mode
    if not set_mode('OFFBOARD'):
        return

    # Wait
    gps_sp.wait_until_reached()

    # Set to AUTO.LOITER
    if not set_mode('AUTO.LOITER'):
        return

    # Wait to allow time for activation of landing or timeout
    logger.log_info('Waiting %d seconds for land signal...' % config.get_landing_loiter_time())

    previous_time = time.time()
    while vehicle.get_mode() == 'AUTO.LOITER':
        if (time.time() - previous_time) > config.get_landing_loiter_time():
            # Status update
            logger.log_warning('Timed out... firing RRS and disarming!')

            # Timeout reached, fire RRS
            Utilities.set_fire_rrs_pin(1)

            # Wait a moment to make sure RRS is fired
            time.sleep(3)

            # Disarm
            vehicle.disarm()

            # Break while
            break

    # Done
    logger.log_info('Done!')
    return

def set_mode(mode):
    # Log mode change
    logger.log_info('Entering %s mode...' % mode)

    # Check mode change status
    if not vehicle.set_mode(mode):
        # Log error
        logger.log_error('Mode change rejected.')

        return False

    # Sleep for a moment
    time.sleep(0.2)

    # Log current mode
    logger.log_info('Current mode: %s' % vehicle.get_mode())

    # Default to true
    return True


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
