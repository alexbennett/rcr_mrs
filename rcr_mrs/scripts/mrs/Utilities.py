import math
import RPi.GPIO as GPIO

# Define GPIO pin numbers
gpio_limit_switch1 = 2
gpio_limit_switch2 = 3
gpio_limit_switch3 = 4
gpio_limit_switch4 = 17
gpio_start_mrs = 25
gpio_shutdown_mrs = 22
gpio_fire_rrs = 24
gpio_breakaway_delay = 23

class Utilities:
    @staticmethod
    def perform_breakaway(logger, config, vel_sp, drift_xy=(1,1,-1)):
        # Calculate drift values
        drift_xy_magnitude = math.sqrt(math.pow(drift_xy[0], 2) + math.pow(drift_xy[1], 2))
        drift_x_unit = drift_xy[0] / drift_xy_magnitude
        drift_y_unit = drift_xy[1] / drift_xy_magnitude

        # Read out values used in calculation
        logger.log_data('Drift: %f, %f, %f' % drift_xy)
        logger.log_data('Magnitude: %f' % drift_xy_magnitude)
        logger.log_data('Unit drift: (%f, %f)' % (drift_x_unit, drift_y_unit))

        # Descend quickly
        logger.log_info('Descending for 2 seconds at max descent velocity...')
        vel_sp.set(0.0, 0.0, -6.0, 2)

        # Tell the RRS to start monitoring velocity
        Utilities.set_breakaway_delay_pin(1)

        # Maneuver away
        logger.log_info('Reversing drift vector...')
        vel_sp.set(config.get_ground_speed_mult() * -drift_x_unit, config.get_ground_speed_mult() * -drift_y_unit, -6.0, 3.0)
        vel_sp.set(config.get_ground_speed_mult() * -drift_x_unit, config.get_ground_speed_mult() * -drift_y_unit, 0.0, 10.0)
        # vel_sp.set(config.get_ground_speed_mult() * -drift_x_unit, config.get_ground_speed_mult() * -drift_y_unit, 6.0, 5.0)

        # Climb
        # logger.log_info('Climbing for 5 seconds...')
        # vel_sp.set(0, 0, 5.0, 5.0)

    @staticmethod
    def setup_gpio():
        # Set pin numbering standard
        GPIO.setmode(GPIO.BCM)

        # Disable warnings
        GPIO.setwarnings(False)

        # Setup pins
        GPIO.setup(gpio_limit_switch1, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(gpio_limit_switch2, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(gpio_limit_switch3, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(gpio_limit_switch4, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(gpio_start_mrs, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(gpio_shutdown_mrs, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(gpio_fire_rrs, GPIO.OUT)
        GPIO.setup(gpio_breakaway_delay, GPIO.OUT)

        # Set initial
        GPIO.output(gpio_fire_rrs, GPIO.LOW)
        GPIO.output(gpio_breakaway_delay, GPIO.LOW)

    @staticmethod
    def read_ls1_pin():
        return GPIO.input(gpio_limit_switch1)

    @staticmethod
    def read_ls2_pin():
        return GPIO.input(gpio_limit_switch2)

    @staticmethod
    def read_ls3_pin():
        return GPIO.input(gpio_limit_switch3)

    @staticmethod
    def read_ls4_pin():
        return GPIO.input(gpio_limit_switch4)

    @staticmethod
    def read_start_mrs_pin():
        return GPIO.input(gpio_start_mrs)

    @staticmethod
    def read_shutdown_mrs_pin():
        return GPIO.input(gpio_shutdown_mrs)

    @staticmethod
    def set_fire_rrs_pin(value):
        if value:
            GPIO.output(gpio_fire_rrs, GPIO.HIGH)
        else:
            GPIO.output(gpio_fire_rrs, GPIO.LOW)

    @staticmethod
    def set_breakaway_delay_pin(value):
        if value:
            GPIO.output(gpio_breakaway_delay, GPIO.HIGH)
        else:
            GPIO.output(gpio_breakaway_delay, GPIO.LOW)
