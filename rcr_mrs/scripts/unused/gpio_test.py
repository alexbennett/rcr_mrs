#!/usr/bin/env python

import time
from mrs import *
from mrs.setpoint import *

def main():
    # Setup GPIO
    Utilities.setup_gpio()

    # while True:
    #     print 'START MRS: %d, SHUTDOWN MRS: %d' % (Utilities.read_start_mrs_pin(), Utilities.read_shutdown_mrs_pin())

    raw_input('Press enter to fire RRS')
    Utilities.set_fire_rrs_pin(1)

    raw_input('Press enter to set breakaway signal')
    Utilities.set_breakaway_delay_pin(1)

if __name__ == '__main__':
    main()
