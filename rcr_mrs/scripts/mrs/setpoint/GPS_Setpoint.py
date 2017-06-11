#!/usr/bin/env python

import rospy
import threading
import time
import mavros
import utm

from math import *
from mavros.utils import *
from std_msgs.msg import *
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped, Quaternion

class GPS_Setpoint:
    def __init__(self, vehicle):

        # Setup navigation status
        self._done = threading.Event()

    def set(self, lat, lon, alt):

        # Reset update status
        self._done.clear()

        # Spawn threads
        try:
            # Create threads
            self._update_thread = threading.Thread(target=self.update_position, args=())
            self._monitor_thread = threading.Thread(target=self.position_monitor, args=())

            # Start threads
            self._update_thread.start()
            self._monitor_thread.start()

            # Allow threads to start
            time.sleep(1)
        except:
            fault("Error: Unable to start thread")

    def wait_until_reached(self):
        while not self._done.is_set():
            self._rate.sleep()

    def update_position(self):
        # Navigate until location is reached
        while not self._done.is_set():
            # Publish
            self._pos_pub.publish(msg)

            # Ensure proper communication rate
            self._rate.sleep()

    def position_monitor(self):
        while not self._done.is_set():
            def is_near(x, y):
                return abs(x - y) < 0.5

            if is_near(self._vehicle.get_position_x(), self._target_utm[0]) and \
               is_near(self._vehicle.get_position_y(), self._target_utm[1]) and \
               is_near(self._vehicle.get_position_z(), self._target_utm[2]):
                # Terminate the thread by setting the done event
                self.terminate()

    def terminate(self):
        # Terminate the thread by setting the done event
        self._done.set()

        # Allow the system time to respond
        time.sleep(0.25)
