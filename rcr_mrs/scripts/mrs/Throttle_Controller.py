#!/usr/bin/env python

import rospy
import threading
import time
import mavros

from math import *
from mavros.utils import *
from std_msgs.msg import *

class Throttle_Controller:
    def __init__(self):
        # Create target coordinate variable
        self._throttle = 0

        # Create publisher to local position
        self._throttle_pub = rospy.Publisher('/mavros/setpoint_attitude/att_throttle', Float64, queue_size=10)

        # Setup rate
        self._rate = rospy.Rate(20)

        # Setup navigation status
        self._done = threading.Event()

    def set(self, throttle):
        # Terminate previous throttle control
        self.terminate()

        # Set velocity
        self._throttle = throttle

        # Reset update status
        self._done.clear()

        # Spawn thread
        try:
            # Create thread
            self._update_thread = threading.Thread(target=self.update_throttle, args=())

            # Start thread
            self._update_thread.start()

            # Allow thread to start
            time.sleep(1)
        except:
            fault("Error: Unable to start thread")

    def update_throttle(self):
        # Create message
        msg = Float64()

        # Set throttle data
        msg.data = self._throttle

        # Set throttle
        while not self._done.is_set() and not rospy.is_shutdown():
            # Publish
            self._throttle_pub.publish(msg)

            # Ensure proper communication rate
            self._rate.sleep()

    def terminate(self):
        # Terminate the thread by setting the done event
        self._done.set()

        # Allow the system time to respond
        time.sleep(0.1)
