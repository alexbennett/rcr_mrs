#!/usr/bin/env python

import rospy
import threading
import time
import mavros

from math import *
from mavros.utils import *
from std_msgs.msg import *
from mavros_msgs.msg import AttitudeTarget
from geometry_msgs.msg import *

class Raw_Setpoint:
    def __init__(self):
        # Create target thrust variable
        self._thrust = 0

        # Create publisher to local position
        self._rawatt_pub = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)

        # Setup rate
        self._rate = rospy.Rate(20)

        # Setup navigation status
        self._done = threading.Event()

    def set(self, thrust):
        # Terminate previous velocity
        self.terminate()

        # Set velocity
        self._thrust = thrust

        # Reset update status
        self._done.clear()

        # Spawn thread
        try:
            # Create thread
            self._update_thread = threading.Thread(target=self.update_attitude, args=())

            # Start thread
            self._update_thread.start()

            # Allow thread to start
            time.sleep(1)
        except:
            fault("Error: Unable to start thread")

    def update_attitude(self):
        # Create message
        msg = AttitudeTarget(
             header = Header(
                stamp=rospy.Time.now()
            )
        )

        # Ignore all but thrust
        msg.type_mask = AttitudeTarget.IGNORE_ROLL_RATE | AttitudeTarget.IGNORE_PITCH_RATE | AttitudeTarget.IGNORE_YAW_RATE | AttitudeTarget.IGNORE_ATTITUDE

        # # Set attitude
        # msg.orientation.x = 0
        # msg.orientation.y = 0
        # msg.orientation.z = 0
        # msg.orientation.w = 0

        # # Set rates
        # msg.body_rate.x = 0.1
        # msg.body_rate.y = 0.1
        # msg.body_rate.z = 0.1

        # Set velocity
        msg.thrust = self._thrust

        # Navigate until location is reached
        while not self._done.is_set() and not rospy.is_shutdown():
            # Publish
            self._rawatt_pub.publish(msg)

            # Ensure proper communication rate
            self._rate.sleep()

    def terminate(self):
        # Terminate the thread by setting the done event
        self._done.set()

        # Allow the system time to respond
        time.sleep(0.1)
