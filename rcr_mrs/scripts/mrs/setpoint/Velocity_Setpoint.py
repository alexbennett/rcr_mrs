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
from geometry_msgs.msg import TwistStamped

class Velocity_Setpoint:
    def __init__(self, vehicle):
        # Store vehicle for reference
        self._vehicle = vehicle

        # Create target coordinate variable
        self._target_vel = (0, 0, 0)

        # Create publisher to local position
        self._vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)

        # Setup rate
        self._rate = rospy.Rate(20)

        # Setup navigation status
        self._done = threading.Event()

    def hold(self, delay=0):
        # Set to negative of current
        self.set(-self._vehicle.get_velocity_x(), -self._vehicle.get_velocity_y(), -self._vehicle.get_velocity_z(), 2)

        # Set all velocities to 0
        self.set(0, 0, 0)

        # Delay if requested and terminate after
        if delay:
            # Delay
            time.sleep(delay)

            # Terminate update thread
            self.terminate()

    def set(self, vel_x, vel_y, vel_z, delay=0):
        # Terminate previous velocity
        self.terminate()

        # Set velocity
        self._target_vel = (vel_x, vel_y, vel_z)

        # Reset update status
        self._done.clear()

        # Spawn thread
        try:
            # Create thread
            self._update_thread = threading.Thread(target=self.update_velocity, args=())

            # Start thread
            self._update_thread.start()

            # Allow thread to start
            time.sleep(1)
        except:
            fault("Error: Unable to start thread")

        # Delay if requested and terminate after
        if delay:
            # Delay
            time.sleep(delay)

            # Terminate update thread
            self.terminate()

    def update_velocity(self):
        # Create message
        msg = TwistStamped(
             header = Header(
                frame_id="base_footprint",
                stamp=rospy.Time.now()
            ),
        )

        # Set velocity
        msg.twist.linear.x = self._target_vel[0]
        msg.twist.linear.y = self._target_vel[1]
        msg.twist.linear.z = self._target_vel[2]

        # Navigate until location is reached
        while not self._done.is_set() and not rospy.is_shutdown():
            # Publish
            self._vel_pub.publish(msg)

            # Ensure proper communication rate
            self._rate.sleep()

    def terminate(self):
        # Terminate the thread by setting the done event
        self._done.set()

        # Allow the system time to respond
        time.sleep(0.1)
