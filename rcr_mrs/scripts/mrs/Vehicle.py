import rospy
import mavros
import time
import utm

from math import *
from mavros_msgs.srv import SetMode, CommandBool
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import NavSatFix

class Vehicle:
    def __init__(self, logger):
        # Initialize variables
        self.state_topic = State()
        self.global_position_topic = NavSatFix()
        self.local_position_topic = PoseStamped()
        self.local_velocity_topic = TwistStamped()

        # Create rate
        self.rate = rospy.Rate(20)

        # Wait for service startup
        rospy.wait_for_service('/mavros/set_mode')
        rospy.wait_for_service('/mavros/cmd/arming')

        # Setup services
        self.set_mode_serv = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.arm_serv = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)

        # Setup subscribers
        self.state_sub = rospy.Subscriber("/mavros/state", State, self.state_callback)
        self.global_position_sub = rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.global_position_callback)
        self.local_position_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.local_position_callback)
        self.local_velocity_sub = rospy.Subscriber('/mavros/local_position/velocity', TwistStamped, self.local_velocity_callback)

        # Wait until connected
        while not rospy.is_shutdown() and not self.is_connected():
            self.rate.sleep()

        # Allow time for complete startup
        time.sleep(1)

        # Save home GPS coordinates
        self.home_gps = self.get_gps()

        # Convert to UTM and save
        home_utm_x, home_utm_y, _, _ = utm.from_latlon(self.home_gps[0], self.home_gps[1])
        self.home_utm = (home_utm_x, home_utm_y, self.home_gps[2])

        logger.log_data('Home: lat %f, lon %f, alt %f' % self.home_gps)

    #################################
    ## Global position information ##
    #################################

    def get_gps(self):
        return (self.global_position_topic.latitude, self.global_position_topic.longitude, self.global_position_topic.altitude)

    def get_latitude(self):
        return self.global_position_topic.latitude

    def get_longitude(self):
        return self.global_position_topic.longitude

    def get_altitude(self):
        return self.global_position_topic.altitude

    ################################
    ## Local position information ##
    ################################

    def get_position(self):
        return (self.local_position_topic.pose.position.x, self.local_position.pose.position.y, self.local_position.pose.position.z)

    def get_position_x(self):
        return self.local_position_topic.pose.position.x

    def get_position_y(self):
        return self.local_position_topic.pose.position.y

    def get_position_z(self):
        return self.local_position_topic.pose.position.z

    def get_orientation(self):
        return (self.local_position_topic.pose.orientation.x, self.local_position_topic.pose.orientation.y, self.local_position_topic.pose.orientation.z, self.local_position_topic.pose.orientation.w)

    def get_orientation_x(self):
        return self.local_position_topic.pose.orientation.x

    def get_orientation_y(self):
        return self.local_position_topic.pose.orientation.y

    def get_orientation_z(self):
        return self.local_position_topic.pose.orientation.z

    def get_orientation_w(self):
        return self.local_position_topic.pose.orientation.w

    ################################
    ## Local velocity information ##
    ################################

    def get_velocity(self):
        return (self.local_velocity_topic.twist.linear.x, self.local_velocity_topic.twist.linear.y, self.local_velocity_topic.twist.linear.z)

    def get_velocity_x(self):
        return self.local_velocity_topic.twist.linear.x

    def get_velocity_y(self):
        return self.local_velocity_topic.twist.linear.y

    def get_velocity_z(self):
        return self.local_velocity_topic.twist.linear.z

    ###############################
    ## Vehicle state information ##
    ###############################

    def arm(self):
        self.arm_serv(True)
        time.sleep(1)

    def disarm(self):
        self.arm_serv(False)

    def set_mode(self, mode):
        # If in AUTO.LAND, reject mode change
        if self.get_mode() != 'AUTO.LAND':
            # Set mode
            self.set_mode_serv(custom_mode=mode)

            # Allow a moment for change to propogate
            time.sleep(0.1)

            # Return true for success
            return True

        # Default false
        return False

    def is_connected(self):
        return self.state_topic.connected

    def is_armed(self):
        return self.state_topic.armed

    def get_mode(self):
        return self.state_topic.mode

    def get_home_gps(self):
        return self.home_gps

    def get_home_lat(self):
        return self.home_gps[0]

    def get_home_lon(self):
        return self.home_gps[1]

    def get_home_alt(self):
        return self.home_gps[2]

    def get_home_utm(self):
        return self.home_utm

    def get_home_utm_x(self):
        return self.home_utm[0]

    def get_home_utm_y(self):
        return self.home_utm[1]

    def get_home_utm_z(self):
        return self.home_utm[2]

    ###############
    ## Callbacks ##
    ###############

    def state_callback(self, topic):
        self.state_topic = topic

    def global_position_callback(self, topic):
        self.global_position_topic = topic

    def local_position_callback(self, topic):
        self.local_position_topic = topic

    def local_velocity_callback(self, topic):
        self.local_velocity_topic = topic
