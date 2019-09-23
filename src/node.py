#!/usr/bin/python

import time
import numpy as np
from datetime import datetime
from math import pi, cos, sin
from global_pose import GlobalPose
from position_controller import PositionController

import tf
import rospy
import rosnode
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from gps_common.msg import GPSFix
from nav_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Quaternion, Twist
from global_position_controller.srv import GoalPosition, GoalPositionResponse

class Node:

    def __init__(self):

        self.controller = PositionController()

        self.pose = GlobalPose()

        self.target_pose = GlobalPose()

        self.response = GoalPositionResponse()

        self.threshold = 3.0

        rospy.init_node('GLOBAL_POS')

        self.rate = 0.1

        self.sub_diag = rospy.Subscriber('/gps_fix', GPSFix, self.gps_callback)

        self.srv_cmd_position = rospy.Service('set_global_position', GoalPosition, self.set_position_callback)

        self.client = rospy.ServiceProxy(' move_base_simple/goal', PoseStamped)

        rospy.loginfo('Starting state observer...')

        self.diag = Diagnostics()


    def gps_callback(self, msg):

        self.pose.latitude = msg.latitude

        self.pose.longitude = msg.longitude

        self.pose.heading = msg.dip 


    def send_move_base_goal(self, goal):

        rospy.wait_for_service('move_base_simple/goal')
        
        try:

            resp = self.client(goal)

            return resp

        except rospy.ServiceException, e:

            print "Service call failed"


    def set_mode_callback(self, msg):

        distance = 2.0 * threshold

        while distance > threshold:

            #first we need to calculate the target global position in local body frame
            local_goal, distance = self.controller.get_local_position(self.pose, self.target_pose)

            if distance > threshold:

                reply = self.send_move_base_goal(local_goal)

        

        return self.response


if __name__ == '__main__':

    try:

        node = Node()

        node.spin()

    except rospy.ROSInterruptException:

        pass

    rospy.loginfo('Exiting')
