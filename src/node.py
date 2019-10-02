#!/usr/bin/python

import time
import numpy as np
from datetime import datetime
from math import pi, cos, sin
from utilities import Utilities
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
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Quaternion, Twist
from global_position_controller.srv import GoalPosition, GoalPositionResponse

class Node:

    def __init__(self):

        self.controller = PositionController()

        self.utilities = Utilities()

        self.pose = GlobalPose()

        self.target_pose = GlobalPose()

        self.goal = PoseStamped()

        self.new_goal = PoseStamped()

        self.response = GoalPositionResponse()

        self.loop_threshold = 3.0 # used to determine  if position is reached

        self.send_threshold = 1.0 # used to determine whether to resend goal or not

        rospy.init_node('GLOBAL_POS')

        self.rate = 0.1

        self.sub_diag = rospy.Subscriber('/gps_fix', GPSFix, self.gps_callback)

        self.srv_cmd_position = rospy.Service('goto_position', GoalPosition, self.goto_position_callback)

        self.client = rospy.ServiceProxy('move_base_simple/goal', PoseStamped)

        rospy.loginfo('Starting state observer...')

        self.diag = Diagnostics()


    def gps_callback(self, msg):

        self.pose.latitude = msg.latitude

        self.pose.longitude = msg.longitude

        self.pose.heading = msg.dip 


    def send_move_base_goal(self, goal):
        
        try:

            resp = self.client(goal)

            return resp

        except rospy.ServiceException, e:

            return "Service call failed"


    def goto_position_callback(self, msg):

        distance = 2.0 * loop_threshold

        while distance > loop_threshold:

            # first we need to calculate the target global position in local body frame
            self.new_goal, distance = self.controller.calculate_new_goal(self.pose, self.target_pose)

            goal_distance = self.utilities.calculate_pose_distance(self.goal, self.new_goal, send_threshold)

            if goal_distance > send_threshold:

                self.goal = self.new_goal

                reply = 'x: ' + self.new_goal.pose.position.x + ' y: ' + self.new_goal.pose.position.y
                #reply = self.send_move_base_goal(self.new_goal)

        return self.response


if __name__ == '__main__':

    try:

        node = Node()

        node.spin()

    except rospy.ROSInterruptException:

        pass

    rospy.loginfo('Exiting')
