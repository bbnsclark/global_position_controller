#!/usr/bin/python

import utm
import time
import math
from global_pose import GlobalPose
from utilities import Utilities

import tf
from nav_msgs.msg import PoseStamped

EAST = 0
NORT = 1
MAX_DISTANCE = 10.0

class PositionController:

    def __init__(self):

        self.goal = PoseStamped()

        self.utilities = Utilities()

    def calculate_new_goal(self, init, target):

        # converting the heading to radians 
        calc_x, calc_y, calc_distance = self.utilities.calculate_distance(init, target)

        # capping the distance
        distance = max(calc_distance, MAX_DISTANCE)

        if distance >= MAX_DISTANCE:

            x = (calc_distance / MAX_DISTANCE ) * calc_x

            y = (calc_distance / MAX_DISTANCE ) * calc_y

        else:

            x = calc_x

            y = calc_y

        quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, theta)

        self.goal.pose.position.x = x

        self.goal.pose.position.y = y

        self.goal.pose.orientation.x = quaternion[0]

        self.goal.pose.orientation.y = quaternion[1]

        self.goal.pose.orientation.z = quaternion[2]

        self.goal.pose.orientation.w = quaternion[3]

        return self.goal