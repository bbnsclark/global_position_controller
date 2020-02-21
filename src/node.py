#!/usr/bin/python

import time
import math
import numpy as np
from math import pi, cos, sin
from datetime import datetime
from utilities import Utilities
from global_pose import GlobalPose
from position_controller import PositionController

import tf
import rospy
import rosnode
import actionlib
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from gps_common.msg import GPSFix
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import NavSatFix
from std_srvs.srv import Empty, EmptyRequest
from move_base_msgs.msg import MoveBaseAction
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Quaternion, Twist
from global_position_controller.srv import GoalPosition, GoalPositionResponse

PENDING         = 0   # The goal has yet to be processed by the action server
ACTIVE          = 1   # The goal is currently being processed by the action server
PREEMPTED       = 2   # The goal received a cancel request after it started executing and has since completed its execution (Terminal State)
SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)
ABORTED         = 4   # The goal was aborted during execution by the action server due to some failure (Terminal State)
REJECTED        = 5   # The goal was rejected by the action server without being processed, because the goal was unattainable or invalid (Terminal State)
PREEMPTING      = 6   # The goal received a cancel request after it started executing and has not yet completed execution
RECALLING       = 7   # The goal received a cancel request before it started executing, but the action server has not yet confirmed that the goal is canceled
RECALLED        = 8   # The goal received a cancel request before it started executing and was successfully cancelled (Terminal State)
LOST            = 9   # An action client can determine that a goal is LOST. This should not be sent over the wire by an action server

class Node:

    def __init__(self):

        self.controller = PositionController()

        self.utilities = Utilities()

        self.pose = GlobalPose()

        self.position = PoseStamped()

        self.target_pose = GlobalPose()

        self.goal = PoseStamped()

        self.new_goal = PoseStamped()

        self.response = GoalPositionResponse()

        self.check_msg = Float64()

        self.loop_threshold = 3.0 # used to determine  if position is reached

        self.send_threshold = 1.0 # used to determine whether to resend goal or not

        self.control_status = 'run'

        rospy.init_node('GLOBAL_POS')

        self.rate = 0.1

        self.current_distance = 0.0

        self.distance = 0.0

        self.speed = 0.0

        self.speed_time = 0.0

        self.speed_time_span = 0.0

        self.stuck = False

        self.stuck_speed_threshold = 0.05

        self.stuck_count = 0

        self.state = PENDING

        self.pub_check = rospy.Publisher('controller_check', Float64, queue_size = 1)

        self.sub_gps = rospy.Subscriber('gps_fix', GPSFix, self.gps_callback)

        self.sub_manage = rospy.Subscriber('manage_controller', String, self.manage_callback)

        self.srv_cmd_position = rospy.Service('goto_position', GoalPosition, self.goto_position_callback)

        self.pub_goal = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size = 1)

        self.sub_base = rospy.Subscriber('move_base/status', GoalStatusArray, self.base_callback)

        self.sub_odom = rospy.Subscriber('odom_inertial', Odometry, self.odom_callback)

        self.client_goal = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        rospy.wait_for_service('/MOVE/clear_costmaps')
        
        self.clear_costmaps_srv = rospy.ServiceProxy('/MOVE/clear_costmaps', Empty)

        rospy.loginfo('Starting global position controller...')
        

    def manage_callback(self, msg):
        
        self.control_status = msg.data


    def base_callback(self, msg):

        if len(msg.status_list) > 0:

            self.state = msg.status_list[-1].status

        else:

            self.state = PENDING


    def gps_callback(self, msg):

        self.pose.latitude = msg.latitude

        self.pose.longitude = msg.longitude

        self.pose.heading = msg.dip * math.pi / 180.0

        self.check_msg.data = 1.0

        self.pub_check.publish(self.check_msg)


    def odom_callback(self, msg):

        current_linear_speed = math.sqrt(math.pow(msg.twist.twist.linear.x, 2) + math.pow(msg.twist.twist.linear.y, 2))

        current_rotational_speed = msg.twist.twist.angular.z

        current_speed = max(current_linear_speed, current_rotational_speed)

        if current_speed <= self.stuck_speed_threshold:

            self.speed_time = rospy.get_rostime().to_nsec() - self.speed_time

            self.speed_time_span += self.speed_time

        else:

            self.speed_time = rospy.get_rostime().to_nsec()

            self.speed_time_span = 0.0

        
        if self.speed_time_span >= 5.0 and self.state != PENDING and self.state != SUCCEEDED and self.state != PREEMPTED and self.state != ABORTED and self.state != REJECTED and self.state != RECALLED and self.state != LOST:

            self.stuck = True

        else:

            self.stuck = False


    def goto_position_callback(self, msg):

        self.control_status = 'run'

        self.target_pose.latitude = msg.target_latitude

        self.target_pose.longitude = msg.target_longitude

        self.target_pose.heading = msg.target_heading

        self.distance = 2.0 * self.loop_threshold

        self.stuck_count = 0

        self.stuck_count = False

        # now we implement a SUPER dumb time-base position control loop
        while self.distance > self.loop_threshold:

            if self.control_status == 'stop':
                
                self.client_goal.cancel_goal()

                self.client_goal.cancel_all_goals()

                break

            elif self.control_status == 'pause':

                # we wait 2 seconds while paused
                self.client_goal.cancel_goal()

                self.client_goal.cancel_all_goals()

                rospy.sleep(2.0)

            elif self.control_status == 'run':

                # first we calculate the target global position in local body frame
                self.new_goal, self.distance = self.controller.calculate_new_goal(self.pose, self.target_pose)

                if self.stuck:

                    rospy.logwarn("Robot is stuck")

                    self.stuck_count += 1

                    self.client_goal.cancel_goal()

                    self.client_goal.cancel_all_goals()

                    if self.stuck_count > 5:

                        # if we are stuck in place, let's cancel all goals, and clear out costmap to try to recover
                        self.clear_costmaps_srv(EmptyRequest())

                        rospy.sleep(1.0)

                        rospy.logwarn('clear costmap recovery activated')

                        if self.stuck_count > 7:

                            rospy.logwarn('rotation recovery activated')

                            recovery = self.controller.get_recovery_goal(self.pose, self.target_pose)

                            self.pub_goal.publish(recovery)

                            self.clear_costmaps_srv(EmptyRequest())

                            rospy.sleep(10.0)

                            self.stuck_count = 0

                else:

                    self.pub_goal.publish(self.new_goal)

                rospy.sleep(3.0)

        # once the platform has reached its goal we cancel all move_base goals
        self.client_goal.cancel_goal()

        self.client_goal.cancel_all_goals()

        # and return the status
        self.response.status = "Done"

        return self.response


if __name__ == '__main__':

    try:

        node = Node()

        rospy.spin()

    except rospy.ROSInterruptException:

        pass

    rospy.loginfo('Exiting')