#!/usr/bin/python

import utm
import time
import math
from global_pose import GlobalPose

class PositionController:

    def __init__(self):

        pass

    def calculate_distance(self, init, target):

        init_utm = utm.from_latlon(init.latitude, init.longitude)

        target_utm = utm.from_latlon(target.latitude, target.longitude)

        position_east = (init_utm[0] - target_utm[0])

        position_north = (init_utm[1] - target_utm[1])

        

        

        

        

    






 
