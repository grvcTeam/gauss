#!/usr/bin/env python
# This Python file uses the following encoding: utf-8

'''This script is developed to define and charge a mission in a fixed wing UAV using the
ual_backend_mavros_fw. Firstly, it has to be executed roslaunch ual_backend_mavros_fw simulations.launch'''

import rospy
import time
import numpy as np
import math
import copy
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from geographic_msgs.msg import *
from std_srvs.srv import *
from uav_abstraction_layer.srv import *
from uav_abstraction_layer.msg import *


class MissionCharger():

    def __init__(self): 
        
        # Initialization
        
        # Publish

        # Wait until services are available and create connection

        # Initialization

        # Initialization


''' The node and the MissionCharger class are initialized'''

if __name__=='__main__':

    rospy.init_node('mission_charger')
    m = MissionCharger()
    rospy.spin()   