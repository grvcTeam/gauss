#!/usr/bin/env python

'''This script is developed to define and load a mission in a fixed wing UAV using the
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

class MissionLoader():

    def __init__(self):
       
        # Wait until service is available and creat connection
        
        rospy.wait_for_service('/ual/set_mission')         
        self._setmission_service = rospy.ServiceProxy('/ual/set_mission', SetMission) 
               
    
    def define_mission(self): 
        #header_empty = std_msgs.msg.Header()
        #header_custom = std_msgs.msg.Header(header_empty.seq, header_empty.stamp, "map")
        self._mission_wps = []

        '''TAKE OFF POSE'''
        take_off_phase = MissionElement()
        take_off_phase.type = MissionElement.TAKEOFF_POSE
        point = Point()
        point.x = 50.0
        point.y = 0.0
        point.z = 5.0
        pose = Pose()
        pose.position = point
        destination = PoseStamped()
        destination.pose = pose
        take_off_phase.waypoints = [destination]
        self._mission_wps.append(take_off_phase.waypoints)

        '''LOITER HEIGHT'''
        loiter_phase = MissionElement()
        loiter_phase.type = MissionElement.LOITER_HEIGHT
        point = Point()
        point.x = 50.0
        point.y = 50.0
        point.z = 5.0
        pose = Pose()
        pose.position = point
        destination = PoseStamped()
        destination.pose = pose
        loiter_phase.waypoints = [destination]
        #loiter_phase.params = self.dictToListOfParamFloat({"heading": 0.0,"radius": 10.0, "forward_moving" : 1.0})
        self._mission_wps.append(loiter_phase.waypoints)
        
        return self._mission_wps

    def send_mission(self):
        request = SetMissionRequest()
        request.mission_elements = self._mission_wps
        #request.blocking = True
        response = self._setmission_service(request)        
        return response 

''' The node and the MissionLoader class are initialized'''

if __name__=='__main__':

    rospy.init_node('mission_loader')
    m = MissionLoader()
    #rospy.spin()   