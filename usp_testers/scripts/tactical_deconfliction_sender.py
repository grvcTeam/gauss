#!/usr/bin/env python
# this Python file uses the following encoding: utf-8
'''This script is a sender of Deconflictionplans for testing
the Deconfliction.srv'''

import rospy
import time
from os import system
from gauss_msgs.srv import Threats, ThreatsRequest, ThreatsResponse
from gauss_msgs.msg import Threat, DeconflictionPlan, Waypoint, WaypointList
from gauss_msgs.srv import Deconfliction, DeconflictionRequest, DeconflictionResponse

''' It is defined a class with all functionabilities'''

class TacticalDeconfliction():
    
    # This is the constructor class.

    def __init__(self): 
        
        # Initialization

        # Publish
    
        # Wait until service is available and creat connection
    
        # Server

        self._deconfliction_service = rospy.Service('/gauss/deconfliction', Deconfliction, self.service_deconfliction_cb) 
        print("Ready to get a Deconfliction request")

    def service_deconfliction_cb(self,request):
        rospy.loginfo("New deconfliction request received:")
        response = DeconflictionResponse()
        response.success = True
        self._deconflictionreq = request # DeconflictionRequest()       
        print(self._deconflictionreq.threat)
        return response     
    
''' The node and the TacticalDeconfliction class are initialized'''

if __name__=='__main__':
    
    rospy.init_node('tactical_deconfliction')
    t = TacticalDeconfliction()
    rospy.spin()


