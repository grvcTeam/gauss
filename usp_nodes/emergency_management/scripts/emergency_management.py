#!/usr/bin/env python
'''This script is the emergency manager node developed to decide what is the best action 
to take in the U-space when some Threats are showed up.'''
import os, sys
import rospy
from math import sqrt
from gauss_msgs.srv import Threats, ThreatsResponse
from gauss_msgs.srv import ReadOperation, ReadOperationRequest
from gauss_msgs.srv import WriteFlightPlan, WriteFlightPlanRequest
from gauss_msgs.srv import WriteGeofences, WriteGeofencesRequest
from gauss_msgs.msg import Threat
#from gauss_msgs.msg import Notification
from gauss_msgs.srv import Deconfliction, DeconflictionRequest

''' It is defined a class with all functionabilities'''

class EmergencyManagement():

    def __init__(self): 
        
        # Initialization

        # Publish
        self._notification_publisher = rospy.Publisher("notification", Notification, queue_size=1)
        
        # Wait until service is available and creat connection
        
        rospy.wait_for_service('/gauss/deconfliction')                    
        self._readOperation_service_handle = rospy.ServiceProxy('/gauss/deconfliction', Deconfliction) 

        rospy.wait_for_service('/gauss/read_operation')                    
        self._readOperation_service_handle = rospy.ServiceProxy('/gauss/read_operation', ReadOperation) 

        rospy.wait_for_service('/gauss/write_flight_plan')                    
        self._writeFlightPlan_service_handle = rospy.ServiceProxy('/gauss/write_flight_plan', WriteFlightPlan) 

        rospy.wait_for_service('/gauss/write_geofences')                    
        self._writeGeofences_service_handle = rospy.ServiceProxy('/gauss/write_geofences', WriteGeofences) 

        # Server     
        self._threats_service = rospy.Service('/gauss/threats', self.service_threats_cb) 

    def service_threats_cb(self, request):
        rospy.loginfo("New threat received:") 
        response = ThreatsResponse()
        response.success = True
        threat2solve = request # ThreatRequest
        action, action_id = action_decision_maker(threat2solve)
        #global gpub
        #notification = Notification()
        #notification.description = action
        #notification.action_id = action_id
        #gpub.publish(notification)
        return response

    def send_notification(self):
        pass
    
    def create_geofence(self):
        pass

    def request_new_trajectory(self):
        pass
    
    def read_operations(self):
        pass

    def action_decision_maker(self):
        pass

    def write_new_flight_plan(self):
        pass

    def estimate_threat_severity(self):
        pass

    def asignate_threat_severity(self):
        pass

    def calculate_landingspot(self):
        pass

''' The node and the Emergency Management class are initialized'''

if __name__== '__main__':
    
    rospy.init_node('emergency_management')
    e = EmergencyManagement()
    
    while not rospy.is_shutdown():
    
        m.main_menu()      
        m.send_threats()
        
        time.sleep(0.1)

