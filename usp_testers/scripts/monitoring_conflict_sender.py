#!/usr/bin/env python
# this Python file uses the following encoding: utf-8
'''This script is a monitoring simulator developed to send conflicts one by one'''
import rospy
import time
from os import system
from gauss_msgs.srv import Threats, ThreatsRequest
from gauss_msgs.msg import Threat

''' It is defined a class with all functionabilities'''

class Monitoring():
    
    # This is the constructor class.

    def __init__(self): 
        
        # Wait until service is available and creat connection
        
        rospy.wait_for_service('/gauss/threats')         
        self._threats_service = rospy.ServiceProxy('/gauss/threats', Threats) 
                              
    # This method sends a subscrition to the threats.srv service.

    def send_threats(self): 
        request = ThreatsRequest()
        request.uav_ids = self._conflicted_uas
        request.threats = self._conflict_flaged   
        response = self._threats_service(request)        
        return response 
    
    # This method defines the conflict configuration.
    
    def conflict_definition_menu(self, conflict_id): 
        self._conflict_flaged = []
        conflict = Threat()
        conflict.threat_type = conflict_id
        
        
        if conflict_id == Threat.UAS_IN_CV:
            self._conflicted_uas = [0]
            self._conflict_id = Threat.UAS_IN_CV
        if conflict_id == Threat.UAS_OUT_OV:
            self._conflicted_uas = [0]
            self._conflict_id = Threat.UAS_OUT_OV
        if conflict_id == Threat.LOSS_OF_SEPARATION:
            self._conflicted_uas = [0, 1]
            self._conflict_id = Threat.LOSS_OF_SEPARATION
        if conflict_id == Threat.GEOFENCE_INTRUSION:
            self._conflicted_uas = [0]
            self._conflict_id = Threat.GEOFENCE_INTRUSION
            conflict.geofence_ids =[1]
        if conflict_id == Threat.GEOFENCE_CONFLICT:
            self._conflicted_uas = [0]
            self._conflict_id = Threat.GEOFENCE_CONFLICT
            conflict.geofence_ids =[0]
        conflict.uav_ids = self._conflicted_uas
        self._conflict_flaged.append(conflict)   
    # This method is an HMI in order to check different conflicts configurations.
        
    def main_menu(self):

        print "\nMain menu of the Monitoring tester. Please choose the conflict to send to Emergency Manager:"
        print "1. UAS in the Contingency Volume"
        print "2. UAS out of the Operational Volume"
        print "3. Loss of separation between UAS"
        print "4. Geofence intrusion"
        print "5. Geofence conflict"
        
        selected = raw_input(" >> ")
        system("clear")
        if selected == "1":
            self.conflict_definition_menu(Threat.UAS_IN_CV)
        elif selected == "2":
            self.conflict_definition_menu(Threat.UAS_OUT_OV)
        elif selected == "3":
            self.conflict_definition_menu(Threat.LOSS_OF_SEPARATION)
        elif selected == "4":
            self.conflict_definition_menu(Threat.GEOFENCE_INTRUSION)
        elif selected == "5":
            self.conflict_definition_menu(Threat.GEOFENCE_CONFLICT)
        else:
            system("clear")
            print "Not a valid option."

''' The node and the Monitoring class are initialized'''

if __name__=='__main__':
    
    rospy.init_node('monitoring')
    m = Monitoring()
    
    while not rospy.is_shutdown():
    
    #We fill the request.

        m.main_menu()      
        m.send_threats()
        
        time.sleep(0.1)

