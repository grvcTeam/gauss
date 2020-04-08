#!/usr/bin/env python
'''This script is a usp manager simulator developed to send alerts one by one'''
import rospy
import time
from os import system
from gauss_msgs.srv import Threats, ThreatsRequest
from gauss_msgs.msg import Threat

''' It is defined a class with all functionabilities'''

class UspManager():
    
    # This is the constructor class.

    def __init__(self): 
        
        # Wait until service is available and creat connection
        
        rospy.wait_for_service('threats')         
        self._threats_service = rospy.ServiceProxy('threats', Threats) 
                              
    # This method sends a subscrition to the threats.srv service.

    def send_threats(self): 
        request = ThreatsRequest()
        request.uav_ids = self._alerted_uas
        request.threats = self._alert_flaged   
        response = self._threats_service(request)        
        return response 
    
    # This method defines the alert configuration.
    
    def alert_definition_menu(self, alert_id): 
        self._alert_flaged = []
        alert = Threat()
        alert.threat_id = alert_id
        
        if alert_id == Threat.ALERT_WARNING:
            self._alerted_uas = [1, 2, 3]
            self._alert_id = Threat.ALERT_WARNING
        if alert_id == Threat.TECHNICAL_FAILURE:
            self._alerted_uas = [1, 2, 3]
            self._alert_id = Threat.TECHNICAL_FAILURE
        if alert_id == Threat.COMMUNICATION_FAILURE:
            self._alerted_uas = [1]
            self._alert_id = Threat.COMMUNICATION_FAILURE
        if alert_id == Threat.LACK_OF_BATTERY:
            self._alerted_uas = [3]
            self._alert_id = Threat.LACK_OF_BATTERY
        if alert_id == Threat.JAMMING_ATTACK:
            self._alerted_uas = [2]
            self._alert_id = Threat.JAMMING_ATTACK
        if alert_id == Threat.SPOOFING_ATTACK:
            self._alerted_uas = [1]
            self._alert_id = Threat.SPOOFING_ATTACK
        alert.uav_ids = self._alerted_uas
        self._alert_flaged.append(alert)  

    # This method is an HMI in order to check different conflicts configurations.
        
    def main_menu(self):

        print "\nMain menu of the USP Manager Alert sender. Please choose the conflict to send to Emergency Manager:"
        print "1. Alert warning (fire detected, authorities notification)"
        print "2. Technical failure"
        print "3. Communication failure"
        print "4. Lack of battery"
        print "5. Jamming attack"
        print "6. Spoofing attack"
        
        selected = raw_input(" >> ")
        system("clear")
        if selected == "1":
            self.alert_definition_menu(Threat.ALERT_WARNING)
        elif selected == "2":
            self.alert_definition_menu(Threat.TECHNICAL_FAILURE)
        elif selected == "3":
            self.alert_definition_menu(Threat.COMMUNICATION_FAILURE)
        elif selected == "4":
            self.alert_definition_menu(Threat.LACK_OF_BATTERY)
        elif selected == "5":
            self.alert_definition_menu(Threat.JAMMING_ATTACK)
        elif selected == "6":
            self.alert_definition_menu(Threat.SPOOFING_ATTACK)
        else:
            system("clear")
            print "Not a valid option."

''' The node and the UspManager class are initialized'''

if __name__=='__main__':
    
    rospy.init_node('usp_manager')
    m = UspManager()
    
    while not rospy.is_shutdown():
    
    #We fill the request.

        m.main_menu()      
        m.send_threats()
        
        time.sleep(0.1)
