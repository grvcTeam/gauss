#!/usr/bin/env python
# this Python file uses the following encoding: utf-8
'''This script is a usp manager simulator developed to send alerts one by one'''
import rospy
#import time
import copy
#from os import system
from gauss_msgs.srv import PilotAnswer, PilotAnswerRequest 
#from gauss_msgs.srv import Threats, ThreatsRequest
from gauss_msgs.srv import Notifications, NotificationsRequest, NotificationsResponse
#from gauss_msgs.msg import Threat, Waypoint
from gauss_msgs.msg import Notification, Threat, Waypoint 

''' It is defined a class with all functionabilities'''

class UspManager():

    def __init__(self): 
        
        # Initialization
        
        self._notification_received = False

        #self._notifications_list = []

        # Server

        self._notifications_service = rospy.Service('/gauss/notifications', Notifications, self.service_notifications_cb)
        
        # Wait until service is available and creat connection
        
        #rospy.wait_for_service('/gauss/threats')         
        #self._threats_service = rospy.ServiceProxy('/gauss/threats', Threats)
        
        rospy.wait_for_service('/gauss/pilotanswer')
        self._pilot_answers_service = rospy.ServiceProxy('/gauss/pilotanswer', PilotAnswer) 

        # Reference time

        #self._reference_time = rospy.Time.now()

        print("Started USP module!. This module receives notifications and gives answers from the pilots.")

    # This method sends a subscrition to the threats.srv service.

    # def send_threats(self): 
    #    request = ThreatsRequest()
    #    request.uav_ids = self._alerted_uas
    #    request.threats = self._alert_flaged   
    #    response = self._threats_service(request)        
    #    return response

    def service_notifications_cb(self, request):
        req = NotificationsRequest()
        req = copy.deepcopy(request)
        num = len(req.notifications)
        print(req.notifications)
        #rospy.loginfo("USP manager has received %d notifications from EM!", num) 
        self._notification_received = True
        res = NotificationsResponse()
        res.success = True
        return res 
    
    def send_answers_1(self):
        rospy.loginfo("USP has sent answers") 
        request = PilotAnswerRequest()
        request.threat_ids = [0]
        request.pilot_answers = ['ACCEPTED']
        response = self._pilot_answers_service(request)
        return response

    # def send_answers_2(self):
    #     rospy.loginfo("USP has sent answers") 
    #     request = PilotAnswerRequest()
    #     request.threat_ids = [1,3,4]
    #     request.pilot_answers = ['ACCEPTED','ACCEPTED','ACCEPTED']
    #     response = self._pilot_answers_service(request)
    #     return response

    # def send_answers_3(self):
    #     rospy.loginfo("USP has sent answers") 
    #     request = PilotAnswerRequest()
    #     request.threat_ids = [7,8,9]
    #     request.pilot_answers = ['ACCEPTED','ACCEPTED','ACCEPTED']
    #     response = self._pilot_answers_service(request)
    #     return response

    # This method defines the alert configuration.
    
    #def alert_definition_menu(self, alert_id): 
    #    self._alert_flaged = []
    #    alert = Threat()
    #    alert.threat_type = alert_id
    #    reference_time = rospy.Time.now()
    #    alert.header.stamp = reference_time
    #    self._location = Waypoint()
        #print(alert.header.stamp)     

    #    if alert_id == Threat.ALERT_WARNING:
    #        self._alerted_uas = [0, 1]
    #        self._location.x = 4
    #        self._location.y = 5
    #        #TODO añadir el tiempo.
    #        self._alert_id = Threat.ALERT_WARNING
            
    #    if alert_id == Threat.TECHNICAL_FAILURE:
    #        self._alerted_uas = [0]
    #        self._alert_id = Threat.TECHNICAL_FAILURE
            
    #    if alert_id == Threat.COMMUNICATION_FAILURE:
    #        self._alerted_uas = [1]
    #        self._alert_id = Threat.COMMUNICATION_FAILURE
            
    #    if alert_id == Threat.LACK_OF_BATTERY:
    #        self._alerted_uas = [0]
    #        self._alert_id = Threat.LACK_OF_BATTERY
            
    #    if alert_id == Threat.JAMMING_ATTACK:
    #        self._alerted_uas = [1]
    #        self._alert_id = Threat.JAMMING_ATTACK
            
    #    if alert_id == Threat.SPOOFING_ATTACK:
    #        self._alerted_uas = [1]
    #        self._alert_id = Threat.SPOOFING_ATTACK
        
    #    if alert_id == Threat.GNSS_DEGRADATION:
    #        self._alerted_uas = [0]
    #        self._alert_id = Threat.GNSS_DEGRADATION
        
    #    alert.uav_ids = self._alerted_uas
    #    alert.location = self._location
    #    self._alert_flaged.append(alert)  
        
    # This method is an HMI in order to check different conflicts configurations.
        
    #def main_menu(self):

    #    print "\nMain menu of the USP Manager Alert sender. Please choose the conflict to send to Emergency Manager:"
    #    print "1. Alert warning (Fire, bad weather or No Drone zones detection)"
    #    print "2. Technical failure"
    #    print "3. Communication failure"
    #    print "4. Lack of battery"
    #    print "5. Jamming attack"
    #    print "6. Spoofing attack"
    #    print "7. GNSS degradation signal"
        
    #    selected = raw_input(" >> ")
    #    system("clear")
    #    if selected == "1":
    #        self.alert_definition_menu(Threat.ALERT_WARNING)
    #    elif selected == "2":
    #        self.alert_definition_menu(Threat.TECHNICAL_FAILURE)
    #    elif selected == "3":
    #        self.alert_definition_menu(Threat.COMMUNICATION_FAILURE)
    #    elif selected == "4":
    #        self.alert_definition_menu(Threat.LACK_OF_BATTERY)
    #    elif selected == "5":
    #        self.alert_definition_menu(Threat.JAMMING_ATTACK)
    #    elif selected == "6":
    #        self.alert_definition_menu(Threat.SPOOFING_ATTACK)
    #    elif selected == "7":
    #        self.alert_definition_menu(Threat.GNSS_DEGRADATION)
    #    else:
    #        system("clear")
    #        print "Not a valid option."

''' The node and the UspManager class are initialized'''

if __name__=='__main__':
    
    rospy.init_node('usp_manager')
    m = UspManager()
    
    rate = rospy.Rate(1) # en Hz.
        
    while not rospy.is_shutdown():
        if m._notification_received:
            rospy.sleep(20) # se duerme cuatro segundos.
            m.send_answers_1() # envio la respuesta.
            # rospy.sleep(20) # se duerme cuatro segundos.
            # m.send_answers_2() # envio la respuesta.
            # rospy.sleep(30) # se duerme cuatro segundos.
            # m.send_answers_3() # envio la respuesta.
        rate.sleep()

     #   m.main_menu()      
      #  m.send_threats()
        
       # time.sleep(0.1)
        