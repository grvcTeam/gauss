#!/usr/bin/env python
import rospy
import time
from os import system
from gauss_msgs.srv import Threats, ThreatsRequest
from gauss_msgs.msg import Threat
from gauss_msgs.msg import Notification

def threat_client(uas_ids, alert_flaged):     
    request = ThreatsRequest()
    request.uas_ids = uas_ids # This is a list.
    request.threats = alert_flaged # This is a list of messages type Threat.  
    response = request
    return response

def alert_definition_menu(alert_id, alerted_uas):
    alert_flaged = []
    alert = Threat()
    alert.threat_id = alert_id
    alert.uas_ids = alerted_uas
    alert_flaged.append(alert)
    return (alerted_uas, alert_flaged)

def main_menu():
    
    print "\nMain menu of the USP manager tester. Please choose the alert to send to Emergency Manager:"
    print "1. Alert warning (fire detected, authorities notification)"
    print "2. Technical failure"
    print "3. Communication failure"
    print "4. Lack of battery"
    print "5. Jamming attack"
    print "6. Spoofing attack"
    
    selected = raw_input(" >> ")
    system("clear")
    if selected == "1":
        (alerted_uas, alert_flaged) = alert_definition_menu(3, [1, 2, 3])
    elif selected == "2":
        (alerted_uas, alert_flaged) = alert_definition_menu(6, [1, 2, 3])
    elif selected == "3":
        (alerted_uas, alert_flaged) = alert_definition_menu(7, [1])
    elif selected == "4":
        (alerted_uas, alert_flaged) = alert_definition_menu(8, [3])
    elif selected == "5":
        (alerted_uas, alert_flaged) = alert_definition_menu(9, [2])
    elif selected == "6":
        (alerted_uas, alert_flaged) = alert_definition_menu(10, [1])
    else:
        system("clear")
        print "Not a valid option."

    return (alerted_uas, alert_flaged)

def notification_callback(data):
    rospy.loginfo("New notification received:") 
    
def main():

    rospy.init_node('usp_node')
    rospy.wait_for_service('threats') # Wait for this service to be running.
    threat_service = rospy.ServiceProxy('threats', Threats) # Create the connection to the service. 
    rospy.Subscriber("notification", Notification, notification_callback)
    
    while not rospy.is_shutdown():
        
        (alerted_uas, alert_flaged) = main_menu()
        
        alert_threat = threat_client(alerted_uas, alert_flaged) # alert_threat would be ThreatsRequest()
        
        result = threat_service(alert_threat)
        
        print(result)
        
        time.sleep(0.1)
    
if __name__ == '__main__':
    main()







