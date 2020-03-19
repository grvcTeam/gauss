#!/usr/bin/env python
import rospy
import time
from os import system
from gauss_msgs.srv import Threats, ThreatsRequest
from gauss_msgs.msg import Threat
#from gauss_msgs.srv import Notification, NotificationRequest, NotificationResponse

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

def main():

    rospy.init_node('usp_node')
    rospy.wait_for_service('threats') # Wait for this service to be running.
    threat_service = rospy.ServiceProxy('threats', Threats) # Create the connection to the service. 

    while not rospy.is_shutdown():
        
        (alerted_uas, alert_flaged) = main_menu()
        
        alert_threat = threat_client(alerted_uas, alert_flaged) # alert_threat would be ThreatsRequest()
        
        result = threat_service(alert_threat)
        
        print(result)
        
        time.sleep(0.1)
    
if __name__ == '__main__':
    main()
#action_notification = NotificationRequest()

#def send_notification(action_notification):
#    uas_notified = action_notification.uas_ids # This is a list.
#    action_id = action_notification.action_id # This is a integer.
#    notification = action_notification.description # This is a string.
#   new_trajectory = action_notification.waypoints 
    #print(uas_notified)
    #print(action_id)
#    print(notification)
#    return notification
#    print(new_trajectory)

#def notification_callback(request):
#    rospy.loginfo("New notification received:") 
#    response = NotificationResponse() # We create the variable which contains the Response.
#    response.success = True
#    notification = send_notification(request) 
#    print(notification)
#    return response

#rospy.init_node('usp_node', anonymous=True) # we iniciate the node.

#def threat_usp_client(uas_ids, alert_flaged):
#    rospy.wait_for_service('threats')
#    try:
#        client = rospy.ServiceProxy('threats', Threats)
        
#        request = ThreatsRequest()
#        request.uas_ids = uas_ids # This is a list.
#        request.threats = alert_flaged # This is a list of messages type Threat.msg
        
#        response = client(request)
        
#    except rospy.ServiceException, e:
#        print "Service call failed: %s"%e
# It is offered a service to notify if there is a new threat.

#s = rospy.Service('notification', Notification, notification_callback) # it is initiated this service.

#alert_flaged = []
#alert = Threat()
#alert.threat_id = 10
#alert.uas_ids = [1]
#reference_time = rospy.Time.now()
#alert.times = [reference_time, reference_time + rospy.Duration(2,0)]
#alert_flaged.append(alert)

#threat_usp_client(alert.uas_ids, alert_flaged)
#print(alert)

# Simply keeps python from exiting until this node is stopped
#rospy.spin()