#!/usr/bin/env python
import rospy
from gauss_msgs.srv import Threats, ThreatsRequest, ThreatsResponse
from gauss_msgs.msg import Threat
from gauss_msgs.srv import Notification, NotificationRequest, NotificationResponse

action_notification = NotificationRequest()

def send_notification(action_notification):
    uas_notified = action_notification.uas_ids # This is a list.
    action_id = action_notification.action_id # This is a integer.
    notification = action_notification.description # This is a string.
#   new_trajectory = action_notification.waypoints 
    #print(uas_notified)
    #print(action_id)
    print(notification)
    return notification
#    print(new_trajectory)

def notification_callback(request):
    rospy.loginfo("New notification received:") 
    response = NotificationResponse() # We create the variable which contains the Response.
    response.success = True
    notification = send_notification(request) 
    print(notification)
    return response

rospy.init_node('usp_node', anonymous=True) # we iniciate the node.

def threat_usp_client(uas_ids, alert_flaged):
    rospy.wait_for_service('threats')
    try:
        client = rospy.ServiceProxy('threats', Threats)
        
        request = ThreatsRequest()
        request.uas_ids = uas_ids # This is a list.
        request.threats = alert_flaged # This is a list of messages type Threat.msg
        
        response = client(request)
        
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
# It is offered a service to notify if there is a new threat.

s = rospy.Service('notification', Notification, notification_callback) # it is initiated this service.

alert_flaged = []
alert = Threat()
alert.threat_id = 10
alert.uas_ids = [1]
reference_time = rospy.Time.now()
alert.times = [reference_time, reference_time + rospy.Duration(2,0)]
alert_flaged.append(alert)

threat_usp_client(alert.uas_ids, alert_flaged)
#print(alert)

# Simply keeps python from exiting until this node is stopped
rospy.spin()