#!/usr/bin/env python
# this Python file uses the following encoding: utf-8
import rospy
import time
from gauss_msgs.srv import Notifications, NotificationsRequest
from gauss_msgs.msg import Notification 

class NotificationSender():
    
    def __init__(self): 
        
        # Initialization
        
        

        # Wait until services are available and create connection

        rospy.wait_for_service('/gauss/notifications')                    
        self._notifications_service_handle = rospy.ServiceProxy('/gauss/notifications', Notifications)

        print("Started Notification sender module!")
    
    def send_notifications(self,notifications): 
        request = NotificationsRequest()
        request.notifications = notifications 
        self._notifications_response = self._notifications_service_handle(request) 
        print(self._notifications_response)
        return self._notifications_response

    def create_notifications(self):
        notifications_list = []
        notification_1 = Notification()
        notification_1.description = "Aterriza"
        notifications_list.append(notification_1)
        notification_2 = Notification()
        notification_2.description = "Vuelve a casa"
        notifications_list.append(notification_2) 
        self.send_notifications(notifications_list)

    ''' The node and the NotificationSender class are initialized'''

if __name__=='__main__':

    rospy.init_node('notifications_sender')
    n = NotificationSender()
    n.create_notifications()
    rospy.spin()