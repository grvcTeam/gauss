#!/usr/bin/env python
import rospy
import json
import os
import sys
import copy
from gauss_msgs.srv import Threats, ThreatsRequest, ThreatsResponse
from gauss_msgs.srv import ChangeFlightStatus, ChangeFlightStatusRequest, ChangeFlightStatusResponse
from gauss_msgs.srv import Notifications, NotificationsRequest, NotificationsResponse
from gauss_msgs.msg import Notification
from gauss_msgs.msg import Waypoint, WaypointList
from gauss_msgs_mqtt.msg import UTMAlternativeFlightPlan
from gauss_msgs_mqtt.msg import Waypoint as WaypointMQTT

class DummyUMT():

    def __init__(self):
        rospy.init_node('dummy_utm')
        rate = rospy.Rate(0.5) # hz

        self.notifications_clt = rospy.ServiceProxy('/gauss/notifications', Notifications)
        self.threats_srv = rospy.Service('/gauss/threats', Threats, self.threatsSrvCb) 
        self.change_flight_status_srv = rospy.Service('/gauss/flight', ChangeFlightStatus, self.changeFlightStatusSrvCb)

        if rospy.has_param('~/operations_json'):
            self.json_file_ = self.inputJSONFile(rospy.get_param('~/operations_json'))
        else:
            self.json_file_ = self.inputJSONFile('')

        rospy.loginfo("[UTM] Ready to receive threats!")

    def inputJSONFile(self, file_name):
        # Setup to get the json file
        usp_tester_config_directory = os.path.abspath(os.path.join(os.path.realpath(__file__), os.pardir, os.pardir, os.pardir)) + '/usp_testers/config/'
        # If there is no paramaterer, ask the user for a file
        if not file_name:
            file_name = raw_input("An example of the file path : atlas/loss_separation\nEnter the path of your file : ")

        file_to_check = usp_tester_config_directory + file_name + '_solution.json'
        assert os.path.exists(file_to_check), "I did not find the file at, "+str(file_to_check)
        return file_to_check

    def prepareNotification(self, json_data):
        notification = Notification()
        notification.uav_id = json_data['operations'][0]['uav_id']
        notification.threat.threat_id = json_data['operations'][0]['threat_id']
        notification.threat.threat_type = json_data['operations'][0]['threat_type']
        notification.description = json_data['operations'][0]['description']
        for wp in json_data['operations'][0]['new_flight_plan']['waypoints']:
            waypoint = Waypoint()
            waypoint.x = wp['x']
            waypoint.y = wp['y']
            waypoint.z = wp['z']
            waypoint.stamp = rospy.Time.from_sec(rospy.Time.now().to_sec() + wp['stamp'])
            notification.new_flight_plan.waypoints.append(waypoint)
        return notification

    def conflictTriggered(self):
        # Get data from json file
        json_data = json.load(open(self.json_file_))
        # Prepare notification
        notifications = NotificationsRequest()
        notifications.notifications.append(self.prepareNotification(json_data))
        # Send notification (to USPM)
        self.notifications_clt(notifications)

    # TODO: This should be replaced by threatSrvCb if we want a bit more flexibility
    # TODO: Or just leave it as it is and use the json file for more configuration
    def changeFlightStatusSrvCb(self, request):
        req = ChangeFlightStatusRequest()
        req = copy.deepcopy(request)
        
        if req.is_started == True:
            self.conflictTriggered()
        
        res = ChangeFlightStatusResponse()
        res.success = True
        return res

    def threatsSrvCb(self, request):
        req = ThreatsRequest()
        req = copy.deepcopy(request)
        rospy.loginfo("[UTM] %d threats received!", len(req.threats)) 
        zero_time = rospy.Time()
        for threat in range(len(req.threats)):
            print(threat)
        res = ThreatsResponse()
        res.success = True
        return res 

if __name__ == '__main__':
    utm = DummyUMT()
    rospy.spin()