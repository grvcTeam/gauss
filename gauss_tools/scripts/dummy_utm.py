#!/usr/bin/env python
import rospy
import json
import os
import sys
import copy
from gauss_msgs.srv import Threats, ThreatsRequest, ThreatsResponse
from gauss_msgs.srv import ChangeFlightStatus, ChangeFlightStatusRequest, ChangeFlightStatusResponse
from gauss_msgs.srv import Notifications, NotificationsRequest, NotificationsResponse
from gauss_msgs.srv import ReadOperation, ReadOperationRequest, ReadOperationResponse
from gauss_msgs.msg import Notification
from gauss_msgs.msg import Waypoint, WaypointList
from gauss_msgs_mqtt.msg import UTMAlternativeFlightPlan
from gauss_msgs_mqtt.msg import Waypoint as WaypointMQTT


class DummyUMT():

    def __init__(self):
        rospy.init_node('dummy_utm')
        rate = rospy.Rate(0.5)  # hz

        rospy.loginfo("[UTM] Waiting for services...")

        rospy.wait_for_service('/gauss/notifications')
        rospy.wait_for_service('/gauss/read_operation')
        self.notifications_clt = rospy.ServiceProxy(
            '/gauss/notifications', Notifications)
        self.read_operation_clt = rospy.ServiceProxy(
            '/gauss/read_operation', ReadOperation)
        self.change_flight_status_srv = rospy.Service(
            '/gauss/flight', ChangeFlightStatus, self.changeFlightStatusSrvCb)

        self.timer = rospy.Timer(rospy.Duration(1), self.timerCb)
        self.started_time = rospy.Time.now().to_sec()

        if rospy.has_param('~/operations_json'):
            json_file = self.inputJSONFile(
                rospy.get_param('~/operations_json'))
        else:
            json_file = self.inputJSONFile('')

        # Get data from json file
        self.json_data_ = json.load(open(json_file))

        # Dictionary to store threat types
        self.threat_types_dictionary_ = {0: 'Spoofing attack', 1: 'Technical failure', 2: 'Communication failure',
                                         3: 'Jamming attack', 4: 'Loss of separation', 5: 'Geofence intrusion',
                                         6: 'UAS out of OV', 7: 'Geofence conflict', 8: 'Alert warning',
                                         9: 'Lack of battery', 10: 'GNSS degradation', 11: 'UAS in CV'}

        rospy.loginfo("[UTM] Ready!")

    def inputJSONFile(self, file_name):
        # Setup to get the json file
        usp_tester_config_directory = os.path.abspath(os.path.join(os.path.realpath(
            __file__), os.pardir, os.pardir, os.pardir)) + '/usp_testers/config/'
        # If there is no paramaterer, ask the user for a file
        if not file_name:
            file_name = raw_input(
                "An example of the file path : atlas/loss_separation\nEnter the path of your file : ")

        file_to_check = usp_tester_config_directory + file_name + '_solution.json'
        assert os.path.exists(
            file_to_check), "I did not find the file at, "+str(file_to_check)
        return file_to_check

    def getActualPosition(self, uav_id):
        read_operation_req = ReadOperationRequest()
        read_operation_req.uav_ids.append(uav_id)
        read_operation_res = self.read_operation_clt(read_operation_req)
        return read_operation_res.operation[0].estimated_trajectory.waypoints[0]

    def prepareNotification(self, conflict_id):
        # Prepare notification message
        notification = Notification()
        notification.uav_id = self.json_data_[
            'notifications'][conflict_id]['uav_id']
        notification.threat.threat_id = self.json_data_[
            'conflicts'][conflict_id]['threat_id']
        notification.threat.threat_type = self.json_data_[
            'conflicts'][conflict_id]['threat_type']
        notification.description = self.threat_types_dictionary_[
            self.json_data_['conflicts'][conflict_id]['threat_type']]
        # Get the current position of the UAV and place it in the new flight plan
        notification.new_flight_plan.waypoints.append(
            self.getActualPosition(notification.uav_id))
        for wp in self.json_data_['notifications'][conflict_id]['new_flight_plan']['waypoints']:
            waypoint = Waypoint()
            waypoint.x = wp['x']
            waypoint.y = wp['y']
            waypoint.z = wp['z']
            waypoint.stamp = rospy.Time.from_sec(
                rospy.Time.now().to_sec() + wp['stamp'])  # TODO: Check if this is correct
            notification.new_flight_plan.waypoints.append(waypoint)
        return notification

    def conflictTriggered(self, conflict_id):
        # Print conflict detected
        rospy.loginfo("[UTM] Conflict detected [%d]! %s! UAVs involved " +
                      str(self.json_data_['conflicts'][conflict_id]['uav_id']) + "!", self.json_data_['conflicts'][conflict_id]['threat_id'], self.threat_types_dictionary_[self.json_data_['conflicts'][conflict_id]['threat_type']])

        # Prepare notification
        notifications = NotificationsRequest()
        notifications.notifications.append(
            self.prepareNotification(conflict_id))
        # Send notification (to USPM)
        self.notifications_clt(notifications)
        rospy.loginfo("[UTM] Notification sent to UAV [%d]!",
                      notifications.notifications[0].uav_id)

    # ! First trigger method.
    # ! Wait for this service and trigger the conflict written in the icao variable.
    def changeFlightStatusSrvCb(self, request):
        req = ChangeFlightStatusRequest()
        req = copy.deepcopy(request)
        if req.is_started == True:
            self.conflictTriggered(req.icao)
        res = ChangeFlightStatusResponse()
        res.success = True
        return res

    # ! Second trigger method.
    # ! Check the current time and the trigger stamp written in the json file.
    def timerCb(self, event):
        delta_time = rospy.Time.now().to_sec() - self.started_time
        for conflict_id, conflict in enumerate(self.json_data_['conflicts']):
            if delta_time > self.json_data_['conflicts'][conflict_id]['trigger_stamp'] and self.json_data_['conflicts'][conflict_id]['trigger_stamp'] > 0:
                self.conflictTriggered(conflict_id)
                # Conflict solved! Do not trigger again.
                self.json_data_['conflicts'][conflict_id]['trigger_stamp'] = -1


if __name__ == '__main__':
    utm = DummyUMT()
    rospy.spin()
