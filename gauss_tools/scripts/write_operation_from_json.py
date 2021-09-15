#!/usr/bin/env python
import rospy
import json
import os
import sys
import copy
# from gauss_msgs.srv import ChangeFlightStatus, ChangeFlightStatusRequest, ChangeFlightStatusResponse
from gauss_msgs.srv import WriteOperation, WriteOperationRequest, WriteOperationResponse
from gauss_msgs.msg import Operation
from gauss_msgs.msg import Waypoint, WaypointList

# TODO: from console parameter
json_filename = 'test.json'  #'../../usp_nodes/db_manager/config/aernosillo/geofence_conflict.json'

class OperationWriter():
    def __init__(self):
        rospy.init_node('operation_writer')
        rospy.loginfo("[UTM] Writing operation...")
        rospy.wait_for_service('/gauss/write_operation')
        self.write_operation_clt = rospy.ServiceProxy('/gauss/write_operation', WriteOperation)
        json_filename = sys.argv[1]
        # Setup to get the json file
        usp_tester_config_directory = os.path.abspath(os.path.join(os.path.realpath(
            __file__), os.pardir, os.pardir, os.pardir)) + '/usp_testers/config/'
        json_filename = usp_tester_config_directory + json_filename + ".json"
        # Get data from json file
        assert os.path.exists(json_filename), "I did not find the file at, " + str(json_filename)
        self.json_data_ = json.load(open(json_filename))
        # Get the started time
        # self.started_time = rospy.Time.from_sec(float(sys.argv[2]))
        self.started_time = rospy.Time.now().to_sec() # ! This node should be initialized at the same time as dummy_utm. Maybe get this parameter from console like json name.
        self.send_operation()
        rospy.loginfo("[UTM] Operation written.")


    def createWaypointList(self, _name_wp_list):
        wp_list = WaypointList()
        for wp in self.json_data_[_name_wp_list]['waypoints']:
            waypoint = Waypoint()
            waypoint.mandatory = wp['mandatory']
            waypoint.stamp = rospy.Time.from_sec(self.started_time + wp['stamp'])
            waypoint.x = wp['x']
            waypoint.y = wp['y']
            waypoint.z = wp['z']
            wp_list.waypoints.append(waypoint)
        return wp_list

    def send_operation(self):
        # Prepare notification message
        operation = Operation()
        operation.uav_id = self.json_data_['uav_id']
        operation.icao_address = str(self.json_data_['icao_address'])
        operation.frame = 0 # ! Error: field must be unsigned integer type
        operation.autonomy = self.json_data_['autonomy']
        operation.priority = self.json_data_['priority']
        operation.is_started = self.json_data_['is_started']
        operation.current_wp = self.json_data_['current_wp']
        operation.flight_plan = self.createWaypointList('flight_plan')
        operation.dT = self.json_data_['dT']
        operation.time_tracked = self.json_data_['time_tracked']
        operation.track = self.createWaypointList('track')
        operation.time_horizon = self.json_data_['time_horizon']
        operation.estimated_trajectory = self.createWaypointList('estimated_trajectory')
        operation.landing_spots = self.createWaypointList('landing_spots')
        operation.flight_geometry = self.json_data_['flight_geometry']
        operation.operational_volume = self.json_data_['operational_volume']
        operation.conop = str(self.json_data_['conop'])
        # Send operation
        write_operation_req = WriteOperationRequest()
        write_operation_req.operation.append(operation)
        write_operation_req.uav_ids = [operation.uav_id]
        self.write_operation_clt(write_operation_req)

if __name__ == '__main__':
    op_writer = OperationWriter()
    rospy.spin()
















