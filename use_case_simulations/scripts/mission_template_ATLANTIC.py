#!/usr/bin/env python

'''This script is developed to define and load a mission in a fixed wing UAV using the
ual_backend_mavros_fw. Firstly, it has to be executed roslaunch ual_backend_mavros_fw simulations.launch'''

import rospy, std_msgs, std_srvs
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from uav_abstraction_layer.srv import SetMission, SetMissionRequest, SetMissionResponse
from uav_abstraction_layer.msg import MissionElement, ParamFloat

class MissionLoader():
    def __init__(self):
       
        # Wait until service is available and creat connection
        
        rospy.wait_for_service('/ATLANTIC/ual/set_mission')         
        self._setmission_service = rospy.ServiceProxy('/ATLANTIC/ual/set_mission', SetMission) 
        
        mission_wps = self.define_mission()
        self.send_mission(mission_wps)
    
    def define_mission(self): 
        wps = []
        header_map = std_msgs.msg.Header()
        header_map.frame_id = "map"

        '''TAKE OFF POSE'''
        take_off_phase = MissionElement()
        take_off_phase.type = MissionElement.TAKEOFF_POSE
        take_off_phase.waypoints = [PoseStamped(header_map,Pose(Point(100,0,10),Quaternion(0,0,0,1)))]        
        take_off_phase.params = self.dictToListOfParamFloat({"minimum_pitch": 0.0})
        wps.append(take_off_phase)

        '''PASS'''
        pass_phase = MissionElement()
        pass_phase.type = MissionElement.PASS
        pass_phase.waypoints = [PoseStamped(header_map,Pose(Point(100,150,80),Quaternion(0,0,0,1)))]
        pass_phase.waypoints.append(PoseStamped(header_map,Pose(Point(200,0,80),Quaternion(0,0,0,1))))
        pass_phase.waypoints.append(PoseStamped(header_map,Pose(Point(100,-150,80),Quaternion(0,0,0,1))))
        pass_phase.params = self.dictToListOfParamFloat({"acceptance_radius": 10.0,"orbit_distance": 0.0, "speed" : 10.0})
        wps.append(pass_phase)

        '''LOITER HEIGHT'''
        loiter_height_phase = MissionElement()
        loiter_height_phase.type = MissionElement.LOITER_HEIGHT
        loiter_height_phase.waypoints = [PoseStamped(header_map,Pose(Point(100,-150,80),Quaternion(0,0,0,1)))]
        loiter_height_phase.waypoints.append(PoseStamped(header_map,Pose(Point(100,-150,30),Quaternion(0,0,0,1))))     
        loiter_height_phase.params = self.dictToListOfParamFloat({"heading": 0.0,"radius": 10.0, "forward_moving" : 1.0})
        wps.append(loiter_height_phase)

        '''LAND POSE'''
        landing_phase = MissionElement()
        landing_phase.type = MissionElement.LAND_POSE
        landing_phase.waypoints = [PoseStamped(header_map,Pose(Point(50,-50,10),Quaternion(0,0,0,1)))]
        landing_phase.waypoints.append(PoseStamped(header_map,Pose(Point(110,20,0),Quaternion(0,0,0,1))))
        landing_phase.params = self.dictToListOfParamFloat({"loit_heading": 0.0, "loit_radius": 0.0,
                                                       "loit_forward_moving": 1.0,"abort_alt": 0.0, "precision_mode": 0.0})
        wps.append(landing_phase)

        return wps

    def send_mission(self, wps):
        request = SetMissionRequest()
        request.mission_elements = wps
        request.blocking = True
        response = self._setmission_service(request)        
        return response 

    def dictToListOfParamFloat(self, dict):

        paramfloat_list = []
        for key in dict.keys():
            paramfloat_list.append(ParamFloat(key, dict[key]))
        return paramfloat_list

''' The node and the MissionLoader class are initialized'''

if __name__=='__main__':

    rospy.init_node('mission_loader', anonymous=True)
    m = MissionLoader()