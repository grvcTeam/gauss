#!/usr/bin/env python
# This Python file uses the following encoding: utf-8

'''This script is developed to define and load a mission in a fixed wing UAV using the
ual_backend_mavros_fw. Firstly, it has to be executed roslaunch ual_backend_mavros_fw simulations.launch'''

import rospy, std_msgs, std_srvs
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from uav_abstraction_layer.srv import SetMission, SetMissionRequest, SetMissionResponse
from uav_abstraction_layer.msg import MissionElement, ParamFloat
from gauss_msgs.msg import Notification, Waypoint, WaypointList

class MissionLoader():
    def __init__(self):
       
        # Subscriber

        self._notification_subscriber = rospy.Subscriber("notification", Notification, self.uav_notification_cb, queue_size=1)

        # Wait until service is available and creat connection
        
        rospy.wait_for_service('/ual/set_mission')         
        self._setmission_service = rospy.ServiceProxy('/ual/set_mission', SetMission) 

        self._uav_notification = Notification()

        #self._min_pitch = rospy.get_param("~minimun pitch", 0.0) 
        #self._accept_radius = rospy.get_param("~acceptance radius", 10.0) 
        #self._orbit_distance = rospy.get_param("~orbit distance", 0.0) 
        #self._speed = rospy.get_param("~speed", 10.0) 
        #self._heading = rospy.get_param("~heading", 0.0) 
        #self._radius = rospy.get_param("~radius", 10.0) 
        #self._forward_moving = rospy.get_param("~forward moving", 1.0) 
        #self._loit_heading = rospy.get_param("~loiter heading", 0.0) 
        #self._loit_radius = rospy.get_param("~loiter radius", 0.0) 
        #self._loit_forward_moving = rospy.get_param("~loiter forward moving", 1.0) 
        #self._abort_alt = rospy.get_param("~abort alt", 0.0) 
        #self._precision_mode = rospy.get_param("~precision mode", 0.0)
                    
    #Lee los wps de la notificación y los guarda en una variable global.
    def uav_notification_cb(self, message):
        self._uav_notification_wps = message.waypoints
        self._mission_wps = [] # Lista de PoseStamped
        mission_wp0 = PoseStamped()      
        header_map = std_msgs.msg.Header()
        header_map.frame_id = "map"
        header_map.stamp = self._uav_notification_wps[0].stamp
        mission_wp0.pose = Pose()
        mission_wp0.pose.position = Point()
        mission_wp0.pose.position.x = self._uav_notification_wps[0].x
        mission_wp0.pose.position.y = self._uav_notification_wps[0].y
        mission_wp0.pose.position.z = self._uav_notification_wps[0].z
        mission_wp0.pose.orientation.x = 0
        mission_wp0.pose.orientation.y = 0
        mission_wp0.pose.orientation.z = 0
        mission_wp0.pose.orientation.w = 1
        self._mission_wps.append(mission_wp0)
        
        mission_wp1 = PoseStamped()      
        header_map = std_msgs.msg.Header()
        header_map.frame_id = "map"
        header_map.stamp = self._uav_notification_wps[1].stamp
        mission_wp1.pose = Pose()
        mission_wp1.pose.position = Point()
        mission_wp1.pose.position.x = self._uav_notification_wps[1].x
        mission_wp1.pose.position.y = self._uav_notification_wps[1].y
        mission_wp1.pose.position.z = self._uav_notification_wps[1].z
        mission_wp1.pose.orientation.x = 0
        mission_wp1.pose.orientation.y = 0
        mission_wp1.pose.orientation.z = 0
        mission_wp1.pose.orientation.w = 1
        self._mission_wps.append(mission_wp1)
         
        wps = []
        #header_map = std_msgs.msg.Header()
        #header_map.frame_id = "map"
        '''PASS'''
        pass_phase = MissionElement()
        pass_phase.type = MissionElement.PASS
        new_path = self._mission_wps
        pass_phase.waypoints = new_path
        #pass_phase.waypoints = [PoseStamped(header_map,Pose(Point(self._uav_notification_wps[0].x, self._uav_notification_wps[0].y,self._uav_notification_wps[0].z),Quaternion(0,0,0,1)))]
        #pass_phase.waypoints.append(PoseStamped(header_map,Pose(Point(self._uav_notification_wps[1].x,self._uav_notification_wps[1].y,self._uav_notification_wps[1].z),Quaternion(0,0,0,1))))
        pass_phase.params = self.dictToListOfParamFloat({"acceptance_radius": 10.0,"orbit_distance": 0.0, "speed" : 10.0})
        wps.append(pass_phase)

        #'''LAND POSE'''
        #landing_phase = MissionElement()
        #landing_phase.type = MissionElement.LAND_POSE
        #landing_phase.waypoints = [PoseStamped(header_map,Pose(Point(0,0,0),Quaternion(0,0,0,1)))]
        #landing_phase.params = self.dictToListOfParamFloat({"loit_heading": 0.0, "loit_radius": 0.0,
        #                                               "loit_forward_moving": 1.0,"abort_alt": 0.0, "precision_mode": 0.0})
        #wps.append(landing_phase)

        request = SetMissionRequest()
        request.mission_elements = wps
        request.blocking = True
        self._setmission_service(request) 

        return wps
        
          

    def dictToListOfParamFloat(self, dict):

        paramfloat_list = []
        for key in dict.keys():
            paramfloat_list.append(ParamFloat(key, dict[key]))
        return paramfloat_list

''' The node and the MissionLoader class are initialized'''

if __name__=='__main__':

    rospy.init_node('mission_loader', anonymous=True)
    m = MissionLoader()
    #mission_wps = m.define_mission()
    #m.send_mission(mission_wps)
    rospy.spin() 