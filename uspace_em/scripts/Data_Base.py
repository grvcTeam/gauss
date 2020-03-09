#!/usr/bin/env python
import rospy
import json
from gauss_msgs.msg import WaypointList
from gauss_msgs.msg import Waypoint
from gauss_msgs.srv import UasInfo, UasInfoRequest, UasInfoResponse

#We open the json files and store the info in variables.
with open('/home/carcapfer/catkin_ws/src/gauss/gauss_msgs/msg/json/uas_info.json') as file:
        full_uas_info = json.load(file)

def handle_uas_info(uas_id):
    rospy.loginfo("Full UAS info stored in the full_uas_info variable") 
     
    response = UasInfoResponse() # Do create a variable which contains the fields of the response
    response.success = True
    response.uas_id = full_uas_info['uas_id']
    response.frame = full_uas_info['frame']
    response.range = full_uas_info['range']
    response.priority = full_uas_info['priority']
    response.fg = full_uas_info['fp']
    response.pose = full_uas_info['pose']
    response.fg = full_uas_info['fg']
    response.conop = full_uas_info['conop']
    response.conformance = full_uas_info['conformance']
    #print(response.conop)
    return response

rospy.init_node('data_base_node', anonymous=True) # it is initiated the node.

#data_base_node node offers two services to share the Full UAS info and the UAS threat info.
s = rospy.Service('uas_info', UasInfo, handle_uas_info) # it is initiated this service.


#rospy.spin()

handle_uas_info(1)



