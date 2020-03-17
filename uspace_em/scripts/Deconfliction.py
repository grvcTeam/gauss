#!/usr/bin/env python
import rospy
from gauss_msgs.srv import Deconfliction, DeconflictionRequest, DeconflictionResponse

conflict_information = DeconflictionRequest()

def trajectory_generator(conflict_information):
    uas_in_conflict = conflict_information.uas_ids # This is a list.
    deconfliction_plans = [36.6, 78.5, 40.5, rospy.Time.now()]
    return deconfliction_plans

def deconfliction_callback(request):
    rospy.loginfo("New 4D trajectories deconfliction calculated:") 
    response = DeconflictionResponse() # We create the variable which contains the Response.
    response.success = True
    print(response.success)
    print("Hello")
    uas_in_conflict = request.uas_ids
    response.deconflicted_plans = trajectory_generator(request) 
    deconflicted_plans = response.deconflicted_plans
    return response

rospy.init_node('deconfliction_node', anonymous=True) # we iniciate the node.

# It is offered a service to generate 4D trajectories free of conflicts.

s = rospy.Service('deconfliction', Deconfliction, deconfliction_callback) # it is initiated this service.

# Simply keeps python from exiting until this node is stopped
rospy.spin()