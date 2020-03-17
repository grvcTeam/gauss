#!/usr/bin/env python
import rospy
from gauss_msgs.srv import Threats, ThreatsRequest, ThreatsResponse
from gauss_msgs.msg import Threat

def threat_monitoring_client(uas_ids, conflict_flaged):
    rospy.wait_for_service('threats')
    try:
        client = rospy.ServiceProxy('threats', Threats)
        
        request = ThreatsRequest()
        request.uas_ids = uas_ids # This is a list.
        request.threats = conflict_flaged # This is a list of messages type Threat.  

        response = client(request)
        
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

rospy.init_node('monitoring_node', anonymous=True) # we initialize the node

conflict_flaged = []
conflict = Threat()
conflict.threat_id = 0
conflict.uas_ids = [1]
reference_time = rospy.Time.now()
conflict.times = [reference_time, reference_time + rospy.Duration(1,0)]
conflict_flaged.append (conflict)

threat_monitoring_client(conflict.uas_ids, conflict_flaged)
#print(conflict)

# Simply keeps python from exiting until this node is stopped
rospy.spin()