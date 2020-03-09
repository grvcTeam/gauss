#!/usr/bin/env python
import rospy
from gauss_msgs.srv import Threats, ThreatsRequest, ThreatsResponse
from gauss_msgs.msg import Threat

def threat_usp_client(uas_ids, alert_flaged):
    rospy.wait_for_service('threats')
    try:
        client = rospy.ServiceProxy('threats', Threats)
        
        request = ThreatsRequest()
        request.uas_ids = uas_ids # This is a list.
        request.threats_flaged = alert_flaged # This is a message type Threat.msg

        response = client(request)
        
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

alert_flaged = []
alert = Threat()
alert.threat_id = 3
alert.uas_ids = [1]
alert.wp_ids = [2]
alert_flaged.append(alert)

rospy.init_node('usp_node', anonymous=True) # we iniciate the node.

threat_usp_client(alert.uas_ids, alert_flaged)
print(alert)