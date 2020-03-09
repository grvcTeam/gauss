#!/usr/bin/env python
import rospy
from gauss_msgs.srv import Threats, ThreatsRequest, ThreatsResponse
from gauss_msgs.msg import Threat

threats_definition = {'0': {'id': 'UAS_IN_CV', 'type': 'conflict', 'severity': 1},
                      '1': {'id': 'UAS_OUT_OV', 'type': 'conflict', 'severity': 2},
                      '2': {'id': 'LOSS_OF_SEPARATION', 'type': 'conflict', 'severity': 2},
                      '3': {'id': 'ALERT_WARNING', 'type': 'alert', 'severity': 2},
                      '4': {'id': 'GEOFENCE_INTRUSION', 'type': 'conflict', 'severity': 2},
                      '5': {'id': 'GEOFENCE_CONFLICT', 'type': 'conflict', 'severity': 1},
                      '6': {'id': 'TECHNICAL_FAILURE', 'type': 'alert', 'severity': 3},
                      '7': {'id': 'COMMUNICATION_FAILURE', 'type': 'alert', 'severity': 3},
                      '8': {'id': 'LACK_OF_BATTERY', 'type': 'conflict', 'severity': 1},
                      '9': {'id': 'JAMMING_ATTACK', 'type': 'alert', 'severity': 2},
                      '10': {'id': 'SPOOFING_ATTACK', 'type': 'alert', 'severity': 3}
                      }

threat2solve = ThreatsRequest()

def resolve_threat(threat2solve):
    action = 'Action to be taken'
    print(action)
    return action


def threats_callback(request):
    rospy.loginfo("New threat received:") 
    response = ThreatsResponse() # We create the variable which contains the Response.
    response.success = True
    action = resolve_threat(request)
    response.action = action
    return response

rospy.init_node('emergency_manager_node', anonymous=True) # it initiates the node.

# It is offered a service to notify if there is a new threat.

s = rospy.Service('threats', Threats, threats_callback) # it is initiated this service.

# Simply keeps python from exiting until this node is stopped
rospy.spin()


