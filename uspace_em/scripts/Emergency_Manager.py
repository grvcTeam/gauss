#!/usr/bin/env python
import rospy
from gauss_msgs.srv import Threats, ThreatsRequest, ThreatsResponse
from gauss_msgs.msg import Threat
from gauss_msgs.srv import Notification, NotificationRequest, NotificationResponse
from gauss_msgs.srv import Deconfliction, DeconflictionRequest, DeconflictionResponse

threats_definition = {'0': {'name': 'UAS_IN_CV', 'type': 'conflict', 'severity': 1},
                      '1': {'name': 'UAS_OUT_OV', 'type': 'conflict', 'severity': 2},
                      '2': {'name': 'LOSS_OF_SEPARATION', 'type': 'conflict', 'severity': 2},
                      '3': {'name': 'ALERT_WARNING', 'type': 'alert', 'severity': 2},
                      '4': {'name': 'GEOFENCE_INTRUSION', 'type': 'conflict', 'severity': 2},
                      '5': {'name': 'GEOFENCE_CONFLICT', 'type': 'conflict', 'severity': 1},
                      '6': {'name': 'TECHNICAL_FAILURE', 'type': 'alert', 'severity': 3},
                      '7': {'name': 'COMMUNICATION_FAILURE', 'type': 'alert', 'severity': 3},
                      '8': {'name': 'LACK_OF_BATTERY', 'type': 'conflict', 'severity': 1},
                      '9': {'name': 'JAMMING_ATTACK', 'type': 'alert', 'severity': 2},
                      '10': {'name': 'SPOOFING_ATTACK', 'type': 'alert', 'severity': 3}
                      }

threat2solve = ThreatsRequest()

def resolve_threat(threat2solve):
    events = threat2solve.threats # This is a list of Threat objects.
    threat_id = events[0].threat_id # This is the threat_id of the first Threat object in the previous list.
    uas_threaten = events[0].uas_ids # This is a list of uas involved in the Threat.
    time = events[0].times #This is a list.
    
    if threat_id == 0:
        threat_severity = threats_definition['0']['severity']
        threat_name = threats_definition['0']['name']
        threat_type = threats_definition['0']['type']
    if threat_id == 1:
        threat_severity = threats_definition['1']['severity']
        threat_name = threats_definition['1']['name']
        threat_type = threats_definition['1']['type']
    if threat_id == 2:
        threat_severity = threats_definition['2']['severity']
        threat_name = threats_definition['2']['name']
        threat_type = threats_definition['2']['type']
    if threat_id == 3:
        threat_severity = threats_definition['3']['severity']
        threat_name = threats_definition['3']['name']
        threat_type = threats_definition['3']['type']
    if threat_id == 4:
        threat_severity = threats_definition['4']['severity']
        threat_name = threats_definition['4']['name']
        threat_type = threats_definition['4']['type']
    if threat_id == 5:
        threat_severity = threats_definition['5']['severity']
        threat_name = threats_definition['5']['name']
        threat_type = threats_definition['5']['type']
    if threat_id == 6:
        threat_severity = threats_definition['6']['severity']
        threat_name = threats_definition['6']['name']
        threat_type = threats_definition['6']['type']
    if threat_id == 7:
        threat_severity = threats_definition['7']['severity']
        threat_name = threats_definition['7']['name']
        threat_type = threats_definition['7']['type']
    if threat_id == 8:
        threat_severity = threats_definition['8']['severity']
        threat_name = threats_definition['8']['name']
        threat_type = threats_definition['8']['type']
    if threat_id == 9:
        threat_severity = threats_definition['9']['severity']
        threat_name = threats_definition['9']['name']
        threat_type = threats_definition['9']['type']
    if threat_id == 10:
        threat_severity = threats_definition['10']['severity']
        threat_name = threats_definition['10']['name']
        threat_type = threats_definition['10']['type']
    
    if threat_severity == 3:
               
        action = 'Send recommendation to Pilot to land as soon as possible' # Use Notification.msg to USP_manager. 

    if threat_severity == 2:

        if threat_id == 1: #UAS_OUT_OV
            action = 'Send recommendation to Pilot to land as soon as possible' # Use Notification.msg to USP_manager. 
        if threat_id == 2: #LOSS_OF_SEPARATION
            action = 'Send new trajectories recommendations to the UAS involved in the conflict' # Use Notification.msg to USP_manager. It is needed support from Tactical Conflict Resolution.     
        if threat_id == 3: #ALERT_WARNING
            action = 'Ask for a Geofence creation and send an alert report to all the pilots' # Use Notification.msg to USP_manager.    
        if threat_id == 4: #GEOFENCE_INTRUSION
            action = 'Ask for leaving the geofence asap and continue its operation' # Needs Tactical Deconfliction support.
        if threat_id == 9: #JAMMING_ATTACK
            action = 'Send recommendation to Pilot to land as soon as possible' # Use Notification.msg to USP_manager. 

    if threat_severity == 1:

        if threat_id == 0: #UAS_IN_CV
            action = 'Send new trajectory to get into the FG' # Needs Tactical deconfliction.
        if threat_id == 5: #GEOFENCE_CONFLICT
            action = 'Send new trajectory recommendation to the UAS involved in the conflict' # Use Notification.msg to USP_manager. It is needed support from Tactical Conflict Resolution.     
        if threat_id == 8: #LACK_OF_BATTERY
            action = 'Send a message to the pilot to land in a landing spot.' # Use Notification.msg to USP_manager. Tactical deconfliction is needed for calculating the new trajectory.    
    
    print("The code name of the threat is:", threat_name)
    print("The type of threat is:", threat_type)
    print("The threat severity is:", threat_severity)
    print("The action to be sent to the RPIC is:", action)    
    return action

def notification_client(uas_notified, action_id, action_description):
    rospy.wait_for_service('notification')
    try:      
        request = NotificationRequest()
        request.uas_ids = uas_notified 
        request.action_id = action_id 
        request.description = action_description 
        response = client(request)
        
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def deconfliction_client(uas_in_conflict):
    rospy.wait_for_service('deconfliction')
    try:      
        request = DeconflictionRequest()
        request.uas_ids = uas_in_conflict 
        response = client(request)
        
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def threats_callback(request):
    rospy.loginfo("New threat received:") 
    response = ThreatsResponse() # We create the variable which contains the Response.
    response.success = True
    uas_notified = request.uas_ids
    uas_in_conflict = uas_notified
    action_id = 10
    action_description = resolve_threat(request)
    #new_trajectory = [36.6, 78.5, 40.5, rospy.Time.now()]
    notification = notification_client(uas_notified, action_id, action_description)
    trajectory = deconfliction_client(uas_in_conflict)
    print(trajectory)
    return response

# main
rospy.init_node('emergency_manager_node', anonymous=True) # it initiates the node.

client = rospy.ServiceProxy('notification', Notification)
client = rospy.ServiceProxy('deconfliction', Deconfliction)
s = rospy.Service('threats', Threats, threats_callback) # it is initiated this service.


# Simply keeps python from exiting until this node is stopped
rospy.spin()


