#!/usr/bin/env python
import rospy
from gauss_msgs.srv import Threats, ThreatsResponse
from gauss_msgs.msg import Threat
from gauss_msgs.msg import Notification
#from gauss_msgs.srv import Deconfliction, DeconflictionRequest, DeconflictionResponse

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

def threat_management(threat2solve):
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
    
    return action

#def notification_client(uas_notified, action_id, action_description):
#    rospy.wait_for_service('notification')
#    try:    
#        notification = rospy.ServiceProxy('notification', Notification)
#        request = NotificationRequest()
#        request.uas_ids = uas_notified 
#        request.action_id = action_id 
#        request.description = action_description 
#        response = notification(request)
        
#    except rospy.ServiceException, e:
#        print "Service call failed: %s"%e

#def deconfliction_client(uas_in_conflict):
#    rospy.wait_for_service('deconfliction')
#    try:  
#        deconfliction = rospy.ServiceProxy('deconfliction', Deconfliction)
#        request = DeconflictionRequest()
#        request.uas_ids = uas_in_conflict 
#       response = deconfliction(request)
        
#    except rospy.ServiceException, e:
#        print "Service call failed: %s"%e

def threats_response(request): # This is the callback
    rospy.loginfo("New threat received:") 
    response = ThreatsResponse() # We create the variable which contains the Response.
    response.success = True
    threat2solve = request
    action = threat_management(threat2solve)
    global gpub
    notification = Notification()
    notification.description = action
    gpub.publish(notification)
    #r = rospy.Rate(1)
    #print(action)
    return response

def main():
    rospy.init_node('emergency_manager_node')
    threat_service = rospy.Service('threats', Threats, threats_response)
    print("Ready to add a threat request")
    global gpub 
    gpub = rospy.Publisher("notification", Notification, queue_size=1)
            
    rospy.spin()


if __name__ == '__main__':
    main()
