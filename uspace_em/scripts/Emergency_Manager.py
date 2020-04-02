#!/usr/bin/env python
# this Python file uses the following encoding: utf-8
import os, sys
import rospy
from math import sqrt
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

def calculate_landingspot(landingspot_list, uas_vel, flight_time, uas_position):
    '''This function select the best point for landing'''
    uas_velocity = uas_vel
    flight_time = flight_time # Time which the UAS can still be flying.
    max_distance = uas_velocity/flight_time # Max distance in meters which the UAS can achieve.
    landing_list = landingspot_list
    landingspot_uasposition_distance = []  
    for point in landing_list:
        distance = sqrt((landing_list[point[0]][0]-uas_position[0])**2 + (landing_list[point[1]][1]-uas_position[1])**2 + (landing_list[point[2]][2]-uas_position[2])**2)
        landingspot_uasposition_distance.append(distance)       
    min_distance = min(landingspot_uasposition_distance)
    pos_min = landingspot_uasposition_distance.index(min(landingspot_uasposition_distance)) # posicion de la lista que tiene la mínima distancia
    best_landingspot = landing_list[pos_min]
    
    return best_landingspot

def threat_management(threat2solve):
    """This function decide what is the fittest action to take""" 
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
               
        action = 'URGENT: Land as soon as possible.'  
        action_id = 0

    if threat_severity == 2:

        if threat_id == 1: #UAS_OUT_OV
            action = 'URGENT: Land as soon as possible.'  
            action_id = 0
        if threat_id == 2: #LOSS_OF_SEPARATION
            action = 'Send new trajectories recommendations to the UAS involved in the conflict.' 
            action_id = 3
        if threat_id == 3: #ALERT_WARNING
            action = 'Ask for a Geofence creation and send an alert report to all the pilots.' 
            action_id = 1
        if threat_id == 4: #GEOFENCE_INTRUSION
            action = 'Ask for leaving the geofence asap and continue its operation.' 
            action_id = 3
        if threat_id == 9: #JAMMING_ATTACK
            action = 'URGENT: Land as soon as possible.'  
            action_id = 0
    if threat_severity == 1:

        if threat_id == 0: #UAS_IN_CV
            action = 'Send new trajectory to get into the FG.' 
            action_id = 3
        if threat_id == 5: #GEOFENCE_CONFLICT
            action = 'Send new trajectory recommendation to the UAS involved in the conflict.'     
            action_id = 3
        if threat_id == 8: #LACK_OF_BATTERY
            action = 'Please, land in the defined landing spot.'     
            action_id = 3
    print(action)
    return action, action_id

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
    action, action_id = threat_management(threat2solve)
    #global gpub
    #notification = Notification()
    #notification.description = action
    #notification.action_id = action_id
    #gpub.publish(notification)
    
    return response

def main():
    rospy.init_node('emergency_manager_node')
    threat_service = rospy.Service('threats', Threats, threats_response)
    print("Ready to add a threat request")
    # We declare actual info for Lack_of_battery solution
    uas_position = [1, 1, 0]
    landingspot_list = [[0, 0, 0], [1, 1, 0], [2, 1, 0]]
    
    uas_vel = 5 # m/s
    flight_time = 600 # seconds
    best_landingspot = calculate_landingspot(landingspot_list, uas_vel, flight_time, uas_position)
    print(best_landingspot)
    #global gpub 
    #gpub = rospy.Publisher("notification", Notification, queue_size=1)
    
            
    rospy.spin()


if __name__ == '__main__':
    main()
