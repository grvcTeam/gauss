#!/usr/bin/env python
import rospy
import time
from os import system
from gauss_msgs.srv import Threats, ThreatsRequest
from gauss_msgs.msg import Threat

def threat_client(uas_ids, conflict_flaged):     
    request = ThreatsRequest()
    request.uas_ids = uas_ids # This is a list.
    request.threats = conflict_flaged # This is a list of messages type Threat.  
    response = request
    return response 

def conflict_definition_menu(conflict_id, conflicted_uas):
    conflict_flaged = []
    conflict = Threat()
    conflict.threat_id = conflict_id
    conflict.uas_ids = conflicted_uas
    conflict_flaged.append(conflict)
    return (conflicted_uas, conflict_flaged)

def main_menu():
    
    print "\nMain menu of the Monitoring tester. Please choose the conflict to send to Emergency Manager:"
    print "1. UAS in the Contingency Volume"
    print "2. UAS out of the Operational Volume"
    print "3. Loss of separation between UAS"
    print "4. Geofence intrusion"
    print "5. Geofence conflict"
    
    selected = raw_input(" >> ")
    system("clear")
    if selected == "1":
        (conflicted_uas, conflict_flaged) = conflict_definition_menu(0, [1])
    elif selected == "2":
        (conflicted_uas, conflict_flaged) = conflict_definition_menu(1, [2])
    elif selected == "3":
        (conflicted_uas, conflict_flaged) = conflict_definition_menu(2, [1, 2])
    elif selected == "4":
        (conflicted_uas, conflict_flaged) = conflict_definition_menu(4, [3])
    elif selected == "5":
        (conflicted_uas, conflict_flaged) = conflict_definition_menu(5, [4])
    else:
        system("clear")
        print "Not a valid option."

    return (conflicted_uas, conflict_flaged)
    
def main():

    rospy.init_node('monitoring_node')
    rospy.wait_for_service('threats') # Wait for this service to be running.
    threat_service = rospy.ServiceProxy('threats', Threats) # Create the connection to the service. 

    while not rospy.is_shutdown():
        
        (conflicted_uas, conflict_flaged) = main_menu()
        
        conflict_threat = threat_client(conflicted_uas, conflict_flaged) # conflict_threat would be ThreatsRequest()
        
        result = threat_service(conflict_threat)
        
        print(result)
        
        time.sleep(0.1)
    
if __name__ == '__main__':
    main()

