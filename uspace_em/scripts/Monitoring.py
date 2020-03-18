#!/usr/bin/env python
import rospy
from gauss_msgs.srv import Threats, ThreatsRequest
from gauss_msgs.msg import Threat

def threat_client(uas_ids, conflict_flaged):     
    request = ThreatsRequest()
    request.uas_ids = uas_ids # This is a list.
    request.threats = conflict_flaged # This is a list of messages type Threat.  
    response = request
    return response   

def main():

    rospy.init_node('monitoring_node')
    rospy.wait_for_service('threats') # Wait for this service to be running.
    threat_service = rospy.ServiceProxy('threats', Threats) # Create the connection to the service. 

    conflict_flaged = []
    conflict = Threat()
    conflict.threat_id = 0
    conflict.uas_ids = [1]
    reference_time = rospy.Time.now()
    conflict.times = [reference_time, reference_time + rospy.Duration(1,0)]
    conflict_flaged.append(conflict)
    
    conflict_threat = threat_client(conflict.uas_ids, conflict_flaged) # conflict_threat would be ThreatsRequest()

    result = threat_service(conflict_threat)
    print(result)


if __name__ == '__main__':
    main()

