#!/usr/bin/env python
# This Python file uses the following encoding: utf-8

'''This script is the emergency management module developed in the UTM to decide what is the best action 
to take in the U-space when some Threats are showed up.'''

import rospy
import time
import copy
from gauss_msgs.srv import Threats, ThreatsResponse, ThreatsRequest
from gauss_msgs.srv import ReadOperation, ReadOperationRequest, ReadOperationResponse
from gauss_msgs.srv import WriteGeofences, WriteGeofencesRequest, WriteGeofencesResponse
from gauss_msgs.msg import Threat, Circle, Notification, Waypoint, WaypointList, Operation, Geofence, DeconflictionPlan
from gauss_msgs.srv import Deconfliction, DeconflictionRequest, DeconflictionResponse

class EmergencyManagement():

    def __init__(self): 
        
        # Initialization
               
        self._threats2solve_list = []

        # Publish

        self._notification_publisher = rospy.Publisher('notification', Notification, queue_size=1)
        
        # Wait until services are available and create connection
        
        rospy.wait_for_service('/gauss/tactical_deconfliction')                    
        self._requestDeconfliction_service_handle = rospy.ServiceProxy('/gauss/tactical_deconfliction', Deconfliction) 

        rospy.wait_for_service('/gauss/read_operation')                    
        self._readOperation_service_handle = rospy.ServiceProxy('/gauss/read_operation', ReadOperation) 

        rospy.wait_for_service('/gauss/write_geofences')                    
        self._writeGeofences_service_handle = rospy.ServiceProxy('/gauss/write_geofences', WriteGeofences) 
               
        # Server     

        self._threats_service = rospy.Service('/gauss/threats', Threats, self.service_threats_cb) 

        # Timer

        self.timer = rospy.Timer(rospy.Duration(5), self.timer_cb)
        
        print("Started Emergency Management module!")
    
    def send_threat2deconfliction(self,threat2deconflicted): 
        'Read the operation of UAVs in conflict and return the Deconfliction response'
        request = DeconflictionRequest()
        request.tactical = True
        request.threat = threat2deconflicted 
        self._deconfliction_response = DeconflictionResponse()
        self._deconfliction_response = self._requestDeconfliction_service_handle(request) 
        return self._deconfliction_response
   
    def select_optimal_route(self, uav):
        'Select the optimal deconfliction route '
        #Lista de deconfliction plans.msg
        deconfliction_plans_list = self._deconfliction_response.deconfliction_plans
        #print(deconfliction_plans_list)
        values = []
        
        for deconfliction_plan in deconfliction_plans_list:
             if deconfliction_plan.uav_id == uav:
                 alfa = 0.25 # Weight of cost
                 beta = 0.75 # Weight of riskiness
                 value = alfa*deconfliction_plan.cost + beta*deconfliction_plan.riskiness
                 values.append(value)
                 value_min = min(values)
                 pos_min = values.index(min(values))
        
        value_min = min(values)
        pos_min = values.index(min(values))
        print(values)
        best_solution = deconfliction_plans_list[pos_min]
        print("The best solution is", best_solution)
        return best_solution

    def action_decision_maker(self, threat2solve):
        threat = Threat()
        threat = threat2solve
        threat_type = threat.threat_type
        threat_time = threat.header.stamp
        uavs_threatened = threat.uav_ids
        notification = Notification()
        rate = rospy.Rate(1)
        if len(uavs_threatened) > 0: 

            '''Threat UAS IN CV: we send a message to the UAV in conflict for going back to the FG.'''

            if threat_type == Threat.UAS_IN_CV:   #OK

                ctrl_c = False        
                while not ctrl_c:
                    connections = self._notification_publisher.get_num_connections()
                    if connections > 0:
                        uav_threatened = uavs_threatened[0]
                        notification.description = 'Go back to your flight plan.'
                        notification.uav_id = uav_threatened
                        ctrl_c = True
                        self._notification_publisher.publish(notification) 
                    else:
                        rate.sleep()
                            
            '''Threat UAS OUT OV: we ask to tactical possible solution trajectories'''

            if threat_type == Threat.UAS_OUT_OV: #OK

                self.send_threat2deconfliction(threat)
                best_solution = self.select_optimal_route(uavs_threatened[0])
                ctrl_c = False        
                while not ctrl_c:
                    connections = self._notification_publisher.get_num_connections()
                    if connections > 0:    
                        notification.uav_id = best_solution.uav_id
                        notification.action = best_solution.maneuver_type
                        notification.waypoints = best_solution.waypoint_list
                        ctrl_c = True
                        self._notification_publisher.publish(notification)
                    else:
                        rate.sleep()

        #TODO waiting for tactical deconfliction development in order to validate this Threat.

            '''Threat LOSS OF SEPARATION: we ask to tactical possible solution trajectories'''

            if threat_type == Threat.LOSS_OF_SEPARATION: #OK
                for uav in uavs_threatened:
                    self.send_threat2deconfliction(threat)
                    best_solution = self.select_optimal_route(uav)
                    ctrl_c = False        
                    while not ctrl_c:
                        connections = self._notification_publisher.get_num_connections()
                        if connections > 0:   
                            notification.uav_id = best_solution.uav_id
                            notification.action = best_solution.maneuver_type
                            notification.waypoints = best_solution.waypoint_list
                            ctrl_c = True
                            self._notification_publisher.publish(notification)
                        else:
                            rate.sleep()

            '''Threat ALERT WARNING: we create a cylindrical geofence with center in "location". Besides, we notifies to all UAVs the alert detected'''
               
            if threat_type == Threat.ALERT_WARNING:     #OK
                    
                #We send a notification for every UAV.
                for uav in uavs_threatened:
                    ctrl_c = False
                    while not ctrl_c:
                        connections = self._notification_publisher.get_num_connections()
                        if connections > 0:
                            notification.description = 'Alert Warning: Bad weather Fire or NDZ detected in the zone.'
                            notification.uav_id = uavs_threatened[uav]
                            ctrl_c = True
                            self._notification_publisher.publish(notification)
                        else:
                            rate.sleep()    
                    
                #Creation of the NDZ.
                geofence_base = Circle()
                alarm_center = threat.location
                geofence_base.x_center = alarm_center.x
                geofence_base.y_center = alarm_center.y
                geofence = Geofence()          
                geofence.id = 3
                geofence.min_altitude = 0.0
                geofence.max_altitude = 50.0
                geofence.circle = geofence_base

                # We write a geofence.
                request = WriteGeofencesRequest()
                request.geofence_ids = [geofence.id]
                request.geofences = [geofence]
                response = WriteGeofencesResponse()
                response = self._writeGeofences_service_handle(request)
                response.message = "Geofence stored in the Data Base."
                print(response.message)

            '''Threat GEOFENCE INTRUSION: we ask to tactical possible solution trajectories'''

            if threat_type == Threat.GEOFENCE_INTRUSION:   #OK
                    
                #Publish the action which the UAV has to make.
                    
                self.send_threat2deconfliction(threat)
                best_solution = self.select_optimal_route(uavs_threatened[0])
                ctrl_c = False
                while not ctrl_c:
                    connections = self._notification_publisher.get_num_connections()
                    if connections > 0:
                        notification.uav_id = best_solution.uav_id
                        notification.action = best_solution.maneuver_type
                        notification.waypoints = best_solution.waypoint_list
                        ctrl_c = True
                        self._notification_publisher.publish(notification)
                    else:
                        rate.sleep()    
                    
            '''Threat GEOFENCE CONFLICT: we ask to tactical possible solution trajectories'''

            if threat_type == Threat.GEOFENCE_CONFLICT:  #OK
                    
                #Publish the action which the UAV has to make.
                    
                self.send_threat2deconfliction(threat)
                best_solution = self.select_optimal_route(uavs_threatened[0])
                ctrl_c = False
                while not ctrl_c:
                    connections = self._notification_publisher.get_num_connections()
                    if connections > 0:
                        notification.uav_id = best_solution.uav_id
                        notification.action = best_solution.maneuver_type
                        notification.waypoints = best_solution.waypoint_list
                        ctrl_c = True
                        self._notification_publisher.publish(notification)
                    else:
                        rate.sleep()

            '''Threat TECHNICAL FAILURE: we send a message to the UAV in conflict for landing now.'''

            if threat_type == Threat.TECHNICAL_FAILURE:  #OK
                                
                #Publish the action which the UAV has to make.
                uav_threatened = uavs_threatened[0]
                ctrl_c = False
                while not ctrl_c:
                    connections = self._notification_publisher.get_num_connections()
                    if connections > 0:
                        notification.description = 'URGENT: Land now.'
                        notification.uav_id = uav_threatened
                        ctrl_c = True
                        self._notification_publisher.publish(notification)           
                    else:
                        rate.sleep()

                # We create a geofence.
                geofence = Geofence()
                geofence.id = 3
                geofence.min_altitude = 0.0
                geofence.max_altitude = 100.0
                    
                # We write a geofence.
                request = WriteGeofencesRequest()
                request.geofence_ids = [geofence.id]
                request.geofences = [geofence]
                response = WriteGeofencesResponse()
                response = self._writeGeofences_service_handle(request)
                response.message = "Geofence stored in the Data Base."
                print(response.message)

            '''Threat COMMUNICATION FAILURE: we EM can not do anything if there is a lost of the link communication between the GCS and/or the
                UAV and USP.'''

            if threat_type == Threat.COMMUNICATION_FAILURE: #OK
                    
                #Publish the action which the UAV has to make.
                uav_threatened = uavs_threatened[0]
                ctrl_c = False
                while not ctrl_c:
                    connections = self._notification_publisher.get_num_connections()
                    if connections > 0:
                        notification.description = 'Change UAV control mode from autonomous to manual.'
                        notification.uav_id = uav_threatened
                        ctrl_c = True
                        self._notification_publisher.publish(notification) 
                    else:
                        rate.sleep()
                
            '''Threat LACK OF BATTERY: we ask to tactical possible solution trajectories'''

        #TODO waiting for tactical deconfliction development in order to validate this Threat.

            if threat_type == Threat.LACK_OF_BATTERY:  #OK
                    
                #Publish the action which the UAV has to make.
                    
                self.send_threat2deconfliction(threat)
                best_solution = self.select_optimal_route(uavs_threatened[0])
                ctrl_c = False
                while not ctrl_c:
                    connections = self._notification_publisher.get_num_connections()
                    if connections > 0:
                        notification.uav_id = best_solution.uav_id
                        notification.action = best_solution.maneuver_type
                        notification.waypoints = best_solution.waypoint_list
                        ctrl_c = True
                        self._notification_publisher.publish(notification)
                    else:
                        rate.sleep()    
  
            '''Threat JAMMING ATTACK: We send a message for landing within the geofence created
                around the UAV.'''

            if threat_type == Threat.JAMMING_ATTACK: #OK
                    
                #Publish the action which the UAV has to make.
                uav_threatened = uavs_threatened[0]
                ctrl_c = False
                while not ctrl_c:
                    connections = self._notification_publisher.get_num_connections()
                    if connections > 0:
                        notification.description = 'Land within the geofence created around the UAV.'
                        notification.uav_id = uav_threatened
                        ctrl_c = True
                        self._notification_publisher.publish(notification)           
                    else:
                        rate.sleep()

                # We create a geofence.
                geofence = Geofence()
                geofence.id = 3
                geofence.min_altitude = 0.0
                geofence.max_altitude = 100.0
                    
                # We write a geofence.
                request = WriteGeofencesRequest()
                request.geofence_ids = [geofence.id]
                request.geofences = [geofence]
                response = WriteGeofencesResponse()
                response = self._writeGeofences_service_handle(request)
                response.message = "Geofence stored in the Data Base."       
                print(response.message)

            '''Threat SPOOFING ATTACK: We send a recommendation to the UAV in order to activate the FTS
                and we create a geofence around the UAV.'''

            if threat_type == Threat.SPOOFING_ATTACK: #OK
                
                #Publish the action which the UAV has to make.
                uav_threatened = uavs_threatened[0]
                ctrl_c = False
                while not ctrl_c:
                    connections = self._notification_publisher.get_num_connections()
                    if connections > 0:
                        notification.description = 'Activate the Flight Termination System (FTS) of the UAV.'
                        notification.uav_id = uav_threatened
                        ctrl_c = True
                        self._notification_publisher.publish(notification)           
                    else:
                        rate.sleep()
                    
                # We create a geofence.
                geofence = Geofence()
                geofence.id = 3
                geofence.min_altitude = 0.0
                geofence.max_altitude = 100.0
                    
                # We write a geofence.
                request = WriteGeofencesRequest()
                request.geofence_ids = [geofence.id]
                request.geofences = [geofence]
                response = WriteGeofencesResponse()
                response = self._writeGeofences_service_handle(request)
                response.message = "Geofence stored in the Data Base."   
                print(response.message)

            '''Threat GNSS DEGRADATION: we wait a period of time and then we ask to tactical
                possible trajectories to landing spots'''

        #TODO waiting for tactical deconfliction development in order to validate this Threat.

            if threat_type == Threat.GNSS_DEGRADATION: 
                    
                #Publish the action which the UAV has to make.
                    
                self.send_threat2deconfliction(threat)
                best_solution = self.select_optimal_route(uavs_threatened[0])
                ctrl_c = False
                while not ctrl_c:
                    connections = self._notification_publisher.get_num_connections()
                    if connections > 0:
                        notification.uav_id = best_solution.uav_id
                        notification.action = best_solution.maneuver_type
                        notification.waypoints = best_solution.waypoint_list
                        ctrl_c = False
                        self._notification_publisher.publish(notification)
                    else:
                        rate.sleep()

    def service_threats_cb(self, request):
        req = ThreatsRequest()
        req = copy.deepcopy(request)
        num = len(req.threats)
        rospy.loginfo("Received %d threats!", num) 
        
        for i in range(num):
            self._threats2solve_list.append(req.threats[i])
        
        res = ThreatsResponse()
        res.success = True
        return res 

    def timer_cb(self, timer):
        num = len(self._threats2solve_list)
        rospy.loginfo("There are %d active threats", num)
        rospy.loginfo("Let's solve the threats by severity order!")
        
        if num > 0:
            for threat in self._threats2solve_list:
                self.action_decision_maker(threat)         
   
''' The node and the EmergencyManagement class are initialized'''

if __name__=='__main__':

    rospy.init_node('emergency_management')
    e = EmergencyManagement()
    rospy.spin()   




    
