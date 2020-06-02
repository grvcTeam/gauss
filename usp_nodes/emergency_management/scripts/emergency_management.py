#!/usr/bin/env python
# This Python file uses the following encoding: utf-8

'''This script is the emergency manager node developed to decide what is the best action 
to take in the U-space when some Threats are showed up.'''

import rospy
import time
from gauss_msgs.srv import Threats, ThreatsResponse, ThreatsRequest
from gauss_msgs.srv import ReadOperation, ReadOperationRequest, ReadOperationResponse
from gauss_msgs.srv import WriteGeofences, WriteGeofencesRequest, WriteGeofencesResponse
from gauss_msgs.msg import Threat, Circle, Notification, Waypoint, WaypointList, Operation, Geofence, DeconflictionPlan
from gauss_msgs.srv import Deconfliction, DeconflictionRequest, DeconflictionResponse

class EmergencyManagement():

    def __init__(self): 
        
        # Initialization
        self._threats2solve = ThreatsRequest()        

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
        
        print("Ready to add a threat request")
    
    #TODO link Threats severity/probability with the SoA references.
        
    def send_threat2deconfliction (self,threat2deconflicted): 
        request = DeconflictionRequest()
        request.tactical = True
        request.threat = threat2deconflicted
        uavs_in_conflict = threat2deconflicted.uav_ids
        self._readoperation_response = ReadOperationResponse()
        self._readoperation_response = self._readOperation_service_handle(uavs_in_conflict)
        priority_ops = []
        for uav in uavs_in_conflict:
            uav_operation = Operation()
            uav_operation = self._readoperation_response.operation[uav]
            uav_priority = uav_operation.priority
            priority_ops.append(uav_priority)
        threat2deconflicted.priority_ops = priority_ops        
        self._deconfliction_response = DeconflictionResponse()
        self._deconfliction_response = self._requestDeconfliction_service_handle(request) 
        return self._deconfliction_response 
   
    def select_optimal_route (self):
        deconfliction_plans_list = self._deconfliction_response.deconfliction_plans
        values = []
        for deconfliction_plan in deconfliction_plans_list:
             if deconfliction_plan.uav_id == 0:
                 alfa = 0.25 # Peso de coste
                 beta = 0.75 # Peso de peligrosidad
                 value = alfa*deconfliction_plan.cost[0] + beta*deconfliction_plan.riskiness[0]
                 values.append(value)
                 value_min = min(values)
                 pos_min = values.index(min(values))
        value_min = min(values)
        pos_min = values.index(min(values))
        print(values)
        best_solution = deconfliction_plans_list[pos_min]
        print("The best solution is", best_solution)
        return best_solution

    def action_decision_maker(self,threats2solve):
        events = threats2solve.threats 
        threat_id = events[0].threat_id
        threat_time = events[0].header.stamp
        uavs_threatened = events[0].uav_ids
        uav_threatened = uavs_threatened[0]
        notification = Notification()
        #print("The Threat has been notified at the second since epoch:", threat_time)
        
        '''Threat UAS IN CV: we send a message to the UAV in conflict for going back to the FG.'''

        if threat_id == Threat.UAS_IN_CV:   

            #Publish the action which the UAV has to make.
            notification.description = 'Go back to your flight plan.'
            notification.uav_id = uav_threatened
            self._notification_publisher.publish(notification) 

#TODO waiting for tactical deconfliction development in order to validate this Threat.
            
        '''Threat UAS OUT OV: we ask to tactical possible solution trajectories'''

        if threat_id == Threat.UAS_OUT_OV: 
            
            #Publish the action which the UAV has to make.
            
            print(self.send_threat2deconfliction(events[0]))
            best_solution = self.select_optimal_route()
            notification.uav_id = best_solution.uav_id
            notification.action = best_solution.maneuver_type
            notification.waypoints = best_solution.waypoint_list
            self._notification_publisher.publish(notification)

#TODO waiting for tactical deconfliction development in order to validate this Threat.

        '''Threat LOSS OF SEPARATION: we ask to tactical possible solution trajectories'''

        if threat_id == Threat.LOSS_OF_SEPARATION: 
            for uav in uavs_threatened:
                print(self.send_threat2deconfliction(events[0]))
                #best_solution = self.select_optimal_route()
                #notification.uav_id = best_solution.uav_id
                #notification.action = best_solution.maneuver_type
                #notification.waypoints = best_solution.waypoint_list
                #self._notification_publisher.publish(notification)

        '''Threat ALERT WARNING: we create a cylindrical geofence with center in "location". Besides, we notifies to all UAVs the alert detected'''
        
        if threat_id == Threat.ALERT_WARNING:             
            
            #We send a notification for every UAV.
            for uav in uavs_threatened:
                notification.description = 'Alert Warning: Bad weather Fire or NDZ detected in the zone.'
                notification.uav_id = uavs_threatened[uav]
                self._notification_publisher.publish(notification)
            
            #Creation of the NDZ.
            geofence_base = Circle()
            alarm_center = events[0].location
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

        if threat_id == Threat.GEOFENCE_INTRUSION: 
            
            #Publish the action which the UAV has to make.
            
            print(self.send_threat2deconfliction(events[0]))
            best_solution = self.select_optimal_route()
            notification.uav_id = best_solution.uav_id
            notification.action = best_solution.maneuver_type
            notification.waypoints = best_solution.waypoint_list
            self._notification_publisher.publish(notification)
            
        '''Threat GEOFENCE CONFLICT: we ask to tactical possible solution trajectories'''

        if threat_id == Threat.GEOFENCE_CONFLICT:
            
            #Publish the action which the UAV has to make.
              
            print(self.send_threat2deconfliction(events[0]))
            best_solution = self.select_optimal_route()
            notification.uav_id = best_solution.uav_id
            notification.action = best_solution.maneuver_type
            notification.waypoints = best_solution.waypoint_list
            self._notification_publisher.publish(notification)

        '''Threat TECHNICAL FAILURE: we send a message to the UAV in conflict for landing now.'''

        if threat_id == Threat.TECHNICAL_FAILURE: 
                        
            #Publish the action which the UAV has to make.
            notification.description = 'URGENT: Land now.'
            notification.uav_id = uav_threatened
            self._notification_publisher.publish(notification)           
            
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
            action = 'URGENT: Land now.'     
        
        '''Threat COMMUNICATION FAILURE: we EM can not do anything if there is a lost of the link communication between the GCS and/or the
        UAV and USP.'''

        if threat_id == Threat.COMMUNICATION_FAILURE: 
            
            #Publish the action which the UAV has to make.
            notification.description = 'Change UAV control mode from autonomous to manual.'
            notification.uav_id = uav_threatened
            self._notification_publisher.publish(notification) 
        
        '''Threat LACK OF BATTERY: we ask to tactical possible solution trajectories'''

#TODO waiting for tactical deconfliction development in order to validate this Threat.

        if threat_id == Threat.LACK_OF_BATTERY: 
            
            #Publish the action which the UAV has to make.
            
            print(self.send_threat2deconfliction(events[0]))
            #best_solution = self.select_optimal_route()
            #notification.uav_id = best_solution.uav_id
            #notification.action = best_solution.maneuver_type
            #notification.waypoints = best_solution.waypoint_list
            #self._notification_publisher.publish(notification)
        
        '''Threat JAMMING ATTACK: We send a message for landing within the geofence created
        around the UAV.'''

        if threat_id == Threat.JAMMING_ATTACK: 
            
            #Publish the action which the UAV has to make.
            notification.description = 'Land within the geofence created around the UAV.'
            notification.uav_id = uav_threatened
            self._notification_publisher.publish(notification)           
            
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

        if threat_id == Threat.SPOOFING_ATTACK: 
        
            #Publish the action which the UAV has to make.
            notification.description = 'Activate the Flight Termination System (FTS) of the UAV.'
            notification.uav_id = uav_threatened
            self._notification_publisher.publish(notification)           
            
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
        
        '''Threat GNSS DEGRADATION: we wait a period of time and then we ask to tactical
        possible trajectories to landing spots'''

#TODO waiting for tactical deconfliction development in order to validate this Threat.

        if threat_id == Threat.GNSS_DEGRADATION: 
            
            #Publish the action which the UAV has to make.
            
            print(self.send_threat2deconfliction(events[0]))
            #best_solution = self.select_optimal_route()
            #notification.uav_id = best_solution.uav_id
            #notification.action = best_solution.maneuver_type
            #notification.waypoints = best_solution.waypoint_list
            #self._notification_publisher.publish(notification)

    def service_threats_cb(self, request):
        rospy.loginfo("New threat received:") 
        response = ThreatsResponse()
        response.success = True
        self._threats2solve = request # ThreatsRequest
        self._threats2solve.uav_ids = list(request.uav_ids) 
        self.action_decision_maker(self._threats2solve) 
        return response        

    def send_uavs_threatened(self, request): 
        return self._readOperation_service_handle(self._threats2solve.uav_ids)

''' The node and the EmergencyManagement class are initialized'''

if __name__=='__main__':

    rospy.init_node('emergency_management')
    e = EmergencyManagement()
    rospy.spin()   


    
