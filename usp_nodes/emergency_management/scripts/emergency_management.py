#!/usr/bin/env python
# This Python file uses the following encoding: utf-8

'''This script is the emergency manager node developed to decide what is the best action 
to take in the U-space when some Threats are showed up.'''

import os, sys
import rospy
import time
from gauss_msgs.srv import Threats, ThreatsResponse, ThreatsRequest
from gauss_msgs.srv import ReadOperation, ReadOperationRequest, ReadOperationResponse
from gauss_msgs.srv import WriteGeofences, WriteGeofencesRequest, WriteGeofencesResponse
from gauss_msgs.msg import Threat, Circle, Notification, Waypoint, WaypointList, Operation, Geofence
from gauss_msgs.srv import Deconfliction, DeconflictionRequest, DeconflictionResponse

class EmergencyManagement():

    def __init__(self): 
        
        # Initialization
        self._threats2solve = ThreatsRequest()        

        # Publish

        self._notification_publisher = rospy.Publisher('notification', Notification, queue_size=1)
        
        # Wait until services are available and create connection
        
        rospy.wait_for_service('/gauss/deconfliction')                    
        self._requestDeconfliction_service_handle = rospy.ServiceProxy('/gauss/deconfliction', Deconfliction) 

        rospy.wait_for_service('/gauss/read_operation')                    
        self._readOperation_service_handle = rospy.ServiceProxy('/gauss/read_operation', ReadOperation) 

        rospy.wait_for_service('/gauss/write_geofences')                    
        self._writeGeofences_service_handle = rospy.ServiceProxy('/gauss/write_geofences', WriteGeofences) 
               
        # Server     

        self._threats_service = rospy.Service('/gauss/threats', Threats, self.service_threats_cb) 
        print("Ready to add a threat request")
    
    #TODO link Threats severity/probability with the SoA references.
    
    #def assign_threat_severity(self):
        #self._threats_definition = {
        #                            Threat.UAS_IN_CV: {'type': 'conflict', 'severity': 1},
        #              	             Threat.UAS_OUT_OV: {'type': 'conflict', 'severity': 2},
        #                            Threat.LOSS_OF_SEPARATION: {'type': 'conflict', 'severity': 2},
        #                            Threat.ALERT_WARNING: {'type': 'alert', 'severity': 2},
        #                            Threat.GEOFENCE_INTRUSION: {'type': 'conflict', 'severity': 2},
        #                            Threat.GEOFENCE_CONFLICT: {'type': 'conflict', 'severity': 1},
        #                            Threat.TECHNICAL_FAILURE: {'type': 'alert', 'severity': 3},
        #                            Threat.COMMUNICATION_FAILURE: {'type': 'alert', 'severity': 3},
        #                            Threat.LACK_OF_BATTERY: {'type': 'conflict', 'severity': 1},
        #                            Threat.JAMMING_ATTACK: {'type': 'alert', 'severity': 2},
        #                            Threat.SPOOFING_ATTACK: {'type': 'alert', 'severity': 3}
        #                            }

    def select_optimal_trajectory ():
        pass

    def send_threat2deconfliction (self,threat2deconflicted): 
        request = DeconflictionRequest()
        request.tactical = True
        request.threat = threat2deconflicted
        response = self._requestDeconfliction_service_handle(request)        
        return response 

    def action_decision_maker(self,threats2solve):
        events = threats2solve.threats # List of Threat.msg
        threat_id = events[0].threat_id # We solve the first Threat in this SW version
        threat_time = events[0].header.stamp
        uavs_threatened = events[0].uav_ids
        uav_threatened = uavs_threatened[0]
        #print("The Threat has been notified at the second since epoch:", threat_time)
        #threat_severity = self._threats_definition[threat_id]['severity']
        #first_operation_priority = self._operation_info_from_db['operations'][0]['priority']
        #second_operation_priority = self._operation_info_from_db['operations'][1]['priority']
        
        ''' Threat UAS IN CV: we send a message to the UAV in conflict for going back to the FG.'''

        if threat_id == Threat.UAS_IN_CV:   

            #Publish the action which the UAV has to make.
            notification = Notification()
            notification.description = 'Go back to your flight plan.'
            notification.uav_id = uav_threatened
            self._notification_publisher.publish(notification) 
            
        '''Threat UAS OUT OV: we ask to tactical possible solution trajectories'''

        if threat_id == Threat.UAS_OUT_OV: 
            
            #Publish the action which the UAV has to make.
            notification = Notification()
            notification.uav_id = uav_threatened
            notification.description = 'URGENT: Land as soon as possible.'   
            self.send_threat2deconfliction(events[0])
            #notification.waypoints

        ''' Threat LOSS OF SEPARATION: we ask to tactical possible solution trajectories'''

        if threat_id == Threat.LOSS_OF_SEPARATION: 
            
            action = 'Send new trajectories recommendations to the UAS involved in the conflict.'     
            self.send_threat2deconfliction(events[0])
        
        ''' Threat ALERT WARNING: we create a cylindrical geofence with center in "location". Besides,
        we notifies to all UAVs the alert detected.'''
        
        if threat_id == Threat.ALERT_WARNING:             
            
            #We send a notification for every UAV.
            for uav in uavs_threatened:
                notification = Notification()
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

        ''' Threat GEOFENCE INTRUSION: we ask to tactical possible solution trajectories'''

        if threat_id == Threat.GEOFENCE_INTRUSION: 
            
            #Publish the action which the UAV has to make.
            notification = Notification()
            notification.uav_id = uav_threatened
            notification.description = 'Ask for leaving the geofence asap and continue its operation.'   
            self.send_threat2deconfliction(events[0])
            #notification.waypoints
            
        '''Threat GEOFENCE CONFLICT: we ask to tactical possible solution trajectories'''

        if threat_id == Threat.GEOFENCE_CONFLICT:
            #Publish the action which the UAV has to make.
            notification = Notification()
            notification.uav_id = uav_threatened
            notification.description = 'Send new trajectory recommendation to the UAS involved in the conflict.'   
            self.send_threat2deconfliction(events[0])
            #notification.waypoints
            
        '''Threat TECHNICAL FAILURE: we send a message to the UAV in conflict for landing now.'''

        if threat_id == Threat.TECHNICAL_FAILURE: 
            
            #request = ReadOperationRequest()
            #request.uav_ids = uav_threatened
            #response = ReadOperationResponse()
            #response = self.send_uavs_threatened(request) 
            
            #Publish the action which the UAV has to make.
            notification = Notification()
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
        
        ''' Threat COMMUNICATION FAILURE: we EM can not do anything if there is a lost of the link communication between the GCS and/or the
        UAV and USP.'''

        if threat_id == Threat.COMMUNICATION_FAILURE: 
            
            #Publish the action which the UAV has to make.
            notification = Notification()
            notification.description = 'Change UAV control mode from autonomous to manual.'
            notification.uav_id = uav_threatened
            self._notification_publisher.publish(notification) 
        
        ''' Threat LACK OF BATTERY: we ask to tactical possible solution trajectories'''

        if threat_id == Threat.LACK_OF_BATTERY: 
            #Publish the action which the UAV has to make.
            notification = Notification()
            notification.description = 'Land in a landing spot'
            notification.uav_id = uav_threatened
            self.send_threat2deconfliction(events[0])
        
        ''' Threat JAMMING ATTACK: We send a message for landing within the geofence created
        around the UAV.'''

        if threat_id == Threat.JAMMING_ATTACK: 
            
            #Publish the action which the UAV has to make.
            notification = Notification()
            notification.description = 'Land within the geofence created around the UAV.'
            notification.uav_id = uavs_threatened
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
                
        ''' Threat SPOOFING ATTACK: We send a recommendation to the UAV in order to activate the FTS
        and we create a geofence around the UAV.'''

        if threat_id == Threat.SPOOFING_ATTACK: 
        
            #Publish the action which the UAV has to make.
            notification = Notification()
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
        
        ''' Threat LACK OF BATTERY: we ask to tactical possible solution trajectories'''

        if threat_id == Threat.GNSS_DEGRADATION: 
            #Publish the action which the UAV has to make.
            notification = Notification()
            notification.description = 'Land in a landing spot'
            notification.uav_id = uav_threatened
            self.send_threat2deconfliction(events[0])
            
        print(notification.description)

    def service_threats_cb(self, request):
        rospy.loginfo("New threat received:") 
        response = ThreatsResponse()
        response.success = True
        self._threats2solve = request # ThreatsRequest
        self._threats2solve.uav_ids = list(request.uav_ids) 
        self.action_decision_maker(self._threats2solve) 
        return response        

   # TODO Since a Threat has been received. It is request operation info of the UAV linked to
   # the Threat. 

    def send_uavs_threatened(self, request): 
        return self._readOperation_service_handle(self._threats2solve.uav_ids)
  
    def request_deconfliction_plans(self):
        request = DeconflictionRequest()
        request.threat = self._
        
        #TODO, rellenar la info necesaria para que me envien una nueva trayectoria.
        
    
    #TODO, definir los parametros para usar esta función. 

''' The node and the EmergencyManagement class are initialized'''

if __name__=='__main__':

    rospy.init_node('emergency_management')
    e = EmergencyManagement()
    rospy.spin()   


    
