#!/usr/bin/env python

## This script is the emergency manager node developed to decide what is the best action 
# to take in the U-space when some Threats are showed up.

import os, sys
import rospy
from math import sqrt
from gauss_msgs.srv import Threats, ThreatsResponse
from gauss_msgs.srv import ReadOperation, ReadOperationRequest
from gauss_msgs.srv import WriteFlightPlan, WriteFlightPlanRequest
from gauss_msgs.srv import WriteGeofences, WriteGeofencesRequest
from gauss_msgs.msg import Threat, Notification, Waypoint, WaypointList, Operation, Geofence
from gauss_msgs.srv import Deconfliction, DeconflictionRequest

## It is defined a class with all functionabilities

# Me llega una lista de Threat y los uavs que los tienen. Le asigno una severidad a cada uno de los Threat. Esta severidad
#debera calcularse con algún criterio. Por otra parte, es necesario read operation de cada UAV para saber cuál es la 
#prioridad que tiene establecida cada misión. La función decision maker debe decidir que amenaza se resuelve primero y qué
#acción se le manda a cada UAV.
#Teniendo en cuenta los diferentes eventos que pueden ocurrir. 
# JAMMING, crear una geofence, land asap
# SPOOFING, crear una geofence, FTS or land asap
# UAS IN CV, go back to FG, new trajectory
# UAS OUT OV, go back to the FG, new trajectory
# LOSS OF SEPARATION, new trajectory for both UAVs
# GEOFENCE CONFLICT, new trajectory for the UAV.
# GEOFENCE INTRUSION, new trajectory for the intruder UAV.
# COMMUNICATION FAILURE, ¿?
# TECHNICAL FAILURE, crear una geofence y activar un FTS o land asap.
# ALERT WARNING,crear una geofence y mandar una notificación a todos los UAVs.
# LACK OF BATTERY, land in a landing spot, calcular el mejor punto en el que aterrizar.

class EmergencyManagement():

    def __init__(self): 
        
        # Initialization

        # Publish
        self._notification_publisher = rospy.Publisher("notification", Notification, queue_size=1)
        
        # Wait until service is available and creat connection
        
        rospy.wait_for_service('/gauss/deconfliction')                    
        self._requestDeconfliction_service_handle = rospy.ServiceProxy('/gauss/deconfliction', Deconfliction) 

        rospy.wait_for_service('/gauss/read_operation')                    
        self._readOperation_service_handle = rospy.ServiceProxy('/gauss/read_operation', ReadOperation) 

        rospy.wait_for_service('/gauss/write_flight_plan')                    
        self._writeFlightPlan_service_handle = rospy.ServiceProxy('/gauss/write_flight_plan', WriteFlightPlan) 

        rospy.wait_for_service('/gauss/write_geofences')                    
        self._writeGeofences_service_handle = rospy.ServiceProxy('/gauss/write_geofences', WriteGeofences) 

        # Server     

        self._threats_service = rospy.Service('/gauss/threats', self.service_threats_cb) 
        
        # Threats definition 

        self._threats_definition = {Threat.UAS_IN_CV: {'type': 'conflict', 'severity': 1},
                      	            Threat.UAS_OUT_OV: {'type': 'conflict', 'severity': 2},
                                    Threat.LOSS_OF_SEPARATION: {'type': 'conflict', 'severity': 2},
                                    Threat.ALERT_WARNING: {'type': 'alert', 'severity': 2},
                                    Threat.GEOFENCE_INTRUSION: {'type': 'conflict', 'severity': 2},
                                    Threat.GEOFENCE_CONFLICT: {'type': 'conflict', 'severity': 1},
                                    Threat.TECHNICAL_FAILURE: {'type': 'alert', 'severity': 3},
                                    Threat.COMMUNICATION_FAILURE{'type': 'alert', 'severity': 3},
                                    Threat.LACK_OF_BATTERY{'type': 'conflict', 'severity': 1},
                                    Threat.JAMMING_ATTACK{'type': 'alert', 'severity': 2},
                                    Threat.SPOOFING_ATTACK{'type': 'alert', 'severity': 3}
                                    }
    ## It is stored the Threat in a ThreatRequest. OK.
    def service_threats_cb(self, request):
        rospy.loginfo("New threat received:") 
        response = ThreatsResponse()
        response.success = True
        self._threats2solve = request # ThreatRequest
        return response

#TODO link SORA criteria with severity Threats

    def asignate_threat_severity(self):
        pass
   
   ## Since a Threat has been received. It is request operation info of the uav linked to
   # the Threat. OK.

    def request_operations_info(self):
        request = ReadOperationRequest()
        request.uav_ids = self._threats2solve.threats[0].uav_ids
        response = self._readOperation_service_handle(request)
        return response  

    ##This function decide what is the fittest action to take
    
    def action_decision_maker(self):
        
        threat_type = self._threats_definition[threat_id]['type']
        threat_severity = self._threats_definition[threat_id]['severity']
        
        if threat_severity == 3:
            UAV_id = self._threats2solve.Threat.uav_ids[0]
            action = 'URGENT: Land as soon as possible.'  
            notification = self.declare_notification_parameters(UAV_id, action)
            self.send_notification(notification)
                    
        if threat_severity == 2:

            if threat_id == 1: #UAS_OUT_OV
                action = 'URGENT: Land as soon as possible.'  
               
            if threat_id == 2: #LOSS_OF_SEPARATION
                action = 'Send new trajectories recommendations to the UAS involved in the conflict.' 
            
            if threat_id == 3: #ALERT_WARNING
                action = 'Ask for a Geofence creation and send an alert report to all the pilots.' 
            
            if threat_id == 4: #GEOFENCE_INTRUSION
                action = 'Ask for leaving the geofence asap and continue its operation.' 
            
            if threat_id == 9: #JAMMING_ATTACK
                action = 'URGENT: Land as soon as possible.'  
            
        if threat_severity == 1:

            if threat_id == 0: #UAS_IN_CV
                action = 'Send new trajectory to get into the FG.' 
            
            if threat_id == 5: #GEOFENCE_CONFLICT
                action = 'Send new trajectory recommendation to the UAS involved in the conflict.'     
            
            if threat_id == 8: #LACK_OF_BATTERY
                action = 'Please, land in the defined landing spot.'         
        print(action)
        
    def declare_notification_parameters(self, UAV_id, action):
        notification = Notification()
        notification.UAV_id = UAV_id
        notification.action = action
        return notification

    def send_notification(self, message): # message será notification de la función anterior
        self._notification_publisher.publish(message)
    
    def declare_geofence_parameters(self):
        geofence = Geofence()
        geofence.id = ?
        geofence.static_geofence = ?
        geofence.cylinder_shape = ? 
        geofence.min_altitude = ?
        geofence.max_altitude = ?
        geofence.start_time = ?
        geofence.end_time = ?
        geofence.circle = ?
        geofence.polygon = ?
        return geofence

    def request_geofence_creation(self):
        request = WriteGeofencesRequest()
        request.geofences[0] = self.declare_geofence_parameters()
        return request
    
    def declare_new_trajectory_parameters (self):
        pass

    def request_new_trajectory(self):
        request = DeconflictionRequest()
        #TODO, rellenar la info necesaria para que me envien una nueva trayectoria.
        pass
    
    def write_new_flight_plan(self):
        request = WriteFlightPlanRequest()
        #TODO, rellenar la info necesaria para poder escribir un nuevo flight plan.
        pass

    def activate_fts(self, fts_signal) # Flight Termination System
        self._notification_publisher.publish(fts_signal)

    #TODO, definir los parametros para usar esta función.
    
    def calculate_best_landingspot(self):
        pass

''' The node and the Emergency Management class are initialized'''

if __name__== '__main__':
    
    rospy.init_node('emergency_management')
    e = EmergencyManagement()
    
    while not rospy.is_shutdown():
    
        e.asignate_threat_severity()
        e.request_operations_info() # Se solicita info de los uav que usará el decision maker.
        
        
        time.sleep(0.1)
