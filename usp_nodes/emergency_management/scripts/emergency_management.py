#!/usr/bin/env python
# This Python file uses the following encoding: utf-8
# Author: Carlos Capitán Fernández

'''This script is the emergency management module developed in the UTM to decide what is the best action 
to take in the U-space when some Threats are showed up.'''

import rospy
import time
import copy
from gauss_msgs.srv import Threats, ThreatsResponse, ThreatsRequest
from gauss_msgs.srv import PilotAnswer, PilotAnswerResponse, PilotAnswerRequest
from gauss_msgs.srv import Notifications, NotificationsRequest
from gauss_msgs.srv import WriteGeofences, WriteGeofencesRequest 
from gauss_msgs.msg import Threat, Circle, Notification, Waypoint, WaypointList
from gauss_msgs.msg import Operation, Geofence, DeconflictionPlan, ConflictiveOperation
from gauss_msgs.srv import Deconfliction, DeconflictionRequest
from gauss_msgs.srv import UpdateThreats, UpdateThreatsRequest

class EmergencyManagement():

    class Threat2Solve:
        def __init__(self, threat_time, threat_status, threat):
            self.timestamp = threat_time 
            self.status = threat_status 
            self.threat_msg = threat 

    def __init__(self): 
        
        # Initialization

        self._threats_list = [] 
        self._notifications_list = []   
        self._conflictive_operations = []
        self._conflictive_geofences = []    
        
        # Server     

        self._threats_service = rospy.Service('/gauss/threats', Threats, self.service_threats_cb) 
        
        self._pilot_answer_service = rospy.Service('/gauss/pilotanswer', PilotAnswer, self.service_pilot_answer_cb)
        
        # Wait until services are available and create connection
        
        rospy.wait_for_service('/gauss/tactical_deconfliction')                    
        self._requestDeconfliction_service_handle = rospy.ServiceProxy('/gauss/tactical_deconfliction', Deconfliction) 

        rospy.wait_for_service('/gauss/write_geofences')                    
        self._writeGeofences_service_handle = rospy.ServiceProxy('/gauss/write_geofences', WriteGeofences) 

        rospy.wait_for_service('/gauss/notifications')                    
        self._notifications_service_handle = rospy.ServiceProxy('/gauss/notifications', Notifications) 

        rospy.wait_for_service('/gauss/update_threats')                    
        self._update_threats_service_handle = rospy.ServiceProxy('/gauss/update_threats', UpdateThreats) 

        # Timer

        self.timer = rospy.Timer(rospy.Duration(10), self.timer_cb)
        
        print("Started Emergency Management(EM) module!. This module proposes threats solutions to the USP module.")

    def send_notifications(self, notifications):
        request = NotificationsRequest()
        request.notifications = self._notifications_list
        request.operations = self._conflictive_operations
        response = self._notifications_service_handle(request)
        return response
    
    def send_threat2deconfliction(self, threat2deconflicted): 
        request = DeconflictionRequest()
        request.tactical = True
        request.threat = threat2deconflicted 
        request.operations = self._conflictive_operations 
        request.geofences = self._conflictive_geofences 
        self._deconfliction_response = self._requestDeconfliction_service_handle(request) 
        return self._deconfliction_response
   
    def select_optimal_route(self):
        'Select the optimal deconfliction route'
        deconfliction_plans_list = self._deconfliction_response.deconfliction_plans
        values = []
        for deconfliction_plan in deconfliction_plans_list:
            alfa = 0.25 # Weight of cost
            beta = 0.75 # Weight of riskiness
            value = alfa*deconfliction_plan.cost + beta*deconfliction_plan.riskiness
            values.append(value)
        value_min = min(values)
        pos_min = values.index(value_min)
        best_solution = deconfliction_plans_list[pos_min]
        self._uav_id_afected = best_solution.uav_id
        return best_solution

    def create_new_flight_plan(self, conflictive_operations, threat2solve, maneuver, tactical_wps):
        print "Create new flight plan function"
        conflictive_operation = ConflictiveOperation()
        conflict_operation_list = conflictive_operations
        new_flight_plan = WaypointList()
        merge2end = False
        flighplansection = 0
        threat = Threat() 
        threat = threat2solve
        maneuver = maneuver
        tactical_wps = WaypointList(tactical_wps)
        
        for conflictive_operation in conflict_operation_list:
            if conflictive_operation.uav_id == self._uav_id_afected:
                flightplan = conflictive_operation.flight_plan_updated
                print "Conflictive operation flight plan updated"
                print flightplan
                current_wp = conflictive_operation.current_wp
                print "current waypoint"
                print current_wp
                actual_wp  = conflictive_operation.actual_wp
                print "actual waypoint"
                print actual_wp

        if threat.threat_type == threat.GEOFENCE_CONFLICT:
            if maneuver == 1: # Route to my destination avoiding a geofence.
                merge2end = True
                flighplansection = 0
            elif maneuver == 3: # Route for going back home.
                merge2end = False
                flighplansection = 2

        elif threat.threat_type == threat.GEOFENCE_INTRUSION:
            if maneuver == 2: # Route to my destination by the shortest way.
                merge2end = False
                flighplansection = 2
            elif maneuver == 3: # Route for going back home.
                merge2end = False
                flighplansection = 2
            elif maneuver == 6: # Route to my destination leaving the geofence asap.
                merge2end = True
                flighplansection = 2   

        elif threat.threat_type == threat.GNSS_DEGRADATION:
            if maneuver == 5: # Route for landing in a landing spot.
                merge2end = False
                flighplansection = 2

        elif threat.threat_type == threat.LACK_OF_BATTERY:
            if maneuver == 5: # Route for landing in a landing spot.
                merge2end = False
                flighplansection = 2

        elif threat.threat_type == threat.LOSS_OF_SEPARATION:
            merge2end = True
            flighplansection = 0
            print "LOSS OF SEPARATION THREAT"
    
        elif threat.threat_type == threat.UAS_OUT_OV:
            if maneuver == 9: # Route for going back to the flight geometry and its flight plan.
                merge2end = True
                flighplansection = 2
            elif maneuver == 10: # Route to keep the flight plan. No matter how much the uav is out of the OV.
                merge2end = True
                flighplansection = 2
        
        # change_path_ref = False
        # rospy.loginfo('to_end = {}, section = {}'.format(merge2end, flighplansection))
        new_flight_plan.waypoints.append(actual_wp)
        for i in range(len(flightplan.waypoints)):
            # print("i = {} ________________________________________________".format(i))
            if flighplansection == 0: # Fist section. Do anything until current waypoint.
                if (flightplan.waypoints[i].x == flightplan.waypoints[current_wp].x and
                flightplan.waypoints[i].y == flightplan.waypoints[current_wp].y and
                flightplan.waypoints[i].z == flightplan.waypoints[current_wp].z):
                    flighplansection = 1
                    # rospy.loginfo('section = {}'.format(flighplansection))

            if flighplansection == 1: # Introduce waypoints between the current waypoint and the first one of the solution
                if (flightplan.waypoints[i].x == tactical_wps.waypoints[0].x and
                flightplan.waypoints[i].y == tactical_wps.waypoints[0].y and
                flightplan.waypoints[i].z == tactical_wps.waypoints[0].z):
                    flighplansection = 2 
                    # rospy.loginfo('section = {}'.format(flighplansection))
                else:
                    temp_pose = Waypoint()
                    temp_pose.x = flightplan.waypoints[i].x
                    temp_pose.y = flightplan.waypoints[i].y
                    temp_pose.z = flightplan.waypoints[i].z
                    temp_pose.stamp = flightplan.waypoints[i].stamp
                    new_flight_plan.waypoints.append(temp_pose)
                    # print("new_flight_plan at {}: {}".format(i, new_flight_plan))

            elif flighplansection == 2: #Introduce the solution
                for j in range(len(tactical_wps.waypoints)):
                    # print("j = {} _____________".format(j))
                    temp_pose = Waypoint()
                    temp_pose.x = tactical_wps.waypoints[j].x
                    temp_pose.y = tactical_wps.waypoints[j].y
                    temp_pose.z = tactical_wps.waypoints[j].z
                    temp_pose.stamp = tactical_wps.waypoints[j].stamp
                    new_flight_plan.waypoints.append(temp_pose)
                    # print("new_flight_plan at {}, {}: {}".format(i, j, new_flight_plan))
                if merge2end:
                    flighplansection = 3
                    # rospy.loginfo('section = {}'.format(flighplansection))
                else:
                    # i = len(flightplan.waypoints)
                    break

            if flighplansection == 3: #Do nothing until matching the solution with the flight plan
                if (flightplan.waypoints[i].x == tactical_wps.waypoints[-1].x and
                flightplan.waypoints[i].y == tactical_wps.waypoints[-1].y and
                flightplan.waypoints[i].z == tactical_wps.waypoints[-1].z):
                    flighplansection = 4

            elif flighplansection == 4: #Introduce the rest of the flight plan
                temp_pose = Waypoint()
                temp_pose.x = flightplan.waypoints[i].x
                temp_pose.y = flightplan.waypoints[i].y
                temp_pose.z = flightplan.waypoints[i].z
                temp_pose.stamp = flightplan.waypoints[i].stamp
                new_flight_plan.waypoints.append(temp_pose)
        print "create_new_flight_plan output: "
        print new_flight_plan
        return new_flight_plan
        

    def ask_update_threat(self, threat_id):
        request = UpdateThreatsRequest()
        request.threat_ids = [threat_id]
        result = self._update_threats_service_handle(request)
        return 
    
    def action_decision_maker(self, threat2solve, conflictive_ops):
        threat = Threat()
        threat = threat2solve
        threat_type = threat.threat_type
        threat_time = threat.header.stamp
        uavs_threatened = list(threat.uav_ids)
        conflictive_operations = conflictive_ops
        actions_dictionary = {1:'Route to my destiny avoiding a geofence', 2:'Route to my destiny for the shortest way',
        3:'Route back home', 4:'Hovering waiting for geofence deactivation', 5:'Route landing in a landing spot',
        6:'Route to my destiny leaving the geofence asap', 7:'Hovering', 8:'Route avoiding the conflict object',
        9:'Route for going back asap to the Flight Geometry and keeping with the Flight Plan'}
        notification = Notification()

                
        if len(uavs_threatened) > 0: 

            '''Threat UAS IN CV: we send a message to the UAV in conflict for going back to the FG.'''
            
            if threat_type == Threat.UAS_IN_CV:
                uav_threatened = uavs_threatened[0]
                notification.description = 'Go back to your flight plan.'
                notification.uav_id = uav_threatened
                self._notifications_list.append(notification)
                            
            '''Threat UAS OUT OV: we ask to TD possible solution trajectories'''

            if threat_type == Threat.UAS_OUT_OV: 
                self.send_threat2deconfliction(threat)
                best_solution = self.select_optimal_route()                
                notification.uav_id = best_solution.uav_id
                notification.action = best_solution.maneuver_type
                notification.description = actions_dictionary[notification.action]
                notification.threat = threat
                notification.maneuver_type = best_solution.maneuver_type
                for conflictive_operation in conflictive_operations:
                    if conflictive_operation.uav_id == self._uav_id_afected:
                        flight_plan = conflictive_operation.flight_plan
                        current_wp = conflictive_operation.current_wp
                notification.current_wp = current_wp
                notification.waypoints = best_solution.waypoint_list
                notification.flight_plan = flight_plan
                notification.new_flight_plan = self.create_new_flight_plan(conflictive_operations, threat, notification.action, best_solution.waypoint_list)
                self._notifications_list.append(notification) 

            '''Threat LOSS OF SEPARATION: we ask to tactical possible solution trajectories'''

            if threat_type == Threat.LOSS_OF_SEPARATION:
                self.send_threat2deconfliction(threat)
                for uav in uavs_threatened:
                    if uav == self._deconfliction_response.deconfliction_plans[0].uav_id:
                        best_solution = self.select_optimal_route()
                        print best_solution.waypoint_list
                notification.uav_id = best_solution.uav_id
                notification.action = best_solution.maneuver_type
                notification.description = actions_dictionary[notification.action]
                notification.threat = threat
                notification.maneuver_type = best_solution.maneuver_type
                for conflictive_operation in conflictive_operations:
                    if conflictive_operation.uav_id == self._uav_id_afected:
                        flight_plan = conflictive_operation.flight_plan
                        current_wp = conflictive_operation.current_wp
                notification.current_wp = current_wp
                notification.waypoints = best_solution.waypoint_list
                notification.flight_plan = flight_plan
                notification.new_flight_plan = self.create_new_flight_plan(conflictive_operations, threat, notification.action, best_solution.waypoint_list)
                print "Emergency management sending alternative flight plan in notification"
                print notification.new_flight_plan
                self._notifications_list.append(notification) 

            '''Threat ALERT WARNING: we create a cylindrical geofence with center in "location". Besides, we notifies to all UAVs the alert detected'''
               
            if threat_type == Threat.ALERT_WARNING:    
                #We send a notification for every UAV.
                for uav in uavs_threatened:
                    notification.description = 'Alert Warning: Bad weather Fire or NDZ detected in the zone.'
                    notification.uav_id = uavs_threatened[uav]
                    self._notifications_list.append(notification) 
                    
                #Creation of the NDZ.
                geofence_base = Circle()
                alarm_center = threat.location
                geofence_base.x_center = alarm_center.x
                geofence_base.y_center = alarm_center.y
                geofence = Geofence()          
                geofence.id = 1
                geofence.cylinder_shape = True
                geofence.min_altitude = 0.0
                geofence.max_altitude = 50.0
                geofence.circle = geofence_base

                # We write a geofence.
                req = WriteGeofencesRequest()
                req.geofence_ids = [geofence.id]
                req.geofences = [geofence]
                res = self._writeGeofences_service_handle(req)
                res.message = "Geofence stored in the Data Base."   

            '''Threat GEOFENCE INTRUSION: we ask to tactical possible solution trajectories'''

            if threat_type == Threat.GEOFENCE_INTRUSION:   
                #Publish the action which the UAV has to make.
                self.send_threat2deconfliction(threat)
                best_solution = self.select_optimal_route()
                notification.uav_id = best_solution.uav_id
                notification.action = best_solution.maneuver_type
                notification.description = actions_dictionary[notification.action]
                notification.threat = threat
                notification.maneuver_type = best_solution.maneuver_type
                for conflictive_operation in conflictive_operations:
                    if conflictive_operation.uav_id == self._uav_id_afected:
                        flight_plan = conflictive_operation.flight_plan
                        current_wp = conflictive_operation.current_wp
                notification.current_wp = current_wp
                notification.waypoints = best_solution.waypoint_list
                notification.flight_plan = flight_plan
                notification.new_flight_plan = self.create_new_flight_plan(conflictive_operations, threat, notification.action, best_solution.waypoint_list)
                self._notifications_list.append(notification) 
                    
            '''Threat GEOFENCE CONFLICT: we ask to tactical possible solution trajectories'''

            if threat_type == Threat.GEOFENCE_CONFLICT:     
                self.send_threat2deconfliction(threat)
                best_solution = self.select_optimal_route()
                notification.uav_id = best_solution.uav_id
                notification.action = best_solution.maneuver_type
                notification.description = actions_dictionary[notification.action]
                notification.threat = threat
                notification.maneuver_type = best_solution.maneuver_type
                for conflictive_operation in conflictive_operations:
                    if conflictive_operation.uav_id == self._uav_id_afected:
                        flight_plan = conflictive_operation.flight_plan
                        current_wp = conflictive_operation.current_wp
                notification.current_wp = current_wp
                notification.waypoints = best_solution.waypoint_list
                notification.flight_plan = flight_plan
                notification.new_flight_plan = self.create_new_flight_plan(conflictive_operations, threat, notification.action, best_solution.waypoint_list)
                self._notifications_list.append(notification) 
                
            '''Threat TECHNICAL FAILURE: we send a message to the UAV in conflict for landing now.'''

            if threat_type == Threat.TECHNICAL_FAILURE:  
                #Publish the action which the UAV has to make.
                uav_threatened = uavs_threatened[0]
                notification.description = 'URGENT: Land now.'
                notification.uav_id = uav_threatened
                self._notifications_list.append(notification) 
                
                # We create a geofence.
                geofence_base = Circle()
                alarm_center = threat.location
                geofence_base.x_center = alarm_center.x
                geofence_base.y_center = alarm_center.y
                geofence = Geofence()
                geofence.id = 2
                geofence.cylinder_shape = True
                geofence.min_altitude = 0.0
                geofence.max_altitude = 100.0
                geofence.circle = geofence_base

                # We write a geofence.
                req = WriteGeofencesRequest()
                req.geofence_ids = [geofence.id]
                req.geofences = [geofence]
                res = self._writeGeofences_service_handle(req)
                res.message = "Geofence stored in the Data Base."

            '''Threat COMMUNICATION FAILURE: we EM can not do anything if there is a lost of the link communication between the GCS and/or the
                UAV and USP.'''

            if threat_type == Threat.COMMUNICATION_FAILURE:         
                #Publish the action which the UAV has to make.
                uav_threatened = uavs_threatened[0]
                notification.description = 'Change UAV control mode from autonomous to manual.'
                notification.uav_id = uav_threatened
                self._notifications_list.append(notification) 
                    
            '''Threat LACK OF BATTERY: we ask to tactical possible solution trajectories'''

            if threat_type == Threat.LACK_OF_BATTERY:  
                #Publish the action which the UAV has to make.
                self.send_threat2deconfliction(threat)
                best_solution = self.select_optimal_route()
                notification.uav_id = best_solution.uav_id
                notification.action = best_solution.maneuver_type
                notification.description = actions_dictionary[notification.action]
                notification.threat = threat
                notification.maneuver_type = best_solution.maneuver_type
                for conflictive_operation in conflictive_operations:
                    if conflictive_operation.uav_id == self._uav_id_afected:
                        flight_plan = conflictive_operation.flight_plan
                        current_wp = conflictive_operation.current_wp
                notification.current_wp = current_wp
                notification.waypoints = best_solution.waypoint_list
                notification.flight_plan = flight_plan
                notification.new_flight_plan = self.create_new_flight_plan(conflictive_operations, threat, notification.action, best_solution.waypoint_list)
                self._notifications_list.append(notification) 
                    
            '''Threat JAMMING ATTACK: We send a message for landing within the geofence created
                around the UAV.'''

            if threat_type == Threat.JAMMING_ATTACK: 
                #Publish the action which the UAV has to make.
                uav_threatened = uavs_threatened[0]
                notification.description = 'Attention: Jamming attack. Land now'
                notification.uav_id = uav_threatened
                notification.threat.threat_type = Threat.JAMMING_ATTACK
                self._notifications_list.append(notification) 
                
                # We create a geofence.
                geofence_base = Circle()
                alarm_center = threat.location
                geofence_base.x_center = alarm_center.x
                geofence_base.y_center = alarm_center.y
                geofence_base.radius = 60
                geofence = Geofence()
                geofence.id = 1
                geofence.min_altitude = 0.0
                geofence.max_altitude = 100.0
                geofence.circle = geofence_base
                geofence.cylinder_shape = True
                geofence.start_time = rospy.Time().now()
                geofence.end_time = rospy.Time().from_sec(rospy.Time.now().to_sec() + 800.0)
    
                # We write a geofence.
                req = WriteGeofencesRequest()
                req.geofence_ids = [geofence.id]
                req.geofences = [geofence]
                res = self._writeGeofences_service_handle(req)
                res.message = "Geofence stored in the Data Base."       

            '''Threat SPOOFING ATTACK: We send a recommendation to the UAV in order to activate the FTS
                and we create a geofence around the UAV.'''

            if threat_type == Threat.SPOOFING_ATTACK: 
                #Publish the action which the UAV has to make.
                uav_threatened = uavs_threatened[0]
                notification.description = 'Activate the Flight Termination System (FTS) of the UAV.'
                notification.uav_id = uav_threatened
                self._notifications_list.append(notification) 
            
                # We create a geofence.
                geofence_base = Circle()
                alarm_center = threat.location
                geofence_base.x_center = alarm_center.x
                geofence_base.y_center = alarm_center.y
                geofence = Geofence()
                geofence.id = 4
                geofence.cylinder_shape = True
                geofence.min_altitude = 0.0
                geofence.max_altitude = 100.0
                geofence.circle = geofence_base
                    
                # We write a geofence.
                req = WriteGeofencesRequest()
                req.geofence_ids = [geofence.id]
                req.geofences = [geofence]
                res = self._writeGeofences_service_handle(req)
                res.message = "Geofence stored in the Data Base."   

            '''Threat GNSS DEGRADATION: we wait a period of time and then we ask to tactical
                possible trajectories to landing spots'''

            if threat_type == Threat.GNSS_DEGRADATION: 
                #Publish the action which the UAV has to make.
                self.send_threat2deconfliction(threat)
                best_solution = self.select_optimal_route()
                notification.uav_id = best_solution.uav_id
                notification.action = best_solution.maneuver_type
                notification.description = actions_dictionary[notification.action]
                notification.threat = threat
                notification.maneuver_type = best_solution.maneuver_type
                for conflictive_operation in conflictive_operations:
                    if conflictive_operation.uav_id == self._uav_id_afected:
                        flight_plan = conflictive_operation.flight_plan
                        current_wp = conflictive_operation.current_wp
                notification.current_wp = current_wp
                notification.waypoints = best_solution.waypoint_list
                notification.flight_plan = flight_plan
                notification.new_flight_plan = self.create_new_flight_plan(conflictive_operations, threat, notification.action, best_solution.waypoint_list)
                self._notifications_list.append(notification) 
        self.send_notifications(self._notifications_list)

    def service_threats_cb(self, request):
        req = ThreatsRequest()
        req = copy.deepcopy(request)
        num = len(req.threats) 
        rospy.loginfo("EM has received %d threats!", num) 
        zero_time = rospy.Time()
        for i in range(num):
            threat = EmergencyManagement.Threat2Solve(rospy.Time.now(), 'TODO', req.threats[i])
            self._threats_list.append(threat)
        for i in range(len(req.operations)):
            self._conflictive_operations.append(req.operations[i])
        for i in range(len(req.geofences)):
            self._conflictive_geofences.append(req.geofences[i])    
        res = ThreatsResponse()
        res.success = True
        return res 

    def service_pilot_answer_cb(self, request):
        req = PilotAnswerRequest()
        req = copy.deepcopy(request)
        rospy.loginfo("There are new pilot answers")
        threat_ids = list(req.threat_ids)
        print("THE PILOT CORRESPONDING TO THREAT_ID:", threat_ids)
        answers = req.pilot_answers 
        print("THE PILOT DECIDES WHAT TO DO WITH REGARD THE UTM PROPOSAL:", req.pilot_answers)
        num = len(threat_ids)
        if num > 0:
            for i in range(num):
                self._threats_list[i].status = answers[i] 
        res = PilotAnswerResponse()
        res.success = True
        return res

    def timer_cb(self, timer):
        num = len(self._threats_list)
        rospy.loginfo("Right now, there are %d active threats in the U-space", num)
        if num > 0:
            for threat in self._threats_list:           
                if threat.status == 'TODO': 
                    conflictive_operations = self._conflictive_operations
                    self.action_decision_maker(threat.threat_msg, conflictive_operations)
                    threat.status = 'DONE'
                if threat.status == 'REJECTED':
                    threat_id = [threat.threat_msg.threat_id]
                    threat_updated = self.ask_update_threat(threat_id).threats
                    conflictive_operations_updated = self.ask_update_threat(threat_id).operations
                    self.action_decision_maker(threat.threat_msg, conflictive_operations_updated)
                    threat.status = 'DONE'
                        
        rospy.loginfo("The number of Threats active in the U-space is %d", num)

''' The node and the EmergencyManagement class are initialized'''

if __name__=='__main__':

    rospy.init_node('emergency_management')
    e = EmergencyManagement()
    rospy.spin()   




    
