#!/usr/bin/env python
# This Python file uses the following encoding: utf-8

import rospy
import time
import copy
from gauss_msgs.srv import Threats, ThreatsResponse, ThreatsRequest
from gauss_msgs.srv import PilotAnswer, PilotAnswerResponse, PilotAnswerRequest
from gauss_msgs.srv import ReadOperation, ReadOperationRequest
from gauss_msgs.srv import Notifications, NotificationsRequest
from gauss_msgs.srv import WriteGeofences, WriteGeofencesRequest 
from gauss_msgs.msg import Threat, Circle, Notification, Waypoint, WaypointList, Operation, Geofence, DeconflictionPlan, PoseStamped
from gauss_msgs.srv import Deconfliction, DeconflictionRequest

def create_new_flight_plan(self, flightplan, threat, maneuver):
    new_flight_plan = ¿? # es una lista de wps? o qué es?
    merge2end = False
    flighplansection = 0
    threat = Threat(threat) #no es necesaria.
    maneuver = maneuver

    if threat.threat_type == threat.GEOFENCE_CONFLICT:
        if maneuver == 1: # Route to my destination avoiding a geofence.
            merge2end = True
            flighplansection = 0
            break
        if maneuver == 3: # Route for going back home.
            merge2end = False
            flighplansection = 2
            break
        break

    elif threat.threat_type == threat.GEOFENCE_INTRUSION:
        if maneuver == 2: # Route to my destination by the shortest way.
            merge2end = False
            flighplansection = 2
            break
        if maneuver == 3: # Route for going back home.
            merge2end = False
            flighplansection = 2
            break
        if maneuver == 6: # Route to my destination leaving the geofence asap.
            merge2end = False
            flighplansection = 2
            break
        break   

    elif threat.threat_type == threat.GNSS_DEGRADATION:
        if maneuver == 5: # Route for landing in a landing spot.
            merge2end = False
            flighplansection = 2
            break
        break

    elif threat.threat_type == threat.LACK_OF_BATTERY:
        if maneuver == 5: # Route for landing in a landing spot.
            merge2end = False
            flighplansection = 2
            break 
        break

    elif threat.threat_type == threat.LOSS_OF_SEPARATION:
        merge2end = True
        flighplansection = 0
        break

    elif threat.threat_type == threat.UAS_OUT_OV:
        if maneuver == 9: # Route for going back to the flight geometry and its flight plan.
            merge2end = True
            flighplansection = 2
            break    
        if maneuver == 10: # Route to keep the flight plan. No matter how much the uav is out of the OV.
            merge2end = True
            flighplansection = 2
            break    
        break
    
    change_path_ref = False
    for i in range(len(flightplan.waypoints)):
        temp_pose = PoseStamped()
        
        if flighplansection == 0: # Fist section. Do anything until current waypoint.
            if ():
                flighplansection = 1
            else:
                break
        elif flighplansection = 1: # Introduce waypoints between the current waypoint and the first one of the solution

        elif flighplansection = 2: #Introduce the solution

            