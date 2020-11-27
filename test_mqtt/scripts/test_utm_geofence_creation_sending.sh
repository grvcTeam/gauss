#!/bin/bash

rostopic pub /gauss/geofence_creation gauss_msgs_mqtt/UTMGeofenceCreation -f Temporal_geofence.yaml
sleep 2
rostopic pub /gauss/geofence_creation gauss_msgs_mqtt/UTMGeofenceCreation -f Permanent_geofence.yaml