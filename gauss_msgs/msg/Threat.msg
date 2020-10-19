Header header
uint8 threat_type  
uint8 threat_id
int8[] uav_ids
time[] times
int8[] geofence_ids
int8[] priority_ops
Waypoint location

uint8 SPOOFING_ATTACK = 0       # Detected by USP
uint8 TECHNICAL_FAILURE = 1     # Detected by USP
uint8 COMMUNICATION_FAILURE = 2 # Detected by USP
uint8 JAMMING_ATTACK = 3        # Detected by USP
uint8 LOSS_OF_SEPARATION = 4    # Detected by Monitoring
uint8 GEOFENCE_INTRUSION = 5    # Detected by Monitoring
uint8 UAS_OUT_OV = 6            # Detected by Monitoring
uint8 GEOFENCE_CONFLICT = 7     # Detected by Monitoring
uint8 ALERT_WARNING = 8         # Detected by USP
uint8 LACK_OF_BATTERY = 9       # Detected by USP
uint8 GNSS_DEGRADATION = 10     # Detected by USP
uint8 UAS_IN_CV = 11            # Detected by Monitoring
