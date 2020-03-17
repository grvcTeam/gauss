#!/usr/bin/env python

import rospy
from test_mqtt.msg import GeofenceCreation

from datetime import datetime

def talker():
    pub = rospy.Publisher('geofence_creation', GeofenceCreation, queue_size=10)
    rospy.init_node('geofence_creation', anonymous=False)
    rate = rospy.Rate(10)
    dt1 = datetime(2020, 04, 01)
    dt2 = datetime(2020, 04, 02)
    utc_ms_1 = (dt1 - datetime(1970, 1, 1)).total_seconds()*1000
    utc_ms_2 = (dt2 - datetime(1970, 1, 1)).total_seconds()*1000
    while not rospy.is_shutdown():
        geofence = GeofenceCreation()
        geofence.latitude = 37.411394
        geofence.longitude = -6.001400
        geofence.circle_radius = 500
        geofence.start_altitude = 0
        geofence.end_altitude = 200
        geofence.start_time = utc_ms_1
        geofence.finish_time = utc_ms_2
        pub.publish(geofence)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

