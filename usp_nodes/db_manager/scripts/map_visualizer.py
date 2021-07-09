#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

def main():
    rospy.init_node('map_visualizer')
    rospy.loginfo('[Viz] Started Map Visualizer node!')
 
    update_rate = 1.0
    global_frame_id = 'map'
    visualization_topic = 'map_visualization_marker_array'
    # Default map is atlas TODO: from parameters!
    map_url = 'package://db_manager/maps/atlas.dae'
    map_position = Point()
    map_scale = 2080.92
    # Uncomment for Loring map
    # map_url = 'package://db_manager/maps/loring.dae'
    # map_position.y = -30  # TODO: visual correction
    # map_scale = 3048

    visualization_pub = rospy.Publisher(visualization_topic, MarkerArray, queue_size=1)
    rate = rospy.Rate(update_rate)
    while not rospy.is_shutdown():

        marker_array = MarkerArray()
        map_marker = Marker()
        map_marker.header.stamp = rospy.Time.now()
        map_marker.header.frame_id = global_frame_id
        map_marker.ns = 'map'
        map_marker.lifetime = rospy.Duration()
        map_marker.id = 0
        map_marker.type = Marker.MESH_RESOURCE
        map_marker.mesh_resource = map_url
        map_marker.action = Marker.ADD
        map_marker.pose.position = map_position
        map_marker.pose.orientation.w = 1
        map_marker.scale.x = map_scale
        map_marker.scale.y = map_scale
        map_marker.scale.z = map_scale
        map_marker.mesh_use_embedded_materials = True
        marker_array.markers.append(map_marker)

        visualization_pub.publish(marker_array)

        rate.sleep()

if __name__ == '__main__':
    main()
