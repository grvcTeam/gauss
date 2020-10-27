#!/usr/bin/env python

import copy
import rospy
from gauss_msgs.srv import ReadIcao, ReadIcaoRequest, ReadIcaoResponse
from gauss_msgs.srv import ReadOperation, ReadOperationRequest, ReadOperationResponse
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

# Colors from An Optimum 16 Color Palette [http://alumni.media.mit.edu/~wad/color/palette.html]
RED         = ColorRGBA(0.678, 0.137, 0.137, 1.0)  # RGB: 173,  35,  35
ORANGE      = ColorRGBA(1.000, 0.573, 0.200, 1.0)  # RGB: 255, 146,  51
YELLOW      = ColorRGBA(1.000, 0.933, 0.200, 1.0)  # RGB: 255, 238,  51
LIGHT_GREEN = ColorRGBA(0.506, 0.773, 0.478, 1.0)  # RGB: 129, 197, 122
GREEN       = ColorRGBA(0.114, 0.412, 0.078, 1.0)  # RGB:  29, 105,  20
CYAN        = ColorRGBA(0.161, 0.816, 0.816, 1.0)  # RGB:  41, 208, 208
LIGHT_BLUE  = ColorRGBA(0.616, 0.686, 1.000, 1.0)  # RGB: 157, 175, 255
BLUE        = ColorRGBA(0.165, 0.294, 0.843, 1.0)  # RGB:  42,  75, 215
PINK        = ColorRGBA(1.000, 0.804, 0.953, 1.0)  # RGB: 255, 205, 243
PURPLE      = ColorRGBA(0.506, 0.149, 0.753, 1.0)  # RGB: 129,  38, 192
TAN         = ColorRGBA(0.914, 0.871, 0.733, 1.0)  # RGB: 233, 222, 187
BROWN       = ColorRGBA(0.506, 0.290, 0.098, 1.0)  # RGB: 129,  74,  25
WHITE       = ColorRGBA(1.000, 1.000, 1.000, 1.0)  # RGB: 255, 255, 255
LIGHT_GRAY  = ColorRGBA(0.627, 0.627, 0.627, 1.0)  # RGB: 160, 160, 160
DARK_GRAY   = ColorRGBA(0.341, 0.341, 0.341, 1.0)  # RGB:  87,  87,  87
BLACK       = ColorRGBA(0.000, 0.000, 0.000, 1.0)  # RGB:   0,   0,   0
# TODO: Alpha variable?

# TODO: Extend to MarkersConfig?
class WaypointListColorScheme(object):
    def __init__(self, path_color, mandatory_wp_color = None, non_mandatory_wp_color = None, stamp_color = None):
        self.path = path_color
        self.mandatory_wp = mandatory_wp_color
        self.non_mandatory_wp = non_mandatory_wp_color
        self.stamp = stamp_color

def create_waypointlist_markerarray(waypointlist, frame_id, ns, lifetime, color_scheme):

    marker_common = Marker()
    marker_common.header.stamp = rospy.Time.now()
    marker_common.header.frame_id = frame_id
    marker_common.ns = ns
    marker_common.lifetime = lifetime

    path_marker = copy.deepcopy(marker_common)
    path_marker.id = 0
    path_marker.type = Marker.LINE_STRIP
    path_marker.action = Marker.ADD
    path_marker.scale.x = 0.1
    path_marker.color = color_scheme.path

    mandatory_wp_marker = copy.deepcopy(marker_common)
    mandatory_wp_marker.id = 1
    mandatory_wp_marker.type = Marker.SPHERE_LIST  # TODO: Or cubes?
    mandatory_wp_marker.action = Marker.ADD
    mandatory_wp_marker.scale.x = 0.2
    mandatory_wp_marker.color = color_scheme.mandatory_wp

    non_mandatory_wp_marker = copy.deepcopy(marker_common)
    non_mandatory_wp_marker.id = 2
    non_mandatory_wp_marker.type = Marker.SPHERE_LIST
    non_mandatory_wp_marker.action = Marker.ADD
    non_mandatory_wp_marker.scale.x = 0.2
    non_mandatory_wp_marker.color = color_scheme.non_mandatory_wp

    stamp_marker = copy.deepcopy(marker_common)
    # stamp_marker.id = 10 ... 10+N
    stamp_marker.type = Marker.TEXT_VIEW_FACING
    stamp_marker.action = Marker.ADD
    stamp_marker.scale.z = 0.2
    stamp_marker.color = color_scheme.stamp
    stamp_marker_array = MarkerArray()

    for i, waypoint in enumerate(waypointlist):
        point = Point()
        point.x = waypoint.x
        point.y = waypoint.y
        point.z = waypoint.z

        path_marker.points.append(point)

        if waypoint.mandatory:
            mandatory_wp_marker.points.append(point)
        else:
            non_mandatory_wp_marker.points.append(point)

        stamp_marker.id = 10 + i
        text_point = copy.deepcopy(point)
        text_point.z += 0.2
        stamp_marker.pose.position = text_point
        stamp_marker.text = 't = {}s'.format(waypoint.stamp.to_sec())
        stamp_marker_array.markers.append(copy.deepcopy(stamp_marker))

    markerarray = MarkerArray()
    markerarray.markers.append(path_marker)

    if color_scheme.mandatory_wp:
        markerarray.markers.append(mandatory_wp_marker)

    if color_scheme.non_mandatory_wp:
        markerarray.markers.append(non_mandatory_wp_marker)

    if color_scheme.stamp:
        markerarray.markers.extend(stamp_marker_array.markers)

    return markerarray

def main():
    rospy.init_node('visualizer')
 
    # TODO: From param
    update_rate = 1.0
    global_frame_id = 'map'
    read_icao_srv_url = '/gauss/read_icao'
    read_operation_srv_url = '/gauss/read_operation'
    visualization_topic = 'visualization_marker_array'
    id_to_color = {0: ORANGE, 1: CYAN, 2: PURPLE}

    rospy.logwarn('Waiting for service {}'.format(read_icao_srv_url))
    rospy.wait_for_service(read_icao_srv_url)
    read_icao_service = rospy.ServiceProxy(read_icao_srv_url, ReadIcao)

    rospy.logwarn('Waiting for service {}'.format(read_operation_srv_url))
    rospy.wait_for_service(read_operation_srv_url)
    read_operation_service = rospy.ServiceProxy(read_operation_srv_url, ReadOperation)

    visualization_pub = rospy.Publisher(visualization_topic, MarkerArray, queue_size=1)

    rospy.loginfo('Started visualization node!')
    rate = rospy.Rate(update_rate)
    while not rospy.is_shutdown():

        read_icao_response = read_icao_service.call(ReadIcaoRequest())
        if not read_icao_response.success:
            rospy.logwarn('Read icao did not succeed')
            rospy.logwarn(read_icao_response.message)

        read_operation_request = ReadOperationRequest()
        read_operation_request.uav_ids = read_icao_response.uav_id
        read_operation_response = read_operation_service.call(read_operation_request)

        if not read_operation_response.success:
            rospy.logwarn('Read operation did not succeed')
            rospy.logwarn(read_operation_response.message)

        marker_frame_id = global_frame_id
        marker_lifetime = rospy.Duration(1.0/update_rate)
        marker_array = MarkerArray()
        for operation in read_operation_response.operation:
            operation_ns = operation.icao_address
            id_color = id_to_color[operation.uav_id]

            flight_plan_ns = operation_ns + '/flight_plan'
            flight_plan_color_scheme = WaypointListColorScheme(id_color, id_color, None, id_color)
            flight_plan = create_waypointlist_markerarray(operation.flight_plan.waypoints, marker_frame_id, flight_plan_ns, marker_lifetime, flight_plan_color_scheme)
            marker_array.markers.extend(flight_plan.markers)

            track_ns = operation_ns + '/track'
            track_color_scheme = WaypointListColorScheme(id_color, WHITE, id_color, id_color)
            track = create_waypointlist_markerarray(operation.track.waypoints, marker_frame_id, track_ns, marker_lifetime, track_color_scheme)
            marker_array.markers.extend(track.markers)

        visualization_pub.publish(marker_array)

        rate.sleep()

if __name__ == '__main__':
    main()
