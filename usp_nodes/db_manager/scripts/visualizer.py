#!/usr/bin/env python

import copy
import rospy
from gauss_msgs.msg import Operation
from gauss_msgs.srv import ReadIcao, ReadIcaoRequest, ReadIcaoResponse
from gauss_msgs.srv import ReadOperation, ReadOperationRequest, ReadOperationResponse
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import ColorRGBA

class ColorPalette(object):
    def __init__(self):
        self.colors = {}
        self.indexed_colors = {}

    def add_color(self, label, r, g, b):
        self.colors[label] = (r, g, b)

    def get_color(self, label, alpha = 1.0):
        if label in self.colors:
            (r, g, b) = self.colors[label]
            return ColorRGBA(r/255.0, g/255.0, b/255.0, alpha)
        else:
            rospy.logwarn('Color [{}] not found in palette, using black'.format(label))
            return ColorRGBA(0, 0, 0, alpha)

    # def set_color_index(self, label, index):
    #     if label in self.colors:
    #         self.indexed_colors[index] = label            
    #     else:
    #         rospy.logwarn('Color [{}] not found in palette, ignoring indexing'.format(label))

    # def get_indexed_color(self, index, alpha = 1.0):
    #     if index in self.indexed_colors:
    #         return self.get_color(self.indexed_colors[index], alpha)
    #     else:
    #         rospy.logwarn('Index [{}] not found in palette, using black'.format(index))
    #         return ColorRGBA(0, 0, 0, alpha)


palette = ColorPalette()
# Colors from An Optimum 16 Color Palette [http://alumni.media.mit.edu/~wad/color/palette.html]
palette.add_color('red',         173,  35,  35)
palette.add_color('orange',      255, 146,  51)
palette.add_color('yellow',      255, 238,  51)
palette.add_color('light_green', 129, 197, 122)
palette.add_color('green',        29, 105,  20)
palette.add_color('cyan',         41, 208, 208)
palette.add_color('light_blue',  157, 175, 255)
palette.add_color('blue',         42,  75, 215)
palette.add_color('pink',        255, 205, 243)
palette.add_color('purple',      129,  38, 192)
palette.add_color('tan',         233, 222, 187)
palette.add_color('brown',       129,  74,  25)
palette.add_color('white',       255, 255, 255)
palette.add_color('light_gray',  160, 160, 160)
palette.add_color('dark_gray',    87,  87,  87)
palette.add_color('black',         0,   0,   0)

# palette.set_color_index('orange',      0)
# palette.set_color_index('yellow',      1)
# palette.set_color_index('light_green', 2)
# palette.set_color_index('cyan',        3)
# palette.set_color_index('light_blue',  4)
# palette.set_color_index('pink',        5)
# palette.set_color_index('purple',      6)
# palette.set_color_index('tan',         7)
# palette.set_color_index('brown',       8)
# palette.set_color_index('light_gray',  9)

class WaypointListColorScheme(object):
    def __init__(self, path_color = None, mandatory_wp_color = None, non_mandatory_wp_color = None, stamp_color = None):
        self.path = path_color
        self.mandatory_wp = mandatory_wp_color
        self.non_mandatory_wp = non_mandatory_wp_color
        self.stamp = stamp_color

class WaypointListViz(object):
    def __init__(self, frame_id, lifetime):
        self.frame_id = frame_id
        self.lifetime = lifetime
        # Default values for markers:
        self.path_type = Marker.LINE_STRIP
        self.path_scale = Vector3(0.1, 0, 0)
        self.mandatory_wp_type = Marker.CUBE_LIST
        self.mandatory_wp_scale = Vector3(0.15, 0.15, 0.15)
        self.non_mandatory_wp_type = Marker.SPHERE_LIST
        self.non_mandatory_wp_scale = Vector3(0.15, 0, 0)
        self.stamp_font_size = 0.2
        self.stamp_offset = Vector3(0, 0, 0.2)

    def config_path(self, marker_type, marker_scale):
        self.path_type = marker_type
        self.path_scale = marker_scale

    def config_mandatory_wp(self, marker_type, marker_scale):
        self.mandatory_wp_type = marker_type
        self.mandatory_wp_scale = marker_scale

    def config_non_mandatory_wp(self, marker_type, marker_scale):
        self.non_mandatory_wp_type = marker_type
        self.non_mandatory_wp_scale = marker_scale

    def config_stamp(self, font_size, offset):
        self.stamp_font_size = font_size
        self.stamp_offset = offset

    def get_markerarray(self, waypointlist, ns, color_scheme):
        marker_common = Marker()
        marker_common.header.stamp = rospy.Time.now()
        marker_common.header.frame_id = self.frame_id
        marker_common.ns = ns
        marker_common.lifetime = self.lifetime

        path_marker = copy.deepcopy(marker_common)
        path_marker.id = 0
        path_marker.type = self.path_type
        path_marker.action = Marker.ADD
        path_marker.scale = self.path_scale
        path_marker.color = color_scheme.path

        mandatory_wp_marker = copy.deepcopy(marker_common)
        mandatory_wp_marker.id = 1
        mandatory_wp_marker.type = self.mandatory_wp_type
        mandatory_wp_marker.action = Marker.ADD
        mandatory_wp_marker.scale = self.mandatory_wp_scale
        mandatory_wp_marker.color = color_scheme.mandatory_wp

        non_mandatory_wp_marker = copy.deepcopy(marker_common)
        non_mandatory_wp_marker.id = 2
        non_mandatory_wp_marker.type = self.non_mandatory_wp_type
        non_mandatory_wp_marker.action = Marker.ADD
        non_mandatory_wp_marker.scale = self.non_mandatory_wp_scale
        non_mandatory_wp_marker.color = color_scheme.non_mandatory_wp

        stamp_marker = copy.deepcopy(marker_common)
        # stamp_marker.id = 10 ... 10+N
        stamp_marker.type = Marker.TEXT_VIEW_FACING
        stamp_marker.action = Marker.ADD
        stamp_marker.scale.z = self.stamp_font_size
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
            text_point.x += self.stamp_offset.x
            text_point.y += self.stamp_offset.y
            text_point.z += self.stamp_offset.z
            stamp_marker.pose.position = text_point
            stamp_marker.text = 't = {}s'.format(waypoint.stamp.to_sec())
            stamp_marker_array.markers.append(copy.deepcopy(stamp_marker))

        markerarray = MarkerArray()

        if color_scheme.path:
            markerarray.markers.append(path_marker)

        if color_scheme.mandatory_wp:
            markerarray.markers.append(mandatory_wp_marker)

        if color_scheme.non_mandatory_wp:
            markerarray.markers.append(non_mandatory_wp_marker)

        if color_scheme.stamp:
            markerarray.markers.extend(stamp_marker_array.markers)

        return markerarray

# class OperationViz(object):

def main():
    rospy.init_node('visualizer')
 
    # TODO: From param
    update_rate = 1.0
    global_frame_id = 'map'
    read_icao_srv_url = '/gauss/read_icao'
    read_operation_srv_url = '/gauss/read_operation'
    visualization_topic = 'visualization_marker_array'
    id_to_color = {}
    id_to_color[0] = 'orange'
    id_to_color[1] = 'yellow'
    id_to_color[2] = 'light_green'
    id_to_color[3] = 'cyan'
    id_to_color[4] = 'light_blue'
    id_to_color[5] = 'pink'
    id_to_color[6] = 'purple'
    id_to_color[7] = 'tan'
    id_to_color[8] = 'brown'
    id_to_color[9] = 'light_gray'

    rospy.loginfo('Waiting for service {}'.format(read_icao_srv_url))
    rospy.wait_for_service(read_icao_srv_url)
    read_icao_service = rospy.ServiceProxy(read_icao_srv_url, ReadIcao)
    rospy.loginfo('Successfully connected to service {}'.format(read_icao_srv_url))

    rospy.loginfo('Waiting for service {}'.format(read_operation_srv_url))
    rospy.wait_for_service(read_operation_srv_url)
    read_operation_service = rospy.ServiceProxy(read_operation_srv_url, ReadOperation)
    rospy.loginfo('Successfully connected to service {}'.format(read_operation_srv_url))

    visualization_pub = rospy.Publisher(visualization_topic, MarkerArray, queue_size=1)

    rospy.loginfo('Started visualization node!')
    rate = rospy.Rate(update_rate)
    while not rospy.is_shutdown():

        read_icao_response = read_icao_service.call(ReadIcaoRequest())  # TODO: try/catch
        if not read_icao_response.success:
            rospy.logwarn('Read icao did not succeed')
            rospy.logwarn(read_icao_response.message)

        read_operation_request = ReadOperationRequest()
        read_operation_request.uav_ids = read_icao_response.uav_id
        read_operation_response = read_operation_service.call(read_operation_request)

        if not read_operation_response.success:
            rospy.logwarn('Read operation did not succeed')
            rospy.logwarn(read_operation_response.message)

        flight_plan_viz = WaypointListViz(global_frame_id, rospy.Duration(1.0/update_rate))
        track_viz = WaypointListViz(global_frame_id, rospy.Duration(1.0/update_rate))
        landing_spots_viz = WaypointListViz(global_frame_id, rospy.Duration(1.0/update_rate))
        landing_spots_viz.config_non_mandatory_wp(Marker.CUBE_LIST, Vector3(0.5, 0.5, 0))

        marker_array = MarkerArray()
        for operation in read_operation_response.operation:
            operation_ns = operation.icao_address

            # Visualize non-trajectory information
            if operation.track.waypoints:
                current_position = operation.track.waypoints[-1]

                info_marker = Marker()
                info_marker.header.stamp = rospy.Time.now()
                info_marker.header.frame_id = global_frame_id
                info_marker.ns = operation_ns + '/operation_info'
                info_marker.lifetime = rospy.Duration(1.0/update_rate)
                info_marker.id = 0
                info_marker.type = Marker.TEXT_VIEW_FACING
                info_marker.action = Marker.ADD
                info_marker.scale.z = 0.2  # TODO: param font_size
                info_marker.color = palette.get_color(id_to_color[operation.uav_id % 10])  # TODO: param!

                current_point = Point()
                current_point.x = current_position.x
                current_point.y = current_position.y
                current_point.z = current_position.z

                text_point = copy.deepcopy(current_point)
                text_point.x += 0.0  # TODO: param offset (as a function of font_size?)
                text_point.y += 0.0
                text_point.z += -1.2

                # TODO: Check unused information
                info_marker.pose.position = text_point
                info_marker.text = '{} [{}]\n'.format(operation.icao_address, operation.uav_id)
                info_marker.text += 'autonomy: {}m\n'.format(operation.autonomy)
                info_marker.text += 'priority: {}\n'.format(operation.priority)
                info_marker.text += 'dt: {}s\n'.format(operation.dT)
                info_marker.text += 't_mod: {}s\n'.format(operation.flight_plan_mod_t)
                info_marker.text += 't_tracked: {}s\n'.format(operation.time_tracked)
                info_marker.text += 't_horizon: {}s\n'.format(operation.time_horizon)
                info_marker.text += 'flight_geom: {}m\n'.format(operation.flight_geometry)
                info_marker.text += 'operational_vol: {}m\n'.format(operation.operational_volume)
                info_marker.text += 'conop: {}\n'.format(operation.conop)

                marker_array.markers.append(info_marker)

                frame_marker = Marker()
                frame_marker.header.stamp = info_marker.header.stamp
                frame_marker.header.frame_id = info_marker.header.frame_id
                frame_marker.ns = info_marker.ns
                frame_marker.lifetime = info_marker.lifetime
                frame_marker.id = 1
                frame_marker.type = Marker.MESH_RESOURCE
                frame_marker.action = Marker.ADD
                frame_marker.pose.position = current_point
                frame_marker.scale.x = 0.4  # TODO: Param!
                frame_marker.scale.y = 0.4
                frame_marker.scale.z = 0.4
                frame_marker.color = palette.get_color(id_to_color[operation.uav_id % 10])
                if operation.frame == Operation.FRAME_ROTOR:
                    frame_marker.mesh_resource = 'package://db_manager/config/rotor.dae'
                elif operation.frame == Operation.FRAME_FIXEDWING:
                    frame_marker.mesh_resource = 'package://db_manager/config/fixedwing.dae'
                else:
                    rospy.logwarn('Unkwown frame type [{}]'.format(operation.frame))
                    frame_marker.mesh_resource = 'package://db_manager/config/arrow.dae'

                marker_array.markers.append(frame_marker)

                current_wp = operation.flight_plan.waypoints[operation.current_wp]

                current_wp_marker = Marker()
                current_wp_marker.header.stamp = info_marker.header.stamp
                current_wp_marker.header.frame_id = info_marker.header.frame_id
                current_wp_marker.ns = info_marker.ns
                current_wp_marker.lifetime = info_marker.lifetime
                current_wp_marker.id = 2
                current_wp_marker.type = Marker.SPHERE
                current_wp_marker.action = Marker.ADD
                current_wp_marker.pose.position.x = current_wp.x
                current_wp_marker.pose.position.y = current_wp.y
                current_wp_marker.pose.position.z = current_wp.z
                current_wp_marker.scale.x = 0.4  # TODO: Param!
                current_wp_marker.scale.y = 0.4
                current_wp_marker.scale.z = 0.4
                current_wp_marker.color = palette.get_color('white', 0.4)

                marker_array.markers.append(current_wp_marker)

            else:
                rospy.logwarn('Tracking information from {} is empty!'.format(operation.icao_address))

            # Visualize flight_plan
            ns = operation_ns + '/flight_plan'
            color = palette.get_color(id_to_color[operation.uav_id % 10])
            color_scheme = WaypointListColorScheme(color, color, color, color)
            flight_plan = flight_plan_viz.get_markerarray(operation.flight_plan.waypoints, ns, color_scheme)
            marker_array.markers.extend(flight_plan.markers)

            # Visualize flight_plan_updated
            ns = operation_ns + '/flight_plan_updated'
            color = palette.get_color(id_to_color[operation.uav_id % 10], 0.25)
            color_scheme = WaypointListColorScheme(color, color, color, color)
            flight_plan_updated = flight_plan_viz.get_markerarray(operation.flight_plan_updated.waypoints, ns, color_scheme)
            marker_array.markers.extend(flight_plan_updated.markers)

            # Visualize track
            ns = operation_ns + '/track'
            color = palette.get_color(id_to_color[operation.uav_id % 10])
            color_scheme = WaypointListColorScheme(color, None, None, None)
            track = track_viz.get_markerarray(operation.track.waypoints, ns, color_scheme)
            marker_array.markers.extend(track.markers)

            # Visualize estimated_trajectory
            ns = operation_ns + '/estimated_trajectory'
            color = palette.get_color(id_to_color[operation.uav_id % 10], 0.25)
            color_scheme = WaypointListColorScheme(color, None, None, None)
            estimated_trajectory = track_viz.get_markerarray(operation.estimated_trajectory.waypoints, ns, color_scheme)
            marker_array.markers.extend(estimated_trajectory.markers)

            # Visualize landing_spots
            ns = operation_ns + '/landing_spots'
            color = palette.get_color(id_to_color[operation.uav_id % 10])
            color_scheme = WaypointListColorScheme(None, None, color, None)
            landing_spots = landing_spots_viz.get_markerarray(operation.landing_spots.waypoints, ns, color_scheme)
            marker_array.markers.extend(landing_spots.markers)

        visualization_pub.publish(marker_array)

        rate.sleep()

if __name__ == '__main__':
    main()
