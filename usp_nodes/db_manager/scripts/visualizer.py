#!/usr/bin/env python

import copy
import rospy
from gauss_msgs.msg import Operation
from gauss_msgs.srv import ReadIcao, ReadIcaoRequest, ReadIcaoResponse
from gauss_msgs.srv import ReadOperation, ReadOperationRequest, ReadOperationResponse
from gauss_msgs.srv import ReadGeofences, ReadGeofencesRequest, ReadGeofencesResponse
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
            rospy.logwarn('[Viz] Color [{}] not found in palette, using black'.format(label))
            return ColorRGBA(0, 0, 0, alpha)

    # def set_color_index(self, label, index):
    #     if label in self.colors:
    #         self.indexed_colors[index] = label            
    #     else:
    #         rospy.logwarn('[Viz] Color [{}] not found in palette, ignoring indexing'.format(label))

    # def get_indexed_color(self, index, alpha = 1.0):
    #     if index in self.indexed_colors:
    #         return self.get_color(self.indexed_colors[index], alpha)
    #     else:
    #         rospy.logwarn('[Viz] Index [{}] not found in palette, using black'.format(index))
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
        self.path_scale = Vector3(1, 0, 0)
        self.mandatory_wp_type = Marker.CUBE_LIST
        self.mandatory_wp_scale = Vector3(1.5, 1.5, 1.5)
        self.non_mandatory_wp_type = Marker.SPHERE_LIST
        self.non_mandatory_wp_scale = Vector3(1.5, 0, 0)
        self.stamp_font_size = 2
        self.stamp_offset = Vector3(0, 0, 2)

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

class GeofenceViz(object):
    def __init__(self, frame_id, lifetime):
        self.frame_id = frame_id
        self.lifetime = lifetime
        # Default values for markers:
        self.surface_type = Marker.CYLINDER
        self.line_type = Marker.LINE_STRIP
        self.line_scale = Vector3(1, 0, 0)
        self.stamp_font_size = 2
        self.stamp_offset = Vector3(0, 0, 2)

    def get_markerarray(self, geofence, ns, color):
        marker_common = Marker()
        marker_common.header.stamp = rospy.Time.now()
        marker_common.header.frame_id = self.frame_id
        marker_common.ns = ns
        marker_common.lifetime = self.lifetime

        markerarray = MarkerArray()
        text_point = Point()

        if geofence.cylinder_shape:
            surface_marker = copy.deepcopy(marker_common)
            surface_marker.id = 0
            surface_marker.type = self.surface_type
            surface_marker.action = Marker.ADD
            surface_marker.pose.position.x = geofence.circle.x_center
            surface_marker.pose.position.y = geofence.circle.y_center
            surface_marker.pose.position.z = 0.5 * (geofence.min_altitude + geofence.max_altitude)
            diameter = 2 * geofence.circle.radius
            surface_marker.scale = Vector3(diameter, diameter, geofence.max_altitude - geofence.min_altitude)
            surface_marker.color = copy.deepcopy(color)
            surface_marker.color.a = 0.4  # TODO: force alpha?
            markerarray.markers.append(surface_marker)
            text_point.x = geofence.circle.x_center
            text_point.y = geofence.circle.y_center
            text_point.z = geofence.max_altitude
        else:
            top_line_marker = copy.deepcopy(marker_common)
            top_line_marker.id = 1
            top_line_marker.type = self.line_type
            top_line_marker.action = Marker.ADD
            top_line_marker.scale = self.line_scale
            top_line_marker.color = copy.deepcopy(color)
            top_line_marker.color.a = 0.4  # TODO: force alpha?

            bottom_line_marker = copy.deepcopy(top_line_marker)
            bottom_line_marker.id = 2

            lateral_line_marker = copy.deepcopy(marker_common)
            lateral_line_marker.id = 3
            lateral_line_marker.type = Marker.LINE_LIST
            lateral_line_marker.action = Marker.ADD
            lateral_line_marker.scale = self.line_scale
            lateral_line_marker.color = copy.deepcopy(color)
            lateral_line_marker.color.a = 0.4  # TODO: force alpha?

            lateral_surface_marker = copy.deepcopy(marker_common)
            lateral_surface_marker.id = 4
            lateral_surface_marker.type = Marker.TRIANGLE_LIST
            lateral_surface_marker.action = Marker.ADD
            lateral_surface_marker.scale = Vector3(1, 1, 1)
            lateral_surface_marker.color = copy.deepcopy(color)
            lateral_surface_marker.color.a = 0.4  # TODO: force alpha?

            top_surface_marker = copy.deepcopy(lateral_surface_marker)
            top_surface_marker.id = 5

            bottom_surface_marker = copy.deepcopy(lateral_surface_marker)
            bottom_surface_marker.id = 6

            if len(geofence.polygon.x) != len(geofence.polygon.y):
                rospy.logerr('[Viz] Length mismatch at geofence polygon description: x[{}] != y[{}]'.format(len(geofence.polygon.x), len(geofence.polygon.y)))
                return
            point_count = len(geofence.polygon.x)
            if point_count <= 0:
                rospy.logerr('[Viz] Unexpected length at geofence polygon description: point_count = {}'.format(point_count))
                return

            x_sum = 0.0
            y_sum = 0.0
            for i in range(point_count):
                point = Point()
                point.x = geofence.polygon.x[i]
                point.y = geofence.polygon.y[i]
                point.z = geofence.max_altitude
                x_sum += geofence.polygon.x[i]
                y_sum += geofence.polygon.y[i]
                top_line_marker.points.append(copy.deepcopy(point))
                lateral_line_marker.points.append(copy.deepcopy(point))
                point.z = geofence.min_altitude
                bottom_line_marker.points.append(copy.deepcopy(point))
                lateral_line_marker.points.append(copy.deepcopy(point))

            # Close the polygon (so length will be point_count+1)
            point = Point()
            point.x = geofence.polygon.x[0]
            point.y = geofence.polygon.y[0]
            point.z = geofence.max_altitude
            top_line_marker.points.append(copy.deepcopy(point))
            point.z = geofence.min_altitude
            bottom_line_marker.points.append(copy.deepcopy(point))

            for i in range(point_count):
                lateral_surface_marker.points.append(bottom_line_marker.points[i])
                lateral_surface_marker.points.append(bottom_line_marker.points[i + 1])
                lateral_surface_marker.points.append(top_line_marker.points[i])

                lateral_surface_marker.points.append(bottom_line_marker.points[i + 1])
                lateral_surface_marker.points.append(top_line_marker.points[i + 1])
                lateral_surface_marker.points.append(top_line_marker.points[i])

                if i != 0 and i != (point_count - 1):
                    top_surface_marker.points.append(top_line_marker.points[0])
                    top_surface_marker.points.append(top_line_marker.points[i])
                    top_surface_marker.points.append(top_line_marker.points[i + 1])

                    bottom_surface_marker.points.append(bottom_line_marker.points[0])
                    bottom_surface_marker.points.append(bottom_line_marker.points[point_count - i])
                    bottom_surface_marker.points.append(bottom_line_marker.points[point_count - i - 1])

            markerarray.markers.append(top_line_marker)
            markerarray.markers.append(bottom_line_marker)
            markerarray.markers.append(lateral_line_marker)
            markerarray.markers.append(lateral_surface_marker)
            markerarray.markers.append(top_surface_marker)
            markerarray.markers.append(bottom_surface_marker)

            text_point.x = x_sum / point_count
            text_point.y = y_sum / point_count
            text_point.z = geofence.max_altitude

        stamp_marker = copy.deepcopy(marker_common)
        stamp_marker.id = 10
        stamp_marker.type = Marker.TEXT_VIEW_FACING
        stamp_marker.action = Marker.ADD
        stamp_marker.scale.z = self.stamp_font_size
        stamp_marker.color = color
        text_point.x += self.stamp_offset.x
        text_point.y += self.stamp_offset.y
        text_point.z += self.stamp_offset.z
        stamp_marker.pose.position = text_point
        if geofence.static_geofence:
            stamp_marker.text = 'id: {}; static'.format(geofence.id)
        else:
            stamp_marker.text = 'id: {}; t: [{}, {}]s'.format(geofence.id, geofence.start_time.to_sec(), geofence.end_time.to_sec())
        markerarray.markers.append(stamp_marker)

        return markerarray


class VolumeViz(object):
    def __init__(self, frame_id, lifetime):
        self.frame_id = frame_id
        self.lifetime = lifetime

    def get_markerarray(self, operation, ns, fg_color, ov_color):
        marker_common = Marker()
        marker_common.header.stamp = rospy.Time.now()
        marker_common.header.frame_id = self.frame_id
        marker_common.lifetime = self.lifetime
        marker_common.type = Marker.ARROW
        marker_common.action = Marker.ADD

        flight_geometry_marker_array = MarkerArray()
        operational_volume_marker_array = MarkerArray()

        flight_geometry = operation.flight_geometry
        operational_volume = operation.operational_volume
        waypointlist = operation.flight_plan.waypoints
        for i, (prev, current) in enumerate(zip(waypointlist, waypointlist[1:])):
            prev_point    = Point(prev.x,    prev.y,    prev.z)
            current_point = Point(current.x, current.y, current.z)

            flight_geometry_marker = copy.deepcopy(marker_common)
            flight_geometry_marker.ns = ns + '/flight_geometry'
            flight_geometry_marker.id = i
            flight_geometry_marker.color = fg_color
            flight_geometry_marker.points.append(prev_point)
            flight_geometry_marker.points.append(current_point)
            flight_geometry_marker.scale = Vector3(flight_geometry, flight_geometry, 0.01)
            flight_geometry_marker_array.markers.append(copy.deepcopy(flight_geometry_marker))

            operational_volume_marker = copy.deepcopy(marker_common)
            operational_volume_marker.ns = ns + '/operational_volume'
            operational_volume_marker.id = i
            operational_volume_marker.color = ov_color
            operational_volume_marker.points.append(prev_point)
            operational_volume_marker.points.append(current_point)
            operational_volume_marker.scale = Vector3(operational_volume, operational_volume, 0.01)
            operational_volume_marker_array.markers.append(copy.deepcopy(operational_volume_marker))

        markerarray = MarkerArray()
        markerarray.markers.extend(flight_geometry_marker_array.markers)
        markerarray.markers.extend(operational_volume_marker_array.markers)

        return markerarray

def main():
    rospy.init_node('visualizer')
    rospy.loginfo('[Viz] Started Visualizer node!')
 
    # TODO: From param
    update_rate = 1.0
    global_frame_id = 'map'
    read_icao_srv_url = '/gauss/read_icao'
    read_operation_srv_url = '/gauss/read_operation'
    read_geofences_srv_url = '/gauss/read_geofences'
    visualization_topic = 'visualization_marker_array'
    id_to_color = {}
    id_to_color[0] = 'orange'
    id_to_color[1] = 'light_green'
    id_to_color[2] = 'light_blue'
    id_to_color[3] = 'pink'
    id_to_color[4] = 'yellow'
    id_to_color[5] = 'cyan'
    id_to_color[6] = 'purple'
    id_to_color[7] = 'tan'
    id_to_color[8] = 'brown'
    id_to_color[9] = 'light_gray'

    rospy.loginfo('[Viz] Waiting for service {}'.format(read_icao_srv_url))
    rospy.wait_for_service(read_icao_srv_url)
    read_icao_service = rospy.ServiceProxy(read_icao_srv_url, ReadIcao)
    rospy.loginfo('[Viz] Successfully connected to service {}'.format(read_icao_srv_url))

    rospy.loginfo('[Viz] Waiting for service {}'.format(read_operation_srv_url))
    rospy.wait_for_service(read_operation_srv_url)
    read_operation_service = rospy.ServiceProxy(read_operation_srv_url, ReadOperation)
    rospy.loginfo('[Viz] Successfully connected to service {}'.format(read_operation_srv_url))

    rospy.loginfo('[Viz] Waiting for service {}'.format(read_geofences_srv_url))
    rospy.wait_for_service(read_geofences_srv_url)
    read_geofences_service = rospy.ServiceProxy(read_geofences_srv_url, ReadGeofences)
    rospy.loginfo('[Viz] Successfully connected to service {}'.format(read_geofences_srv_url))

    visualization_pub = rospy.Publisher(visualization_topic, MarkerArray, queue_size=1)

    rospy.loginfo('[Viz] Started visualization node!')
    rate = rospy.Rate(update_rate)
    while not rospy.is_shutdown():

        read_icao_response = read_icao_service.call(ReadIcaoRequest())  # TODO: try/catch
        if not read_icao_response.success:
            rospy.logwarn('[Viz] Read icao did not succeed')
            rospy.logwarn(read_icao_response.message)

        read_operation_request = ReadOperationRequest()
        read_operation_request.uav_ids = read_icao_response.uav_id
        read_operation_response = read_operation_service.call(read_operation_request)

        if not read_operation_response.success:
            rospy.logwarn('[Viz] Read operation did not succeed')
            rospy.logwarn(read_operation_response.message)

        read_geofences_request = ReadGeofencesRequest()
        read_geofences_request.geofences_ids = read_icao_response.geofence_id
        read_geofences_response = read_geofences_service.call(read_geofences_request)

        if not read_geofences_response.success:
            rospy.logwarn('[Viz] Read geofences did not succeed')
            rospy.logwarn(read_geofences_response.message)

        flight_plan_viz = WaypointListViz(global_frame_id, rospy.Duration(1.0/update_rate))
        track_viz = WaypointListViz(global_frame_id, rospy.Duration(1.0/update_rate))
        landing_spots_viz = WaypointListViz(global_frame_id, rospy.Duration(1.0/update_rate))
        landing_spots_viz.config_non_mandatory_wp(Marker.CUBE_LIST, Vector3(0.5, 0.5, 0))
        volume_viz = VolumeViz(global_frame_id, rospy.Duration(1.0/update_rate))

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
                info_marker.scale.z = 2  # TODO: param font_size
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
                frame_marker.scale.x = 4  # TODO: Param!
                frame_marker.scale.y = 4
                frame_marker.scale.z = 4
                frame_marker.color = palette.get_color(id_to_color[operation.uav_id % 10])
                if operation.frame == Operation.FRAME_ROTOR:
                    frame_marker.mesh_resource = 'package://db_manager/config/rotor.dae'
                elif operation.frame == Operation.FRAME_FIXEDWING:
                    frame_marker.mesh_resource = 'package://db_manager/config/fixedwing.dae'
                else:
                    rospy.logwarn('[Viz] Unkwown frame type [{}]'.format(operation.frame))
                    frame_marker.mesh_resource = 'package://db_manager/config/arrow.dae'

                marker_array.markers.append(frame_marker)

                if operation.current_wp >= 0 and operation.current_wp < len(operation.flight_plan.waypoints):
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
                    current_wp_marker.scale.x = 4  # TODO: Param!
                    current_wp_marker.scale.y = 4
                    current_wp_marker.scale.z = 4
                    current_wp_marker.color = palette.get_color('white', 0.4)

                    marker_array.markers.append(current_wp_marker)

            else:
                rospy.logwarn('[Viz] Tracking information from {} is empty!'.format(operation.icao_address))

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

            # Visualize volumes (flight geometry, operational volume)
            ns = operation_ns + '/volumes'
            fg_color = palette.get_color(id_to_color[operation.uav_id % 10], 0.5)
            ov_color = palette.get_color(id_to_color[operation.uav_id % 10], 0.2)
            volume = volume_viz.get_markerarray(operation, ns, fg_color, ov_color)
            marker_array.markers.extend(volume.markers)

        geofence_viz = GeofenceViz(global_frame_id, rospy.Duration(1.0/update_rate))
        for geofence in read_geofences_response.geofences:
            ns = 'geofence_' + str(geofence.id)
            color = palette.get_color('red')  # TODO: Force red?
            if geofence.start_time.secs < rospy.get_rostime().secs and rospy.get_rostime().secs < geofence.end_time.secs: 
                marker_array.markers.extend(geofence_viz.get_markerarray(geofence, ns, color).markers)

        visualization_pub.publish(marker_array)

        rate.sleep()

if __name__ == '__main__':
    main()
