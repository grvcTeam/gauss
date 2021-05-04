#include <gauss_msgs/CheckConflicts.h>
#include <gauss_msgs/Circle.h>
#include <gauss_msgs/NewDeconfliction.h>
#include <gauss_msgs/ReadIcao.h>
#include <gauss_msgs/ReadOperation.h>
#include <gauss_msgs/Waypoint.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>
#include <tactical_deconfliction/path_finder.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Eigen>

double safety_distance_;
bool actual_wp_on_merge_;
ros::Publisher visualization_pub_;

std::vector<Eigen::Vector3f> perpendicularSeparationVector(const gauss_msgs::Waypoint &_pA, const gauss_msgs::Waypoint &_pB, const double &_op_vol_A, const double &_op_vol_B) {
    std::vector<Eigen::Vector3f> out_avoid_vector;
    Eigen::Vector3f p_a, p_b, unit_vec_ab, unit_vec_ba, avoid_vector_a, avoid_vector_b;
    p_a = Eigen::Vector3f(_pA.x, _pA.y, _pA.z);
    p_b = Eigen::Vector3f(_pB.x, _pB.y, _pB.z);

    unit_vec_ab = (p_a - p_b) / (p_a - p_b).norm();
    unit_vec_ba = (p_b - p_a) / (p_b - p_a).norm();

    double distance_to_avoid;
    double distance_between_points = (p_a - p_b).norm();
    double distance_operational_volumes = _op_vol_A + _op_vol_B;
    if (safety_distance_ >= distance_operational_volumes) {
        distance_to_avoid = safety_distance_;
        distance_to_avoid -= distance_between_points;
    } else {
        distance_to_avoid = abs(_op_vol_A - _op_vol_B);
    }
    double extra_safety_margin = 1.1;  // Increase 10% the distance
    distance_to_avoid *= extra_safety_margin;

    avoid_vector_a = -unit_vec_ba * distance_to_avoid;
    avoid_vector_b = -unit_vec_ab * distance_to_avoid;

    out_avoid_vector.push_back(avoid_vector_a);
    out_avoid_vector.push_back(avoid_vector_b);

    return out_avoid_vector;
}

std::vector<gauss_msgs::Waypoint> applySeparation(const Eigen::Vector3f &_avoid_vector, const std::vector<gauss_msgs::Waypoint> &_extremes) {
    // TODO: Check why _extremes has repeated elements.
    std::vector<gauss_msgs::Waypoint> out_waypoints;
    gauss_msgs::Waypoint pA, pB;  // (pA) ------------------- (pB)
    pA = _extremes.front();
    pB = _extremes.back();

    for (auto wp : _extremes) {
        wp.x = wp.x + _avoid_vector[0];
        wp.y = wp.y + _avoid_vector[1];
        wp.z = wp.z + _avoid_vector[2];
        out_waypoints.push_back(wp);
    }

    return out_waypoints;
}

void checkGroundCollision(const std::vector<gauss_msgs::Waypoint> &_solution, const double &_operational_volume) {
    for (auto wp : _solution) {
        ROS_ERROR_COND(wp.z - _operational_volume <= 0.0, "[Tactical] Proposed solution hits the ground. Waypoint height is [%.2f].", wp.z);
    }
}

geometry_msgs::Polygon circleToPolygon(double &_x, double &_y, double &_radius, double _nVertices = 9) {
    geometry_msgs::Polygon out_polygon;
    Eigen::Vector2d centerToVertex(_radius, 0.0), centerToVertexTemp;
    for (int i = 0; i < _nVertices; i++) {
        double theta = i * 2 * M_PI / (_nVertices - 1);
        Eigen::Rotation2D<double> rot2d(theta);
        centerToVertexTemp = rot2d.toRotationMatrix() * centerToVertex;
        geometry_msgs::Point32 temp_point;
        temp_point.x = _x + centerToVertexTemp[0];
        temp_point.y = _y + centerToVertexTemp[1];
        out_polygon.points.push_back(temp_point);
    }

    return out_polygon;
}

// nvert        - Number of vertices in the polygon. Whether to repeat the first vertex at the end is discussed below.
// vertx, verty	- Arrays containing the x- and y-coordinates of the polygon's vertices.
// testx, testy	- X&Y coordinate of the test point.
// [https://wrf.ecse.rpi.edu/Research/Short_Notes/pnpoly.html]
int pointInPolygon(int nvert, std::vector<float> &vertx, std::vector<float> &verty, float testx, float testy) {
    int i, j, c = 0;
    for (i = 0, j = nvert - 1; i < nvert; j = i++) {
        if (((verty.at(i) > testy) != (verty.at(j) > testy)) &&
            (testx < (vertx.at(j) - vertx.at(i)) * (testy - verty.at(i)) / (verty.at(j) - verty.at(i)) + vertx.at(i)))
            c = !c;
    }
    return c;
}

double signedArea(const geometry_msgs::Polygon &p) {
    double A = 0;
    //========================================================//
    // Assumes:                                               //
    //    N+1 vertices:   p[0], p[1], ... , p[N-1], p[N]      //
    //    Closed polygon: p[0] = p[N]                         //
    // Returns:                                               //
    //    Signed area: +ve if anticlockwise, -ve if clockwise //
    //========================================================//
    int N = p.points.size() - 1;
    for (int i = 0; i < N; i++) A += p.points.at(i).x * p.points.at(i + 1).y - p.points.at(i + 1).x * p.points.at(i).y;
    A *= 0.5;
    return A;
}

geometry_msgs::Polygon decreasePolygon(const geometry_msgs::Polygon &p, double thickness) {
    //=====================================================//
    // Assumes:                                            //
    //    N+1 vertices:   p[0], p[1], ... , p[N-1], p[N]   //
    //    Closed polygon: p[0] = p[N]                      //
    //    No zero-length sides                             //
    // Returns (by reference, as a parameter):             //
    //    Internal poly:  q[0], q[1], ... , q[N-1], q[N]   //
    //=====================================================//
    geometry_msgs::Polygon q;
    int N = p.points.size() - 1;
    q.points.resize(N + 1);
    double a, b, A, B, d, cross;
    double displacement = thickness;
    if (signedArea(p) < 0) displacement = -displacement;  // Detects clockwise order
    // Unit vector (a,b) along last edge
    a = p.points.at(N).x - p.points.at(N - 1).x;
    b = p.points.at(N).y - p.points.at(N - 1).y;
    d = sqrt(a * a + b * b);
    a /= d;
    b /= d;
    for (int i = 0; i < N; i++) {  // Loop round the polygon, dealing with successive intersections of lines
        // Unit vector (A,B) along previous edge
        A = a;
        B = b;
        // Unit vector (a,b) along next edge
        a = p.points.at(i + 1).x - p.points.at(i).x;
        b = p.points.at(i + 1).y - p.points.at(i).y;
        d = sqrt(a * a + b * b);
        a /= d;
        b /= d;
        // New vertex
        cross = A * b - a * B;
        const double SMALL = 1.0e-10;
        if (abs(cross) < SMALL) {  // Degenerate cases: 0 or 180 degrees at vertex
            q.points.at(i).x = p.points.at(i).x - displacement * b;
            q.points.at(i).y = p.points.at(i).y + displacement * a;
        } else {  // Usual case
            q.points.at(i).x = p.points.at(i).x + displacement * (a - A) / cross;
            q.points.at(i).y = p.points.at(i).y + displacement * (b - B) / cross;
        }
    }
    // Close the inside polygon
    q.points.at(N) = q.points.at(0);

    return q;
}

std::vector<double> findGridBorders(geometry_msgs::Polygon &_polygon, geometry_msgs::Point _init_point, geometry_msgs::Point _goal_point, double _operational_volume) {
    geometry_msgs::Point obs_min, obs_max, out_point;
    std::vector<float> vert_x, vert_y;
    for (int i = 0; i < _polygon.points.size(); i++) {
        vert_x.push_back(_polygon.points.at(i).x);
        vert_y.push_back(_polygon.points.at(i).y);
    }
    vert_x.push_back(_polygon.points.front().x);
    vert_y.push_back(_polygon.points.front().y);
    obs_min.x = *std::min_element(vert_x.begin(), vert_x.end());
    obs_min.y = *std::min_element(vert_y.begin(), vert_y.end());
    obs_max.x = *std::max_element(vert_x.begin(), vert_x.end());
    obs_max.y = *std::max_element(vert_y.begin(), vert_y.end());

    std::vector<double> out_grid_borders, temp_x, temp_y;
    temp_x.push_back(_init_point.x);
    temp_x.push_back(_goal_point.x);
    temp_y.push_back(_init_point.y);
    temp_y.push_back(_goal_point.y);
    double min_x = *std::min_element(temp_x.begin(), temp_x.end());
    double min_y = *std::min_element(temp_y.begin(), temp_y.end());
    double max_x = *std::max_element(temp_x.begin(), temp_x.end());
    double max_y = *std::max_element(temp_y.begin(), temp_y.end());

    while (min_x >= obs_min.x || min_y >= obs_min.y || max_x <= obs_max.x || max_y <= obs_max.y) {
        if (min_x >= obs_min.x) min_x = _operational_volume * 2 - min_x;
        if (min_y >= obs_min.y) min_y = _operational_volume * 2 - min_y;
        if (max_x <= obs_max.x) max_x = _operational_volume * 2 + max_x;
        if (max_y <= obs_max.y) max_y = _operational_volume * 2 + max_y;
    }

    out_grid_borders.push_back(min_x);
    out_grid_borders.push_back(min_y);
    out_grid_borders.push_back(max_x);
    out_grid_borders.push_back(max_y);

    return out_grid_borders;
}

geometry_msgs::Point translateToPoint(const gauss_msgs::Waypoint &wp) {
    geometry_msgs::Point p;
    p.x = wp.x;
    p.y = wp.y;
    p.z = wp.z;
    return p;
}

nav_msgs::Path translateToPath(const gauss_msgs::WaypointList &_wp_list) {
    nav_msgs::Path out_path;
    for (auto wp : _wp_list.waypoints) {
        geometry_msgs::PoseStamped temp_wp;
        temp_wp.pose.position.x = wp.x;
        temp_wp.pose.position.y = wp.y;
        temp_wp.pose.position.z = wp.z;
        temp_wp.header.stamp = wp.stamp;
    }
    return out_path;
}

std::vector<gauss_msgs::Waypoint> pathAStartToWPVector(const nav_msgs::Path &_path, const std::vector<double> &_times) {
    std::vector<gauss_msgs::Waypoint> out_wp_vector;
    ROS_ERROR_COND(_path.poses.size() != _times.size(), "[Tactical] A Start solution must have the same amount of waypoints (space and time)!");
    for (int i = 0; i < _path.poses.size(); i++) {
        gauss_msgs::Waypoint temp_wp;
        temp_wp.x = _path.poses.at(i).pose.position.x;
        temp_wp.y = _path.poses.at(i).pose.position.y;
        temp_wp.z = _path.poses.at(i).pose.position.z;
        temp_wp.stamp = ros::Time(_times.at(i));  // !Careful
        out_wp_vector.push_back(temp_wp);
    }
    return out_wp_vector;
}

std::vector<gauss_msgs::Waypoint> findAlternativePathAStar(geometry_msgs::Point &_p_init, geometry_msgs::Point &_p_end, ros::Time &_t_init, ros::Time &_t_end, gauss_msgs::Geofence &_geofence, gauss_msgs::ConflictiveOperation &_conflictive_operation) {
    // Setup polygon geofence according on its shape
    geometry_msgs::Polygon polygon_geofence;
    if (_geofence.cylinder_shape) {
        polygon_geofence = circleToPolygon(_geofence.circle.x_center, _geofence.circle.y_center, _geofence.circle.radius);
    } else {
        for (int i = 0; i < _geofence.polygon.x.size(); i++) {
            geometry_msgs::Point32 temp_points;
            temp_points.x = _geofence.polygon.x.at(i);
            temp_points.y = _geofence.polygon.y.at(i);
            polygon_geofence.points.push_back(temp_points);
        }
    }
    // Inflate polygon to take operational volume into account
    if (!_geofence.cylinder_shape) polygon_geofence.points.push_back(polygon_geofence.points.front());
    geometry_msgs::Polygon inflated_geofence = decreasePolygon(polygon_geofence, -_conflictive_operation.operational_volume * 1.1);
    if (!_geofence.cylinder_shape) inflated_geofence.points.pop_back();
    // Get borders of a local greed for the A* path finder
    geometry_msgs::Point p_min_local_grid, p_max_local_grid;
    std::vector<double> grid_borders = findGridBorders(inflated_geofence, _p_init, _p_end, _conflictive_operation.operational_volume);
    p_min_local_grid.x = grid_borders[0];
    p_min_local_grid.y = grid_borders[1];
    p_max_local_grid.x = grid_borders[2];
    p_max_local_grid.y = grid_borders[3];
    nav_msgs::Path estimated_traj_path = translateToPath(_conflictive_operation.estimated_trajectory);
    // Use A* path finder to get an alternative path
    PathFinder path_finder(estimated_traj_path, _p_init, _p_end, inflated_geofence, p_min_local_grid, p_max_local_grid);
    nav_msgs::Path a_star_path = path_finder.findNewPath();
    // Fix times
    std::vector<double> interp_times, a_star_times;
    interp_times.push_back(_t_init.toSec());
    interp_times.push_back(_t_end.toSec());
    a_star_times = path_finder.interpWaypointList(interp_times, a_star_path.poses.size() - 1);
    a_star_times.push_back(_t_end.toSec());

    return pathAStartToWPVector(a_star_path, a_star_times);
}

std::vector<gauss_msgs::Waypoint> mergeSolutionWithFlightPlan(std::vector<gauss_msgs::Waypoint> &_solution, gauss_msgs::WaypointList &_flight_plan, int &_current_wp, gauss_msgs::Waypoint &_actual_wp) {
    std::vector<gauss_msgs::Waypoint> out_merged_solution;
    bool do_once = true;
    if (actual_wp_on_merge_) out_merged_solution.push_back(_actual_wp);  // Insert the actual wp
    for (auto fp_wp : _flight_plan.waypoints) {
        if (_flight_plan.waypoints.at(_current_wp).stamp <= fp_wp.stamp) {  // Do nothing before current wp
            if (fp_wp.stamp <= _solution.front().stamp) {                   // Between current wp and first wp of the solution
                out_merged_solution.push_back(fp_wp);
            } else if (do_once && _solution.back().stamp < fp_wp.stamp) {  // Insert all the solution wps
                for (auto solution_wp : _solution) {
                    out_merged_solution.push_back(solution_wp);
                }
                out_merged_solution.push_back(fp_wp);  // Insert the wp after the solution
                do_once = false;
            } else if (_solution.back().stamp < fp_wp.stamp) {  // Insert the remaining wps of the flight plan
                out_merged_solution.push_back(fp_wp);
            }
        }
    }

    return out_merged_solution;
}

std::vector<gauss_msgs::Waypoint> delayFlightPlan(std::vector<gauss_msgs::Waypoint> &_segment, gauss_msgs::WaypointList &_flight_plan, int &_current_wp, gauss_msgs::Waypoint &_actual_wp) {
    std::vector<gauss_msgs::Waypoint> out_merged_solution;
    bool do_once = true;
    double dtime = _segment.back().stamp.sec - _segment.front().stamp.sec;
    if (actual_wp_on_merge_) out_merged_solution.push_back(_actual_wp);  // Insert the actual wp
    for (auto fp_wp : _flight_plan.waypoints) {
        if (_flight_plan.waypoints.at(_current_wp).stamp <= fp_wp.stamp) {  // Do nothing before current wp
            if (fp_wp.stamp <= _segment.front().stamp) {                   // Between current wp and first wp of the solution
                out_merged_solution.push_back(fp_wp);
            } else if (do_once && _segment.back().stamp < fp_wp.stamp) {  // Insert all the solution wps
                for (auto segment_wp : _segment) {
                    segment_wp.stamp.fromSec(segment_wp.stamp.toSec() + dtime);
                    out_merged_solution.push_back(segment_wp);
                }
                fp_wp.stamp.fromSec(fp_wp.stamp.toSec() + dtime);
                out_merged_solution.push_back(fp_wp);  // Insert the wp after the solution
                do_once = false;
            } else if (_segment.back().stamp < fp_wp.stamp) {  // Insert the remaining wps of the flight plan
                fp_wp.stamp.fromSec(fp_wp.stamp.toSec() + dtime);
                out_merged_solution.push_back(fp_wp);
            }
        }
    }

    return out_merged_solution;
}

visualization_msgs::Marker createMarkerSpheres(const gauss_msgs::Waypoint &_p_at_t_min_first, const gauss_msgs::Waypoint &_p_at_t_min_second) {
    std_msgs::ColorRGBA white;
    white.r = 1.0;
    white.g = 1.0;
    white.b = 1.0;
    white.a = 1.0;

    visualization_msgs::Marker marker_spheres;
    marker_spheres.header.stamp = ros::Time::now();
    marker_spheres.header.frame_id = "map";
    marker_spheres.ns = "avoid_points";
    marker_spheres.id = 0;
    marker_spheres.type = visualization_msgs::Marker::SPHERE_LIST;
    marker_spheres.action = visualization_msgs::Marker::ADD;
    marker_spheres.pose.orientation.w = 1;
    marker_spheres.scale.x = 5.0;
    marker_spheres.scale.y = 5.0;
    marker_spheres.scale.z = 5.0;
    marker_spheres.lifetime = ros::Duration(1.0);
    marker_spheres.points.push_back(translateToPoint(_p_at_t_min_first));
    marker_spheres.colors.push_back(white);
    marker_spheres.points.push_back(translateToPoint(_p_at_t_min_second));
    marker_spheres.colors.push_back(white);
    return marker_spheres;
}

visualization_msgs::Marker createMarkerLines(const std::vector<gauss_msgs::Waypoint> &_solution) {
    std_msgs::ColorRGBA blue;
    blue.b = 1.0;
    blue.a = 1.0;
    static int sol_count = 0;

    visualization_msgs::Marker marker_lines;
    marker_lines.header.stamp = ros::Time::now();
    marker_lines.header.frame_id = "map";
    marker_lines.ns = "lines_" + std::to_string(sol_count);
    sol_count++;
    marker_lines.id = 1;
    marker_lines.type = visualization_msgs::Marker::LINE_STRIP;
    marker_lines.action = visualization_msgs::Marker::ADD;
    marker_lines.pose.orientation.w = 1;
    marker_lines.scale.x = 5.0;
    marker_lines.color = blue;
    marker_lines.lifetime = ros::Duration(1.0);

    for (auto wp : _solution) {
        marker_lines.points.push_back(translateToPoint(wp));
    }

    return marker_lines;
}

bool deconflictCB(gauss_msgs::NewDeconfliction::Request &req, gauss_msgs::NewDeconfliction::Response &res) {
    ROS_INFO("[Tactical] Threat to solve [%d, %d]", req.threat.threat_id, req.threat.threat_type);
    switch (req.threat.threat_type) {
        case req.threat.LOSS_OF_SEPARATION: {
            std::vector<std::vector<gauss_msgs::Waypoint>> segments_first_second;
            segments_first_second.push_back(req.conflictive_segments.segment_first);
            segments_first_second.push_back(req.conflictive_segments.segment_second);
            std::vector<gauss_msgs::Waypoint> points_at_t_min;
            points_at_t_min.push_back(req.conflictive_segments.point_at_t_min_segment_first);
            points_at_t_min.push_back(req.conflictive_segments.point_at_t_min_segment_second);
            ROS_ERROR_COND(req.conflictive_operations.size() != 2, "[Tactical] Deconflictive server should receive 2 conflictive operations to solve LOSS OF SEPARATION!");
            // Calculate a vector to separate perpendiculary one trajectory
            std::vector<Eigen::Vector3f> avoid_vectors = perpendicularSeparationVector(points_at_t_min.front(), points_at_t_min.back(), req.conflictive_operations.front().operational_volume, req.conflictive_operations.back().operational_volume);
            // Solution applying separation to one operation
            double fake_value = 1.0;
            for (int i = 0; i < 2; i++) {
                gauss_msgs::DeconflictionPlan possible_solution;
                possible_solution.maneuver_type = 8;
                possible_solution.cost = possible_solution.riskiness = fake_value;
                possible_solution.uav_id = req.conflictive_operations.at(i).uav_id;
                std::vector<gauss_msgs::Waypoint> temp_solution = applySeparation(avoid_vectors.at(i), segments_first_second.at(i));
                // TODO: Should another alternative be proposed if the current one hits the ground?
                checkGroundCollision(temp_solution, req.conflictive_operations.at(i).operational_volume);
                // TODO: Who should do the merge?
                possible_solution.waypoint_list = mergeSolutionWithFlightPlan(temp_solution, req.conflictive_operations.at(i).flight_plan_updated, req.conflictive_operations.at(i).current_wp, req.conflictive_operations.at(i).actual_wp);
                res.deconfliction_plans.push_back(possible_solution);
            }
            // !Solution delaying one operation
            fake_value = 5.0;
            for (int i = 0; i < 2; i++) {
                gauss_msgs::DeconflictionPlan possible_solution;
                possible_solution.maneuver_type = 8;  // !Should be another maneuver type?
                possible_solution.cost = possible_solution.riskiness = fake_value;
                possible_solution.uav_id = req.conflictive_operations.at(i).uav_id;
                std::vector<gauss_msgs::Waypoint> temp_solution = segments_first_second.at(i);
                possible_solution.waypoint_list = delayFlightPlan(segments_first_second.at(i), req.conflictive_operations.at(i).flight_plan_updated, req.conflictive_operations.at(i).current_wp, req.conflictive_operations.at(i).actual_wp);
                res.deconfliction_plans.push_back(possible_solution);
            }
            // Visualize "space" results
            visualization_msgs::MarkerArray marker_array;
            visualization_msgs::Marker marker_spheres = createMarkerSpheres(req.conflictive_segments.point_at_t_min_segment_first, req.conflictive_segments.point_at_t_min_segment_second);
            marker_array.markers.push_back(marker_spheres);
            // Visualize results
            for (auto i : res.deconfliction_plans) marker_array.markers.push_back(createMarkerLines(i.waypoint_list));
            visualization_pub_.publish(marker_array);
        } break;
        case req.threat.GEOFENCE_CONFLICT: {
            ROS_ERROR_COND(req.geofences.size() != 1, "[Tactical] Deconflictive server should receive 1 geofence to solve GEOFENCE CONFLICT!");
            // * Assume inputs from monitoring
            // TODO: Check if init and end points have to be further apart from the geofence!
            geometry_msgs::Point p_init_conflict, p_end_conflict;
            p_init_conflict = translateToPoint(req.conflictive_segments.segment_first.front());
            p_end_conflict = translateToPoint(req.conflictive_segments.segment_first.back());
            ros::Time t_init_conflict = req.conflictive_segments.segment_first.front().stamp;
            ros::Time t_end_conflict = req.conflictive_segments.segment_first.front().stamp;

            gauss_msgs::DeconflictionPlan possible_solution;
            // [1] Ruta a mi destino evitando una geofence
            possible_solution.maneuver_type = 1;
            possible_solution.waypoint_list = findAlternativePathAStar(p_init_conflict, p_end_conflict, t_init_conflict, t_end_conflict, req.geofences.front(), req.conflictive_operations.front());
            res.deconfliction_plans.push_back(possible_solution);
            // [3] Ruta que me manda devuelta a casa
            possible_solution.maneuver_type = 3;
            possible_solution.waypoint_list.clear();
            possible_solution.waypoint_list.push_back(req.conflictive_operations.front().estimated_trajectory.waypoints.front());
            possible_solution.waypoint_list.push_back(req.conflictive_operations.front().flight_plan.waypoints.front());
            res.deconfliction_plans.push_back(possible_solution);
        } break;
        case req.threat.GEOFENCE_INTRUSION: {
            ROS_ERROR_COND(req.geofences.size() != 1, "[Tactical] Deconflictive server should receive 1 geofence to solve GEOFENCE INTRUSION!");
            // * Assume inputs from monitoring
            // TODO: Check if init and end points have to be further apart from the geofence!
            geometry_msgs::Point p_init_conflict, p_end_conflict;
            p_init_conflict = translateToPoint(req.conflictive_segments.segment_first.front());  // * Should we assume that this is the escape point?
            p_end_conflict = translateToPoint(req.conflictive_segments.segment_first.back());
            ros::Time t_init_conflict = req.conflictive_segments.segment_first.front().stamp;
            ros::Time t_end_conflict = req.conflictive_segments.segment_first.front().stamp;

            gauss_msgs::DeconflictionPlan possible_solution;
            possible_solution.uav_id = req.conflictive_operations.front().uav_id;
            // [6] Ruta a mi destino saliendo lo antes posible de la geofence
            possible_solution.maneuver_type = 1;
            possible_solution.waypoint_list = findAlternativePathAStar(p_init_conflict, p_end_conflict, t_init_conflict, t_end_conflict, req.geofences.front(), req.conflictive_operations.front());
            res.deconfliction_plans.push_back(possible_solution);
            // [2] Ruta a mi destino por el camino mas corto
            possible_solution.maneuver_type = 2;
            possible_solution.waypoint_list.clear();
            possible_solution.waypoint_list.push_back(req.conflictive_operations.front().estimated_trajectory.waypoints.front());
            possible_solution.waypoint_list.push_back(req.conflictive_operations.front().flight_plan.waypoints.back());
            res.deconfliction_plans.push_back(possible_solution);
            // [3] Ruta que me manda de vuelta a casa
            possible_solution.maneuver_type = 3;
            possible_solution.waypoint_list.clear();
            possible_solution.waypoint_list.push_back(req.conflictive_operations.front().estimated_trajectory.waypoints.front());
            possible_solution.waypoint_list.push_back(req.conflictive_operations.front().flight_plan.waypoints.front());
            res.deconfliction_plans.push_back(possible_solution);
        } break;
        case req.threat.UAS_OUT_OV: {
            ROS_ERROR_COND(req.conflictive_operations.size() != 1, "[Tactical] Deconflictive server should receive 1 conflictive operations to solve UAS OUT OV!");
            gauss_msgs::DeconflictionPlan possible_solution;
            possible_solution.uav_id = req.conflictive_operations.front().uav_id;
            // [9] Ruta para volver lo antes posible al flight geometry y seguir el plan de vuelo.
            // TODO: Should we use the same strategy described in ConflictSolver.cpp?

            // [10] Ruta para seguir con el plan de vuelo, da igual que esté más tiempo fuera del Operational Volume.
            possible_solution.maneuver_type = 10;
            possible_solution.waypoint_list.clear();
            possible_solution.waypoint_list.push_back(req.conflictive_operations.front().estimated_trajectory.waypoints.back());
            // ! current wp + 1 or just current wp?
            possible_solution.waypoint_list.push_back(req.conflictive_operations.front().flight_plan.waypoints.at(req.conflictive_operations.front().current_wp + 1));
            res.deconfliction_plans.push_back(possible_solution);
        } break;
        case req.threat.GNSS_DEGRADATION: {
            ROS_ERROR_COND(req.conflictive_operations.size() != 1, "[Tactical] Deconflictive server should receive 1 conflictive operations to solve GNSS DEGRADATION!");
            // [5] Ruta que aterrice en un landing spot
            for (auto landing_wp : req.conflictive_operations.front().landing_spots.waypoints) {
                gauss_msgs::DeconflictionPlan possible_solution;
                possible_solution.uav_id = req.conflictive_operations.front().uav_id;
                possible_solution.maneuver_type = 5;
                possible_solution.waypoint_list.push_back(req.conflictive_operations.front().estimated_trajectory.waypoints.front());
                possible_solution.waypoint_list.push_back(landing_wp);
                res.deconfliction_plans.push_back(possible_solution);
            }
        } break;
        case req.threat.LACK_OF_BATTERY: {
            ROS_ERROR_COND(req.conflictive_operations.size() != 1, "[Tactical] Deconflictive server should receive 1 conflictive operations to solve LACK OF BATTERY!");
            // [5] Ruta que aterrice en un landing spot
            for (auto landing_wp : req.conflictive_operations.front().landing_spots.waypoints) {
                gauss_msgs::DeconflictionPlan possible_solution;
                possible_solution.uav_id = req.conflictive_operations.front().uav_id;
                possible_solution.maneuver_type = 5;
                possible_solution.waypoint_list.push_back(req.conflictive_operations.front().estimated_trajectory.waypoints.front());
                possible_solution.waypoint_list.push_back(landing_wp);
                res.deconfliction_plans.push_back(possible_solution);
            }
        } break;
        default:
            break;
    }

    res.message = "Conflict solved";
    res.success = true;
    return res.success;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "tactical_deconfliction");

    ros::NodeHandle nh;
    nh.param("safetyDistance", safety_distance_, 10.0);
    nh.param("actual_wp_on_merge", actual_wp_on_merge_, true);

    ros::ServiceServer deconflict_server = nh.advertiseService("/gauss/new_tactical_deconfliction", deconflictCB);
    ros::ServiceClient check_client = nh.serviceClient<gauss_msgs::CheckConflicts>("/gauss/check_conflicts");

    auto visualization_topic_url = "/gauss/visualize_tactical";

    visualization_pub_ = nh.advertise<visualization_msgs::MarkerArray>(visualization_topic_url, 1);

    ros::Rate rate(1);  // [Hz]
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    ros::spin();
    return 0;
}
