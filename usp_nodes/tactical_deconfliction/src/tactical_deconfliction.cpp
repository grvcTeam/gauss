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
ros::Publisher visualization_pub_;

std::vector<Eigen::Vector3f> perpendicularSeparationVector(const gauss_msgs::Waypoint &_pA, const gauss_msgs::Waypoint &_pB, const double &_safety_distance) {
    std::vector<Eigen::Vector3f> out_avoid_vector;
    Eigen::Vector3f p_a, p_b, unit_vec_ab, unit_vec_ba;
    p_a = Eigen::Vector3f(_pA.x, _pA.y, _pA.z);
    p_b = Eigen::Vector3f(_pB.x, _pB.y, _pB.z);

    unit_vec_ab = (p_a - p_b) / (p_a - p_b).norm();
    unit_vec_ba = (p_b - p_a) / (p_b - p_a).norm();

    double distance_to_check, distance_between_points, distance_operational_volumes;
    distance_between_points = (p_a - p_b).norm();
    distance_operational_volumes = 2.0;
    if (distance_between_points >= distance_operational_volumes) {
        distance_to_check = distance_between_points;
    } else {
        distance_to_check = distance_operational_volumes;
    }
    double distance_to_avoid = _safety_distance;
    if (distance_to_check > _safety_distance) {
        distance_to_avoid = distance_to_check;
    }

    unit_vec_ab = unit_vec_ab * distance_to_avoid;
    unit_vec_ba = unit_vec_ba * distance_to_avoid;

    out_avoid_vector.push_back(unit_vec_ab);
    out_avoid_vector.push_back(unit_vec_ba);

    return out_avoid_vector;
}

std::vector<gauss_msgs::Waypoint> applySeparation(const Eigen::Vector3f &_avoid_vector, const std::vector<gauss_msgs::Waypoint> &_extremes, const gauss_msgs::WaypointList &_estimated) {
    // TODO: This function is applying separation to _extremes, but it should apply separation to _estimated!
    // TODO: Check why _extremes has repeated elements.
    std::vector<gauss_msgs::Waypoint> out_waypoints;
    gauss_msgs::Waypoint pA, pB;  // (pA) ------------------- (pB)
    pA = _extremes.front();
    pB = _extremes.back();

    // pA.x = pA.x + _avoid_vector[0];
    // pA.y = pA.y + _avoid_vector[1];
    // pA.z = pA.z + _avoid_vector[2];
    // out_waypoints.push_back(pA);

    for (auto wp : _extremes) {
        // if (pA.stamp < wp.stamp && wp.stamp < pB.stamp) {
        wp.x = wp.x + _avoid_vector[0];
        wp.y = wp.y + _avoid_vector[1];
        wp.z = wp.z + _avoid_vector[2];
        out_waypoints.push_back(wp);
        // }
    }

    // pB.x = pB.x + _avoid_vector[0];
    // pB.y = pB.y + _avoid_vector[1];
    // pB.z = pB.z + _avoid_vector[2];
    // out_waypoints.push_back(pB);

    return out_waypoints;
}

std::vector<gauss_msgs::Waypoint> delayOperation(const gauss_msgs::ConflictiveOperation &_operation, const gauss_msgs::CheckSegmentsLossResult &_segments) {
    std::vector<gauss_msgs::Waypoint> out_solution;
    double dtime = _segments.segment_first.back().stamp.sec - _segments.segment_first.front().stamp.sec;
    for (int itx = _operation.current_wp; itx < _operation.estimated_trajectory.waypoints.size(); itx++) {
        if (_operation.estimated_trajectory.waypoints.at(itx).stamp > _segments.segment_first.back().stamp && out_solution.size() == 0) {
            gauss_msgs::Waypoint temp_wp;
            temp_wp = _segments.point_at_t_min_segment_first;
            temp_wp.stamp = _segments.segment_first.back().stamp;
            out_solution.push_back(temp_wp);
        } else if (_operation.estimated_trajectory.waypoints.at(itx).stamp > _segments.segment_first.front().stamp) {
            gauss_msgs::Waypoint temp_wp;
            temp_wp = _operation.estimated_trajectory.waypoints.at(itx);
            temp_wp.stamp.fromSec(temp_wp.stamp.toSec() + dtime);
            out_solution.push_back(temp_wp);
        }
    }

    return out_solution;
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
    marker_spheres.scale.x = 2.0;
    marker_spheres.scale.y = 2.0;
    marker_spheres.scale.z = 2.0;
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
    marker_lines.type = visualization_msgs::Marker::LINE_LIST;
    marker_lines.action = visualization_msgs::Marker::ADD;
    marker_lines.pose.orientation.w = 1;
    marker_lines.scale.x = 2.0;
    marker_lines.color = blue;
    marker_lines.lifetime = ros::Duration(1.0);

    for (auto wp : _solution) {
        marker_lines.points.push_back(translateToPoint(wp));
    }

    return marker_lines;
}

bool deconflictCB(gauss_msgs::NewDeconfliction::Request &req, gauss_msgs::NewDeconfliction::Response &res) {
    switch (req.threat.threat_type) {
        case req.threat.LOSS_OF_SEPARATION: {
            std::vector<std::vector<gauss_msgs::Waypoint>> solution_list;
            ROS_ERROR_COND(req.conflictive_operations.size() != 2, "[Tactical] Deconflictive server should receive 2 conflictive operations to solve LOSS OF SEPARATION!");
            // Calculate a vector to separate perpendiculary one trajectory
            std::vector<Eigen::Vector3f> avoid_vectors = perpendicularSeparationVector(req.conflictive_segments.point_at_t_min_segment_first, req.conflictive_segments.point_at_t_min_segment_second, safety_distance_);
            // Solution applying separation to one operation
            solution_list.push_back(applySeparation(avoid_vectors.front(), req.conflictive_segments.segment_first, req.conflictive_operations.front().estimated_trajectory));
            solution_list.push_back(applySeparation(avoid_vectors.back(), req.conflictive_segments.segment_second, req.conflictive_operations.back().estimated_trajectory));
            // Solution delaying one operation
            solution_list.push_back(delayOperation(req.conflictive_operations.front(), req.conflictive_segments));
            solution_list.push_back(delayOperation(req.conflictive_operations.back(), req.conflictive_segments));
            // Visualize "space" results
            visualization_msgs::MarkerArray marker_array;
            visualization_msgs::Marker marker_spheres = createMarkerSpheres(req.conflictive_segments.point_at_t_min_segment_first, req.conflictive_segments.point_at_t_min_segment_second);
            marker_array.markers.push_back(marker_spheres);
            // Visualize just "space" results (solution 0 and 1)
            for (int i = 0; i < 2; i++) { 
                marker_array.markers.push_back(createMarkerLines(solution_list.at(i)));
            }
            visualization_pub_.publish(marker_array);
            // Fill server response
            for (auto solution : solution_list){
                gauss_msgs::DeconflictionPlan possible_solution;
                // possible_solution.maneuver_type = 1;
                possible_solution.waypoint_list = solution;
                res.deconfliction_plans.push_back(possible_solution);
            }
            res.message = "Conflict solved";
            res.success = true;

        } break;
        case req.threat.GEOFENCE_CONFLICT: {
            ROS_ERROR_COND(req.geofences.size() != 1, "[Tactical] Deconflictive server should receive 1 geofence to solve GEOFENCE CONFLICT!");
            // * Assume inputs from monitoring
            // TODO: Check if init and end points have to be further apart from the geofence!
            geometry_msgs::Point p_init_conflict, p_end_conflict, p_min_local_grid, p_max_local_grid;
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

            res.message = "Conflict solved";
            res.success = true;
        } break;
        case req.threat.GEOFENCE_INTRUSION: {
            ROS_ERROR_COND(req.geofences.size() != 1, "[Tactical] Deconflictive server should receive 1 geofence to solve GEOFENCE INTRUSION!");
            // * Assume inputs from monitoring
            // TODO: Check if init and end points have to be further apart from the geofence!
            geometry_msgs::Point p_init_conflict, p_end_conflict, p_min_local_grid, p_max_local_grid;
            p_init_conflict = translateToPoint(req.conflictive_segments.segment_first.front()); // * Should we assume that this is the escape point?
            p_end_conflict = translateToPoint(req.conflictive_segments.segment_first.back());
            ros::Time t_init_conflict = req.conflictive_segments.segment_first.front().stamp;
            ros::Time t_end_conflict = req.conflictive_segments.segment_first.front().stamp;

            gauss_msgs::DeconflictionPlan possible_solution;
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

            res.message = "Conflict solved";
            res.success = true;
        } break;
        case req.threat.UAS_OUT_OV:
            break;
        case req.threat.GNSS_DEGRADATION:
            break;
        case req.threat.LACK_OF_BATTERY:
            break;
        default:
            break;
    }
    res.success = true;
    return res.success;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "tactical_deconfliction");

    ros::NodeHandle nh;
    nh.param("safetyDistance", safety_distance_, 10.0);

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
