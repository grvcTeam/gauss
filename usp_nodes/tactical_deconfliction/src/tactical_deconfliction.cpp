#include <gauss_msgs/CheckConflicts.h>
#include <gauss_msgs/NewDeconfliction.h>
#include <gauss_msgs/ReadIcao.h>
#include <gauss_msgs/ReadOperation.h>
#include <gauss_msgs/Waypoint.h>
#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Eigen>

double safety_distance_;
ros::Publisher visualization_pub_;

geometry_msgs::Point translateToPoint(const gauss_msgs::Waypoint &wp) {
    geometry_msgs::Point p;
    p.x = wp.x;
    p.y = wp.y;
    p.z = wp.z;
    return p;
}

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
            // Calculate a vector to separate perpendiculary one trajectory
            std::vector<Eigen::Vector3f> avoid_vectors = perpendicularSeparationVector(req.conflictive_segments.point_at_t_min_segment_first, req.conflictive_segments.point_at_t_min_segment_second, safety_distance_);
            std::vector<std::vector<gauss_msgs::Waypoint>> solution_list;
            solution_list.push_back(applySeparation(avoid_vectors.front(), req.conflictive_segments.segment_first, req.conflictive_operations.front().estimated_trajectory));
            solution_list.push_back(applySeparation(avoid_vectors.back(), req.conflictive_segments.segment_second, req.conflictive_operations.back().estimated_trajectory));
            // Visualize results
            visualization_msgs::MarkerArray marker_array;
            visualization_msgs::Marker marker_spheres = createMarkerSpheres(req.conflictive_segments.point_at_t_min_segment_first, req.conflictive_segments.point_at_t_min_segment_second);
            marker_array.markers.push_back(marker_spheres);
            for (auto solution : solution_list) {
                marker_array.markers.push_back(createMarkerLines(solution));
            }
            visualization_pub_.publish(marker_array);

        } break;
        case req.threat.GEOFENCE_CONFLICT:
            break;
        case req.threat.GEOFENCE_INTRUSION:
            break;
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
