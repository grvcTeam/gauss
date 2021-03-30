#include <gauss_msgs/CheckConflicts.h>
#include <gauss_msgs/Deconfliction.h>
#include <gauss_msgs/ReadIcao.h>
#include <gauss_msgs/ReadOperation.h>
#include <gauss_msgs/Waypoint.h>
#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Eigen>

bool deconflictCB(gauss_msgs::Deconfliction::Request &req, gauss_msgs::Deconfliction::Response &res) {
    if (req.tactical && req.threat.threat_type == req.threat.LOSS_OF_SEPARATION) {
    }
    return res.success;
}

std::vector<gauss_msgs::Waypoint> perpendicularSeparationPoints(const gauss_msgs::Waypoint &_pA, const gauss_msgs::Waypoint &_pB, const double &_safety_distance) {
    std::vector<gauss_msgs::Waypoint> out_avoid_points;
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

    gauss_msgs::Waypoint avoid_point;
    avoid_point.x = p_a[0] + unit_vec_ab[0];
    avoid_point.y = p_a[1] + unit_vec_ab[1];
    avoid_point.z = p_a[2] + unit_vec_ab[2];
    out_avoid_points.push_back(avoid_point);
    avoid_point.x = p_b[0] + unit_vec_ba[0];
    avoid_point.y = p_b[1] + unit_vec_ba[1];
    avoid_point.z = p_b[2] + unit_vec_ba[2];
    out_avoid_points.push_back(avoid_point);

    return out_avoid_points;
}

geometry_msgs::Point translateToPoint(const gauss_msgs::Waypoint &wp) {
    geometry_msgs::Point p;
    p.x = wp.x;
    p.y = wp.y;
    p.z = wp.z;
    return p;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "tactical_deconfliction");

    ros::NodeHandle nh;
    // Read parameters
    double safety_distance;
    nh.param("safetyDistance", safety_distance, 10.0);
    // Server
    ros::ServiceServer deconflict_server = nh.advertiseService("/gauss/tactical_deconfliction", deconflictCB);
    // Cient
    ros::ServiceClient check_client = nh.serviceClient<gauss_msgs::CheckConflicts>("/gauss/check_conflicts");

    auto visualization_topic_url = "/gauss/visualize_tactical";

    ros::Publisher visualization_pub = nh.advertise<visualization_msgs::MarkerArray>(visualization_topic_url, 1);

    ros::Rate rate(1);  // [Hz]
    while (ros::ok()) {
        gauss_msgs::Waypoint pA, pB;
        pA.x = 3;   // This is a test!
        pA.y = 3;   // This is a test!
        pA.z = 20;  // This is a test!
        pB.x = 9;   // This is a test!
        pB.y = 9;   // This is a test!
        pB.z = 10;  // This is a test!

        std::vector<gauss_msgs::Waypoint> avoid_points = perpendicularSeparationPoints(pA, pB, safety_distance);

        visualization_msgs::MarkerArray marker_array;
        visualization_msgs::Marker marker;
        std_msgs::ColorRGBA red, green, blue;  
        red.r = 1.0;
        red.a = 1.0;
        green.g = 1.0;
        green.a = 1.0;
        blue.b = 1.0;
        blue.a = 1.0;

        marker.header.stamp = ros::Time::now();
        marker.header.frame_id = "map"; 
        marker.ns = "avoid_points";     
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1;
        marker.scale.x = 20.0;
        marker.scale.y = 20.0;
        marker.scale.z = 20.0;
        marker.lifetime = ros::Duration(1.0);  
        marker.points.push_back(translateToPoint(pA));
        marker.colors.push_back(blue);
        marker.points.push_back(translateToPoint(pB));
        marker.colors.push_back(green);
        marker.points.push_back(translateToPoint(avoid_points.front()));
        marker.colors.push_back(red);
        marker.points.push_back(translateToPoint(avoid_points.back()));
        marker.colors.push_back(red);
        marker_array.markers.push_back(marker);

        visualization_pub.publish(marker_array);

        ros::spinOnce();
        rate.sleep();
    }

    ros::spin();
    return 0;
}
