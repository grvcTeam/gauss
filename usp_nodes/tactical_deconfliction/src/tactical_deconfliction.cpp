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

visualization_msgs::MarkerArray monitoring_array_;

bool deconflictCB(gauss_msgs::Deconfliction::Request &req, gauss_msgs::Deconfliction::Response &res) {
    if (req.tactical && req.threat.threat_type == req.threat.LOSS_OF_SEPARATION) {
    }
    return res.success;
}

void monitoringVisCb(const visualization_msgs::MarkerArray &_msg) {
    monitoring_array_ = _msg;
}

std::vector<Eigen::Vector3f> perpendicularSeparationVector(const geometry_msgs::Point &_pA, const geometry_msgs::Point &_pB, const double &_safety_distance) {
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

void applySeparation(std::vector<geometry_msgs::Point> &_p_extremes_0, std::vector<geometry_msgs::Point> &_p_extremes_1, const std::vector<Eigen::Vector3f> &_avoid_vectors) {
    _p_extremes_0.front().x = _p_extremes_0.front().x + _avoid_vectors.front()[0];
    _p_extremes_0.front().y = _p_extremes_0.front().y + _avoid_vectors.front()[1];
    _p_extremes_0.front().z = _p_extremes_0.front().z + _avoid_vectors.front()[2];
    _p_extremes_0.back().x = _p_extremes_0.back().x + _avoid_vectors.front()[0];
    _p_extremes_0.back().y = _p_extremes_0.back().y + _avoid_vectors.front()[1];
    _p_extremes_0.back().z = _p_extremes_0.back().z + _avoid_vectors.front()[2];
    _p_extremes_1.front().x = _p_extremes_1.front().x + _avoid_vectors.back()[0];
    _p_extremes_1.front().y = _p_extremes_1.front().y + _avoid_vectors.back()[1];
    _p_extremes_1.front().z = _p_extremes_1.front().z + _avoid_vectors.back()[2];
    _p_extremes_1.back().x = _p_extremes_1.back().x + _avoid_vectors.back()[0];
    _p_extremes_1.back().y = _p_extremes_1.back().y + _avoid_vectors.back()[1];
    _p_extremes_1.back().z = _p_extremes_1.back().z + _avoid_vectors.back()[2];
}

visualization_msgs::Marker createMarkerSpheres(const std::vector<geometry_msgs::Point> &_p_extremes_0, const std::vector<geometry_msgs::Point> &_p_extremes_1, const geometry_msgs::Point &_p_middle_0, const geometry_msgs::Point &_p_middle_1) {
    std_msgs::ColorRGBA blue;
    blue.b = 1.0;
    blue.a = 1.0;

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
    marker_spheres.points.push_back(_p_extremes_0.front());
    marker_spheres.colors.push_back(blue);
    marker_spheres.points.push_back(_p_extremes_0.back());
    marker_spheres.colors.push_back(blue);
    marker_spheres.points.push_back(_p_extremes_1.front());
    marker_spheres.colors.push_back(blue);
    marker_spheres.points.push_back(_p_extremes_1.back());
    marker_spheres.colors.push_back(blue);
    marker_spheres.points.push_back(_p_middle_0);
    marker_spheres.colors.push_back(blue);
    marker_spheres.points.push_back(_p_middle_1);
    marker_spheres.colors.push_back(blue);
    return marker_spheres;
}

visualization_msgs::Marker createMarkerLines(const std::vector<geometry_msgs::Point> &_p_extremes_0, const std::vector<geometry_msgs::Point> &_p_extremes_1) {
    std_msgs::ColorRGBA blue;
    blue.b = 1.0;
    blue.a = 1.0;

    visualization_msgs::Marker marker_lines;
    marker_lines.header.stamp = ros::Time::now();
    marker_lines.header.frame_id = "map";
    marker_lines.ns = "lines";
    marker_lines.id = 1;
    marker_lines.type = visualization_msgs::Marker::LINE_LIST;
    marker_lines.action = visualization_msgs::Marker::ADD;
    marker_lines.pose.orientation.w = 1;
    marker_lines.scale.x = 1.0;
    marker_lines.color = blue;
    marker_lines.lifetime = ros::Duration(1.0);
    marker_lines.points.push_back(_p_extremes_0.front());
    marker_lines.points.push_back(_p_extremes_0.back());
    marker_lines.points.push_back(_p_extremes_1.front());
    marker_lines.points.push_back(_p_extremes_1.back());

    return marker_lines;
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
    ros::Subscriber monitoring_vis_sub = nh.subscribe("/gauss/visualize_monitoring", 1, monitoringVisCb);

    ros::Rate rate(1);  // [Hz]
    while (ros::ok()) {
        geometry_msgs::Point p_middle_0, p_middle_1;
        std::vector<geometry_msgs::Point> p_extremes_0, p_extremes_1;
        for (auto marker : monitoring_array_.markers) {
            if (marker.ns == "extremes" && marker.id == 0) {
                p_extremes_0.push_back(marker.points.front());
                p_extremes_0.push_back(marker.points.back());
                p_middle_0.x = (marker.points.front().x + marker.points.back().x) / 2;
                p_middle_0.y = (marker.points.front().y + marker.points.back().y) / 2;
                p_middle_0.z = (marker.points.front().z + marker.points.back().z) / 2;
            }
            if (marker.ns == "extremes" && marker.id == 1) {
                p_extremes_1.push_back(marker.points.front());
                p_extremes_1.push_back(marker.points.back());
                p_middle_1.x = (marker.points.front().x + marker.points.back().x) / 2;
                p_middle_1.y = (marker.points.front().y + marker.points.back().y) / 2;
                p_middle_1.z = (marker.points.front().z + marker.points.back().z) / 2;
            }
        }
        if (p_extremes_0.size() > 0 && p_extremes_1.size() > 0) {
            std::vector<Eigen::Vector3f> avoid_vectors = perpendicularSeparationVector(p_middle_0, p_middle_1, safety_distance);
            applySeparation(p_extremes_0, p_extremes_1, avoid_vectors);
            visualization_msgs::MarkerArray marker_array;
            visualization_msgs::Marker marker_spheres = createMarkerSpheres(p_extremes_0, p_extremes_1, p_middle_0, p_middle_1);
            marker_array.markers.push_back(marker_spheres);
            visualization_msgs::Marker marker_lines = createMarkerLines(p_extremes_0, p_extremes_1);
            marker_array.markers.push_back(marker_lines);
            visualization_pub.publish(marker_array);
        }

        ros::spinOnce();
        rate.sleep();
    }

    ros::spin();
    return 0;
}
