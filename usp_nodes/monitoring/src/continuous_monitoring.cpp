#include <gauss_msgs/NewDeconfliction.h>
#include <gauss_msgs/NewThreats.h>
#include <gauss_msgs/ReadIcao.h>
#include <gauss_msgs/ReadOperation.h>
#include <gauss_msgs/ReadGeofences.h>
#include <gauss_msgs/Waypoint.h>
#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Eigen>

bool in_range(double x, double min_x, double max_x) {
    return (x > min_x) && (x < max_x);
}

double clamp(double x, double min_x, double max_x) {
    if (min_x > max_x) {
        ROS_ERROR("[Monitoring] min_x[%lf] > max_x[%lf], swapping!", min_x, max_x);
        std::swap(min_x, max_x);
    }
    return std::max(std::min(x, max_x), min_x);
}

double dot(const geometry_msgs::Vector3& u, const geometry_msgs::Vector3& v) {
    return u.x * v.x + u.y * v.y + u.z * v.z;
}

double length(const geometry_msgs::Vector3& u) {
    return sqrt(u.x * u.x + u.y * u.y + u.z * u.z);
}

// TODO: Use geometry_msgs/Point instead of Waypoint?
geometry_msgs::Vector3 vector_from_point_to_point(const gauss_msgs::Waypoint& A, const gauss_msgs::Waypoint& B) {
    geometry_msgs::Vector3 AB;
    AB.x = B.x - A.x;
    AB.y = B.y - A.y;
    AB.z = B.z - A.z;
    return AB;
}

// Signed Distance Fucntion from P to:
// a sphere centered in C with radius r
double sdSphere(const gauss_msgs::Waypoint& P, const gauss_msgs::Waypoint& C, double r) {
    return length(vector_from_point_to_point(C, P)) - r;
}

// Signed Distance Fucntion from P to:
// a segment (A,B) with some radius r
double sdSegment(const gauss_msgs::Waypoint& P, const gauss_msgs::Waypoint& A, const gauss_msgs::Waypoint& B, double r = 0) {
    geometry_msgs::Vector3 AP = vector_from_point_to_point(A, P);
    geometry_msgs::Vector3 AB = vector_from_point_to_point(A, B);
    double h = clamp(dot(AP, AB) / dot(AB, AB), 0.0, 1.0);
    geometry_msgs::Vector3 aux;
    aux.x = AP.x - AB.x * h;  // TODO: Use eigen?
    aux.y = AP.y - AB.y * h;
    aux.z = AP.z - AB.z * h;
    return length(aux) - r;
}

geometry_msgs::Point translateToPoint(const gauss_msgs::Waypoint& WP) {
    geometry_msgs::Point P;
    P.x = WP.x;
    P.y = WP.y;
    P.z = WP.z;
    return P;
}

struct Segment {
    Segment() = default;
    Segment(gauss_msgs::Waypoint A, gauss_msgs::Waypoint B) {
        // if (A == B) { ROS_WARN("A == B == [%lf, %lf, %lf, %lf]", A.x, A.y, A.z, A.stamp.toSec()); }  // TODO: compare function
        point_A = A;
        point_B = B;
        t_A = A.stamp.toSec();
        t_B = B.stamp.toSec();
        if (t_A >= t_B) {
            // ROS_WARN("t_A[%lf] >= t_B[%lf]", t_A, t_B);
        }
    }

    gauss_msgs::Waypoint point_at_time(double t) const {
        if (t < t_A) {
            // ROS_WARN("t[%lf] < t_A[%lf]", t, t_A);
            return point_A;
        }
        if (t > t_B) {
            // ROS_WARN("t[%lf] > t_B[%lf]", t, t_B);
            return point_B;
        }
        if (t_A == t_B) {
            // ROS_WARN("t_A == t_B == %lf", t_A);
            return point_A;
        }
        if (std::isnan(t)) {
            // ROS_WARN("t is NaN");
            return point_A;
        }

        double m = (t - t_A) / (t_B - t_A);
        gauss_msgs::Waypoint point;
        point.x = point_A.x + m * (point_B.x - point_A.x);
        point.y = point_A.y + m * (point_B.y - point_A.y);
        point.z = point_A.z + m * (point_B.z - point_A.z);
        point.stamp.fromSec(t);
        return point;
    }

    gauss_msgs::Waypoint point_at_param(double m) const {
        gauss_msgs::Waypoint point;
        point.x = point_A.x + m * (point_B.x - point_A.x);
        point.y = point_A.y + m * (point_B.y - point_A.y);
        point.z = point_A.z + m * (point_B.z - point_A.z);
        double t =  t_A + m * (t_B - t_A);
        point.stamp.fromSec(t);
        return point;
    }

    visualization_msgs::Marker translateToMarker(int id, std_msgs::ColorRGBA color) {
        visualization_msgs::Marker marker;
        marker.header.stamp = ros::Time::now();
        marker.header.frame_id = "map";  // TODO: other?
        marker.ns = "segments";
        marker.id = id;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1;
        marker.scale.x = 5.0;  // shaft diameter
        marker.scale.y = 1.5;  // head diameter
        marker.scale.z = 1.0;  // head length (if not zero)
        marker.color = color;
        marker.lifetime = ros::Duration(1.0);  // TODO: pair with frequency
        marker.points.push_back(translateToPoint(point_A));
        marker.points.push_back(translateToPoint(point_B));
        return marker;
    }

    friend std::ostream& operator<<(std::ostream& out, const Segment& s);
    gauss_msgs::Waypoint point_A;
    gauss_msgs::Waypoint point_B;
    double t_A = 0;
    double t_B = 0;
};

std::ostream& operator<<(std::ostream& out, const Segment& s) {
    out << "[(" << s.point_A << "); (" << s.point_B << ")]";
    return out;
}

std::pair<geometry_msgs::Vector3, geometry_msgs::Vector3> delta(const Segment& first, const Segment& second) {
    geometry_msgs::Vector3 delta_alpha;
    delta_alpha.x = second.point_A.x - first.point_A.x;
    delta_alpha.y = second.point_A.y - first.point_A.y;
    delta_alpha.z = second.point_A.z - first.point_A.z;
    geometry_msgs::Vector3 delta_beta;
    delta_beta.x = second.point_B.x - first.point_B.x;
    delta_beta.y = second.point_B.y - first.point_B.y;
    delta_beta.z = second.point_B.z - first.point_B.z;
    return std::make_pair(delta_alpha, delta_beta);
}

double sq_distance(const Segment& first, const Segment& second, double mu) {
    if (mu < 0) {
        // ROS_WARN("mu[%lf] < 0, clamping!", mu);
        mu = 0;
    }

    if (mu > 1) {
        // ROS_WARN("mu[%lf] > 1, clamping!", mu);
        mu = 1;
    }

    auto d = delta(first, second);
    double delta_x = d.first.x + mu * (d.second.x - d.first.x);
    double delta_y = d.first.y + mu * (d.second.y - d.first.y);
    double delta_z = d.first.z + mu * (d.second.z - d.first.z);
    return pow(delta_x, 2) + pow(delta_y, 2) + pow(delta_z, 2);
}

std::pair<double, double> quadratic_roots(double a, double b, double c) {
    if ((a == 0) && (b == 0) && (c == 0)) {
        // ROS_WARN("a = b = c = 0, any number is a solution!");
        return std::make_pair(std::nan(""), std::nan(""));
    }
    if ((a == 0) && (b == 0) && (c != 0)) {
        // ROS_WARN("a = b = 0, there is no solution!");
        return std::make_pair(std::nan(""), std::nan(""));
    }
    if ((a == 0) && (b != 0)) {
        // ROS_WARN("a = 0, non quadratic!");
        return std::make_pair(-c / b, -c / b);
    }

    float d = b * b - 4 * a * c;
    if (d < 0) {
        // ROS_WARN("d = [%lf], complex solutions!", d);
        return std::make_pair(std::nan(""), std::nan(""));
    }
    double e = sqrt(d);
    return std::make_pair((-b - e) / (2 * a), (-b + e) / (2 * a));
}

std::vector<Segment> getFirstSetOfContiguousSegments(const std::vector<Segment>& input) {
    std::vector<Segment> output;
    if (input.size() < 1) {
        ROS_ERROR("[Monitoring] input.size() < 1");
        return output;
    }

    output.push_back(input[0]);
    float t_gap_threshold = 1.0;  // [s]  TODO: as a parameter?
    for (int i = 1; i < input.size(); i++) {
        double t_gap = fabs(input[i].t_A - input[i - 1].t_B);
        if (t_gap > t_gap_threshold) {
            // i-th Segment is not contiguous
            break;
        }
        output.push_back(input[i]);
    }

    return output;
}

geometry_msgs::Vector3 getUnitOutwardVector(const gauss_msgs::Circle& circle, const gauss_msgs::Waypoint wp) {
    geometry_msgs::Vector3 out;
    out.x = wp.x - circle.x_center;
    out.y = wp.y - circle.y_center;
    auto len = sqrt(pow(out.x, 2) + pow(out.y, 2));
    out.x /= len;
    out.y /= len;
    return out;
}

struct LossConflictiveSegments {
    LossConflictiveSegments(const Segment& first, const Segment& second) : first(first), second(second) {}

    friend std::ostream& operator<<(std::ostream& out, const LossConflictiveSegments& r);
    Segment first;
    Segment second;
    double t_min = std::nan("");
    double s_min = std::nan("");
    double t_crossing_0 = std::nan("");
    double t_crossing_1 = std::nan("");
    bool threshold_is_violated = false;
};

std::ostream& operator<<(std::ostream& out, const LossConflictiveSegments& r) {
    out << "first = " << r.first << '\n';
    out << "second = " << r.second << '\n';
    out << "t_min[s] = " << r.t_min << '\n';
    out << "s_min[m2] = " << r.s_min << '\n';
    out << "t_crossing_0[s] = " << r.t_crossing_0 << '\n';
    out << "t_crossing_1[s] = " << r.t_crossing_1 << '\n';
    out << "threshold_is_violated = " << r.threshold_is_violated << '\n';
    return out;
}

LossConflictiveSegments checkUnifiedSegmentsLoss(Segment first, Segment second, double s_threshold) {
    // print('checkUnifiedSegmentsLoss:')
    // print(first.point_A)
    // print(first.point_B)
    // print('___________')
    // print(second.point_A)
    // print(second.point_B)
    auto d = delta(first, second);
    // print(d)
    double c_x = pow(d.first.x, 2);
    double c_y = pow(d.first.y, 2);
    double c_z = pow(d.first.z, 2);
    double b_x = 2 * (d.first.x * d.second.x - c_x);
    double b_y = 2 * (d.first.y * d.second.y - c_y);
    double b_z = 2 * (d.first.z * d.second.z - c_z);
    double a_x = pow(d.second.x - d.first.x, 2);
    double a_y = pow(d.second.y - d.first.y, 2);
    double a_z = pow(d.second.z - d.first.z, 2);
    double a = a_x + a_y + a_z;
    double b = b_x + b_y + b_z;
    double c = c_x + c_y + c_z;

    double mu_min, t_min, s_min;
    if (a == 0) {
        // ROS_WARN("a = 0");
        if (b >= 0) {
            mu_min = 0;
            t_min = first.t_A;
            s_min = c;
        } else {
            mu_min = 1;
            t_min = first.t_B;
            s_min = b + c;
        }

    } else {  // a != 0
        double mu_star = -0.5 * b / a;
        double t_star = first.t_A + mu_star * (first.t_B - first.t_A);
        // print(mu_star)
        // print(t_star)
        // print(sq_distance(first, second, mu_star))
        mu_min = clamp(mu_star, 0, 1);
        t_min = first.t_A + mu_min * (first.t_B - first.t_A);
        s_min = sq_distance(first, second, mu_min);
        // print(mu_min)
        // print(t_min)
        // print(s_min)
    }
    auto result = LossConflictiveSegments(first, second);
    result.t_min = t_min;
    result.s_min = s_min;

    if (s_min > s_threshold) {
        // ROS_INFO("s_min[%lf] > s_threshold[%lf]", s_min, s_threshold);
        result.threshold_is_violated = false;
        return result;
    }

    auto mu_bar = quadratic_roots(a, b, c - s_threshold);
    double t_bar_0 = first.t_A + mu_bar.first * (first.t_B - first.t_A);
    double t_bar_1 = first.t_A + mu_bar.second * (first.t_B - first.t_A);
    // print(mu_bar)
    // print(t_bar_0, t_bar_1)
    double mu_crossing_0 = clamp(mu_bar.first, 0, 1);
    double mu_crossing_1 = clamp(mu_bar.second, 0, 1);
    double t_crossing_0 = first.t_A + mu_crossing_0 * (first.t_B - first.t_A);
    double t_crossing_1 = first.t_A + mu_crossing_1 * (first.t_B - first.t_A);
    // print(mu_crossing_0, mu_crossing_1)
    // print(t_crossing_0, t_crossing_1)
    auto first_in_conflict = Segment(first.point_at_time(t_crossing_0), first.point_at_time(t_crossing_1));
    auto second_in_conflict = Segment(second.point_at_time(t_crossing_0), second.point_at_time(t_crossing_1));

    result.first = first_in_conflict;
    result.second = second_in_conflict;
    result.t_crossing_0 = t_crossing_0;
    result.t_crossing_1 = t_crossing_1;
    result.threshold_is_violated = true;
    return result;
}

LossConflictiveSegments checkSegmentsLoss(const std::pair<Segment, Segment>& segments, double s_threshold) {
    // print('checkSegmentsLoss:')
    // print(first.point_A)
    // print(first.point_B)
    // print('___________')
    // print(second.point_A)
    // print(second.point_B)
    double t_A1 = segments.first.t_A;
    double t_B1 = segments.first.t_B;
    double t_A2 = segments.second.t_A;
    double t_B2 = segments.second.t_B;
    double t_alpha = std::max(t_A1, t_A2);
    double t_beta = std::min(t_B1, t_B2);
    if (t_alpha > t_beta) {
        // ROS_INFO("t_alpha[%lf] > t_beta[%lf]", t_alpha, t_beta);
        return LossConflictiveSegments(segments.first, segments.second);
    }

    auto P_alpha1 = segments.first.point_at_time(t_alpha);
    auto P_beta1 = segments.first.point_at_time(t_beta);
    auto P_alpha2 = segments.second.point_at_time(t_alpha);
    auto P_beta2 = segments.second.point_at_time(t_beta);
    return checkUnifiedSegmentsLoss(Segment(P_alpha1, P_beta1), Segment(P_alpha2, P_beta2), s_threshold);
}

std::vector<LossConflictiveSegments> checkTrajectoriesLoss(const std::pair<gauss_msgs::WaypointList, gauss_msgs::WaypointList>& trajectories, double s_threshold) {
    std::vector<LossConflictiveSegments> segment_loss_results;
    if (trajectories.first.waypoints.size() < 2) {
        // TODO: Warn and push the same point twice?
        ROS_ERROR("[Monitoring] Trajectory must contain at least 2 points, [%ld] found in first argument", trajectories.first.waypoints.size());
        return segment_loss_results;
    }
    if (trajectories.second.waypoints.size() < 2) {
        // TODO: Warn and push the same point twice?
        ROS_ERROR("[Monitoring] Trajectory must contain at least 2 points, [%ld] found in second argument", trajectories.second.waypoints.size());
        return segment_loss_results;
    }

    for (int i = 0; i < trajectories.first.waypoints.size() - 1; i++) {
        std::pair<Segment, Segment> segments;
        // printf("First segment, i = %d\n", i);
        segments.first = Segment(trajectories.first.waypoints[i], trajectories.first.waypoints[i + 1]);
        // std::cout << segments.first.point_A << "_____________\n" << segments.first.point_B << '\n';
        for (int j = 0; j < trajectories.second.waypoints.size() - 1; j++) {
            // printf("Second segment, j = %d\n", j);
            segments.second = Segment(trajectories.second.waypoints[j], trajectories.second.waypoints[j + 1]);
            // std::cout << segments.second.point_A << "_____________\n" << segments.second.point_B << '\n';
            auto loss_check = checkSegmentsLoss(segments, s_threshold);
            if (loss_check.threshold_is_violated) {
                ROS_ERROR("[Monitoring] Loss of separation! [i = %d, j = %d]", i, j);
                // std::cout << loss_check << '\n';
                segment_loss_results.push_back(loss_check);
            }
        }
    }
    return segment_loss_results;
}

struct LossExtreme {
    gauss_msgs::Waypoint in_point;
    gauss_msgs::Waypoint out_point;
};

visualization_msgs::Marker translateToMarker(const LossExtreme& extremes, int id = 0) {
    visualization_msgs::Marker marker;
    std_msgs::ColorRGBA red, green;  // TODO: constants
    red.r = 1.0;
    red.a = 1.0;
    green.g = 1.0;
    green.a = 1.0;

    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = "map";  // TODO: other?
    marker.ns = "extremes";          // TODO: other?
    marker.id = id;
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1;
    marker.scale.x = 5.0;
    marker.scale.y = 5.0;
    marker.scale.z = 5.0;
    marker.lifetime = ros::Duration(1.0);  // TODO: pair with frequency
    marker.points.push_back(translateToPoint(extremes.in_point));
    marker.colors.push_back(red);
    marker.points.push_back(translateToPoint(extremes.out_point));
    marker.colors.push_back(green);
    return marker;
}

gauss_msgs::ConflictiveOperation fillConflictiveOperation(const int& _trajectory_index, const std::map<int, gauss_msgs::Operation>& _index_to_operation_map) {
    gauss_msgs::ConflictiveOperation out_msg;
    out_msg.actual_wp = _index_to_operation_map.at(_trajectory_index).track.waypoints.back();
    out_msg.current_wp = _index_to_operation_map.at(_trajectory_index).current_wp;
    out_msg.estimated_trajectory = _index_to_operation_map.at(_trajectory_index).estimated_trajectory;
    out_msg.flight_plan = _index_to_operation_map.at(_trajectory_index).flight_plan;
    out_msg.flight_plan_updated = _index_to_operation_map.at(_trajectory_index).flight_plan_updated;
    out_msg.landing_spots = _index_to_operation_map.at(_trajectory_index).landing_spots;
    out_msg.operational_volume = _index_to_operation_map.at(_trajectory_index).operational_volume;
    out_msg.uav_id = _index_to_operation_map.at(_trajectory_index).uav_id;

    return out_msg;
}

struct LossResult {
    LossResult(const int i, const int j) : first_trajectory_index(i), second_trajectory_index(j) {}
    friend std::ostream& operator<<(std::ostream& out, const LossResult& r);
    int first_trajectory_index;
    int second_trajectory_index;
    std::vector<LossConflictiveSegments> loss_conflictive_segments;

    bool isEqual(const LossResult& other) {
        const float check_time_margin = 10.0;  // [s]
        return (first_trajectory_index == other.first_trajectory_index && second_trajectory_index == other.second_trajectory_index) &&
               (std::abs(loss_conflictive_segments.front().t_crossing_0 - other.loss_conflictive_segments.front().t_crossing_0) <= check_time_margin ||
                std::abs(loss_conflictive_segments.front().t_crossing_1 - other.loss_conflictive_segments.front().t_crossing_1) <= check_time_margin ||
                std::abs(loss_conflictive_segments.back().t_crossing_0 - other.loss_conflictive_segments.back().t_crossing_0) <= check_time_margin ||
                std::abs(loss_conflictive_segments.back().t_crossing_1 - other.loss_conflictive_segments.back().t_crossing_1) <= check_time_margin);
    }
    gauss_msgs::NewThreat convertToThreat(const std::map<int, gauss_msgs::Operation>& _index_to_operation_map, double& count_id) {
        gauss_msgs::NewThreat out_threat;
        out_threat.threat_id = count_id++;
        out_threat.threat_type = out_threat.LOSS_OF_SEPARATION;
        out_threat.uav_ids.push_back(_index_to_operation_map.at(first_trajectory_index).uav_id);
        out_threat.uav_ids.push_back(_index_to_operation_map.at(second_trajectory_index).uav_id);
        out_threat.priority_ops.push_back(_index_to_operation_map.at(first_trajectory_index).priority);
        out_threat.priority_ops.push_back(_index_to_operation_map.at(second_trajectory_index).priority);
        out_threat.conflictive_operations.push_back(fillConflictiveOperation(first_trajectory_index, _index_to_operation_map));
        out_threat.conflictive_operations.push_back(fillConflictiveOperation(second_trajectory_index, _index_to_operation_map));
        for (auto j : loss_conflictive_segments) {
            out_threat.loss_conflictive_segments.segment_first.push_back(j.first.point_A);
            out_threat.loss_conflictive_segments.segment_first.push_back(j.first.point_B);
            out_threat.loss_conflictive_segments.segment_second.push_back(j.second.point_A);
            out_threat.loss_conflictive_segments.segment_second.push_back(j.second.point_B);
            out_threat.loss_conflictive_segments.t_min = j.t_min;
            out_threat.loss_conflictive_segments.s_min = j.s_min;
            out_threat.loss_conflictive_segments.t_crossing_0 = j.t_crossing_0;
            out_threat.loss_conflictive_segments.t_crossing_1 = j.t_crossing_1;
            out_threat.loss_conflictive_segments.point_at_t_min_segment_first = j.first.point_at_time(j.t_min);
            out_threat.loss_conflictive_segments.point_at_t_min_segment_second = j.second.point_at_time(j.t_min);
        }

        return out_threat;
    }
};

struct GeoConflictiveTrajectory {
    GeoConflictiveTrajectory(int i): trajectory_index(i) {}
    int trajectory_index;
    std::vector<Segment> geofence_conflictive_segments;
    gauss_msgs::Waypoint closest_exit_wp;  // Use the mandatory field as intrusion flag

    bool isEqual(const GeoConflictiveTrajectory& other) {
        const double check_time_margin = 10.0;
        return (trajectory_index == other.trajectory_index &&
                closest_exit_wp.mandatory == other.closest_exit_wp.mandatory &&
               (geofence_conflictive_segments.front().t_A - other.geofence_conflictive_segments.front().t_A <= check_time_margin ||
                geofence_conflictive_segments.back().t_B - other.geofence_conflictive_segments.back().t_B <= check_time_margin));
    }
};

struct GeofenceResult {
    GeofenceResult(int i): geofence_id(i) {}
    int geofence_id;
    std::vector<GeoConflictiveTrajectory> geo_conflictive_trajectories;
    
    bool isEqual(const GeofenceResult& other) {
        const float check_time_margin = 10.0;  // [s]
        if (geofence_id != other.geofence_id) {
            return false;
        } else {
            for (auto geo_trajectory : geo_conflictive_trajectories) {
                std::vector<GeoConflictiveTrajectory>::const_iterator traj_it = std::find_if(other.geo_conflictive_trajectories.begin(), other.geo_conflictive_trajectories.end(),
                                                                                             [geo_trajectory](GeoConflictiveTrajectory _traj) { return _traj.isEqual(geo_trajectory);});
                return (traj_it != other.geo_conflictive_trajectories.end());
            }
        }
    }
    std::vector<gauss_msgs::NewThreat> convertToThreat(const std::map<int, gauss_msgs::Operation>& _index_to_operation_map, const std::map<int, gauss_msgs::Geofence>& _index_to_geofence_map, double& count_id) {
        std::vector<gauss_msgs::NewThreat> out_threats;
        for (auto geo_conflictive_trajectory : geo_conflictive_trajectories) {
            gauss_msgs::NewThreat aux_threat;
            aux_threat.threat_id = count_id++;
            aux_threat.geofence_ids.push_back(geofence_id);
            aux_threat.uav_ids.push_back(_index_to_operation_map.at(geo_conflictive_trajectory.trajectory_index).uav_id);
            aux_threat.conflictive_geofences.push_back(_index_to_geofence_map.at(geofence_id));
            aux_threat.conflictive_operations.push_back(fillConflictiveOperation(geo_conflictive_trajectory.trajectory_index, _index_to_operation_map));
            std::vector<Segment> aux_vec = getFirstSetOfContiguousSegments(geo_conflictive_trajectory.geofence_conflictive_segments);
            for (auto segment : aux_vec) {
                aux_threat.geofence_conflictive_segments.first_contiguous_segment.push_back(segment.point_A);
                aux_threat.geofence_conflictive_segments.first_contiguous_segment.push_back(segment.point_B);
            }
            for (auto segment : geo_conflictive_trajectory.geofence_conflictive_segments) {
                aux_threat.geofence_conflictive_segments.all_segments.push_back(segment.point_A);
                aux_threat.geofence_conflictive_segments.all_segments.push_back(segment.point_B);
            }

            auto geo_circle = _index_to_geofence_map.at(geofence_id).circle;
            auto crossing_0 = aux_threat.geofence_conflictive_segments.first_contiguous_segment.front();
            auto crossing_1 = aux_threat.geofence_conflictive_segments.first_contiguous_segment.back();
            aux_threat.geofence_conflictive_segments.crossing_0_out_vector = getUnitOutwardVector(geo_circle, crossing_0);
            aux_threat.geofence_conflictive_segments.crossing_1_out_vector = getUnitOutwardVector(geo_circle, crossing_1);
            if (geo_conflictive_trajectory.closest_exit_wp.mandatory) {
                aux_threat.threat_type = aux_threat.GEOFENCE_INTRUSION;
                auto closest_exit = geo_conflictive_trajectory.closest_exit_wp;
                aux_threat.geofence_conflictive_segments.closest_exit_wp = closest_exit;
                aux_threat.geofence_conflictive_segments.closest_exit_out_vector = getUnitOutwardVector(geo_circle, closest_exit);
            } else {
                aux_threat.threat_type = aux_threat.GEOFENCE_CONFLICT;
            }
            out_threats.push_back(aux_threat);
        }

        return out_threats;
    }
};

std::vector<LossResult> getContiguousResults(const LossResult& input) {
    std::vector<LossResult> output;
    if (input.loss_conflictive_segments.size() < 1) {
        ROS_ERROR("[Monitoring] input.loss_conflictive_segments.size() < 1");
        return output;
    }

    float t_gap_threshold = 1.0;  // [s]  TODO: as a parameter?
    auto current_loss_result = input;
    current_loss_result.loss_conflictive_segments.clear();
    current_loss_result.loss_conflictive_segments.push_back(input.loss_conflictive_segments[0]);
    for (int i = 1; i < input.loss_conflictive_segments.size(); i++) {
        double t_gap_first = fabs(input.loss_conflictive_segments[i].first.t_A - input.loss_conflictive_segments[i - 1].first.t_B);
        double t_gap_second = fabs(input.loss_conflictive_segments[i].second.t_A - input.loss_conflictive_segments[i - 1].second.t_B);
        if ((t_gap_first > t_gap_threshold) || (t_gap_second > t_gap_threshold)) {
            // i-th element is not contiguous
            output.push_back(current_loss_result);
            current_loss_result.loss_conflictive_segments.clear();
        }
        current_loss_result.loss_conflictive_segments.push_back(input.loss_conflictive_segments[i]);
    }
    output.push_back(current_loss_result);

    return output;
}

std::pair<LossExtreme, LossExtreme> calculateExtremes(const LossResult& result) {
    std::pair<LossExtreme, LossExtreme> extremes;
    if (result.loss_conflictive_segments.size() < 1) {
        ROS_ERROR("[Monitoring] result.loss_conflictive_segments.size() < 1");
        return extremes;
    }

    // In points are for sure A's from segment 0
    extremes.first.in_point = result.loss_conflictive_segments[0].first.point_A;
    extremes.second.in_point = result.loss_conflictive_segments[0].second.point_A;

    // Initialize out points as B's from segment 0...
    extremes.first.out_point = result.loss_conflictive_segments[0].first.point_B;
    extremes.second.out_point = result.loss_conflictive_segments[0].second.point_B;
    //  ...but update if more contiguous segments are available
    float t_gap_threshold = 1.0;  // [s]
    // Check for first
    for (int i = 1; i < result.loss_conflictive_segments.size(); i++) {
        double t_gap = fabs(result.loss_conflictive_segments[i].first.t_A - result.loss_conflictive_segments[i - 1].first.t_B);
        if (t_gap > t_gap_threshold) {
            break;
        }
        extremes.first.out_point = result.loss_conflictive_segments[i].first.point_B;
    }
    // Check for second
    for (int i = 1; i < result.loss_conflictive_segments.size(); i++) {
        double t_gap = fabs(result.loss_conflictive_segments[i].second.t_A - result.loss_conflictive_segments[i - 1].second.t_B);
        if (t_gap > t_gap_threshold) {
            break;
        }
        extremes.second.out_point = result.loss_conflictive_segments[i].second.point_B;
    }

    return extremes;
}

visualization_msgs::Marker translateToMarker(const LossResult& result) {
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = "map";  // TODO: other?
    marker.ns = "loss_" + std::to_string(result.first_trajectory_index) + "_" + std::to_string(result.second_trajectory_index);
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1;
    marker.scale.x = 1.0;
    marker.color.r = 1.0;  // TODO: color?
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(1.0);  // TODO: pair with frequency
    for (auto segments_loss : result.loss_conflictive_segments) {
        marker.points.push_back(translateToPoint(segments_loss.first.point_A));
        marker.points.push_back(translateToPoint(segments_loss.first.point_B));
        marker.points.push_back(translateToPoint(segments_loss.second.point_A));
        marker.points.push_back(translateToPoint(segments_loss.second.point_B));
    }
    /*
  double t_min = std::nan("");
  double s_min = std::nan("");
  double t_crossing_0 = std::nan("");
  double t_crossing_1 = std::nan("");
  bool threshold_is_violated = false;
*/
    return marker;
}

std::ostream& operator<<(std::ostream& out, const LossResult& r) {
    out << "first_trajectory_index = " << r.first_trajectory_index << '\n';
    out << "second_trajectory_index = " << r.second_trajectory_index << '\n';
    out << "loss_conflictive_segments = [" << r.second_trajectory_index << '\n';
    for (int i = 0; i < r.loss_conflictive_segments.size(); i++) {
        out << r.loss_conflictive_segments[i] << '\n';
    }
    out << "]\n";
    return out;
}

bool happensBefore(const LossResult& a, const LossResult& b) {
    if (a.loss_conflictive_segments.size() < 1) {
        ROS_ERROR("[Monitoring] a.loss_conflictive_segments.size() < 1");
        return false;
    }
    if (b.loss_conflictive_segments.size() < 1) {
        ROS_ERROR("[Monitoring] b.loss_conflictive_segments.size() < 1");
        return true;
    }

    return a.loss_conflictive_segments[0].t_crossing_0 < b.loss_conflictive_segments[0].t_crossing_0;
}

void cleanStoredList(std::vector<LossResult>& _stored_loss_result_list, std::vector<GeofenceResult>& _stored_geofence_result_list, const std::vector<LossResult>& _actual_loss_result_list, const std::vector<GeofenceResult>& _actual_geofence_result_list, const double& _check_time_margin) {
    if (_actual_loss_result_list.size() == 0) {
        _stored_loss_result_list.clear();
    } else {
        for (auto _stored_result = _stored_loss_result_list.begin(); _stored_result != _stored_loss_result_list.end();) {
            std::vector<LossResult>::const_iterator actual_it = std::find_if(_actual_loss_result_list.begin(), _actual_loss_result_list.end(),
                                                                             [_stored_result](LossResult _actual_result) { return _actual_result.isEqual(*_stored_result); });
            bool do_erase = (actual_it == _actual_loss_result_list.end());
            if (do_erase) {
                _stored_result = _stored_loss_result_list.erase(_stored_result);
            } else {
                _stored_result++;
            }
        }
    }
}

gauss_msgs::NewThreats manageResultList(std::vector<LossResult>& _loss_result_list, std::vector<GeofenceResult>& _geofence_result_list, const std::map<int, gauss_msgs::Operation>& _index_to_operation_map, const std::map<int, gauss_msgs::Geofence>& _index_to_geofence_map) {
    double check_time_margin = 10.0;
    static double threat_count_id = 0;
    static double stored_loss_id_count = 0;
    static double stored_geofence_id_count = 0;
    gauss_msgs::NewThreats out_threats;
    static std::vector<LossResult> stored_loss_result_list;
    static std::vector<GeofenceResult> stored_geofence_result_list;

    if (stored_loss_result_list.size() > 0 || stored_geofence_result_list.size() > 0) cleanStoredList(stored_loss_result_list, stored_geofence_result_list, _loss_result_list, _geofence_result_list, check_time_margin);
    if (_loss_result_list.size() > 0) {
        if (stored_loss_result_list.size() == 0) {
            stored_loss_result_list.push_back(_loss_result_list.front());
            out_threats.request.threats.push_back(_loss_result_list[0].convertToThreat(_index_to_operation_map, threat_count_id));
        } else {
            for (auto _loss_result : _loss_result_list) {
                bool save_loss_result = false;
                // Using lambda, check if both trajectory index of _loss_result are in stored_loss_result_list
                std::vector<LossResult>::iterator stored_it = std::find_if(stored_loss_result_list.begin(), stored_loss_result_list.end(),
                                                                           [_loss_result](LossResult stored_loss_result) { return stored_loss_result.isEqual(_loss_result); });
                save_loss_result = (stored_it == stored_loss_result_list.end());
                if (save_loss_result) {
                    stored_loss_result_list.push_back(_loss_result);
                    out_threats.request.threats.push_back(_loss_result.convertToThreat(_index_to_operation_map, threat_count_id));
                }
            }
        }
    }
    if (_geofence_result_list.size() > 0) {
        if (stored_geofence_result_list.size() == 0) {
            stored_geofence_result_list.push_back(_geofence_result_list.front());
            std::vector<gauss_msgs::NewThreat> aux_vec = _geofence_result_list[0].convertToThreat(_index_to_operation_map, _index_to_geofence_map, threat_count_id);
            out_threats.request.threats.insert(out_threats.request.threats.end(), aux_vec.begin(), aux_vec.end());
        } else {
            for (auto _geofence_result : _geofence_result_list) {
                bool save_geofence_result = false;
                // Using lambda, check if both trajectory index of _geofence_result are in stored_geofence_result_list
                std::vector<GeofenceResult>::iterator stored_it = std::find_if(stored_geofence_result_list.begin(), stored_geofence_result_list.end(),
                                                                               [_geofence_result](GeofenceResult stored_loss_result) { return stored_loss_result.isEqual(_geofence_result); });
                save_geofence_result = (stored_it == stored_geofence_result_list.end());
                if (save_geofence_result) {
                    stored_geofence_result_list.push_back(_geofence_result);
                    std::vector<gauss_msgs::NewThreat> aux_vec = _geofence_result.convertToThreat(_index_to_operation_map, _index_to_geofence_map, threat_count_id);
                    out_threats.request.threats.insert(out_threats.request.threats.end(), aux_vec.begin(), aux_vec.end());
                }
            }
        }
    }

    return out_threats;
}

std::pair<double, double> checkGeofence2D(const Segment& segment, const gauss_msgs::Circle& circle) {

    auto translated_segment = segment;
    translated_segment.point_A.x -= circle.x_center;
    translated_segment.point_A.y -= circle.y_center;
    translated_segment.point_B.x -= circle.x_center;
    translated_segment.point_B.y -= circle.y_center;

    float sq_distance_A = pow(translated_segment.point_A.x, 2) + pow(translated_segment.point_A.y, 2);
    float sq_distance_B = pow(translated_segment.point_B.x, 2) + pow(translated_segment.point_B.y, 2);
    float sq_radius = pow(circle.radius, 2);

    bool point_A_is_in = (sq_distance_A < sq_radius);
    bool point_B_is_in = (sq_distance_B < sq_radius);

    // Geofence2DResult result;
    if (point_A_is_in && point_B_is_in) {
        // A and B inside the circle
        // ROS_INFO("A and B inside the circle");
        return std::make_pair(segment.t_A, segment.t_B);
    }

    float a_x = pow(translated_segment.point_B.x - translated_segment.point_A.x, 2);
    float b_x = 2.0 * (translated_segment.point_B.x - translated_segment.point_A.x) * translated_segment.point_A.x;
    float c_x = pow(translated_segment.point_A.x, 2);
    float a_y = pow(translated_segment.point_B.y - translated_segment.point_A.y, 2);
    float b_y = 2.0 * (translated_segment.point_B.y - translated_segment.point_A.y) * translated_segment.point_A.y;
    float c_y = pow(translated_segment.point_A.y, 2);

    auto m_crossing = quadratic_roots(a_x + a_y, b_x + b_y, c_x + c_y - sq_radius);
    // ROS_INFO("Roots: m = [%lf, %lf]", m_crossing.first, m_crossing.second);

    if (std::isnan(m_crossing.first)) {  // m_crossing.second should also be nan
        // There is no intersection at all
        // ROS_INFO("There is no intersection at all");
        return std::make_pair(std::nan(""), std::nan(""));
    }

    auto t_crossing_0 = translated_segment.t_A + clamp(m_crossing.first,  0, 1) * (translated_segment.t_B - translated_segment.t_A);
    auto t_crossing_1 = translated_segment.t_A + clamp(m_crossing.second, 0, 1) * (translated_segment.t_B - translated_segment.t_A);
    return std::make_pair(t_crossing_0, t_crossing_1);
}

bool checkOverlappingInTime(std::pair<double, double> time_interval_a, std::pair<double, double> time_interval_b) {
    return (time_interval_a.first <= time_interval_b.second) && (time_interval_a.second >= time_interval_b.first);
}

geometry_msgs::Point calculateClosestExit(const geometry_msgs::Point& current, const gauss_msgs::Circle& circle) {
    geometry_msgs::Point out;
    float delta_x = current.x - circle.x_center;
    float delta_y = current.y - circle.y_center;
    auto distance = sqrt(pow(delta_x, 2) + pow(delta_y, 2));
    if (distance < 1e-3) {
        // At the center of the geofence? Go East!
        out.x = circle.x_center + circle.radius;
        out.y = circle.y_center;
        // out.x = std::nan("");  // TODO: Better NaN and handle later?
        // out.y = std::nan("");
        return out;
    }
    out.x = circle.x_center + (delta_x / distance) * circle.radius;
    out.y = circle.y_center + (delta_y / distance) * circle.radius;
    return out;
}

std::vector<GeoConflictiveTrajectory> checkGeofenceConflict(const std::vector<gauss_msgs::WaypointList>& trajectories, const std::vector<double>& volumes, const gauss_msgs::Geofence& geofence) {
    std::vector<GeoConflictiveTrajectory> result;

    if (!geofence.cylinder_shape) {
        // TODO: implement also for polygons
        ROS_ERROR("[Monitoring] Polygon geofences not implemented yet");
        return result;
        // float64 min_altitude	# meters
        // float64 max_altitude	# meters
        // Polygon polygon
        //     float64[] x
        //     float64[] y
    }

    if (trajectories.size() != volumes.size()) {
        ROS_ERROR("[Monitoring] Sizes do not match: trajectories.size() = %ld, volumes.size() = %ld", trajectories.size(), volumes.size());
        return result;
    }

    for (int i = 0; i < trajectories.size(); i++) {
        // ROS_INFO("Checking trajectory [%d]", i);
        GeoConflictiveTrajectory current_result(i);

        if (trajectories[i].waypoints.size() < 2) {
            // TODO: Warn and push the same point twice?
            ROS_ERROR("[Monitoring]: trajectory must contain at least 2 points, [%ld] found in second argument", trajectories[i].waypoints.size());
            continue;
        }
        auto operational_volume = volumes[i];
        auto rectified_geofence = geofence;
        rectified_geofence.min_altitude -= operational_volume;
        rectified_geofence.max_altitude += operational_volume;
        rectified_geofence.circle.radius += operational_volume;

        for (int j = 0; j < trajectories[i].waypoints.size() - 1; j++) {
            // ROS_INFO("Checking segment [%d, %d]", j, j + 1);
            auto segment = Segment(trajectories[i].waypoints[j], trajectories[i].waypoints[j + 1]);

            // TODO: combine the following two ifs into a single one?
            if ((segment.point_A.z < rectified_geofence.min_altitude) && (segment.point_B.z < rectified_geofence.min_altitude)) {
                // The whole segment lies below the rectified_geofence
                // ROS_INFO("The whole segment lies below the geofence");
                continue;
            }
            if ((segment.point_A.z > rectified_geofence.max_altitude) && (segment.point_B.z > rectified_geofence.max_altitude)) {
                // The whole segment lies above the rectified_geofence
                // ROS_INFO("The whole segment lies above the geofence");
                continue;
            }

            // TODO: enlarge the circle with operational volume (* some security_gain)
            float delta_z = segment.point_B.z - segment.point_A.z;
            // if (fabs(delta_z) < 1e-3) {  // TODO: Early return?
            //     // We can consider the whole segment lies inside the geofence z interval
            //     // TODO: Consider the special case of j == 0 for geofence intrusion!
            //     ROS_INFO("We can consider the whole segment lies inside the geofence z interval");
            // }

            // Get the segment that does lie between min_alt, max_alt
            if (segment.point_A.z < rectified_geofence.min_altitude) {
                // Point A lies below the geofence z interval
                // ROS_INFO("Point A lies below the geofence z interval");
                float m_min = (rectified_geofence.min_altitude - segment.point_A.z) / delta_z;
                segment.point_A = segment.point_at_param(m_min);
                // TODO: Consider the special case of j == 0 for geofence intrusion!
            } else if (segment.point_A.z > rectified_geofence.max_altitude) {
                // Point A lies above the geofence z interval
                // ROS_INFO("Point A lies above the geofence z interval");
                float m_max = (rectified_geofence.max_altitude - segment.point_A.z) / delta_z;
                segment.point_A = segment.point_at_param(m_max);
                // TODO: Consider the special case of j == 0 for geofence intrusion!
            }

            // Same for point B  // TODO: repeated code!
            if (segment.point_B.z < rectified_geofence.min_altitude) {
                // Point B lies below the geofence z interval
                // ROS_INFO("Point B lies below the geofence z interval");
                float m_min = (rectified_geofence.min_altitude - segment.point_B.z) / delta_z;
                segment.point_B = segment.point_at_param(m_min);
            } else if (segment.point_B.z > rectified_geofence.max_altitude) {
                // Point B lies above the geofence z interval
                // ROS_INFO("Point B lies above the geofence z interval");
                float m_max = (rectified_geofence.max_altitude - segment.point_B.z) / delta_z;
                segment.point_B = segment.point_at_param(m_max);
            }

            auto conflict_times = checkGeofence2D(segment, rectified_geofence.circle);
            if (std::isnan(conflict_times.first)) {  // conflict_times.second should be also nan
                // ROS_INFO("No conflicts");
                // return result; // TODO: Only for debug!
                continue;
            }  // TODO: Rename to GeofenceConflict?

            auto current_time = ros::Time::now().toSec();
            if (current_time > conflict_times.second) {  // Should be also > conflict_times.second
                // ROS_INFO("Past conflicts do not count :)");
                // return result; // TODO: Only for debug!
                continue;
            }

            if (checkOverlappingInTime(conflict_times, std::make_pair(rectified_geofence.start_time.toSec(), rectified_geofence.end_time.toSec()))) {
                // ROS_INFO("Conflict!");  // TODO
                auto current_position = trajectories[i].waypoints[j];
                // Check also for intrusion:
                if ((j == 0)
                    && in_range(current_position.z, rectified_geofence.min_altitude, rectified_geofence.max_altitude)
                    && (pow(current_position.x - rectified_geofence.circle.x_center, 2) + pow(current_position.y - rectified_geofence.circle.y_center, 2) < pow(rectified_geofence.circle.radius, 2))
                    ) {
                    ROS_ERROR("[Monitoring] Geofence intrusion! [i = %d]", current_result.trajectory_index);
                    current_result.closest_exit_wp.mandatory = true;
                    auto exit_circle = geofence.circle;
                    exit_circle.radius += operational_volume * 2.0;
                    auto xy_closest_exit = calculateClosestExit(translateToPoint(current_position), exit_circle);
                    // auto xy_closest_exit = calculateClosestExit(translateToPoint(current_position), rectified_geofence.circle);
                    current_result.closest_exit_wp.x = xy_closest_exit.x;
                    current_result.closest_exit_wp.y = xy_closest_exit.y;
                    current_result.closest_exit_wp.z = current_position.z;  // Suppose we want the closest exit with no changes in altuitude!
                }
                current_result.geofence_conflictive_segments.push_back(Segment(segment.point_at_time(conflict_times.first), segment.point_at_time(conflict_times.second)));
            }
            // result.push_back(current_result);  // TODO: Only for debug!
            // return result;  // TODO: Only for debug!
        }
        if (current_result.geofence_conflictive_segments.size() > 0) {
            result.push_back(current_result);
        }
    }

    return result;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "continuous_monitoring");

    ros::NodeHandle n;
    // ros::NodeHandle np("~");
    ROS_INFO("[Monitoring] Started monitoring node!");
    double safety_distance;
    n.param("safetyDistance", safety_distance, 10.0);
    double safety_distance_sq = pow(safety_distance, 2);

    auto read_icao_srv_url = "/gauss/read_icao";
    auto read_operation_srv_url = "/gauss/read_operation";
    auto read_geofences_srv_url = "/gauss/read_geofences";
    auto tactical_srv_url = "/gauss/new_tactical_deconfliction";
    auto alternatives_topic_url = "/gauss/possible_alternatives";
    auto new_threats_srv_url = "/gauss/new_threats";
    auto visualization_topic_url = "/gauss/visualize_monitoring";

    ros::ServiceClient icao_client = n.serviceClient<gauss_msgs::ReadIcao>(read_icao_srv_url);
    ros::ServiceClient operation_client = n.serviceClient<gauss_msgs::ReadOperation>(read_operation_srv_url);
    ros::ServiceClient geofences_client = n.serviceClient<gauss_msgs::ReadGeofences>(read_geofences_srv_url);
    ros::ServiceClient tactical_client = n.serviceClient<gauss_msgs::NewDeconfliction>(tactical_srv_url);
    ros::ServiceClient possible_alternatives_client = n.serviceClient<gauss_msgs::NewDeconfliction>(alternatives_topic_url);
    ros::ServiceClient new_threats_client = n.serviceClient<gauss_msgs::NewThreats>(new_threats_srv_url);
    ros::Publisher visualization_pub = n.advertise<visualization_msgs::MarkerArray>(visualization_topic_url, 1);

    ROS_INFO("[Monitoring] Waiting for required services...");
    ros::service::waitForService(read_icao_srv_url, -1);
    ROS_INFO("[Monitoring] %s: ok", read_icao_srv_url);
    ros::service::waitForService(read_operation_srv_url, -1);
    ROS_INFO("[Monitoring] %s: ok", read_operation_srv_url);
    ros::service::waitForService(read_geofences_srv_url, -1);
    ROS_INFO("[Monitoring] %s: ok", read_geofences_srv_url);
    ros::service::waitForService(tactical_srv_url, -1);
    ROS_INFO("[Monitoring] %s: ok", tactical_srv_url);
    ros::service::waitForService(new_threats_srv_url, -1);
    ROS_INFO("[Monitoring] %s: ok", new_threats_srv_url);
    ros::Rate rate(1);  // [Hz]
    while (ros::ok()) {
        gauss_msgs::ReadIcao read_icao;
        if (icao_client.call(read_icao)) {
            // ROS_INFO("[Monitoring] Read icao addresses... ok");
            // std::cout << read_icao.response << '\n';
        } else {
            ROS_ERROR("[Monitoring] Failed to call service: [%s]", read_icao_srv_url);
            return 1;
        }

        gauss_msgs::ReadOperation read_operation;
        read_operation.request.uav_ids = read_icao.response.uav_id;
        if (operation_client.call(read_operation)) {
            // ROS_INFO("[Monitoring] Read operations... ok");
            // std::cout << read_operation.response << '\n';
        } else {
            ROS_ERROR("[Monitoring] Failed to call service: [%s]", read_operation_srv_url);
            return 1;
        }

        gauss_msgs::ReadGeofences read_geofences;
        read_geofences.request.geofences_ids = read_icao.response.geofence_id;
        if (geofences_client.call(read_geofences)) {
            // ROS_INFO("[Monitoring] Read geofences... ok");
            // std::cout << read_geofences.response << '\n';
        } else {
            ROS_ERROR("[Monitoring] Failed to call service: [%s]", read_geofences_srv_url);
            return 1;
        }

        std::map<std::string, int> icao_to_index_map;
        std::map<int, gauss_msgs::Operation> index_to_operation_map;
        std::map<int, gauss_msgs::Geofence> index_to_geofence_map;
        std::vector<gauss_msgs::WaypointList> estimated_trajectories;
        std::vector<double> operational_volumes;
        for (auto operation : read_operation.response.operation) {
            // std::cout << operation << '\n';
            if (operation.is_started) {
                icao_to_index_map[operation.icao_address] = estimated_trajectories.size();
                index_to_operation_map[estimated_trajectories.size()] = operation;
                estimated_trajectories.push_back(operation.estimated_trajectory);
                operational_volumes.push_back(operation.operational_volume);
            }
        }

        std::vector<GeofenceResult> geofence_results_list;
        for (auto geofence : read_geofences.response.geofences) {
            index_to_geofence_map[geofence.id] = geofence;
            // std::cout << geofence << '\n';
            // ROS_INFO("_________________________");
            // ROS_INFO("Checking geofence id [%d]", geofence.id);
            GeofenceResult current_result(geofence.id);
            current_result.geo_conflictive_trajectories = checkGeofenceConflict(estimated_trajectories, operational_volumes, geofence);
            if (current_result.geo_conflictive_trajectories.size() > 0) {
                geofence_results_list.push_back(current_result);
            }
        }
        // Visualize...
        visualization_msgs::MarkerArray marker_array;
        for (int i = 0; i < geofence_results_list.size(); i++) {
            auto geofence_result = geofence_results_list[i];
            // geofence_result.geofence_id
            for (int j = 0; j < geofence_result.geo_conflictive_trajectories.size(); j++) {
                auto trajectory = geofence_result.geo_conflictive_trajectories[j];
                // trajectory.trajectory_index;
                auto all_conflicts = trajectory.geofence_conflictive_segments;
                auto first_conflict = getFirstSetOfContiguousSegments(all_conflicts);
                auto conflicts = first_conflict;
                int segment_id = 1e6 * i + 1e3 * j;
                std_msgs::ColorRGBA segment_color;
                segment_color.a = 1.0;
                segment_color.r = 1.0;
                if (trajectory.closest_exit_wp.mandatory) {
                    // Means it is an intrusion!
                    Segment way_out(trajectory.closest_exit_wp, conflicts.back().point_B);
                    marker_array.markers.push_back(way_out.translateToMarker(segment_id+1e9, segment_color));
                    // continue;
                }
                // else:
                for (int k = 0; k < conflicts.size(); k++) {
                    segment_id += k;
                    segment_color.g = 0.5;
                    marker_array.markers.push_back(conflicts[k].translateToMarker(segment_id, segment_color));
                }
            }
        }

        std::vector<LossResult> loss_results_list;
        if (estimated_trajectories.size() >= 2) {
            for (int i = 0; i < estimated_trajectories.size() - 1; i++) {
                std::pair<gauss_msgs::WaypointList, gauss_msgs::WaypointList> trajectories;
                trajectories.first = estimated_trajectories[i];
                for (int j = i + 1; j < estimated_trajectories.size(); j++) {
                    // ROS_INFO("Checking trajectories: [%d, %d]", i, j);
                    trajectories.second = estimated_trajectories[j];
                    double s_threshold = std::max(safety_distance_sq, pow(operational_volumes[i] + operational_volumes[j], 2));
                    auto loss_conflictive_segments = checkTrajectoriesLoss(trajectories, s_threshold);
                    if (loss_conflictive_segments.size() > 0) {
                        LossResult loss_result(i, j);
                        loss_result.loss_conflictive_segments = loss_conflictive_segments;
                        loss_results_list.push_back(loss_result);
                    }
                }
            }
        }

        std::sort(loss_results_list.begin(), loss_results_list.end(), happensBefore);

        gauss_msgs::NewThreats threats_msg = manageResultList(loss_results_list, geofence_results_list, index_to_operation_map, index_to_geofence_map);
        if (threats_msg.request.threats.size() > 0) {
            std::string cout_threats;
            for (auto threat : threats_msg.request.threats) {
                cout_threats = cout_threats + " [" + std::to_string(threat.threat_id) + " " + std::to_string(threat.threat_type) + " |";
                for (auto uav_id : threat.uav_ids) cout_threats = cout_threats + " " + std::to_string(uav_id);
                cout_threats = cout_threats + "]";
            }
            ROS_INFO_STREAM("[Monitoring] Threats detected: [id type | uav] " + cout_threats);
            if (new_threats_client.call(threats_msg)) {
                // ROS_INFO("[Monitoring] Call tactical... ok");
            } else {
                ROS_ERROR("[Monitoring] Failed to call service: [%s]", tactical_srv_url);
                return 1;
            }
        }

        for (int i = 0; i < loss_results_list.size(); i++) {
            // std::cout << loss_results_list[i] << '\n';
            marker_array.markers.push_back(translateToMarker(loss_results_list[i]));
            auto extremes = calculateExtremes(loss_results_list[i]);
            marker_array.markers.push_back(translateToMarker(extremes.first, 10 * i));
            marker_array.markers.push_back(translateToMarker(extremes.second, 10 * i + 1));
        }
        visualization_pub.publish(marker_array);

        ros::spinOnce();
        rate.sleep();
    }

    // ros::spin();
    return 0;
}
