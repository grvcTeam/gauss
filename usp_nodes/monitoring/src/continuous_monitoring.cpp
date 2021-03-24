#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <gauss_msgs/Waypoint.h>
#include <gauss_msgs/ReadIcao.h>
#include <gauss_msgs/ReadOperation.h>

double clamp(double x, double min_x, double max_x) {
  if (min_x > max_x) {
    ROS_ERROR("min_x[%lf] > max_x[%lf], swapping!", min_x, max_x);
    std::swap(min_x, max_x);
  }
  return std::max(std::min(x, max_x), min_x);
}

double dot(const geometry_msgs::Vector3& u, const geometry_msgs::Vector3& v) {
  return u.x*v.x + u.y*v.y + u.z*v.z;
}

double length(const geometry_msgs::Vector3& u) {
  return sqrt(u.x*u.x + u.y*u.y + u.z*u.z);
}

// TODO: Use geometry_msgs/Point instead of Waypoint?
geometry_msgs::Vector3 vector_from_point_to_point(const gauss_msgs::Waypoint& a, const gauss_msgs::Waypoint& b) {
  geometry_msgs::Vector3 ab;
  ab.x = b.x - a.x;
  ab.y = b.y - a.y;
  ab.z = b.z - a.z;
  return ab;
}

// Signed Distance Fucntion from P to:
// a sphere centered in C with radius R
double sdSphere(const gauss_msgs::Waypoint& p, const gauss_msgs::Waypoint& c, double r) {
  return length(vector_from_point_to_point(c, p)) - r;
}

// Signed Distance Fucntion from P to:
// a segment (A,B) with some radius R
double sdSegment(const gauss_msgs::Waypoint& p, const gauss_msgs::Waypoint& a, const gauss_msgs::Waypoint& b, double r = 0) {
  geometry_msgs::Vector3 ap = vector_from_point_to_point(a, p);
  geometry_msgs::Vector3 ab = vector_from_point_to_point(a, b);
  double h = clamp(dot(ap, ab) / dot(ab, ab), 0.0, 1.0);
  geometry_msgs::Vector3 aux;
  aux.x = ap.x - ab.x * h;  // TODO: Use eigen?
  aux.y = ap.y - ab.y * h;
  aux.z = ap.z - ab.z * h;
  return length(aux) - r;
}

struct Segment {
  Segment() = default;
  Segment(gauss_msgs::Waypoint a, gauss_msgs::Waypoint b) {
    if (a == b) { ROS_WARN("a == b == [%lf, %lf, %lf, %lf]", a.x, a.y, a.z, a.stamp.toSec()); }
    point_a = a;
    point_b = b;
    t_a = a.stamp.toSec();
    t_b = b.stamp.toSec();
    if (t_a >= t_b) { ROS_WARN("t_a[%lf] >= t_b[%lf]", t_a, t_b); }
  }

  gauss_msgs::Waypoint point_at_time(double t) const {
    if (t < t_a) {
      ROS_WARN("t[%lf] < t_a[%lf]", t, t_a);
      return point_a;
    }
    if (t > t_b) {
      ROS_WARN("t[%lf] > t_b[%lf]", t, t_b);
      return point_b;
    }
    if (t_a == t_b) {
      ROS_WARN("t_a == t_b == %lf", t_a);
      return point_a;
    }
    if (isnan(t)) {
      ROS_WARN("t is NaN");
      return point_a;
    }

    double u = (t - t_a) / (t_b - t_a);
    gauss_msgs::Waypoint point;
    point.x = point_a.x * (1.0 - u) + point_b.x * u;
    point.y = point_a.y * (1.0 - u) + point_b.y * u;
    point.z = point_a.z * (1.0 - u) + point_b.z * u;
    point.stamp.fromSec(t);
    return point;
  }

  friend std::ostream& operator<< (std::ostream& out, const Segment& s);
  gauss_msgs::Waypoint point_a;
  gauss_msgs::Waypoint point_b;
  double t_a = 0;
  double t_b = 0;
};

std::ostream& operator<< (std::ostream& out, const Segment& s) {
  out << "[(" << s.point_a << "); (" << s.point_b << ")]";
  return out;
}

std::pair<geometry_msgs::Vector3, geometry_msgs::Vector3> delta(const Segment& first, const Segment& second) {
  geometry_msgs::Vector3 delta_a;
  delta_a.x = second.point_a.x - first.point_a.x;
  delta_a.y = second.point_a.y - first.point_a.y;
  delta_a.z = second.point_a.z - first.point_a.z;
  geometry_msgs::Vector3 delta_b;
  delta_b.x = second.point_b.x - first.point_b.x;
  delta_b.y = second.point_b.y - first.point_b.y;
  delta_b.z = second.point_b.z - first.point_b.z;
  return std::make_pair(delta_a, delta_b);
}

double sq_distance(const Segment& first, const Segment& second, double u) {
  if (u < 0) {
    ROS_WARN("u[%lf] < 0, clamping!", u);
    u = 0;
  }

  if (u > 1) {
    ROS_WARN("u[%lf] > 1, clamping!", u);
    u = 1;
  }

  auto d = delta(first, second);
  double delta_x = d.first.x * (1 - u) + d.second.x * u;
  double delta_y = d.first.y * (1 - u) + d.second.y * u;
  double delta_z = d.first.z * (1 - u) + d.second.z * u;
  return pow(delta_x, 2) + pow(delta_y, 2) + pow(delta_z, 2);
}

std::pair<double, double> quadratic_roots(double a, double b, double c) {
  if ((a == 0) && (b == 0) && (c == 0)) {
    ROS_WARN("a = b = c = 0, any number is a solution!");
    return std::make_pair(std::nan(""), std::nan(""));
  }
  if ((a == 0) && (b == 0) && (c != 0)) {
    ROS_WARN("a = b = 0, there is no solution!");
    return std::make_pair(std::nan(""), std::nan(""));
  }
  if ((a == 0) && (b != 0)) {
    ROS_WARN("a = 0, non quadratic!");
    return std::make_pair(-c/b, -c/b);
  }

  float d = b*b - 4*a*c;
  if (d < 0) {
    ROS_WARN("d = [%lf], complex solutions!", d);
    return std::make_pair(std::nan(""), std::nan(""));
  }
  double e = sqrt(d);
  return std::make_pair((-b - e)/(2*a), (-b + e)/(2*a));
}

struct CheckSegmentsLossResult {
  
  CheckSegmentsLossResult(const Segment& first, const Segment& second): first(first), second(second) {}

  friend std::ostream& operator<< (std::ostream& out, const CheckSegmentsLossResult& r);
  Segment first;
  Segment second;
  double t_min = std::nan("");
  double s_min = std::nan("");
  double t_crossing_0 = std::nan("");
  double t_crossing_1 = std::nan("");
  bool threshold_is_violated = false;
};

std::ostream& operator<< (std::ostream& out, const CheckSegmentsLossResult& r) {
    out << "first = " << r.first << '\n';
    out << "second = " << r.second << '\n';
    out << "t_min[s] = " << r.t_min << '\n';
    out << "s_min[m2] = " << r.s_min << '\n';
    out << "t_crossing_0[s] = " << r.t_crossing_0 << '\n';
    out << "t_crossing_1[s] = " << r.t_crossing_1 << '\n';
    out << "threshold_is_violated = " << r.threshold_is_violated << '\n';
    return out;
}

CheckSegmentsLossResult checkUnifiedSegmentsLoss(Segment first, Segment second, double s_threshold) {
  // print('checkUnifiedSegmentsLoss:')
  // print(first.point_a)
  // print(first.point_b)
  // print('___________')
  // print(second.point_a)
  // print(second.point_b)
  auto d = delta(first, second);
  // print(d)
  double C_x = pow(d.first.x, 2);
  double C_y = pow(d.first.y, 2);
  double C_z = pow(d.first.z, 2);
  double B_x = 2 * (d.first.x * d.second.x - C_x);
  double B_y = 2 * (d.first.y * d.second.y - C_y);
  double B_z = 2 * (d.first.z * d.second.z - C_z);
  double A_x = pow(d.second.x - d.first.x, 2);
  double A_y = pow(d.second.y - d.first.y, 2);
  double A_z = pow(d.second.z - d.first.z, 2);
  double A = A_x + A_y + A_z;
  double B = B_x + B_y + B_z;
  double C = C_x + C_y + C_z;

  double u_min, t_min, s_min;
  if (A == 0) {
    ROS_WARN("A = 0");
    if (B >= 0) {
      u_min = 0;
      t_min = first.t_a;
      s_min = C;
    } else {
      u_min = 1;
      t_min = first.t_b;
      s_min = B+C;
    }

  } else {  // A != 0
    double u_star = -0.5 * B / A;
    double t_star = first.t_a * (1-u_star) + first.t_b * u_star;
    // print(u_star)
    // print(t_star)
    // print(sq_distance(first, second, u_star))
    u_min = clamp(u_star, 0, 1);
    t_min = first.t_a * (1-u_min) + first.t_b * u_min;
    s_min = sq_distance(first, second, u_min);
    // print(u_min)
    // print(t_min)
    // print(s_min)
  }
  auto result = CheckSegmentsLossResult(first, second);
  result.t_min = t_min;
  result.s_min = s_min;

  if (s_min > s_threshold) {
    ROS_INFO("s_min[%lf] > s_threshold[%lf]", s_min, s_threshold);
    result.threshold_is_violated = false;
    return result;
  }

  auto u_bar = quadratic_roots(A, B, C - s_threshold);
  double t_bar_0 = first.t_a * (1-u_bar.first) + first.t_b * u_bar.first;
  double t_bar_1 = first.t_a * (1-u_bar.second) + first.t_b * u_bar.second;
  // print(u_bar)
  // print(t_bar_0, t_bar_1)
  double u_crossing_0 = clamp(u_bar.first, 0, 1);
  double u_crossing_1 = clamp(u_bar.second, 0, 1);
  double t_crossing_0 = first.t_a * (1-u_crossing_0) + first.t_b * u_crossing_0;
  double t_crossing_1 = first.t_a * (1-u_crossing_1) + first.t_b * u_crossing_1;
  // print(u_crossing_0, u_crossing_1)
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

CheckSegmentsLossResult checkSegmentsLoss(const std::pair<Segment, Segment>& segments, double s_threshold) {
  // print('checkSegmentsLoss:')
  // print(first.point_a)
  // print(first.point_b)
  // print('___________')
  // print(second.point_a)
  // print(second.point_b)
  double t_a1 = segments.first.t_a;
  double t_b1 = segments.first.t_b;
  double t_a2 = segments.second.t_a;
  double t_b2 = segments.second.t_b;
  double t_alpha = std::max(t_a1, t_a2);
  double t_beta  = std::min(t_b1, t_b2);
  if (t_alpha > t_beta) {
    ROS_INFO("t_alpha[%lf] > t_beta[%lf]", t_alpha, t_beta);
    return CheckSegmentsLossResult(segments.first, segments.second);
  }

  auto p_alpha1 = segments.first.point_at_time(t_alpha);
  auto p_beta1 = segments.first.point_at_time(t_beta);
  auto p_alpha2 = segments.second.point_at_time(t_alpha);
  auto p_beta2 = segments.second.point_at_time(t_beta);
  return checkUnifiedSegmentsLoss(Segment(p_alpha1, p_beta1), Segment(p_alpha2, p_beta2), s_threshold);
}

std::vector<CheckSegmentsLossResult> checkTrajectoriesLoss(const std::pair<gauss_msgs::WaypointList, gauss_msgs::WaypointList>& trajectories, double s_threshold) {
    std::vector<CheckSegmentsLossResult> segment_loss_results;
    if (trajectories.first.waypoints.size() < 2) {
        // TODO: Warn and push the same point twice?
        ROS_ERROR("[Monitoring]: trajectory must contain at least 2 points, [%ld] found in first argument", trajectories.first.waypoints.size());
        return segment_loss_results;
    }
    if (trajectories.second.waypoints.size() < 2) {
        // TODO: Warn and push the same point twice?
        ROS_ERROR("[Monitoring]: trajectory must contain at least 2 points, [%ld] found in second argument", trajectories.second.waypoints.size());
        return segment_loss_results;
    }

    for (int i = 0; i < trajectories.first.waypoints.size() - 1; i++) {
        std::pair<Segment, Segment> segments;
        printf("First segment, i = %d\n", i);
        segments.first = Segment(trajectories.first.waypoints[i], trajectories.first.waypoints[i+1]);
        // std::cout << segments.first.point_a << "_____________\n" << segments.first.point_b << '\n';
        for (int j = 0; j < trajectories.second.waypoints.size() - 1; j++) {
            printf("Second segment, j = %d\n", j);
            segments.second = Segment(trajectories.second.waypoints[j], trajectories.second.waypoints[j+1]);
            // std::cout << segments.second.point_a << "_____________\n" << segments.second.point_b << '\n';
            auto loss_check = checkSegmentsLoss(segments, s_threshold);
            if (loss_check.threshold_is_violated) {
                ROS_ERROR("Loss of separation!");
                std::cout << loss_check << '\n';
                segment_loss_results.push_back(loss_check);
            }
        }
    }
    return segment_loss_results;
}

struct LossResult {
  LossResult(const int i, const int j): first_trajectory_index(i), second_trajectory_index(j) {}

  friend std::ostream& operator<< (std::ostream& out, const LossResult& r);
  int first_trajectory_index;
  int second_trajectory_index;
  std::vector<CheckSegmentsLossResult> segments_loss_results;
};

geometry_msgs::Point translateToPoint(const gauss_msgs::Waypoint& wp) {
  geometry_msgs::Point p;
  p.x = wp.x;
  p.y = wp.y;
  p.z = wp.z;
  return p;
}

visualization_msgs::Marker translateToMarker(const LossResult& result) {
  visualization_msgs::Marker marker;
  marker.header.stamp = ros::Time::now();  // std_msgs/Header header
  marker.header.frame_id = "map";  // TODO: other?
  marker.ns = "loss_" + std::to_string(result.first_trajectory_index) + "_" + std::to_string(result.second_trajectory_index);
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1;
  marker.scale.x = 1.0;
  marker.color.r = 1.0;  // TODO: color?
  marker.color.a = 1.0;
  marker.lifetime = ros::Duration();  // duration lifetime
  for (auto segments_loss: result.segments_loss_results) {
    marker.points.push_back(translateToPoint(segments_loss.first.point_a));
    marker.points.push_back(translateToPoint(segments_loss.first.point_b));
    marker.points.push_back(translateToPoint(segments_loss.second.point_a));
    marker.points.push_back(translateToPoint(segments_loss.second.point_b));
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

std::ostream& operator<< (std::ostream& out, const LossResult& r) {
    out << "first_trajectory_index = " << r.first_trajectory_index << '\n';
    out << "second_trajectory_index = " << r.second_trajectory_index << '\n';
    out << "segments_loss_results = [" << r.second_trajectory_index << '\n';
    for (int i = 0; i < r.segments_loss_results.size(); i++) {
      out << r.segments_loss_results[i] << '\n';
    }
    out << "]\n";
    return out;
}

bool happensBefore(const LossResult& a, const LossResult& b) {
  if (a.segments_loss_results.size() < 1) {
    ROS_ERROR("a.segments_loss_results.size() < 1");
    return false;
  }
  if (b.segments_loss_results.size() < 1) {
    ROS_ERROR("b.segments_loss_results.size() < 1");
    return true;
  }

  return a.segments_loss_results[0].t_crossing_0 < b.segments_loss_results[0].t_crossing_0;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "continuous_monitoring");


    ros::NodeHandle n;
    // ros::NodeHandle np("~");
    ROS_INFO("[Monitoring] Started monitoring node!");

    auto read_icao_srv_url = "/gauss/read_icao";
    auto read_operation_srv_url = "/gauss/read_operation";
    auto visualization_topic_url = "/gauss/visualize_monitoring";

    ros::ServiceClient icao_client = n.serviceClient<gauss_msgs::ReadIcao>(read_icao_srv_url);
    ros::ServiceClient operation_client = n.serviceClient<gauss_msgs::ReadOperation>(read_operation_srv_url);
    ros::Publisher visualization_pub = n.advertise<visualization_msgs::MarkerArray>(visualization_topic_url, 1);

    ROS_INFO("[Monitoring] Waiting for required services...");
    ros::service::waitForService(read_icao_srv_url, -1);
    ROS_INFO("[Monitoring] %s: ok", read_icao_srv_url);
    ros::service::waitForService(read_operation_srv_url, -1);
    ROS_INFO("[Monitoring] %s: ok", read_operation_srv_url);

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

        std::map<std::string, int> icao_to_index_map;
        std::vector<gauss_msgs::WaypointList> estimated_trajectories;
        std::vector<double> operational_volumes;
        for (auto operation: read_operation.response.operation) {
            // std::cout << operation << '\n';
            icao_to_index_map[operation.icao_address] = estimated_trajectories.size();
            estimated_trajectories.push_back(operation.estimated_trajectory);
            operational_volumes.push_back(operation.operational_volume);
        }

        auto trajectories_count = estimated_trajectories.size();
        if (trajectories_count < 2) { continue; }

        std::vector<LossResult> loss_results_list;
        for (int i = 0; i < trajectories_count - 1; i++) {
            std::pair<gauss_msgs::WaypointList, gauss_msgs::WaypointList> trajectories;
            trajectories.first = estimated_trajectories[i];
            for (int j = i + 1; j < trajectories_count; j++) {
                printf("[%d, %d]\n", i, j);
                trajectories.second = estimated_trajectories[j];
                //double s_threshold = 109000;  // TODO: param (330m)^2
                double s_threshold = pow(operational_volumes[i] + operational_volumes[j], 2);
                auto segments_loss_results = checkTrajectoriesLoss(trajectories, s_threshold);
                if (segments_loss_results.size() > 0) {
                  LossResult loss_result(i, j);
                  loss_result.segments_loss_results = segments_loss_results;
                  loss_results_list.push_back(loss_result);
                }
            }
        }

        std::sort(loss_results_list.begin(), loss_results_list.end(), happensBefore);
        visualization_msgs::MarkerArray marker_array;
        for (int i = 0; i < loss_results_list.size(); i++) {
          //std::cout << loss_results_list[i] << '\n';
          marker_array.markers.push_back(translateToMarker(loss_results_list[i]));
        }
        visualization_pub.publish(marker_array);

        ros::spinOnce();
        rate.sleep();
    }

    ros::spin();
    return 0;
}
