#include <gauss_light_sim/ChangeParam.h>
#include <gauss_light_sim/ChangeFlightPlan.h>
#include <gauss_msgs/ReadIcao.h>
#include <gauss_msgs/ReadOperation.h>
#include <gauss_msgs_mqtt/RPAStateInfo.h>
#include <gauss_msgs_mqtt/RPSChangeFlightStatus.h>
#include <gauss_msgs_mqtt/UTMAlternativeFlightPlan.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Eigen>
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <limits>
#include <random>

#define ARENOSILLO_LATITUDE 37.094784
#define ARENOSILLO_LONGITUDE -6.735478
#define ARENOSILLO_ELLIPSOIDAL_HEIGHT 0.0 // TODO: MEASURE IT 

/*
gauss_msgs::Waypoint interpolate(const gauss_msgs::Waypoint& from, const gauss_msgs::Waypoint& to, const ros::Time& t) {
    // Make sure that from.stamp < to.stamp
    if (from.stamp > to.stamp) {
        ROS_ERROR("[Sim] from.stamp > to.stamp (%lf > %lf)", from.stamp.toSec(), to.stamp.toSec());
        return from;
    } else if (from.stamp == to.stamp) {
        ROS_WARN("[Sim] from.stamp == to.stamp (%lf)", from.stamp.toSec());
        return to;
    }

    // Make sure that from.stamp < t < to.stamp
    if (t <= from.stamp) { return from; } else if (t >= to.stamp) { return to; }

    // Now safely interpolate in space-time :)
    double delta_t = to.stamp.toSec() - from.stamp.toSec();
    double delta_x = to.x - from.x;
    double delta_y = to.y - from.y;
    double delta_z = to.z - from.z;
    gauss_msgs::Waypoint interpolated;
    auto elapsed_t = t.toSec() - from.stamp.toSec();
    interpolated.x = from.x + (delta_x / delta_t) * elapsed_t;
    interpolated.y = from.y + (delta_y / delta_t) * elapsed_t;
    interpolated.z = from.z + (delta_z / delta_t) * elapsed_t;
    interpolated.stamp = t;

    return interpolated;
}
*/

inline double distanceBetweenWaypoints(const gauss_msgs::Waypoint &waypoint_a, const gauss_msgs::Waypoint &waypoint_b)
{
    return sqrt(pow(waypoint_a.x - waypoint_b.x,2) + pow(waypoint_a.y - waypoint_b.y, 2) + pow(waypoint_a.z - waypoint_b.z, 2));
}

void findSegmentWaypointsIndices(const gauss_msgs::Waypoint &current_position, const gauss_msgs::WaypointList &flight_plan, int &a_index, int &b_index, double &distance_to_segment, double &distance_to_point_a, double &distance_to_point_b)
{
    int first_index = 0;
    int second_index = 0;
    distance_to_segment = std::numeric_limits<double>::max();
    double distance_to_waypoint_a = std::numeric_limits<double>::max();
    bool flag_distance_to_waypoint_a = false;
    bool flag_distance_to_waypoint_b = false;

    for (int i=0; i < flight_plan.waypoints.size()-1; i++)
    {
        gauss_msgs::Waypoint waypoint_a = flight_plan.waypoints[i];
        gauss_msgs::Waypoint waypoint_b = flight_plan.waypoints[i+1];
        Eigen::Vector3d vector_u;
        flag_distance_to_waypoint_a = false;
        flag_distance_to_waypoint_b = false;
        vector_u.x() = waypoint_b.x - waypoint_a.x;
        vector_u.y() = waypoint_b.y - waypoint_a.y;
        vector_u.z() = waypoint_b.z - waypoint_a.z;
        /*
            Solve this equation
            |alpha_1|     | 1    0    0    u_1|^-1   | a_1 - x_0|
            |alpha_2|     | 0    1    0    u_2|      | a_2 - y_0|
            |alpha_3|  =  | 0    0    1    u_3|   *  | a_3 - z_0|
            |    t  |     | u_1  u_2  u_3  0  |      |     0    |

            b = A^-1*x
        */
        Eigen::Matrix4d A;
        Eigen::Vector4d b;
        Eigen::Vector4d x;

        A = Eigen::Matrix4d::Identity();
        A(3,3) = 0;
        A(0,3) = vector_u.x();
        A(1,3) = vector_u.y();
        A(2,3) = vector_u.z();
        A(3,0) = vector_u.x();
        A(3,1) = vector_u.y();
        A(3,2) = vector_u.z();

        x.x() = current_position.x - waypoint_a.x;
        x.y() = current_position.y - waypoint_a.y;
        x.z() = current_position.z - waypoint_a.z;
        x.w() = 0.0;

        b = A.inverse()*x;

        double distance;
        if (b[3] >= 0 && b[3] <= 1)
        {
            distance = sqrt(b[0]*b[0] + b[1]*b[1] + b[2]*b[2]);
        }
        else
        {
            double distance_to_waypoint_a = distanceBetweenWaypoints(current_position, waypoint_a);
            double distance_to_waypoint_b = distanceBetweenWaypoints(current_position, waypoint_b);

            if (distance_to_waypoint_a < distance_to_waypoint_b)
            {
                distance = distance_to_waypoint_a;
                flag_distance_to_waypoint_a = true;
            }
            else
            {
                distance = distance_to_waypoint_b;
                flag_distance_to_waypoint_b = true;
            }
        }
        // Then check the distance to the line that contains the segment
        if (distance <= distance_to_segment)
        {
            distance_to_segment = distance;
            if(flag_distance_to_waypoint_a)
            {
                first_index = i;
                second_index = i;
            }
            else if(flag_distance_to_waypoint_b)
            {
                first_index = i+1;
                second_index = i+1;                
            }
            else
            {
                first_index = i;
                second_index = i+1;
            }
        }
    }

    a_index = first_index;
    b_index = second_index;

    gauss_msgs::Waypoint waypoint_a = flight_plan.waypoints[first_index];
    gauss_msgs::Waypoint waypoint_b = flight_plan.waypoints[second_index];

    distance_to_point_a = distanceBetweenWaypoints(waypoint_a, current_position);
    distance_to_point_b = distanceBetweenWaypoints(waypoint_b, current_position);
}

// This is a trickier version of the previous function, as time-base issues must be addressed (TODO!)
gauss_msgs::Waypoint interpolate(const gauss_msgs::Waypoint &from, const gauss_msgs::Waypoint &to, const ros::Duration &t, double* yaw = nullptr) {
    // Make sure that from.stamp < to.stamp
    if (from.stamp > to.stamp) {
        ROS_ERROR("[Sim] from.stamp > to.stamp (%lf > %lf)", from.stamp.toSec(), to.stamp.toSec());
        return from;
    } else if (from.stamp == to.stamp) {
        ROS_WARN("[Sim] from.stamp == to.stamp (%lf)", from.stamp.toSec());
        return to;
    }

    // Make sure that from.stamp < dt < to.stamp (now we have to convert it to seconds before comparing)
    if (t.toSec() <= from.stamp.toSec()) {
        return from;
    } else if (t.toSec() >= to.stamp.toSec()) {
        return to;
    }

    // Now safely interpolate in space-time :)
    double delta_t = to.stamp.toSec() - from.stamp.toSec();
    double delta_x = to.x - from.x;
    double delta_y = to.y - from.y;
    double delta_z = to.z - from.z;
    gauss_msgs::Waypoint interpolated;
    auto elapsed_t = t.toSec() - from.stamp.toSec();
    interpolated.x = from.x + (delta_x / delta_t) * elapsed_t;
    interpolated.y = from.y + (delta_y / delta_t) * elapsed_t;
    interpolated.z = from.z + (delta_z / delta_t) * elapsed_t;
    // interpolated.stamp = t;  // Cannot do this anymore

    if (yaw != nullptr) { *yaw = atan2(delta_y, delta_x); }

    return interpolated;
}

// TODO: Much code from interpolate is reused and repeated, see how to merge?
float calculateMeanSpeed(const gauss_msgs::Waypoint &from, const gauss_msgs::Waypoint &to) {
    // Make sure that from.stamp < to.stamp
    if (from.stamp > to.stamp) {
        ROS_ERROR("[Sim] from.stamp > to.stamp (%lf > %lf)", from.stamp.toSec(), to.stamp.toSec());
        return 0.0;
    } else if (from.stamp == to.stamp) {
        // ROS_WARN("[Sim] from.stamp == to.stamp (%lf)", from.stamp.toSec());
        return 0.0;
    }

    double delta_t = to.stamp.toSec() - from.stamp.toSec();
    double delta_x = to.x - from.x;
    double delta_y = to.y - from.y;
    double delta_z = to.z - from.z;
    double distance = sqrt(delta_x * delta_x + delta_y * delta_y + delta_z * delta_z);

    return distance / delta_t;
}

void copyTarget(const gauss_msgs::Waypoint& target_position, double target_yaw, geometry_msgs::Transform* current_transform, float* current_yaw) {
    tf2::Quaternion q;
    q.setRPY(0, 0, target_yaw);  // Only yaw!
    current_transform->rotation.x = q.x();
    current_transform->rotation.y = q.y();
    current_transform->rotation.z = q.z();
    current_transform->rotation.w = q.w();
    current_transform->translation.x = target_position.x;
    current_transform->translation.y = target_position.y;
    current_transform->translation.z = target_position.z;
    *current_yaw = target_yaw;
}

void smoothTarget(const gauss_msgs::Waypoint& target_position, double target_yaw, geometry_msgs::Transform* current_transform, float* current_yaw) {
    double alpha = 0.5;  // TODO: as param?
    double beta = 1 - alpha;
    tf2::Quaternion q_target;
    q_target.setRPY(0, 0, target_yaw);
    tf2::Quaternion q_current(current_transform->rotation.x, current_transform->rotation.y, current_transform->rotation.z, current_transform->rotation.w);
    auto q_smooth = q_current.slerp(q_target, beta);
    current_transform->rotation.x = q_smooth.x();
    current_transform->rotation.y = q_smooth.y();
    current_transform->rotation.z = q_smooth.z();
    current_transform->rotation.w = q_smooth.w();
    current_transform->translation.x = alpha*current_transform->translation.x + beta*target_position.x;
    current_transform->translation.y = alpha*current_transform->translation.y + beta*target_position.y;
    current_transform->translation.z = alpha*current_transform->translation.z + beta*target_position.z;
    *current_yaw = 2.0 * atan2(current_transform->rotation.z, current_transform->rotation.w);  // Only yaw!
}

void simTarget(const gauss_msgs::Waypoint& target_position, double target_yaw, geometry_msgs::Transform* current_transform, float* current_yaw) {
    static std::default_random_engine generator;
    static std::normal_distribution<double> xyz_distribution(0, 1.00);  // N(mean, stddev) TODO: as params?
    static std::normal_distribution<double> yaw_distribution(0, 0.01);  // N(mean, stddev) TODO: as params?

    // Add noise to current data...
    double tf_yaw_with_noise = 2.0 * atan2(current_transform->rotation.z, current_transform->rotation.w);
    tf_yaw_with_noise += yaw_distribution(generator);
    tf2::Quaternion q_yaw_with_noise;  // Only yaw!
    q_yaw_with_noise.setRPY(0, 0, tf_yaw_with_noise);
    current_transform->rotation.x = q_yaw_with_noise.x();
    current_transform->rotation.y = q_yaw_with_noise.y();
    current_transform->rotation.z = q_yaw_with_noise.z();
    current_transform->rotation.w = q_yaw_with_noise.w();
    *current_yaw = tf_yaw_with_noise;  // Not needed...

    current_transform->translation.x += xyz_distribution(generator);
    current_transform->translation.y += xyz_distribution(generator);
    current_transform->translation.z += xyz_distribution(generator);

    smoothTarget(target_position, target_yaw, current_transform, current_yaw);

/*
    // ...or add noise to target data
    auto new_target_position = target_position;
    new_target_position.x += xyz_distribution(generator);
    new_target_position.y += xyz_distribution(generator);
    new_target_position.z += xyz_distribution(generator);

    auto new_target_yaw = target_yaw + yaw_distribution(generator);

    smoothTarget(new_target_position, new_target_yaw, current_transform, current_yaw);
*/
}

class RPAStateInfoWrapper {
   public:
    // RPAStateInfoWrapper wraps (has-a) RPAStateInfo named data
    gauss_msgs_mqtt::RPAStateInfo data;
    geometry_msgs::TransformStamped tf;

    // Each function updates a subset of fields in data:
    // uint32 icao                 - update
    // float64 latitude            -------- updatePhysics
    // float64 longitude           -------- updatePhysics
    // float32 altitude            -------- updatePhysics
    // float32 yaw                 -------- updatePhysics
    // float32 pitch               -------- updatePhysics
    // float32 roll                -------- updatePhysics
    // float32 groundspeed         -------- updatePhysics
    // float32 covariance_h        ---------------------- applyChange
    // float32 covariance_v        ---------------------- applyChange
    // float32 hpl                 ---------------------- applyChange
    // float32 hal                 ---------------------- applyChange
    // float32 vpl                 ---------------------- applyChange
    // float32 val                 ---------------------- applyChange
    // string solution_mode        ---------------------- applyChange
    // uint64 timestamp            - update
    // float32 jamming             ---------------------- applyChange
    // float32 spoofing            ---------------------- applyChange
    // bool anomalous_clock_drift  ---------------------- applyChange
    // bool anomalous_pos_drift    ---------------------- applyChange
    // float32 signal_noise_ratio  ---------------------- applyChange
    // float32 received_power      ---------------------- applyChange

    bool update(const ros::Duration &elapsed, const gauss_msgs::Operation &operation, const std::map<std::string, double> &icao_to_speed_map, const double &sim_rate) {
        // TODO: Solve icao string vs uint32 issue
        data.icao = std::stoi(operation.icao_address);

        // TODO: Solve timing issues
        data.timestamp = ros::Time::now().toNSec() / 1000000;

        auto it = change_param_request_list.begin();
        while (it != change_param_request_list.end()) {
            auto change = *it;
            if (elapsed.toSec() > change.stamp.toSec()) {
                try {
                    YAML::Node yaml_change = YAML::Load(change.yaml);
                    auto icao = operation.icao_address.c_str();
                    if (!applyChange(yaml_change)) {
                        ROS_ERROR("[Sim] RPA[%s] apply: %s", icao, change.yaml.c_str());
                    } else {
                        ROS_INFO("[Sim] RPA[%s] apply: %s", icao, change.yaml.c_str());
                    }

                } catch (const std::runtime_error &error) {
                    auto icao = operation.icao_address.c_str();
                    ROS_ERROR("[Sim] RPA[%s] could not apply [%s]: %s", icao, change.yaml.c_str(), error.what());
                }
                // And erase this change, keepin valid it
                it = change_param_request_list.erase(it);
            } else {
                ++it;
            }
        }

        if (icao_to_speed_map.find(operation.icao_address)->second != 0.0) { // If cruising speed is 0, use updatePhysics
            return updatePhysicsCruisingSpeed(elapsed, operation.flight_plan, icao_to_speed_map.find(operation.icao_address)->second, sim_rate);
        } else {
            return updatePhysics(elapsed, operation.flight_plan);
        }
    }

    void setProjection(const GeographicLib::LocalCartesian &projection) { proj = projection; }

    void initPhysics(const gauss_msgs::WaypointList &flight_plan) {
        double latitude, longitude, altitude;
        if (flight_plan.waypoints.size() == 0) {
            ROS_ERROR("[Sim] Flight plan is empty");
            return;
        } else {
            // At least one point, get position
            tf.transform.translation.x = flight_plan.waypoints[0].x;
            tf.transform.translation.y = flight_plan.waypoints[0].y;
            tf.transform.translation.z = flight_plan.waypoints[0].z;
            proj.Reverse(tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z, latitude, longitude, altitude);
            data.latitude = latitude;
            data.longitude = longitude;
            data.altitude = altitude;
            data.groundspeed = 0;
        }

        if(flight_plan.waypoints.size() >= 2) {
            // At least two points, get also orientation!
            gauss_msgs::Waypoint prev, next;
            prev = flight_plan.waypoints[0];
            next = flight_plan.waypoints[1];
            double delta_x = next.x - prev.x;
            double delta_y = next.y - prev.y;
            double yaw = atan2(delta_y, delta_x);
            tf2::Quaternion q;
            q.setRPY(0, 0, yaw);
            tf.transform.rotation.x = q.x();
            tf.transform.rotation.y = q.y();
            tf.transform.rotation.z = q.z();
            tf.transform.rotation.w = q.w();
            data.yaw = yaw;
        }
    }

    bool updatePhysics(const ros::Duration &elapsed, const gauss_msgs::WaypointList &flight_plan) {
        // This function returns true if 'physics' is running
        bool running = false;  // otherwise, it returns false
        double latitude, longitude, altitude;

        // Update common tf data
        tf.header.stamp = ros::Time::now();
        tf.header.frame_id = "map";
        tf.child_frame_id = std::to_string(data.icao);

        if (flight_plan.waypoints.size() == 0) {
            ROS_ERROR("[Sim] Flight plan is empty");
            return false;

        } else if (flight_plan.waypoints.size() == 1) {
            tf.transform.translation.x = flight_plan.waypoints[0].x;
            tf.transform.translation.y = flight_plan.waypoints[0].y;
            tf.transform.translation.z = flight_plan.waypoints[0].z;
            proj.Reverse(tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z, latitude, longitude, altitude);
            data.latitude = latitude;
            data.longitude = longitude;
            data.altitude = altitude;
            data.groundspeed = 0;
            return false;
        }

        // flight_plan.waypoints.size() >= 2
        gauss_msgs::Waypoint prev, next;
        prev = flight_plan.waypoints[0];
        auto it = flight_plan.waypoints.begin();
        while (it != flight_plan.waypoints.end()) {
            next = *it;
            if (next.stamp.toSec() > elapsed.toSec()) {
                break;
            } else {
                prev = next;
                ++it;
            }
        }

        if (prev.stamp == next.stamp) {
            // Last waypoint is reached
            running = false;
            tf.transform.translation.x = next.x;
            tf.transform.translation.y = next.y;
            tf.transform.translation.z = next.z;
        } else {
            running = true;
            double target_yaw;
            gauss_msgs::Waypoint target_point;
            target_point = interpolate(prev, next, elapsed, &target_yaw);
            // TODO: Realism level as a parameter!
            //copyTarget(target_point, target_yaw, &tf.transform, &data.yaw);
            smoothTarget(target_point, target_yaw, &tf.transform, &data.yaw);
            //simTarget(target_point, target_yaw, &tf.transform, &data.yaw);  // TODO: Merge copy, smooth and sim into one function?
        }

        // Cartesian to geographic conversion
        proj.Reverse(tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z, latitude, longitude, altitude);
        data.latitude = latitude;
        data.longitude = longitude;
        data.altitude = altitude;
        data.groundspeed = calculateMeanSpeed(prev, next);
        return running;
    }

    bool updatePhysicsCruisingSpeed(const ros::Duration &elapsed, const gauss_msgs::WaypointList &flight_plan, const double &cruising_speed, const double sim_rate) {
        // This function returns true if 'physics' is running
        bool running = false;  // otherwise, it returns false
        double latitude, longitude, altitude;

        // Update common tf data
        tf.header.stamp = ros::Time::now();
        tf.header.frame_id = "map";
        tf.child_frame_id = std::to_string(data.icao);

        if (flight_plan.waypoints.size() == 0) {
            ROS_ERROR("[Sim] Flight plan is empty");
            return false;

        } else if (flight_plan.waypoints.size() == 1) {
            tf.transform.translation.x = flight_plan.waypoints[0].x;
            tf.transform.translation.y = flight_plan.waypoints[0].y;
            tf.transform.translation.z = flight_plan.waypoints[0].z;
            proj.Reverse(tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z, latitude, longitude, altitude);
            data.latitude = latitude;
            data.longitude = longitude;
            data.altitude = altitude;
            data.groundspeed = 0;
            return false;
        }
        // At first step, curret position should be the first waypoint of the flight plan
        static gauss_msgs::Waypoint current_position = flight_plan.waypoints[0];

        int a_index, b_index;
        double dist_to_segment, dist_to_a, dist_to_b;
        findSegmentWaypointsIndices(current_position, flight_plan, a_index, b_index, dist_to_segment, dist_to_a, dist_to_b);
        Eigen::Vector3f p_a, p_b, unit_vec;
        p_a = Eigen::Vector3f(current_position.x, current_position.y, current_position.z);
        p_b = Eigen::Vector3f(flight_plan.waypoints.at(b_index).x, flight_plan.waypoints.at(b_index).y, flight_plan.waypoints.at(b_index).z);
        unit_vec = (p_b - p_a) / (p_b - p_a).norm();

        gauss_msgs::Waypoint target_point;
        double target_yaw, remaining_dist_to_travel;
        // v = m / s -> m = v * s -> s = 1 / hz
        double dist_to_move = (1 /  sim_rate) * cruising_speed;

        if (dist_to_move <= dist_to_b) {
            // If there is enough distance to next waypoint, apply all the distance
            running = true;
            unit_vec *= dist_to_move;
            target_point.x = current_position.x + unit_vec[0];
            target_point.y = current_position.y + unit_vec[1];
            target_point.z = current_position.z + unit_vec[2];
            target_yaw = atan2(flight_plan.waypoints[b_index].y - flight_plan.waypoints[a_index].y, 
                               flight_plan.waypoints[b_index].x - flight_plan.waypoints[a_index].x);
        } else if (dist_to_move > dist_to_b && b_index != flight_plan.waypoints.size() - 1) {
            // If there is NOT enough distance to next waypoint, apply the remaining distance to the next segment
            // TODO: The remaining distance can be bigger than the next segment
            running = true;
            remaining_dist_to_travel = dist_to_move - dist_to_b; 
            p_a = Eigen::Vector3f(flight_plan.waypoints.at(a_index + 1).x, flight_plan.waypoints.at(a_index + 1).y, flight_plan.waypoints.at(a_index + 1).z);
            p_b = Eigen::Vector3f(flight_plan.waypoints.at(b_index + 1).x, flight_plan.waypoints.at(b_index + 1).y, flight_plan.waypoints.at(b_index + 1).z);
            unit_vec = (p_b - p_a) / (p_b - p_a).norm();
            unit_vec *= remaining_dist_to_travel;
            target_point.x = flight_plan.waypoints[a_index + 1].x + unit_vec[0];
            target_point.y = flight_plan.waypoints[a_index + 1].y + unit_vec[1];
            target_point.z = flight_plan.waypoints[a_index + 1].z + unit_vec[2];
            target_yaw = atan2(flight_plan.waypoints[b_index + 1].y - flight_plan.waypoints[a_index + 1].y, 
                               flight_plan.waypoints[b_index + 1].x - flight_plan.waypoints[a_index + 1].x);

        } else {
            // If there is NOT enough distance to next waypoint and it is the last one
            running = false;
            unit_vec *= dist_to_b;
            target_point.x = current_position.x + unit_vec[0];
            target_point.y = current_position.y + unit_vec[1];
            target_point.z = current_position.z + unit_vec[2];
            target_yaw = atan2(flight_plan.waypoints[b_index].y - flight_plan.waypoints[a_index].y, 
                               flight_plan.waypoints[b_index].x - flight_plan.waypoints[a_index].x);
        }
        // Convert to TF
        copyTarget(target_point, target_yaw, &tf.transform, &data.yaw);
        // Store current position for next step
        current_position.x = tf.transform.translation.x;
        current_position.y = tf.transform.translation.y;
        current_position.z = tf.transform.translation.z;
        // Cartesian to geographic conversion
        proj.Reverse(tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z, latitude, longitude, altitude);
        data.latitude = latitude;
        data.longitude = longitude;
        data.altitude = altitude;
        data.groundspeed = cruising_speed;
        return running;
    }

    bool applyChange(const YAML::Node &yaml_change) {
        // This function returns true if change is correctly applied
        if (!yaml_change.IsMap()) {
            ROS_ERROR("[Sim] A map {name: param_name, type: float|bool|string, value: param_value} is expected!");
            return false;

        } else if (!yaml_change["name"]) {
            ROS_ERROR("[Sim] Key [name] not found!");
            return false;

        } else if (!yaml_change["type"]) {
            ROS_ERROR("[Sim] Key [type] not found!");
            return false;

        } else if (!yaml_change["value"]) {
            ROS_ERROR("[Sim] Key [value] not found!");
            return false;
        }

        std::string param_name = yaml_change["name"].as<std::string>();
        std::string param_type = yaml_change["type"].as<std::string>();

        if ((param_name == "covariance_h") && (param_type == "float")) {
            data.covariance_h = yaml_change["value"].as<float>();

        } else if ((param_name == "covariance_v") && (param_type == "float")) {
            data.covariance_v = yaml_change["value"].as<float>();

        } else if ((param_name == "hpl") && (param_type == "float")) {
            data.hpl = yaml_change["value"].as<float>();

        } else if ((param_name == "hal") && (param_type == "float")) {
            data.hal = yaml_change["value"].as<float>();

        } else if ((param_name == "vpl") && (param_type == "float")) {
            data.vpl = yaml_change["value"].as<float>();

        } else if ((param_name == "val") && (param_type == "float")) {
            data.val = yaml_change["value"].as<float>();

        } else if ((param_name == "solution_mode") && (param_type == "int")) {
            data.solution_mode = yaml_change["value"].as<int>();

        } else if ((param_name == "jamming") && (param_type == "float")) {
            data.jamming = yaml_change["value"].as<float>();

        } else if ((param_name == "spoofing") && (param_type == "float")) {
            data.spoofing = yaml_change["value"].as<float>();

        } else if ((param_name == "anomalous_clock_drift") && (param_type == "bool")) {
            data.anomalous_clock_drift = yaml_change["value"].as<bool>();

        } else if ((param_name == "anomalous_pos_drift") && (param_type == "bool")) {
            data.anomalous_pos_drift = yaml_change["value"].as<bool>();

        } else if ((param_name == "signal_noise_ratio") && (param_type == "float")) {
            data.signal_noise_ratio = yaml_change["value"].as<float>();

        } else if ((param_name == "received_power") && (param_type == "float")) {
            data.received_power = yaml_change["value"].as<float>();

        } else {
            ROS_ERROR("[Sim] Unexpected param [%s] of type [%s]", param_name.c_str(), param_type.c_str());
            return false;
        }

        return true;
    }

    void addChangeParamRequest(const gauss_light_sim::ChangeParam::Request &req) {
        change_param_request_list.push_back(req);
    }

   protected:
    std::vector<gauss_light_sim::ChangeParam::Request> change_param_request_list;
    GeographicLib::LocalCartesian proj;
};

class LightSim {
   public:
    LightSim(ros::NodeHandle &n, const std::vector<std::string> &icao_addresses, const GeographicLib::LocalCartesian &projection) : 
                                                                                                          n(n), proj_(projection){
        for (auto icao : icao_addresses) {
            icao_to_is_started_map[icao] = false;
            icao_to_time_zero_map[icao] = ros::Time(0);
            icao_to_operation_map[icao] = gauss_msgs::Operation();
            icao_to_state_info_map[icao] = RPAStateInfoWrapper();
            icao_to_state_info_map[icao].setProjection(projection);
            ROS_INFO("[Sim] Ready to simulate icao [%s]", icao.c_str());
        }
        change_param_service = n.advertiseService("gauss_light_sim/change_param", &LightSim::changeParamCallback, this);
        change_flight_plan_service = n.advertiseService("gauss_light_sim/change_flight_plan", &LightSim::changeFlightPlanCallback, this);
        status_sub = n.subscribe("/gauss/flight", 10, &LightSim::flightStatusCallback, this);
        status_pub = n.advertise<gauss_msgs_mqtt::RPSChangeFlightStatus>("/gauss/flight", 10);
        rpa_state_info_pub = n.advertise<gauss_msgs_mqtt::RPAStateInfo>("/gauss/rpastateinfo", 10);
        n.getParam("sim_rate", sim_rate);
    }

    void start() {
        ROS_INFO("[Sim] Starting simulation at t = [%lf]s", ros::Time::now().toSec());
        timer = n.createTimer(ros::Duration(1/sim_rate), &LightSim::updateCallback, this);
    }

    void setOperations(const std::vector<gauss_msgs::Operation> &operations) {
        for (auto operation : operations) {
            // There should be one operation for each icao_address
            icao_to_operation_map[operation.icao_address] = operation;
            icao_to_current_position_map[operation.icao_address] = operation.flight_plan.waypoints.front();
            icao_to_state_info_map[operation.icao_address].initPhysics(operation.flight_plan);
            ROS_INFO("[Sim] Loaded operation for icao [%s]", operation.icao_address.c_str());
        }
    }

    void setAutoStart(const std::map<std::string, ros::Time> &icao_to_start_time_map) {
        auto now = ros::Time::now();  // So it is the same for all operations
        for (auto const& auto_start : icao_to_start_time_map) {
            auto icao = auto_start.first;
            auto countdown = auto_start.second - now;
            ROS_INFO("[Sim] Operation icao [%s] will auto start in [%lf] seconds", icao.c_str(), countdown.toSec());

            auto callback = [icao, countdown, this](const ros::TimerEvent& event) {
                ROS_INFO("[Sim] Operation icao [%s] auto starting after [%lf] seconds", icao.c_str(), countdown.toSec());
                // this->startOperation(icao);  // Possible loop here: start->callback->start...
                // ...publish status instead:
                gauss_msgs_mqtt::RPSChangeFlightStatus status_msg;
                status_msg.icao = std::stoi(icao);
                status_msg.status = "start";
                this->status_pub.publish(status_msg);
            };
            auto_start_timers.push_back(n.createTimer(countdown, callback, true));
        }
    }
    
    void setCruisingSpeed(const std::map<std::string, double> &_icao_to_cruising_speed_map) {
        icao_to_cruising_speed_map = _icao_to_cruising_speed_map;
        for (auto const& cruising_speed_item : icao_to_cruising_speed_map) {
            auto icao = cruising_speed_item.first;
            auto speed = cruising_speed_item.second;
            ROS_INFO("[Sim] Operation icao [%s] will be simulated at [%lf] meters per seconds", icao.c_str(), speed);
        }
    }

// protected:  // Leave it public to allow calling from main (TODO: add new public function?)
    bool changeParamCallback(gauss_light_sim::ChangeParam::Request &req, gauss_light_sim::ChangeParam::Response &res) {
        if (icao_to_state_info_map.count(req.icao_address) == 0) {
            ROS_WARN("[Sim] Discarding ChangeParam request for unknown RPA[%s]", req.icao_address.c_str());
            return false;
        }

        ROS_INFO("[Sim] RPA[%s] at t = %lf will change: %s", req.icao_address.c_str(), req.stamp.toSec(), req.yaml.c_str());
        icao_to_state_info_map[req.icao_address].addChangeParamRequest(req);
        return true;
    }

   protected:
    bool changeFlightPlanCallback(gauss_light_sim::ChangeFlightPlan::Request &req, gauss_light_sim::ChangeFlightPlan::Response &res) {
        icao_to_operation_map[std::to_string(req.alternative.icao)].flight_plan.waypoints.clear();
        for (auto geo_wp : req.alternative.new_flight_plan){
            gauss_msgs::Waypoint temp_wp;
            double longitude, latitude, altitude;
            longitude = geo_wp.waypoint_elements[0];
            latitude = geo_wp.waypoint_elements[1];
            altitude = geo_wp.waypoint_elements[2];
            temp_wp.stamp = ros::Time(geo_wp.waypoint_elements[3]);
            proj_.Forward(latitude, longitude, altitude, temp_wp.x, temp_wp.y, temp_wp.z);
            // std::cout << req.alternative.icao << ":" << temp_wp.x << " " << temp_wp.y << " " << temp_wp.z << " " << temp_wp.stamp.toNSec() << "\n";
            icao_to_operation_map[std::to_string(req.alternative.icao)].flight_plan.waypoints.push_back(temp_wp);
        }
        return true;
    }

    void flightStatusCallback(const gauss_msgs_mqtt::RPSChangeFlightStatus::ConstPtr &msg) {
        // TODO: Decide if icao is a string or a uint32
        std::string icao_address = std::to_string(msg->icao);
        if (icao_to_is_started_map.count(icao_address) == 0) {
            ROS_WARN("[Sim] Discarding RPSChangeFlightStatus for unknown RPA[%s]", icao_address.c_str());
            return;
        }

        // TODO: Magic word for start?
        if (msg->status == "start") {
            startOperation(icao_address);
        } else if (msg->status == "stop") {
            stopOperation(icao_address);
        }
    }

    void startOperation(const std::string& icao_address) {
        bool started = icao_to_is_started_map[icao_address];
        if (started) {
            ROS_WARN("[Sim] Operation icao [%s] already started", icao_address.c_str());
        } else {
            icao_to_time_zero_map[icao_address] = ros::Time::now();
            icao_to_is_started_map[icao_address] = true;
            // gauss_msgs_mqtt::RPSChangeFlightStatus status_msg;
            // status_msg.icao = std::stoi(icao_address);
            // status_msg.status = "start";
            // status_pub.publish(status_msg);  // Possible loop here: start->callback->start...
            ROS_INFO("[Sim] RPA[%s] starting (t = %lf)", icao_address.c_str(), icao_to_time_zero_map[icao_address].toSec());
        }
    }

    void stopOperation(const std::string& icao_address) {
        bool started = icao_to_is_started_map[icao_address];
        if (!started) {
            ROS_WARN("[Sim] Operation icao [%s] not started yet", icao_address.c_str());
        } else {
            icao_to_is_started_map[icao_address] = false;
            ROS_INFO("[Sim] RPA[%s] stopping (t = %lf)", icao_address.c_str(), ros::Time::now().toSec());
        }
    }

    void updateCallback(const ros::TimerEvent &time) {
        for (auto element : icao_to_is_started_map) {
            auto icao = element.first;
            bool is_started = element.second;
            if (is_started) {
                // TODO: Fix base-time issues!
                // ros::Duration elapsed = time.current_real - icao_to_time_zero_map[icao];
                ros::Duration elapsed = time.current_real - ros::Time(0);
                auto operation = icao_to_operation_map[icao];
                if (!icao_to_state_info_map[icao].update(elapsed, operation, icao_to_cruising_speed_map, sim_rate)) {
                    // Operation is finished
                    ROS_INFO("[Sim] RPA[%s] finished operation", icao.c_str());
                    // TODO: Check tracking, it stop instantly the update of the operation instead of wait for the next service call (writeTracking)
                    gauss_msgs_mqtt::RPSChangeFlightStatus status_msg;
                    status_msg.icao = std::stoi(icao);
                    status_msg.status = "stop";
                    status_pub.publish(status_msg);
                }
                rpa_state_info_pub.publish(icao_to_state_info_map[icao].data);
                tf_broadcaster.sendTransform(icao_to_state_info_map[icao].tf);
            }
        }
    }

    std::map<std::string, bool> icao_to_is_started_map;
    std::map<std::string, ros::Time> icao_to_time_zero_map;
    std::map<std::string, double> icao_to_cruising_speed_map;
    std::map<std::string, gauss_msgs::Operation> icao_to_operation_map;
    std::map<std::string, RPAStateInfoWrapper> icao_to_state_info_map;
    std::map<std::string, gauss_msgs::Waypoint> icao_to_current_position_map;
    // std::vector<geometry_msgs::TransformStamped> tf_vector;  // TODO?

    // Param
    double sim_rate = 10.0;
    // Auxiliary variable for cartesian to geographic conversion
    GeographicLib::LocalCartesian proj_;

    ros::NodeHandle n;
    ros::Timer timer;
    std::vector<ros::Timer> auto_start_timers;
    ros::Subscriber status_sub;
    ros::Publisher rpa_state_info_pub, status_pub;
    ros::ServiceServer change_param_service;
    ros::ServiceServer change_flight_plan_service;
    tf2_ros::TransformBroadcaster tf_broadcaster;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "gauss_light_sim_node");
    ros::NodeHandle n;
    ros::NodeHandle np("~");
    ROS_INFO("[Sim] Started gauss_light_sim_node!");

    // Private param
    std::string timing_file = "";
    np.getParam("timing_file", timing_file);

    // Public params
    double time_param = 0.0;
    n.getParam("init_time", time_param);

    // Auxiliary variables for cartesian to geographic conversion
    double origin_latitude = ARENOSILLO_LATITUDE;
    double origin_longitude = ARENOSILLO_LONGITUDE;
    double origin_ellipsoidal_height = ARENOSILLO_ELLIPSOIDAL_HEIGHT;
    n.getParam("origin_latitude", origin_latitude);
    n.getParam("origin_longitude", origin_longitude);
    n.getParam("origin_ellipsoidal_height", origin_ellipsoidal_height);
    static GeographicLib::Geocentric earth(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f());
    static GeographicLib::LocalCartesian projection(origin_latitude, origin_longitude, origin_ellipsoidal_height, earth);

    auto read_icao_srv_url = "/gauss/read_icao";
    auto read_operation_srv_url = "/gauss/read_operation";

    ros::ServiceClient icao_client = n.serviceClient<gauss_msgs::ReadIcao>(read_icao_srv_url);
    ros::ServiceClient operation_client = n.serviceClient<gauss_msgs::ReadOperation>(read_operation_srv_url);

    ROS_INFO("[Sim] Waiting for required services...");
    ros::service::waitForService(read_icao_srv_url, -1);
    ROS_INFO("[Sim] %s: ok", read_icao_srv_url);
    ros::service::waitForService(read_operation_srv_url, -1);
    ROS_INFO("[Sim] %s: ok", read_operation_srv_url);

    gauss_msgs::ReadIcao read_icao;
    if (icao_client.call(read_icao)) {
        ROS_INFO("[Sim] Read icao addresses... ok");
        // std::cout << read_icao.response << '\n';
    } else {
        ROS_ERROR("[Sim] Failed to call service: [%s]", read_icao_srv_url);
        return 1;
    }

    // std::map<int8_t, std::string> id_to_icao_map;
    // std::map<std::string, int8_t> icao_to_id_map;
    // for (size_t i = 0; i < read_icao.response.uav_id.size(); i++) {
    //     id_to_icao_map[read_icao.response.uav_id[i]] = read_icao.response.icao_address[i];
    //     icao_to_id_map[read_icao.response.icao_address[i]] = read_icao.response.uav_id[i];
    // }

    gauss_msgs::ReadOperation read_operation;
    read_operation.request.uav_ids = read_icao.response.uav_id;
    if (operation_client.call(read_operation)) {
        ROS_INFO("[Sim] Read operations... ok");
        // std::cout << read_operation.response << '\n';
    } else {
        ROS_ERROR("[Sim] Failed to call service: [%s]", read_operation_srv_url);
        return 1;
    }

    LightSim sim(n, read_icao.response.icao_address, projection);
    sim.setOperations(read_operation.response.operation);

    ros::Time init_time;  // So it is the same for all
    if (time_param == 0.0){
        init_time = ros::Time::now();
    } else {
        init_time = ros::Time(time_param);
    }

    YAML::Node timing_yaml;
    if (timing_file != "") {
        ROS_INFO("[Sim] Loading timing from %s", timing_file.c_str());
        try {
            timing_yaml = YAML::LoadFile(timing_file);
        } catch(std::runtime_error& e) {
            ROS_ERROR("[Sim] Ignoring yaml, as file may not exist or it is bad defined: %s", e.what());
            // return 1;  // TODO: exit?
        }
    }

    if (timing_yaml["simulation"]["auto_start"]) {
        // Load auto_start_map from config file
        auto auto_start_yaml = timing_yaml["simulation"]["auto_start"];
        std::map<std::string, ros::Time> auto_start_map;
        ROS_INFO("[Sim] auto_start:");
        for (auto auto_start_item: auto_start_yaml) {
            ROS_INFO_STREAM("[Sim] - " << auto_start_item);
            auto icao = auto_start_item["icao"].as<std::string>();
            auto delay = auto_start_item["delay"].as<float>();
            auto_start_map[icao] = init_time + ros::Duration(delay);
            // std::cout << icao << ": " << auto_start_map[icao] << '\n';
        }
        //auto_start_map[read_icao.response.icao_address.front()] = init_time + ros::Duration(43);
        //auto_start_map[read_icao.response.icao_address.back()] = init_time + ros::Duration(8);
        sim.setAutoStart(auto_start_map);
    }

    if (timing_yaml["simulation"]["change_param"]) {
        // Load change_param from config file
        auto change_param_yaml = timing_yaml["simulation"]["change_param"];
        ROS_INFO("[Sim] change_param:");
        for (auto change_param_item: change_param_yaml) {
            ROS_INFO_STREAM("[Sim] - " << change_param_item);
            auto icao = change_param_item["icao"].as<std::string>();
            auto delay = change_param_item["delay"].as<float>();
            YAML::Node inner_yaml = change_param_item["yaml"];
            std::stringstream ss_yaml;
            ss_yaml << inner_yaml;
            gauss_light_sim::ChangeParam::Request req;
            gauss_light_sim::ChangeParam::Response res;
            req.icao_address = icao;
            req.stamp = init_time + ros::Duration(delay);
            req.yaml = ss_yaml.str();
            // std::cout << req << '\n';
            sim.changeParamCallback(req, res);
        }
    }

    if (timing_yaml["simulation"]["cruising_speed"]) {
        // Load auto_start_map from config file
        auto cruising_speed_yaml = timing_yaml["simulation"]["cruising_speed"];
        std::map<std::string, double> cruising_speed_map;
        ROS_INFO("[Sim] cruising_speed:");
        for (auto cruising_speed_item: cruising_speed_yaml) {
            ROS_INFO_STREAM("[Sim] - " << cruising_speed_item);
            auto icao = cruising_speed_item["icao"].as<std::string>();
            auto speed = cruising_speed_item["speed"].as<double>();
            cruising_speed_map[icao] = speed;
        }
        sim.setCruisingSpeed(cruising_speed_map);
    }

    sim.start();
    ros::spin();
    return 0;
}
