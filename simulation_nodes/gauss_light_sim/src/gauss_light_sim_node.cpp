#include <gauss_light_sim/ChangeParam.h>
#include <gauss_msgs/ReadIcao.h>
#include <gauss_msgs/ReadOperation.h>
#include <gauss_msgs_mqtt/RPAStateInfo.h>
#include <gauss_msgs_mqtt/RPSChangeFlightStatus.h>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Eigen>
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

#define ARENOSILLO_LATITUDE 37.094784
#define ARENOSILLO_LONGITUDE -6.735478

#define REFERENCE_LATITUDE ARENOSILLO_LATITUDE
#define REFERENCE_LONGITUDE ARENOSILLO_LONGITUDE

class RPAStateInfoWrapper : public gauss_msgs_mqtt::RPAStateInfo {
   public:
    void addChangeParamRequest(const gauss_light_sim::ChangeParam::Request &req) {
        change_param_request_list.push_back(req);
        // TODO: Is it worth sorting based on time?
    }

    void update(const ros::Duration &elapsed) {
        // TODO: Update also physics?
        // Pass flight_plan from operation?
        auto it = change_param_request_list.begin();
        while (it != change_param_request_list.end()) {
            auto change = *it;
            if (elapsed.toSec() > change.stamp.toSec()) {
                YAML::Node yaml_change = YAML::Load(change.yaml);
                applyChange(yaml_change);
                std::cout << this->hal << '\n';
                // And erase this change, keepin valid it
                it = change_param_request_list.erase(it);
            } else {
                ++it;
            }
        }
    }

    void applyChange(const YAML::Node &yaml_change) {
        std::string param_name = yaml_change["name"].as<std::string>();
        std::string param_type = yaml_change["type"].as<std::string>();

        if ((param_name == "covariance_h") && (param_type == "float")) {
            this->covariance_h = yaml_change["value"].as<float>();

        } else if ((param_name == "covariance_v") && (param_type == "float")) {
            this->covariance_v = yaml_change["value"].as<float>();

        } else if ((param_name == "hpl") && (param_type == "float")) {
            this->hpl = yaml_change["value"].as<float>();

        } else if ((param_name == "hal") && (param_type == "float")) {
            this->hal = yaml_change["value"].as<float>();

        } else if ((param_name == "vpl") && (param_type == "float")) {
            this->vpl = yaml_change["value"].as<float>();

        } else if ((param_name == "val") && (param_type == "float")) {
            this->val = yaml_change["value"].as<float>();

        } else if ((param_name == "solution_mode") && (param_type == "string")) {
            this->solution_mode = yaml_change["value"].as<std::string>();

        } else if ((param_name == "jamming") && (param_type == "float")) {
            this->jamming = yaml_change["value"].as<float>();

        } else if ((param_name == "spoofing") && (param_type == "float")) {
            this->spoofing = yaml_change["value"].as<float>();

        } else if ((param_name == "anomalous_clock_drift") && (param_type == "bool")) {
            this->anomalous_clock_drift = yaml_change["value"].as<bool>();

        } else if ((param_name == "anomalous_pos_drift") && (param_type == "bool")) {
            this->anomalous_pos_drift = yaml_change["value"].as<bool>();

        } else if ((param_name == "signal_noise_ratio") && (param_type == "float")) {
            this->signal_noise_ratio = yaml_change["value"].as<float>();

        } else if ((param_name == "received_power") && (param_type == "float")) {
            this->received_power = yaml_change["value"].as<float>();

        } else {
            ROS_ERROR("Unexpected param [%s] of type [%s]", param_name.c_str(), param_type.c_str());
        }
    }

   protected:
    std::vector<gauss_light_sim::ChangeParam::Request> change_param_request_list;  // TODO: other container?
};

class LightSim {
   public:
    LightSim(ros::NodeHandle &n) : n(n) {
        change_param_service = n.advertiseService("gauss_light_sim/change_param", &LightSim::changeParamCallback, this);
        status_sub = n.subscribe("flight_status", 10, &LightSim::flightStatusCallback, this);  // TODO: Check topic url
        rpa_state_info_pub = n.advertise<gauss_msgs_mqtt::RPAStateInfo>("/gauss/rpa_state", 1);
        timer = n.createTimer(ros::Duration(1), &LightSim::updateCallback, this);  // TODO: rate 1Hz?
    }

    void addRPAs(const std::vector<std::string> &icao_addresses) {
        for (auto icao : icao_addresses) {
            icao_to_is_started_map[icao] = false;
        }
    }

    void addOperations(const std::vector<gauss_msgs::Operation> &operations) {
        for (auto operation : operations) {
            // TODO: More than one operation for one icao_address?
            icao_to_operations_map[operation.icao_address].push_back(operation);
        }
    }

   protected:
    bool changeParamCallback(gauss_light_sim::ChangeParam::Request &req, gauss_light_sim::ChangeParam::Response &res) {
        icao_to_state_info_map[req.icao_address].addChangeParamRequest(req);
        return true;
    }

    void flightStatusCallback(const gauss_msgs_mqtt::RPSChangeFlightStatus::ConstPtr &msg) {
        // TODO: Decide if icao is a string or a uint32
        std::string icao_address = std::to_string(msg->icao);
        bool started = icao_to_is_started_map[icao_address];
        // TODO: Magic word for start?
        if ((msg->status == "start") && !started) {
            icao_to_time_zero_map[icao_address] = ros::Time::now();
            icao_to_is_started_map[icao_address] = true;
        } else if ((msg->status == "stop") && started) {
            icao_to_is_started_map[icao_address] = false;
        }
    }

    void updateCallback(const ros::TimerEvent &time) {
        for (auto element : icao_to_is_started_map) {
            auto icao = element.first;
            bool is_started = element.second;
            if (is_started) {
                ros::Duration elapsed = time.current_real - icao_to_time_zero_map[icao];
                icao_to_state_info_map[icao].update(elapsed);  // TODO: pass operation
                // TODO: update also physics and publish
                rpa_state_info_pub.publish(createRPAStateInfoMsg(icao));
            }
        }
    }

    gauss_msgs_mqtt::RPAStateInfo createRPAStateInfoMsg(std::string icao) {
        gauss_msgs_mqtt::RPAStateInfo out_msg;
        // Auxiliary variables for cartesian to geographic conversion
        static double origin_latitude(REFERENCE_LATITUDE);
        static double origin_longitude(REFERENCE_LONGITUDE);
        static GeographicLib::Geocentric earth(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f());
        static GeographicLib::LocalCartesian proj(origin_latitude, origin_longitude, 0, earth);
        double latitude, longitude, altitude;
        // Calculate next waypoint
        static gauss_msgs::Waypoint current_position;
        current_position = nextWaypoint(current_position, icao_to_operations_map[icao].front().flight_plan);
        // Cartesian to geographic conversion
        proj.Reverse(current_position.x, current_position.y, current_position.z, latitude, longitude, altitude);
        out_msg.altitude = altitude;
        out_msg.latitude = latitude;
        out_msg.longitude = longitude;
        out_msg.timestamp = current_position.stamp.toNSec() / 1000000;
        out_msg.icao = atoi(icao.c_str());

        return out_msg;
    }

    double distanceBetweenWaypoints(gauss_msgs::Waypoint &waypoint_a, gauss_msgs::Waypoint &waypoint_b) {
        return sqrt(pow(waypoint_a.x - waypoint_b.x, 2) + pow(waypoint_a.y - waypoint_b.y, 2) + pow(waypoint_a.z - waypoint_b.z, 2));
    }

    void findSegmentWaypointsIndices(gauss_msgs::Waypoint &current_position, gauss_msgs::WaypointList &flight_plan, int &a_index, int &b_index, double &distance_to_segment, double &distance_to_point_a, double &distance_to_point_b) {
        int first_index = 0;
        int second_index = 0;
        distance_to_segment = std::numeric_limits<double>::max();
        double distance_to_waypoint_a = std::numeric_limits<double>::max();
        bool flag_distance_to_waypoint_a = false;
        bool flag_distance_to_waypoint_b = false;

        for (int i = 0; i < flight_plan.waypoints.size() - 1; i++) {
            gauss_msgs::Waypoint &waypoint_a = flight_plan.waypoints[i];
            gauss_msgs::Waypoint &waypoint_b = flight_plan.waypoints[i + 1];

            Eigen::Vector3d vector_u;
            flag_distance_to_waypoint_a = false;
            flag_distance_to_waypoint_b = false;
            vector_u.x() = waypoint_b.x - waypoint_a.x;
            vector_u.y() = waypoint_b.y - waypoint_a.y;
            vector_u.z() = waypoint_b.z - waypoint_a.z;

            Eigen::Matrix4d A;
            Eigen::Vector4d b;
            Eigen::Vector4d x;

            A = Eigen::Matrix4d::Identity();
            A(3, 3) = 0;
            A(0, 3) = vector_u.x();
            A(1, 3) = vector_u.y();
            A(2, 3) = vector_u.z();
            A(3, 0) = vector_u.x();
            A(3, 1) = vector_u.y();
            A(3, 2) = vector_u.z();

            x.x() = current_position.x - waypoint_a.x;
            x.y() = current_position.y - waypoint_a.y;
            x.z() = current_position.z - waypoint_a.z;

            b = A.inverse() * x;

            double distance;
            if (b[3] >= 0 && b[3] <= 1) {
                distance = sqrt(b[0] * b[0] + b[1] * b[1] + b[2] * b[2]);
            } else {
                double distance_to_waypoint_a = distanceBetweenWaypoints(current_position, waypoint_a);
                double distance_to_waypoint_b = distanceBetweenWaypoints(current_position, waypoint_b);

                if (distance_to_waypoint_a < distance_to_waypoint_b) {
                    distance = distance_to_waypoint_a;
                    flag_distance_to_waypoint_a = true;
                } else {
                    distance = distance_to_waypoint_b;
                    flag_distance_to_waypoint_b = true;
                }
            }

            if (distance <= distance_to_segment) {
                distance_to_segment = distance;
                if (flag_distance_to_waypoint_a) {
                    first_index = i;
                    second_index = i;
                } else if (flag_distance_to_waypoint_b) {
                    first_index = i + 1;
                    second_index = i + 1;
                } else {
                    first_index = i;
                    second_index = i + 1;
                }
            }
        }

        a_index = first_index;
        b_index = second_index;

        gauss_msgs::Waypoint &waypoint_a = flight_plan.waypoints[first_index];
        gauss_msgs::Waypoint &waypoint_b = flight_plan.waypoints[second_index];

        distance_to_point_a = distanceBetweenWaypoints(waypoint_a, current_position);
        distance_to_point_b = distanceBetweenWaypoints(waypoint_b, current_position);
    }

    gauss_msgs::Waypoint nextWaypoint(gauss_msgs::Waypoint &_current_position, gauss_msgs::WaypointList &_flight_plan) {
        gauss_msgs::Waypoint out_wp;
        int a_waypoint_index = 0;
        int b_waypoint_index = 0;
        double distance_to_segment = 0;
        double distance_to_point_a = 0;
        double distance_to_point_b = 0;
        findSegmentWaypointsIndices(_current_position, _flight_plan, a_waypoint_index, b_waypoint_index, distance_to_segment, distance_to_point_a, distance_to_point_b);
        // Trick results of findSegmentWaypointsIndices
        if (a_waypoint_index == b_waypoint_index) {
            b_waypoint_index++;
            // std::cout << "b + 1: " << b_waypoint_index << "\n";
        }
        // Trick results of findSegmentWaypointsIndices
        if (b_waypoint_index != 0 && a_waypoint_index < _flight_plan.waypoints.size() - 1 && b_waypoint_index < _flight_plan.waypoints.size()) {
            // Get unit vector between wpa and wpb
            Eigen::Vector3f wpa = Eigen::Vector3f(_flight_plan.waypoints.at(a_waypoint_index).x, _flight_plan.waypoints.at(a_waypoint_index).y, _flight_plan.waypoints.at(a_waypoint_index).z);
            Eigen::Vector3f wpb = Eigen::Vector3f(_flight_plan.waypoints.at(b_waypoint_index).x, _flight_plan.waypoints.at(b_waypoint_index).y, _flight_plan.waypoints.at(b_waypoint_index).z);
            Eigen::Vector3f unit_vec = (wpb - wpa) / (wpb - wpa).norm();
            unit_vec = unit_vec / unit_vec.norm();
            // Get speed to go from A to B
            double time_between_wps = (_flight_plan.waypoints.at(b_waypoint_index).stamp - _flight_plan.waypoints.at(a_waypoint_index).stamp).toSec();
            double speed = (wpb - wpa).norm() / time_between_wps;  // m/s
            Eigen::Vector3f sum_vec = unit_vec * speed;
            // Create next waypoint using the speed and the unit vector
            out_wp.x = _current_position.x + sum_vec(0);
            out_wp.y = _current_position.y + sum_vec(1);
            out_wp.z = _current_position.z + sum_vec(2);
            // Check if the next waypoint is inside the current segment
            double dist_ab = distanceBetweenWaypoints(_flight_plan.waypoints.at(a_waypoint_index), _flight_plan.waypoints.at(b_waypoint_index));
            double dist_ao = distanceBetweenWaypoints(_flight_plan.waypoints.at(a_waypoint_index), out_wp);
            double dist_bo = distanceBetweenWaypoints(_flight_plan.waypoints.at(b_waypoint_index), out_wp);
            if (dist_ao + dist_bo != dist_ab && b_waypoint_index + 1 < _flight_plan.waypoints.size()) {  // A ------- B --- O ----- C
                // If next waypoint is not inside the current segment, it should be modified adding to B the distance BO aiming to C
                Eigen::Vector3f wpc = Eigen::Vector3f(_flight_plan.waypoints.at(b_waypoint_index + 1).x, _flight_plan.waypoints.at(b_waypoint_index + 1).y, _flight_plan.waypoints.at(b_waypoint_index + 1).z);
                Eigen::Vector3f unit_vec = (wpc - wpb) / (wpc - wpb).norm();
                unit_vec = unit_vec / unit_vec.norm();
                // Get speed to go from B to C
                double next_time_between_wps = (_flight_plan.waypoints.at(b_waypoint_index + 1).stamp - _flight_plan.waypoints.at(b_waypoint_index).stamp).toSec();
                double next_speed = (wpc - wpb).norm() / next_time_between_wps;  // m/s
                // Rule of three. BO was calculated with previous speed, get new speed and calculate new BO distance.
                double dist_bo_next_speed = next_speed * dist_bo / speed;
                sum_vec = unit_vec * dist_bo_next_speed;
                // Create next waypoint using the speed and the unit vector
                out_wp.x = _flight_plan.waypoints.at(b_waypoint_index).x + sum_vec(0);
                out_wp.y = _flight_plan.waypoints.at(b_waypoint_index).y + sum_vec(1);
                out_wp.z = _flight_plan.waypoints.at(b_waypoint_index).z + sum_vec(2);
            }  // A ----- O -- B
            // Do not go underground
            if (out_wp.z < 0) out_wp.z = 0;
        } else {
            out_wp = _current_position;
        }
        // Assuming the node is publishing RPAStateInfo every second.
        out_wp.stamp = ros::Time(_current_position.stamp.toSec() + 1.0);

        return out_wp;
    }

    std::map<std::string, bool> icao_to_is_started_map;
    std::map<std::string, ros::Time> icao_to_time_zero_map;
    std::map<std::string, std::vector<gauss_msgs::Operation>> icao_to_operations_map;  // TODO: more than one operation is possible?
    std::map<std::string, RPAStateInfoWrapper> icao_to_state_info_map;

    ros::NodeHandle n;
    ros::Timer timer;
    ros::Subscriber status_sub;
    ros::Publisher rpa_state_info_pub;
    ros::ServiceServer change_param_service;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "gauss_light_sim_node");
    ros::NodeHandle n;

    auto read_icao_srv_url = "/gauss/read_icao";
    auto read_operation_srv_url = "/gauss/read_operation";

    ros::ServiceClient icao_client = n.serviceClient<gauss_msgs::ReadIcao>(read_icao_srv_url);
    ros::ServiceClient operation_client = n.serviceClient<gauss_msgs::ReadOperation>(read_operation_srv_url);

    gauss_msgs::ReadIcao read_icao;
    if (icao_client.call(read_icao)) {
        std::cout << read_icao.response << '\n';
    } else {
        ROS_ERROR("Failed to call service: [%s]", read_icao_srv_url);
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
        std::cout << read_operation.response << '\n';
    } else {
        ROS_ERROR("Failed to call service: [%s]", read_operation_srv_url);
        return 1;
    }

    LightSim sim(n);
    sim.addRPAs(read_icao.response.icao_address);
    sim.addOperations(read_operation.response.operation);
    ros::spin();
    return 0;
}
