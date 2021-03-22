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

struct RPYAngles {
    double roll = 0;
    double pitch = 0;
    double yaw = 0;
};

// This is a trickier version of the previous function, as time-base issues must be addressed (TODO!)
gauss_msgs::Waypoint interpolate(const gauss_msgs::Waypoint &from, const gauss_msgs::Waypoint &to, const ros::Duration &t, RPYAngles* angles = nullptr) {
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

    if (angles) {
        angles->yaw = atan2(delta_y, delta_x);
        // double xy_distance = sqrt(delta_x*delta_x + delta_y*delta_y);
        // angles->pitch = atan2(delta_z, xy_distance);  // TODO: only fixed_wing!
        angles->pitch = 0;
        angles->roll = 0;
    }

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

    bool update(const ros::Duration &elapsed, const gauss_msgs::Operation &operation) {
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

        return updatePhysics(elapsed, operation.flight_plan);
    }

    void setProjection(const GeographicLib::LocalCartesian &projection) { proj = projection; }

    // TODO: Initialize phyics?
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
            proj.Reverse(flight_plan.waypoints[0].x, flight_plan.waypoints[0].y, flight_plan.waypoints[0].z, latitude, longitude, altitude);
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
        gauss_msgs::Waypoint target_point;
        if (prev.stamp == next.stamp) {
            // Last waypoint is reached
            running = false;
            target_point = next;
        } else {
            running = true;
            RPYAngles angles;
            target_point = interpolate(prev, next, elapsed, &angles);
            data.yaw = angles.yaw;
            data.pitch = angles.pitch;
            data.roll = angles.roll;
            // Update tf transform data
            tf2::Quaternion q;
            q.setRPY(data.roll, data.pitch, data.yaw);
            tf.transform.rotation.x = q.x();
            tf.transform.rotation.y = q.y();
            tf.transform.rotation.z = q.z();
            tf.transform.rotation.w = q.w();
            tf.transform.translation.x = target_point.x;
            tf.transform.translation.y = target_point.y;
            tf.transform.translation.z = target_point.z;
        }

        // TODO: Play with a current_point and target_point to add some memory and behave more realistically if flight_plan is changed
        // Cartesian to geographic conversion
        proj.Reverse(target_point.x, target_point.y, target_point.z, latitude, longitude, altitude);
        data.latitude = latitude;
        data.longitude = longitude;
        data.altitude = altitude;
        data.groundspeed = calculateMeanSpeed(prev, next);
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
    }

    void start() {
        ROS_INFO("[Sim] Starting simulation at t = [%lf]s", ros::Time::now().toSec());
        timer = n.createTimer(ros::Duration(1), &LightSim::updateCallback, this);
    }

    void setOperations(const std::vector<gauss_msgs::Operation> &operations) {
        for (auto operation : operations) {
            // There should be one operation for each icao_address
            icao_to_operation_map[operation.icao_address] = operation;
            icao_to_current_position_map[operation.icao_address] = operation.flight_plan.waypoints.front();
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
                if (!icao_to_state_info_map[icao].update(elapsed, operation)) {
                    // TODO: Check why it stops after receiving a new flight plan <-----------------
                    // // Operation is finished
                    // ROS_INFO("RPA[%s] finished operation", icao.c_str());
                    // icao_to_is_started_map[icao] = false;
                    // // TODO: Check tracking, it stop instantly the update of the operation instead of wait for the next service call (writeTracking)
                    // gauss_msgs_mqtt::RPSChangeFlightStatus status_msg;
                    // status_msg.icao = std::stoi(icao);
                    // status_msg.status = "stop";
                    // status_pub.publish(status_msg);
                }
                rpa_state_info_pub.publish(icao_to_state_info_map[icao].data);
                tf_broadcaster.sendTransform(icao_to_state_info_map[icao].tf);
            }
        }
    }

    std::map<std::string, bool> icao_to_is_started_map;
    std::map<std::string, ros::Time> icao_to_time_zero_map;
    std::map<std::string, gauss_msgs::Operation> icao_to_operation_map;
    std::map<std::string, RPAStateInfoWrapper> icao_to_state_info_map;
    std::map<std::string, gauss_msgs::Waypoint> icao_to_current_position_map;
    // std::vector<geometry_msgs::TransformStamped> tf_vector;  // TODO?

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

    sim.start();
    ros::spin();
    return 0;
}
