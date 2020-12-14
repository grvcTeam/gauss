#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <gauss_msgs/ReadIcao.h>
#include <gauss_msgs/ReadOperation.h>
#include <gauss_msgs_mqtt/RPSChangeFlightStatus.h>
#include <gauss_msgs_mqtt/RPAStateInfo.h>
#include <gauss_light_sim/ChangeParam.h>

class RPAStateInfoWrapper: public gauss_msgs_mqtt::RPAStateInfo {
public:

    void addChangeParamRequest(const gauss_light_sim::ChangeParam::Request& req) {
        change_param_request_list.push_back(req);
        // TODO: Is it worth sorting based on time?
    }

    void update(const ros::Duration& elapsed) {
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

    void applyChange(const YAML::Node& yaml_change) {

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

    LightSim(ros::NodeHandle &n): n(n) {

        change_param_service = n.advertiseService("gauss_light_sim/change_param", &LightSim::changeParamCallback, this);
        status_sub = n.subscribe("flight_status", 10, &LightSim::flightStatusCallback, this);  // TODO: Check topic url
        timer = n.createTimer(ros::Duration(1), &LightSim::updateCallback, this);  // TODO: rate 1Hz?
    }

    void addRPAs(const std::vector<std::string>& icao_addresses) {
        for (auto icao: icao_addresses) {
            icao_to_is_started_map[icao] = false;
        }
    }

    void addOperations(const std::vector<gauss_msgs::Operation>& operations) {
        for (auto operation: operations) {
            // TODO: More than one operation for one icao_address?
            icao_to_operations_map[operation.icao_address].push_back(operation);
        }
    }

protected:

    bool changeParamCallback(gauss_light_sim::ChangeParam::Request &req, gauss_light_sim::ChangeParam::Response &res) {
        icao_to_state_info_map[req.icao_address].addChangeParamRequest(req);
        return true;
    }

    void flightStatusCallback(const gauss_msgs_mqtt::RPSChangeFlightStatus::ConstPtr& msg) {
        // TODO: Decide if icao is a string or a uint32
        std::string icao_address = std::to_string(msg->icao);
        bool started = icao_to_is_started_map[icao_address];
        // TODO: Magic word for start?
        if ((msg->status == "start") && !started) {
            icao_to_time_zero_map[icao_address] = ros::Time::now();
            icao_to_is_started_map[icao_address] = true;
        }
    }

    void updateCallback(const ros::TimerEvent& time) {
        for (auto element: icao_to_is_started_map) {
            auto icao = element.first;
            bool is_started = element.second;
            if (is_started) {
                ros::Duration elapsed = time.current_real - icao_to_time_zero_map[icao];
                icao_to_state_info_map[icao].update(elapsed);  // TODO: pass operation
                // TODO: update also physics and publish
            }
        }
    }

    std::map<std::string, bool> icao_to_is_started_map;
    std::map<std::string, ros::Time> icao_to_time_zero_map;
    std::map<std::string, std::vector<gauss_msgs::Operation>> icao_to_operations_map;  // TODO: more than one operation is possible?
    std::map<std::string, RPAStateInfoWrapper> icao_to_state_info_map;

    ros::NodeHandle n;
    ros::Timer timer;
    ros::Subscriber status_sub;
    ros::ServiceServer change_param_service;
};


int main(int argc, char** argv) {
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
