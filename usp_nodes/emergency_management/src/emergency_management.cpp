#include <gauss_msgs/NewDeconfliction.h>
#include <gauss_msgs/NewThreat.h>
#include <gauss_msgs/NewThreats.h>
#include <gauss_msgs/Notifications.h>
#include <gauss_msgs/PilotAnswer.h>
#include <ros/ros.h>

#include <Eigen/Eigen>

ros::ServiceClient tactical_client_, notification_client_;

gauss_msgs::Threat translateToThreat(const gauss_msgs::NewThreat _threat) {
    gauss_msgs::Threat out_threat;
    out_threat.geofence_ids = _threat.geofence_ids;
    out_threat.header = _threat.header;
    out_threat.location = _threat.location;
    out_threat.priority_ops = _threat.priority_ops;
    out_threat.threat_id = _threat.threat_id;
    out_threat.threat_type = _threat.threat_type;
    out_threat.times = _threat.times;
    out_threat.uav_ids = _threat.uav_ids;

    return out_threat;
}

gauss_msgs::Notification selectBestSolution(const gauss_msgs::NewDeconfliction &_msg, const int &_uav_id) {
    gauss_msgs::Notification out_solution;
    double check_cost_risk = std::numeric_limits<double>::max();
    for (auto possible_solution : _msg.response.deconfliction_plans) {
        // If both UAV have the same priority, uav_id value is -1. We should check all the solutions.
        // If not, just check solution for the UAV with less priority.
        if ((possible_solution.uav_id == _uav_id || _uav_id == -1) && check_cost_risk >= (possible_solution.cost + possible_solution.riskiness)) {
            out_solution.description = "";
            out_solution.threat = translateToThreat(_msg.request.threat);
            out_solution.uav_id = possible_solution.uav_id;
            out_solution.action = possible_solution.maneuver_type;
            out_solution.maneuver_type = possible_solution.maneuver_type;
            out_solution.waypoints = possible_solution.waypoint_list;
            out_solution.new_flight_plan.waypoints = possible_solution.waypoint_list;

            check_cost_risk = possible_solution.cost + possible_solution.riskiness;
        }
    }
    for (auto operation : _msg.request.threat.conflictive_operations) {
        if (operation.uav_id == out_solution.uav_id) {
            out_solution.actual_wp = operation.actual_wp;
            out_solution.current_wp = operation.current_wp;
            out_solution.flight_plan = operation.flight_plan;
        }
    }

    return out_solution;
}

int selectSmallerPriority(const gauss_msgs::NewThreat &_threat) {
    int out_uav_id_priority;
    int smaller_priority = std::numeric_limits<int>::max();
    ROS_ERROR_COND(_threat.uav_ids.size() != _threat.priority_ops.size(), "[EM] Threat uav_ids size [%zd] should match priority_ops size [%zd]!", _threat.uav_ids.size(), _threat.priority_ops.size());
    for (int idx = 0; idx < _threat.uav_ids.size(); idx++) {
        if (smaller_priority > _threat.priority_ops.at(idx)) {
            smaller_priority = _threat.priority_ops.at(idx);
            out_uav_id_priority = _threat.uav_ids.at(idx);
        } else if (smaller_priority == _threat.priority_ops.at(idx)) {
            smaller_priority = _threat.priority_ops.at(idx);
            out_uav_id_priority = -1;  // Both UAVs have the same priority, we should check all the posible solutions
        }
    }

    return out_uav_id_priority;
}

bool pilotAnswerCb(gauss_msgs::PilotAnswerRequest &_req, gauss_msgs::PilotAnswerResponse &_res) {
    ROS_INFO_STREAM("[EM] The pilot decided " << _req.pilot_answers.front() << " corresponding to threat id " << (int)_req.threat_ids.front());
    _res.success = true;
    return _res.success;
}

bool threatsCb(gauss_msgs::NewThreatsRequest &_req, gauss_msgs::NewThreatsResponse &_res) {
    std::string cout_threats;
    for (auto threat : _req.threats) {
        cout_threats = cout_threats + " [" + std::to_string(threat.threat_id) + " " + std::to_string(threat.threat_type) + " |";
        for (auto uav_id : threat.uav_ids) cout_threats = cout_threats + " " + std::to_string(uav_id);
        cout_threats = cout_threats + "]";
    }
    ROS_INFO_STREAM_COND(_req.threats.size() > 0, "[EM] Threats received: [id type | uav] " + cout_threats);
    for (auto threat : _req.threats) {
        if (threat.threat_type == threat.GEOFENCE_CONFLICT || threat.threat_type == threat.GEOFENCE_INTRUSION ||
            threat.threat_type == threat.GNSS_DEGRADATION || threat.threat_type == threat.LACK_OF_BATTERY ||
            threat.threat_type == threat.LOSS_OF_SEPARATION || threat.threat_type == threat.UAS_OUT_OV) {
            // Call tactical
            gauss_msgs::NewDeconfliction tactical_msg;
            tactical_msg.request.threat = threat;
            if (tactical_client_.call(tactical_msg)) {
                int uav_id_smaller_priority = threat.uav_ids.front();
                if (threat.threat_type == threat.LOSS_OF_SEPARATION) uav_id_smaller_priority = selectSmallerPriority(threat);  // Get the UAV id with less priority
                gauss_msgs::Notifications notifications_msg;
                notifications_msg.request.notifications.push_back(selectBestSolution(tactical_msg, uav_id_smaller_priority));
                if (!notification_client_.call(notifications_msg)) ROS_WARN("[EM] Failed to call USP manager");
            } else {
                ROS_WARN("[EM] Failed to call tactical deconfliction!");
            }
        }
    }

    _res.success = true;
    return _res.success;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "Emergency_management");

    ros::NodeHandle nh;

    auto threats_srv_url = "/gauss/new_threats";
    auto pilot_answer_srv_url = "/gauss/pilotanswer";
    auto notifications_clt_url = "/gauss/notifications";
    auto tactical_clt_url = "/gauss/new_tactical_deconfliction";

    ros::ServiceServer threats_server = nh.advertiseService(threats_srv_url, threatsCb);
    ros::ServiceServer pilot_answer_server = nh.advertiseService(pilot_answer_srv_url, pilotAnswerCb);
    notification_client_ = nh.serviceClient<gauss_msgs::Notifications>(notifications_clt_url);
    tactical_client_ = nh.serviceClient<gauss_msgs::NewDeconfliction>(tactical_clt_url);

    ROS_INFO("[EM] Waiting for required services...");
    ros::service::waitForService(notifications_clt_url, -1);
    ROS_INFO("[EM] %s: ok", notifications_clt_url);
    ros::service::waitForService(tactical_clt_url, -1);
    ROS_INFO("[EM] %s: ok", tactical_clt_url);

    ros::Rate rate(1);  // [Hz]
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    ros::spin();
    return 0;
}
