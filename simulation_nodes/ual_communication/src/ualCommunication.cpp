//----------------------------------------------------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2018 Hector Perez Leon
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
// documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
// Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
// OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
// OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//----------------------------------------------------------------------------------------------------------------------

#include <upat_follower/ual_communication.h>
#include <gauss_msgs/ReadFlightPlan.h>
#include <gauss_msgs/ReadOperation.h>
#include <gauss_msgs/Notification.h>
#include <gauss_msgs/PositionReport.h>

int uav_id_;
bool new_notification_ = false;
gauss_msgs::Notification notification_;
uav_abstraction_layer::State ual_state_;
geometry_msgs::PoseStamped ual_pose_;
geometry_msgs::TwistStamped ual_vel_;
ros::ServiceClient read_operation_client_;

void ualStateCb(const uav_abstraction_layer::State msg) {
    ual_state_.state = msg.state;
}

void ualPoseCb(const geometry_msgs::PoseStamped msg) {
    ual_pose_ = msg;
}

void ualVelocityCb(const geometry_msgs::TwistStamped msg) {
    ual_vel_ = msg;
}

void notificationCb(const gauss_msgs::Notification msg) {
    if (msg.uav_id == uav_id_ && !new_notification_){
        notification_ = msg;
        new_notification_ = true;
    }
}

nav_msgs::Path mergeFlightPlan(const gauss_msgs::WaypointList &_flight_plan, const gauss_msgs::Threat &_threat, const int &_maneuver_type, const int &_current_wp){
    nav_msgs::Path out_path;

    bool merge_to_the_end = false;
    int flight_plan_section = 0;
    
    switch (_threat.threat_id) // TODO: Check using examples. Geofence conflict working. 
    {
    case _threat.GEOFENCE_CONFLICT: 
        switch (_maneuver_type)
        {
        case 1: // Ruta a mi destino evitando una geofence
            ROS_WARN("1");
            merge_to_the_end = true;
            flight_plan_section = 0;
            break;
        case 3: // Ruta que me manda devuelta a casa
            ROS_WARN("3");
            merge_to_the_end = false;
            flight_plan_section = 2; 
            break;
        }
        break;
    case _threat.GEOFENCE_INTRUSION:
        switch (_maneuver_type)
        {
        case 2: // Ruta a mi destino por el camino mas corto
            merge_to_the_end = false;
            flight_plan_section = 2;
            break;
        case 3: // Ruta que me manda de vuelta a casa
            merge_to_the_end = false;
            flight_plan_section = 2;
            break;
        case 6: // Ruta a mi destino saliendo lo antes posible de la geofence
            merge_to_the_end = true;
            flight_plan_section = 2;
            break;
        }
        break;
    case _threat.GNSS_DEGRADATION:
        switch (_maneuver_type)
        {
        case 5: // Ruta que aterrice en un landing spot
            merge_to_the_end = false;
            flight_plan_section = 2;
            break;
        }
        break;
    case _threat.JAMMING_ATTACK:
        // LAND NOW!
        break;
    case _threat.LACK_OF_BATTERY:
        switch (_maneuver_type)
        {
        case 5: // Ruta que aterrice en un landing spot
            merge_to_the_end = false;
            flight_plan_section = 2;
            break;
        }
        break;
    case _threat.LOSS_OF_SEPARATION:
        merge_to_the_end = true;
        flight_plan_section = 0;
        break;
    case _threat.TECHNICAL_FAILURE:
        // LAND NOW!
        break;
    case _threat.UAS_OUT_OV:
        switch (_maneuver_type)
        {
        case 9: // Ruta para volver lo antes posible al flight geometry y seguir el plan de vuelo
            merge_to_the_end = true;
            flight_plan_section = 2;
            break;
        case 10: // Ruta para seguir con el plan de vuelo, da igual que esté más tiempo fuera del Operational Volume
            merge_to_the_end = true;
            flight_plan_section = 2;
            break;
        }
        break;
    } 

    bool change_path_reference = false;
    for (int i = 0; i < _flight_plan.waypoints.size(); i++){
        geometry_msgs::PoseStamped temp_pose;
        switch (flight_plan_section){
            case 0: // First section. Do nothing until current waypoint.
                if (_flight_plan.waypoints.at(i).x == _flight_plan.waypoints.at(_current_wp).x &&
                    _flight_plan.waypoints.at(i).y == _flight_plan.waypoints.at(_current_wp).y &&
                    _flight_plan.waypoints.at(i).z == _flight_plan.waypoints.at(_current_wp).z){
                        flight_plan_section = 1;
                } else {
                    break;
                }
            case 1: // Introduce waypoints between current waypoint and the first one of the solution (tactical)
                if (_flight_plan.waypoints.at(i).x == notification_.waypoints.front().x &&
                    _flight_plan.waypoints.at(i).y == notification_.waypoints.front().y &&
                    _flight_plan.waypoints.at(i).z == notification_.waypoints.front().z){
                        flight_plan_section = 2;
                } else {
                    temp_pose.pose.position.x = _flight_plan.waypoints.at(i).x;
                    temp_pose.pose.position.y = _flight_plan.waypoints.at(i).y;
                    temp_pose.pose.position.z = _flight_plan.waypoints.at(i).z;
                    temp_pose.header.stamp = _flight_plan.waypoints.at(i).stamp;
                    out_path.poses.push_back(temp_pose);
                }
                break;
            case 2: // Introduce the solution
                for(int i = 0; i < notification_.waypoints.size(); i++){
                    geometry_msgs::PoseStamped temp_pose;
                    temp_pose.pose.position.x = notification_.waypoints.at(i).x;
                    temp_pose.pose.position.y = notification_.waypoints.at(i).y;
                    temp_pose.pose.position.z = notification_.waypoints.at(i).z;
                    temp_pose.header.stamp = notification_.waypoints.at(i).stamp;
                    out_path.poses.push_back(temp_pose);
                }
                if (merge_to_the_end){
                    flight_plan_section = 3;
                } else {
                    i = _flight_plan.waypoints.size(); // TODO: Check using examples
                }
            case 3: // Do nothing until matching the solution with the flight plan
                if (_flight_plan.waypoints.at(i).x == notification_.waypoints.back().x &&
                    _flight_plan.waypoints.at(i).y == notification_.waypoints.back().y &&
                    _flight_plan.waypoints.at(i).z == notification_.waypoints.back().z){
                        flight_plan_section = 4;
                    }
                break;
            case 4: // Introduce the rest of the flight plan
                temp_pose.pose.position.x = _flight_plan.waypoints.at(i).x;
                temp_pose.pose.position.y = _flight_plan.waypoints.at(i).y;
                temp_pose.pose.position.z = _flight_plan.waypoints.at(i).z;
                temp_pose.header.stamp = _flight_plan.waypoints.at(i).stamp;
                out_path.poses.push_back(temp_pose);
                break;
        }
    }

    for (auto i : out_path.poses) std::cout << i << std::endl;

    return out_path;
}

gauss_msgs::Operation getOperation(const int &_uav_id){
    gauss_msgs::Operation out_operation;
    gauss_msgs::ReadOperation operation_msg;
    operation_msg.request.uav_ids.push_back(uav_id_);
    if (!read_operation_client_.call(operation_msg) || !operation_msg.response.success)
    {
        ROS_ERROR("Failed to read a flight plan");
    } else {
        out_operation = operation_msg.response.operation.front();
    }

    return out_operation;
}

gauss_msgs::PositionReport updatePositionReport(gauss_msgs::PositionReport &_position_report){
    // _position_report.confidence = ?
    _position_report.header = ual_pose_.header;
    // _position_report.heading = ?
    // _position_report.icao_address = ?
    _position_report.position.x = ual_pose_.pose.position.x;
    _position_report.position.y = ual_pose_.pose.position.y;
    _position_report.position.z = ual_pose_.pose.position.z;
    _position_report.position.stamp = ual_pose_.header.stamp; // ?
    _position_report.position.mandatory = 0; // ?
    _position_report.speed = sqrt(ual_vel_.twist.linear.x * ual_vel_.twist.linear.x + 
                                    ual_vel_.twist.linear.y * ual_vel_.twist.linear.y + 
                                    ual_vel_.twist.linear.z * ual_vel_.twist.linear.z);
    // _position_report.source == ?
    _position_report.uav_id = uav_id_;
    return _position_report;
}

int main(int _argc, char **_argv) {
    ros::init(_argc, _argv, "ualCommunication_node");

    upat_follower::UALCommunication ual_communication;
    int pub_rate;
    bool light;
    std::string ns_prefix;
    ros::param::param<int>("~uav_id", uav_id_, 0);
    ros::param::param<std::string>("~ns_prefix", ns_prefix, "uav_");
    ros::param::param<int>("~pub_rate", pub_rate, 30);
    ros::param::param<bool>("~light", light, false);
    ros::Rate rate(pub_rate);   
    ros::NodeHandle nh;
    ros::Subscriber sub_state_ = nh.subscribe("/" + ns_prefix + std::to_string(uav_id_) + "/ual/state", 0, ualStateCb);
    ros::Subscriber sub_pose_ = nh.subscribe("/" + ns_prefix + std::to_string(uav_id_) + "/ual/pose", 0, ualPoseCb);
    ros::Subscriber sub_vel_ = nh.subscribe("/" + ns_prefix + std::to_string(uav_id_) + "/ual/velocity", 0, ualVelocityCb);
    ros::Subscriber sub_notification_ = nh.subscribe("/notification", 0, notificationCb);
    ros::Publisher pub_position_report_ = nh.advertise<gauss_msgs::PositionReport>("/gauss/position_report", 1);

    read_operation_client_ = nh.serviceClient<gauss_msgs::ReadOperation>("/gauss/read_operation");

    // Let UAL and MAVROS get ready
    if (light) {
        ROS_WARN("[UPAT] UAL %d not ready!", uav_id_);
        while (ual_state_.state == 0) {
            ros::spinOnce();
            sleep(1.0);
        }
        ROS_INFO("[UPAT] UAL %d ready!", uav_id_);
    } else {
        ROS_WARN("[UPAT] UAL %d and MAVROS not ready!", uav_id_);
        while (!ros::service::exists("/" + ns_prefix + std::to_string(uav_id_) + "/mavros/param/get", false) || ual_state_.state == 0) {
            ros::spinOnce();
            sleep(1.0);
        }

        while (!ual_communication.setPX4Param("MPC_XY_VEL_MAX", ual_communication.max_vxy_)) sleep(1.0);
        while (!ual_communication.setPX4Param("MPC_Z_VEL_MAX_UP", ual_communication.max_vz_up_)) sleep(1.0);
        while (!ual_communication.setPX4Param("MPC_Z_VEL_MAX_DN", ual_communication.max_vz_dn_)) sleep(1.0);
        ROS_INFO("[UPAT] UAL %d and MAVROS ready!", uav_id_);
    }
    sleep(1.0);

    gauss_msgs::Operation uav_operation = getOperation(uav_id_);
    nav_msgs::Path res_path, alternative_path;
    res_path.header.frame_id = "map"; // world
    std::vector<double> res_times, alternative_times;
    for (int i = 0; i < uav_operation.flight_plan.waypoints.size(); i++){
        geometry_msgs::PoseStamped temp_pose;
        temp_pose.pose.position.x = uav_operation.flight_plan.waypoints.at(i).x;
        temp_pose.pose.position.y = uav_operation.flight_plan.waypoints.at(i).y;
        temp_pose.pose.position.z = uav_operation.flight_plan.waypoints.at(i).z;
        res_path.poses.push_back(temp_pose);
        res_times.push_back(uav_operation.flight_plan.waypoints.at(i).stamp.toSec());
    }

    ual_communication.flag_redo_ = true;
    ual_communication.init_path_.poses.clear();
    ual_communication.init_path_.poses = res_path.poses;
    ual_communication.init_path_.header = res_path.header;
    ual_communication.times_.clear();
    ual_communication.times_ = res_times;
    
    bool once = false;
    gauss_msgs::PositionReport position_report;
    while (ros::ok()) {
        ual_communication.runMission();
        ual_communication.callVisualization();
    
        if(new_notification_){
            alternative_path.poses.clear();
            alternative_times.clear();
            uav_operation = getOperation(uav_id_); // To get current_wp updated
            alternative_path = mergeFlightPlan(uav_operation.flight_plan, notification_.threat, notification_.maneuver_type, uav_operation.current_wp);
            for (auto i : alternative_path.poses){
                alternative_times.push_back(i.header.stamp.toSec());
            }
            ual_communication.flag_redo_ = true;
            ual_communication.init_path_.poses.clear();
            ual_communication.init_path_.poses = alternative_path.poses;
            ual_communication.init_path_.header = res_path.header;
            ual_communication.times_.clear();
            ual_communication.times_ = alternative_times;

            new_notification_ = false;
        }

        pub_position_report_.publish(updatePositionReport(position_report));

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}