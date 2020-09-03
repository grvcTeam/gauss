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
#include <gauss_msgs/Notification.h>

int uav_id_;
bool new_notification_ = false;
uav_abstraction_layer::State ual_state_;
gauss_msgs::Notification notification_;

void ualStateCb(const uav_abstraction_layer::State msg) {
    ual_state_.state = msg.state;
}

void notificationCb(const gauss_msgs::Notification msg) {
    if (msg.uav_id == uav_id_ && !new_notification_){
        notification_ = msg;
        new_notification_ = true;
        // std::cout << notification_ << std::endl;
    }
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
    ros::Subscriber sub_notification_ = nh.subscribe("/notification", 0, notificationCb);

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

    ros::ServiceClient read_plan_client_;
    read_plan_client_ = nh.serviceClient<gauss_msgs::ReadFlightPlan>("/gauss/read_flight_plan");
    gauss_msgs::ReadFlightPlan plan_msg;
    plan_msg.request.uav_ids.push_back(uav_id_);
    if (!read_plan_client_.call(plan_msg) || !plan_msg.response.success)
    {
        ROS_ERROR("Failed to read a flight plan");
        return false;
    }
    nav_msgs::Path res_path, alternative_path;
    res_path.header.frame_id = "map"; // world
    std::vector<double> res_times, alternative_times;
    for (int i = 0; i < plan_msg.response.plans.front().waypoints.size(); i++){
        geometry_msgs::PoseStamped temp_pose;
        temp_pose.pose.position.x = plan_msg.response.plans.front().waypoints.at(i).x;
        temp_pose.pose.position.y = plan_msg.response.plans.front().waypoints.at(i).y;
        temp_pose.pose.position.z = plan_msg.response.plans.front().waypoints.at(i).z;
        res_path.poses.push_back(temp_pose);
        res_times.push_back(plan_msg.response.plans.front().waypoints.at(i).stamp.toSec());
    }

    ual_communication.flag_redo_ = true;
    ual_communication.init_path_.poses.clear();
    ual_communication.init_path_.poses = res_path.poses;
    ual_communication.init_path_.header = res_path.header;
    ual_communication.times_.clear();
    ual_communication.times_ = res_times;
    
    bool once = false;
    while (ros::ok()) {
        ual_communication.runMission();
        ual_communication.callVisualization();
        if(new_notification_ && !once){
            for(int i = 0; i < notification_.waypoints.size(); i++){
                geometry_msgs::PoseStamped temp_pose;
                temp_pose.pose.position.x = notification_.waypoints.at(i).x;
                temp_pose.pose.position.y = notification_.waypoints.at(i).y;
                temp_pose.pose.position.z = notification_.waypoints.at(i).z;
                alternative_path.poses.push_back(temp_pose);
                alternative_times.push_back(notification_.waypoints.at(i).stamp.toSec());
            }
            bool change_path_reference = false;
            for (int i = 0; i < plan_msg.response.plans.front().waypoints.size(); i++){
                if (plan_msg.response.plans.front().waypoints.at(i).x == notification_.waypoints.back().x &&
                    plan_msg.response.plans.front().waypoints.at(i).y == notification_.waypoints.back().y &&
                    plan_msg.response.plans.front().waypoints.at(i).z == notification_.waypoints.back().z){
                        change_path_reference = true;
                }
                
                if (change_path_reference){
                    geometry_msgs::PoseStamped temp_pose;
                    temp_pose.pose.position.x = plan_msg.response.plans.front().waypoints.at(i).x;
                    temp_pose.pose.position.y = plan_msg.response.plans.front().waypoints.at(i).y;
                    temp_pose.pose.position.z = plan_msg.response.plans.front().waypoints.at(i).z;
                    alternative_path.poses.push_back(temp_pose);
                    alternative_times.push_back(plan_msg.response.plans.front().waypoints.at(i).stamp.toSec());
                }
            }
            once = true;

            ual_communication.flag_redo_ = true;
            ual_communication.init_path_.poses.clear();
            ual_communication.init_path_.poses = alternative_path.poses;
            ual_communication.init_path_.header = res_path.header;
            ual_communication.times_.clear();
            ual_communication.times_ = alternative_times;

            // new_notification_ = false;
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}