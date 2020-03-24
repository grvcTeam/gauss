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

uav_abstraction_layer::State ual_state_;

void ualStateCb(const uav_abstraction_layer::State msg) {
    ual_state_.state = msg.state;
}

int main(int _argc, char **_argv) {
    ros::init(_argc, _argv, "ualCommunication_node");

    upat_follower::UALCommunication ual_communication;
    int pub_rate, uav_id;
    bool light;
    std::string ns_prefix;
    ros::param::param<int>("~uav_id", uav_id, 1);
    ros::param::param<std::string>("~ns_prefix", ns_prefix, "uav_");
    ros::param::param<int>("~pub_rate", pub_rate, 30);
    ros::param::param<bool>("~light", light, false);
    ros::Rate rate(pub_rate);
    ros::NodeHandle nh;
    ros::Subscriber sub_state_ = nh.subscribe("/" + ns_prefix + std::to_string(uav_id) + "/ual/state", 0, ualStateCb);

    // Let UAL and MAVROS get ready
    if (light) {
        ROS_WARN("[UPAT] UAL %d not ready!", uav_id);
        while (ual_state_.state == 0) {
            ros::spinOnce();
            sleep(1.0);
        }
        ROS_INFO("[UPAT] UAL %d ready!", uav_id);
    } else {
        ROS_WARN("[UPAT] UAL %d and MAVROS not ready!", uav_id);
        while (!ros::service::exists("/" + ns_prefix + std::to_string(uav_id) + "/mavros/param/get", false) || ual_state_.state == 0) {
            ros::spinOnce();
            sleep(1.0);
        }

        while (!ual_communication.setPX4Param("MPC_XY_VEL_MAX", ual_communication.max_vxy_)) sleep(1.0);
        while (!ual_communication.setPX4Param("MPC_Z_VEL_MAX_UP", ual_communication.max_vz_up_)) sleep(1.0);
        while (!ual_communication.setPX4Param("MPC_Z_VEL_MAX_DN", ual_communication.max_vz_dn_)) sleep(1.0);
        ROS_INFO("[UPAT] UAL %d and MAVROS ready!", uav_id);
    }
    sleep(1.0);

    while (ros::ok()) {
        ual_communication.runMission();
        ual_communication.callVisualization();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}