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

#include <ros/ros.h>
#include <gauss_msgs/Notification.h>
#include <gauss_msgs/Notifications.h>

int uav_id_;
ros::Publisher pub_notification_;
gauss_msgs::Notification notification_;

bool notificationsCB(gauss_msgs::Notifications::Request &req, gauss_msgs::Notifications::Response &res){
    res.message = "Notification failed";
    res.success = false;
    for (auto i : req.notifications){
        if(i.uav_id == uav_id_){
            notification_ = i;
            notification_.maneuver_type = i.action;
            // std::cout << (int)notification_.threat.threat_type << "\n";
            notification_.threat.threat_type = i.threat.LOSS_OF_SEPARATION; 
            for (auto j : req.operations){
                if (j.uav_id == i.uav_id){
                    notification_.flight_plan = j.flight_plan;
                    notification_.current_wp = j.current_wp;
                }
            }
            // std::cout << (int)notification_.threat.threat_type << "\n";
            res.message = "Notification received";
            res.success = true;
            // std::cout << " o- o - o - o - o - o -o -o \n";
            // std::cout << notification_ << "\n";
            // std::cout << " o- o - o - o - o - o -o -o \n";
            pub_notification_.publish(notification_);
        } 
    }
    
    return true;
}


int main(int _argc, char **_argv) {
    ros::init(_argc, _argv, "notification_bridge_node");
    int pub_rate;
    ros::NodeHandle nh;
    std::string ns_prefix;
    ros::param::param<int>("~uav_id", uav_id_, 0);
    ros::param::param<std::string>("~ns_prefix", ns_prefix, "uav_");
    ros::param::param<int>("~pub_rate", pub_rate, 30);
    ros::Rate rate(pub_rate);   
    pub_notification_ = nh.advertise<gauss_msgs::Notification>("/notification", 1);
    ros::ServiceServer notification_server_ = nh.advertiseService("/gauss/notifications", notificationsCB);

    while (ros::ok()){
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}