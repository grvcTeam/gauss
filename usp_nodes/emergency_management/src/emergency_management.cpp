#include <ros/ros.h>
#include "gauss_msgs/Threats.h"


// Class definition
class EmergencyManagement
{
public:
    EmergencyManagement();

private:
    // Topic Callbacks

    // Service Callbacks
    bool alertEmergencyCB(gauss_msgs::Threats::Request &req, gauss_msgs::Threats::Response &res);

    // Auxilary methods


    // Auxilary variables

    ros::NodeHandle nh_;


    // Subscribers

    // Publisher

    // Timer

    // Server
    ros::ServiceServer alert_server_;
};

// EmergencyManagement Constructor
EmergencyManagement::EmergencyManagement()
{
    // Read parameters
    //nh_.param("desired_altitude",desired_altitude,0.5);


    // Initialization


    // Publish

    // Subscribe

    // Server
    alert_server_=nh_.advertiseService("/gauss/threats",&EmergencyManagement::alertEmergencyCB,this);

    // Clients

    ROS_INFO("Started EmergencyManagement node!");
}

// Auxilary methods


// Alert callback
 bool EmergencyManagement::alertEmergencyCB(gauss_msgs::Threats::Request &req, gauss_msgs::Threats::Response &res)
 {
     int num = req.threats.size();
     ROS_INFO("recevied %d threats from monitoring", num);
     for (int i=0; i<num; i++)
     {
         if (req.threats.at(i).threat_id==req.threats.at(i).GEOFENCE_CONFLICT)
             ROS_INFO("Geofence conflict threat: UAV %d, geofence %d, time %d",req.threats.at(i).uav_ids.at(0), req.threats.at(i).geofence_ids.at(0), req.threats.at(i).times.at(0).sec);
         if (req.threats.at(i).threat_id==req.threats.at(i).GEOFENCE_INTRUSION)
             ROS_INFO("Geofence intrusion threat: UAV %d, geofence %d, time %d",req.threats.at(i).uav_ids.at(0), req.threats.at(i).geofence_ids.at(0), req.threats.at(i).times.at(0).sec);
         if (req.threats.at(i).threat_id==req.threats.at(i).UAS_IN_CV)
             ROS_INFO("UAS in CV threat: UAV %d, time %d",req.threats.at(i).uav_ids.at(0), req.threats.at(i).times.at(0).sec);
         if (req.threats.at(i).threat_id==req.threats.at(i).UAS_OUT_OV)
             ROS_INFO("UAS out OV threat: UAV %d, time %d",req.threats.at(i).uav_ids.at(0), req.threats.at(i).times.at(0).sec);
         if (req.threats.at(i).threat_id==req.threats.at(i).LOSS_OF_SEPARATION)
             ROS_INFO("UAS LOSS OF SEPARATION threat: UAVs %d and %d, times %d and %d",req.threats.at(i).uav_ids.at(0), req.threats.at(i).uav_ids.at(1),req.threats.at(i).times.at(0).sec,req.threats.at(i).times.at(1).sec);



     }
     res.success=true;
     return true;
 }



// MAIN function
int main(int argc, char *argv[])
{
    ros::init(argc,argv,"emergency_management");

    // Create a EmergencyManagement object
    EmergencyManagement *emergency_management = new EmergencyManagement();

    ros::spin();
}
