#include <ros/ros.h>
#include "gauss_msgs/Threats.h"
#include "gauss_msgs/Deconfliction.h"
#include <list>

using namespace std;


// Class definition
class EmergencyManagement
{
public:
    EmergencyManagement();

private:
    // Topic Callbacks

    // Service Callbacks
    bool alertEmergencyCB(gauss_msgs::Threats::Request &req, gauss_msgs::Threats::Response &res);
    // Timer Callbacks
    void timerCallback(const ros::TimerEvent&);

    // Auxilary methods


    // Auxilary variables

    ros::NodeHandle nh_;
    list<gauss_msgs::Threat> lista;

    


    // Subscribers

    // Publisher
    ros::Publisher dec_pub_;

    // Timer
    ros::Timer timer_sub_;

    // Server
    ros::ServiceServer alert_server_;

    // Client
    ros::ServiceClient deconfliction_client_;
};

// EmergencyManagement Constructor
EmergencyManagement::EmergencyManagement()
{
    // Read parameters
    //nh_.param("desired_altitude",desired_altitude,0.5);


    // Initialization


    // Publish
     dec_pub_ = nh_.advertise<gauss_msgs::DeconflictionPlan>("/gauss/deconfliction_plan", 10);

    // Subscribe

    // Server
    alert_server_=nh_.advertiseService("/gauss/threats",&EmergencyManagement::alertEmergencyCB,this);

    // Clients
    deconfliction_client_=nh_.serviceClient<gauss_msgs::Deconfliction>("/gauss/tactical_deconfliction");
    
    timer_sub_=nh_.createTimer(ros::Duration(2),&EmergencyManagement::timerCallback,this);

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
         lista.push_back(req.threats.at(i));
     }

     res.success=true;
     return true;
 }

 void EmergencyManagement::timerCallback(const ros::TimerEvent &)
 {

     int num = lista.size();
     ROS_INFO("There are %d threats in the list", num);

     if (num>0)
     {
         gauss_msgs::Threat first_threat=lista.front();
         if (first_threat.threat_id==first_threat.GEOFENCE_CONFLICT)
             ROS_INFO("Geofence conflict threat: UAV %d, geofence %d, time %d",first_threat.uav_ids.at(0), first_threat.geofence_ids.at(0), first_threat.times.at(0).sec);
         if (first_threat.threat_id==first_threat.GEOFENCE_INTRUSION)
             ROS_INFO("Geofence intrusion threat: UAV %d, geofence %d, time %d",first_threat.uav_ids.at(0), first_threat.geofence_ids.at(0), first_threat.times.at(0).sec);
         if (first_threat.threat_id==first_threat.UAS_IN_CV)
             ROS_INFO("UAS in CV threat: UAV %d, time %d",first_threat.uav_ids.at(0), first_threat.times.at(0).sec);
         if (first_threat.threat_id==first_threat.UAS_OUT_OV)
             ROS_INFO("UAS out OV threat: UAV %d, time %d",first_threat.uav_ids.at(0), first_threat.times.at(0).sec);
         if (first_threat.threat_id==first_threat.LOSS_OF_SEPARATION)
         {
             ROS_INFO("UAS LOSS OF SEPARATION threat: UAVs %d and %d, times %d and %d",first_threat.uav_ids.at(0), first_threat.uav_ids.at(1),first_threat.times.at(0).sec,first_threat.times.at(1).sec);
             gauss_msgs::Deconfliction deconfliction_msg;

             deconfliction_msg.request.tactical=true;
             deconfliction_msg.request.threat.threat_id=deconfliction_msg.request.threat.LOSS_OF_SEPARATION;
             deconfliction_msg.request.threat.uav_ids.push_back(first_threat.uav_ids.at(0));
             deconfliction_msg.request.threat.uav_ids.push_back(first_threat.uav_ids.at(1));
             deconfliction_msg.request.threat.times.push_back(first_threat.times.at(0));
             deconfliction_msg.request.threat.times.push_back(first_threat.times.at(1));
             deconfliction_msg.request.threat.priority_ops.push_back(1);
             deconfliction_msg.request.threat.priority_ops.push_back(1);
             ROS_INFO("sending deconfliction request");
             if(!(deconfliction_client_.call(deconfliction_msg)) || !(deconfliction_msg.response.success))
             {
                 ROS_ERROR("Failed to deconfliction message");
                 return;
             }

             ROS_INFO("Received %d deconfliction plans",deconfliction_msg.response.deconfliction_plans.size());

             for(int j=0; j<deconfliction_msg.response.deconfliction_plans.size();j++)
             {
                 dec_pub_.publish(deconfliction_msg.response.deconfliction_plans.at(j));
              //   ROS_INFO("Plan %d: UAV %d new wp x %f y %f z %f",j+1,deconfliction_msg.response.deconfliction_plans.at(j).uav_id
                   //       deconfliction_msg.response.deconfliction_plans.at(j).waypoint_list.);

             }
         }
         lista.pop_front();
     }

 }


// MAIN function
int main(int argc, char *argv[])
{
    ros::init(argc,argv,"emergency_management");

    // Create a EmergencyManagement object
    EmergencyManagement *emergency_management = new EmergencyManagement();

    ros::spin();
}
