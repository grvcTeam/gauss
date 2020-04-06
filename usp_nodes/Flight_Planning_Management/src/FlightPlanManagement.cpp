#include <ros/ros.h>
#include <gauss_msgs/FlightPlanReq.h>
#include <gauss_msgs/Deconfliction.h>
#include <gauss_msgs/ReadFlightPlan.h>
#include <gauss_msgs/ReadGeofences.h>
#include <gauss_msgs/WriteFlightPlan.h>

// Class definition
class FlightPlanManager
{
public:
    FlightPlanManager();

private:
    // Topic Callbacks

    // Service Callbacks
    bool flightPlanReqCB(gauss_msgs::FlightPlanReq::Request &req, gauss_msgs::FlightPlanReq::Response &res);


    // Auxilary variables


    ros::NodeHandle nh_;


    // Subscribers

    // Publisher

    // Different formats TBD

    // Timer

    // Server
    ros::ServiceServer fpreq_server_;

    // Client
    ros::ServiceClient read_geofence_client_;
    ros::ServiceClient read_fp_client_;
    ros::ServiceClient write_fp_client_;
    ros::ServiceClient deconfliction_client_;
};

// TrafficInformation Constructor
FlightPlanManager::FlightPlanManager()
{
    // Read parameters
    //nh_.param("desired_altitude",desired_altitude,0.5);


    // Initialization


    // Publish

    // Subscribe

    // Server
    fpreq_server_=nh_.advertiseService("/gauss/fight_plan",&FlightPlanManager::flightPlanReqCB,this);

    // Clients
    read_geofence_client_ = nh_.serviceClient<gauss_msgs::ReadGeofences>("/gauss/read_geofences");
    read_fp_client_ = nh_.serviceClient<gauss_msgs::ReadFlightPlan>("/gauss/read_flight_plan");
    write_fp_client_ = nh_.serviceClient<gauss_msgs::WriteFlightPlan>("/gauss/write_flight_plan");
    deconfliction_client_ = nh_.serviceClient<gauss_msgs::Deconfliction>("/gauss/conflict_solver");


    ROS_INFO("Started flightplanreqCB node!");
}



// FPreq callback
bool FlightPlanManager::flightPlanReqCB(gauss_msgs::FlightPlanReq::Request &req, gauss_msgs::FlightPlanReq::Response &res)
{
    int id = req.uav_id;
    int type = req.type;
    gauss_msgs::Waypoint *plan;

    return true;
}


// MAIN function
int main(int argc, char *argv[])
{
    ros::init(argc,argv,"flight_plan_manager");

    // Create a FlightPlanManager object
    FlightPlanManager *flight_plan_manager = new FlightPlanManager();

    ros::spin();
}
