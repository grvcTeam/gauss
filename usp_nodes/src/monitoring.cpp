#include <ros/ros.h>
#include <gauss_msgs/Track.h>
#include <gauss_msgs/ReadTrack.h>
#include <gauss_msgs/ReadGeofence.h>
#include <gauss_msgs/ReadPlan.h>


// Class definition
class Monitoring
{
public:
    Monitoring();

private:
    // Topic Callbacks

    // Service Callbacks

    // Timer Callbacks
    void timerCallback(const ros::TimerEvent&);

    // Auxilary methods
    void conflictDetection();

    // Auxilary variables
    double rate;
    gauss_msgs::Plan plans[];
    gauss_msgs::Track tracks[];
    gauss_msgs::Geofence geofences[];

    ros::NodeHandle nh_;

    // Subscribers


    // Publisher

    // Timer
    ros::Timer timer_sub_;

    // Server

    // Client
    ros::ServiceClient read_track_client_;
    ros::ServiceClient read_geofence_client;
    ros::ServiceClient read_plan_client_;

};

// Monitoring Constructor
Monitoring::Monitoring()
{
    // Read parameters
    nh_.param("/gauss/monitoring_rate",rate,1.0);


    // Initialization


    // Publish

    // Subscribe

    // Server

    // Client
    read_track_client_ = nh_.serviceClient<gauss_msgs::ReadTrack>("/gauss/readTrack");
    read_geofence_client = nh_.serviceClient<gauss_msgs::ReadGeofence>("/gauss_msgs/readGeofence");
    read_plan_client_ = nh_.serviceClient<gauss_msgs::ReadPlan>("/gauss/readPlan");

    // Timer
    timer_sub_=nh_.createTimer(ros::Duration(1.0/rate),&Monitoring::timerCallback,this);

    ROS_INFO("Started Monitoring node!");
}

// Timer Callback
void Monitoring::timerCallback(const ros::TimerEvent &)
{
    // Leer tracks de DB, flight de DB y Geofences de DB
    // Algoritmo de Conflict detection

    // si va a ocurrir un conflicto, se llama al servicio tactical deconfliction

}



// MAIN function
int main(int argc, char *argv[])
{
    ros::init(argc,argv,"monitoring");

    // Create a Monitoring object
    Monitoring *monitoring = new Monitoring();

    ros::spin();
}
