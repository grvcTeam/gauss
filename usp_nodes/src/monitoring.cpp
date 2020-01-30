#include <ros/ros.h>
#include <gauss_msgs/Alert.h>
#include <gauss_msgs/ReadGeofences.h>
#include <gauss_msgs/ReadTracks.h>


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


    // Auxilary variables
    double rate;

    ros::NodeHandle nh_;

    // Subscribers


    // Publisher

    // Timer
    ros::Timer timer_sub_;

    // Server

    // Client
    ros::ServiceClient alert_client_;
    ros::ServiceClient read_geofence_client_;
    ros::ServiceClient read_track_client_;
};

// Monitoring Constructor
Monitoring::Monitoring()
{
    // Read parameters
    nh_.param("/gauss/monitoring_rate",rate,0.5);


    // Initialization


    // Publish

    // Subscribe

    // Server

    // Client
    read_track_client_ = nh_.serviceClient<gauss_msgs::ReadTracks>("/gauss/readTrack");
    read_geofence_client_ = nh_.serviceClient<gauss_msgs::ReadGeofences>("/gauss_msgs/readGeofence");
    alert_client_ = nh_.serviceClient<gauss_msgs::Alert>("/gauss/alert");

    // Timer
    timer_sub_=nh_.createTimer(ros::Duration(1.0/rate),&Monitoring::timerCallback,this);

    ROS_INFO("Started Monitoring node!");
}

// Auxilary methods

// Timer Callback
void Monitoring::timerCallback(const ros::TimerEvent &)
{
    ROS_INFO("Monitoring");


}



// MAIN function
int main(int argc, char *argv[])
{
    ros::init(argc,argv,"monitoring");

    // Create a Monitoring object
    Monitoring *monitoring = new Monitoring();

    ros::spin();
}
