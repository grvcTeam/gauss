#include <ros/ros.h>
#include <gauss_msgs/Traffic.h>

// Class definition
class TrafficInformation
{
public:
    TrafficInformation();

private:
    // Topic Callbacks
    void trafficCB(const gauss_msgs::Traffic::ConstPtr& msg);

    // Service Callbacks

    // Auxilary variables


    ros::NodeHandle nh_;


    // Subscribers
    ros::Subscriber traffic_sub_;

    // Publisher
    ros::Publisher traffic_pub_;
    // Different formats TBD

    // Timer

    // Server
};

// TrafficInformation Constructor
TrafficInformation::TrafficInformation()
{
    // Read parameters
    //nh_.param("desired_altitude",desired_altitude,0.5);


    // Initialization


    // Publish
    traffic_pub_ = nh_.advertise<gauss_msgs::Traffic>("/gauss/traffic_out",1);

    // Subscribe
    traffic_sub_=nh_.subscribe<gauss_msgs::Traffic>("/gauss/traffic",1,&TrafficInformation::trafficCB,this);

    // Server

    ROS_INFO("Started TrafficInformation node!");
}



// traffic callback
void TrafficInformation::trafficCB(const gauss_msgs::Traffic::ConstPtr &msg)
{
    // Lee track y traffic y los escribe en el formato correcto y los publica
    // Lee warnings y los escribe en el formato correcto y los publica
    traffic_pub_.publish(msg); // TBD
}


// MAIN function
int main(int argc, char *argv[])
{
    ros::init(argc,argv,"traffic_information");

    // Create a TrafficInformation object
    TrafficInformation *traffic_information = new TrafficInformation();

    ros::spin();
}
