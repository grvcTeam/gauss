#include <ros/ros.h>
//#include <mavros_msgs/ADSBVehicle.h>  //TBD message to/from UAVs
#include <gauss_msgs/PositionReport.h>
#include <gauss_msgs/Notification.h>
// #include <gauss_msgs/Alert.h>


// Class definition
class USPManager
{
public:
    USPManager();

private:
    // Topic Callbacks
    void notificationCB(const gauss_msgs::Notification::ConstPtr& msg);
    void RPAStatusCB(const gauss_msgs::Notification::ConstPtr& msg);        //TBD message from UAVs

    // Timer Callbacks
    void timerCallback(const ros::TimerEvent&);

    // Auxilary variables
    double rate;
    ros::NodeHandle nh_;

    // Timer
    ros::Timer timer_sub_;

    // Subscribers
    ros::Subscriber notification_sub_;
    ros::Subscriber rpaStatus_sub_;     //TBD message from UAVs

    // Clients
    ros::ServiceClient alert_client_;
    ros::ServiceClient FPreq_client_;

    // Publisher
    ros::Publisher rpacommands_pub_;       //TBD message to UAVs

};

// USPManager Constructor
USPManager::USPManager()
{
    // Read parameters
    nh_.param("/gauss/monitoring_rate",rate,0.5);

    // Initialization


    // Publish
    rpacommands_pub_=nh_.advertise<gauss_msgs::Notification>("/gauss/commands",1);  //TBD message to UAVs

    // Subscribe
    notification_sub_=nh_.subscribe<gauss_msgs::Notification>("/gauss/notification",1,&USPManager::notificationCB,this);
    rpaStatus_sub_=nh_.subscribe<gauss_msgs::Notification>("/gauss/rpa_state",1,&USPManager::RPAStatusCB,this); //TBD message from UAVs

    // Client
    // alert_client_ = nh_.serviceClient<gauss_msgs::Alert>("/gauss/alert");

    // Timer
    timer_sub_=nh_.createTimer(ros::Duration(1.0/rate),&USPManager::timerCallback,this);

    ROS_INFO("Started USPManager node!");
}



// Notification callback
void USPManager::notificationCB(const gauss_msgs::Notification::ConstPtr &msg)
{

}

// RPAStatus Callback TBD messge from UAVs
void USPManager::RPAStatusCB(const gauss_msgs::Notification::ConstPtr &msg)
{

}

// Timer Callback
void USPManager::timerCallback(const ros::TimerEvent &)
{
    ROS_INFO("GAUSS USP running");


}


// MAIN function
int main(int argc, char *argv[])
{
    ros::init(argc,argv,"USPManager");

    // Create a USPManager object
    USPManager *usp_manager = new USPManager();

    ros::spin();
}
