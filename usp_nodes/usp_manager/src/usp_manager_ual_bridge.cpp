#include <ros/ros.h>

#include <boost/bind.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <gauss_msgs_mqtt/ADSBSurveillance.h>
#include <gauss_msgs_mqtt/RPAStateInfo.h>
#include <gauss_msgs_mqtt/UTMAlert.h>
#include <gauss_msgs_mqtt/UTMAlternativeFlightPlan.h>
#include <gauss_msgs_mqtt/RPSFlightPlanAccept.h>
#include <gauss_msgs_mqtt/RPSChangeFlightStatus.h>

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

//#include <message_filters/subscriber.h>
//#include <message_filters/synchronizer.h>
//#include <message_filters/sync_policies/approximate_time.h>

// To solve compilation problems due to GeographicLib, follow the link below
// https://stackoverflow.com/questions/48169653/finding-geographiclib-in-cmake-on-debian

//typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, geometry_msgs::TwistStamped> ApproxTimeSyncPolicy;

#define ARENOSILLO_LATITUDE 37.101973
#define ARENOSILLO_LONGITUDE -6.736522

#define REFERENCE_LATITUDE ARENOSILLO_LATITUDE
#define REFERENCE_LONGITUDE ARENOSILLO_LONGITUDE


// Class definition
class USPUALBridge
{
public:
    USPUALBridge();

private:
    // Topic Callbacks
    //void RPAStateCB(const gauss_msgs_mqtt::RPAStateInfo::ConstPtr& msg); // RPS -> UTM
    //void callbackSyncPoseVelocity(const geometry_msgs::PoseStampedConstPtr& pose_ptr, const geometry_msgs::TwistStampedConstPtr&  velocity_ptr);
    void callbackPose(const geometry_msgs::PoseStampedConstPtr& pose_ptr);
    void callbackVelocity(const geometry_msgs::TwistStampedConstPtr& velocity_ptr);
    // Auxilary variables
    double rate;
    ros::NodeHandle nh_;

    // Subscribers
    //message_filters::Subscriber<geometry_msgs::PoseStamped> pose_sub_;
    //message_filters::Subscriber<geometry_msgs::TwistStamped> velocity_sub_;
    ros::Subscriber pose_sub_;
    ros::Subscriber velocity_sub_;

    //message_filters::Synchronizer<ApproxTimeSyncPolicy> sync_;

    // Publisher
    ros::Publisher rpa_state_info_pub_;
    ros::Publisher adsb_surveillance_pub_;

    bool available_velocity_msg_;
    geometry_msgs::TwistStampedConstPtr latest_velocity_msg_ptr_;
    geometry_msgs::PoseStampedConstPtr latest_pose_msg_ptr_;
    
    // Variables for cartesian to geographic conversion
    double lat0_, lon0_;
    GeographicLib::Geocentric earth_;
    GeographicLib::LocalCartesian proj_;
};

// USPManager Constructor
USPUALBridge::USPUALBridge():
available_velocity_msg_(false),
nh_(),
lat0_(REFERENCE_LATITUDE),
lon0_(REFERENCE_LONGITUDE),
earth_(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f()),
proj_(lat0_, lon0_, 0, earth_)//,
//pose_sub_(nh_, "ual/pose", 1),
//velocity_sub_(nh_, "ual/velocity", 1),
//sync_(ApproxTimeSyncPolicy(10), pose_sub_, velocity_sub_)
{
    // Read parameters
    nh_.param("/gauss/monitoring_rate",rate,0.5);

    // Initialization
    //sync_.registerCallback(boost::bind(&USPUALBridge::callbackSyncPoseVelocity, this, _1, _2));
    
    // Publish
    //rpacommands_pub_ = nh_.advertise<gauss_msgs::Notification>("/gauss/commands",1);  //TBD message to UAVs
    rpa_state_info_pub_ = nh_.advertise<gauss_msgs_mqtt::RPAStateInfo>("gauss/rpa_state", 1);
    adsb_surveillance_pub_ = nh_.advertise<gauss_msgs_mqtt::ADSBSurveillance>("gauss/adsb", 1);

    // Subscribe
    //notification_sub_ = nh_.subscribe<gauss_msgs::Notification>("/gauss/notification",1,&USPManager::notificationCB,this);
    pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("ual/pose", 1, &USPUALBridge::callbackPose, this);
    velocity_sub_ = nh_.subscribe<geometry_msgs::TwistStamped>("ual/velocity", 1, &USPUALBridge::callbackVelocity, this);

    ROS_INFO("Started usp_ual_bridge node!");
}

/*
void USPUALBridge::callbackSyncPoseVelocity(const geometry_msgs::PoseStampedConstPtr& pose_ptr, const geometry_msgs::TwistStampedConstPtr&  velocity_ptr)
{
    std::cout << "Pose stamp " << pose_ptr->header.stamp << std::endl;
    std::cout << "Velocity stamp " << velocity_ptr->header.stamp << std::endl;
}*/

void USPUALBridge::callbackPose(const geometry_msgs::PoseStampedConstPtr& pose_ptr)
{
    if (available_velocity_msg_)
    {
        gauss_msgs_mqtt::RPAStateInfo rpa_state_info_msg;
        gauss_msgs_mqtt::ADSBSurveillance adsb_surveillance_msg;

        double lat,lon,h;
        proj_.Reverse(pose_ptr->pose.position.x, pose_ptr->pose.position.y, pose_ptr->pose.position.z, lat, lon, h);

        rpa_state_info_msg.altitude = h;
        rpa_state_info_msg.latitude = lat;
        rpa_state_info_msg.longitude = lon;
        rpa_state_info_msg.timestamp = pose_ptr->header.stamp.toNSec()/1000000;

        adsb_surveillance_msg.altitude = h;
        adsb_surveillance_msg.latitude = lat;
        adsb_surveillance_msg.longitude = lon;

        // TODO: Fill remaining fields of messages

        rpa_state_info_pub_.publish(rpa_state_info_msg);
        adsb_surveillance_pub_.publish(adsb_surveillance_msg);
    }
}

void USPUALBridge::callbackVelocity(const geometry_msgs::TwistStampedConstPtr& velocity_ptr)
{
    latest_velocity_msg_ptr_ = velocity_ptr;
    available_velocity_msg_ = true;
}

// MAIN function
int main(int argc, char *argv[])
{
    ros::init(argc,argv,"usp_ual_bridge");

    // Create a USPManager object
    USPUALBridge *usp_ual_bridge = new USPUALBridge();

    ros::spin();

    delete usp_ual_bridge;
}