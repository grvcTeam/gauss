#include <gauss_msgs/ReadOperation.h>
#include <gauss_msgs_mqtt/ADSBSurveillance.h>
#include <gauss_msgs_mqtt/RPAStateInfo.h>
#include <gauss_msgs_mqtt/RPSChangeFlightStatus.h>
#include <gauss_msgs_mqtt/RPSFlightPlanAccept.h>
#include <gauss_msgs_mqtt/UTMAlert.h>
#include <gauss_msgs_mqtt/UTMAlternativeFlightPlan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <upat_follower/ual_communication.h>

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <boost/bind.hpp>

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
class USPUALBridge {
   public:
    USPUALBridge();
    // UPAT Follower
    upat_follower::UALCommunication ual_communication_;

   private:
    // Topic Callbacks
    //void RPAStateCB(const gauss_msgs_mqtt::RPAStateInfo::ConstPtr& msg); // RPS -> UTM
    //void callbackSyncPoseVelocity(const geometry_msgs::PoseStampedConstPtr& pose_ptr, const geometry_msgs::TwistStampedConstPtr&  velocity_ptr);
    void callbackPose(const geometry_msgs::PoseStampedConstPtr& pose_ptr);
    void callbackState(const uav_abstraction_layer::State& state_ptr);
    void callbackVelocity(const geometry_msgs::TwistStampedConstPtr& velocity_ptr);
    void callbackAlternativeFlightPlan(const gauss_msgs_mqtt::UTMAlternativeFlightPlan& alt_flight_plan);
    // Auxilary variables
    double rate_;
    int uav_id_;
    std::string icao_address_;    
    ros::NodeHandle nh_;

    // Subscribers
    //message_filters::Subscriber<geometry_msgs::PoseStamped> pose_sub_;
    //message_filters::Subscriber<geometry_msgs::TwistStamped> velocity_sub_;
    ros::Subscriber pose_sub_;
    ros::Subscriber state_sub_;
    ros::Subscriber velocity_sub_;
    ros::Subscriber alternative_flight_plan_sub_;

    //message_filters::Synchronizer<ApproxTimeSyncPolicy> sync_;

    // Publisher
    ros::Publisher rpa_state_info_pub_;
    ros::Publisher adsb_surveillance_pub_;

    bool available_velocity_msg_;
    uav_abstraction_layer::State latest_state_msgs_;
    geometry_msgs::TwistStampedConstPtr latest_velocity_msg_ptr_;
    geometry_msgs::PoseStampedConstPtr latest_pose_msg_ptr_;

    // Variables for cartesian to geographic conversion
    double lat0_, lon0_;
    GeographicLib::Geocentric earth_;
    GeographicLib::LocalCartesian proj_;
};

// USPManager Constructor
USPUALBridge::USPUALBridge() : available_velocity_msg_(false),
                               nh_(),
                               lat0_(REFERENCE_LATITUDE),
                               lon0_(REFERENCE_LONGITUDE),
                               earth_(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f()),
                               proj_(lat0_, lon0_, 0, earth_)  //,
//pose_sub_(nh_, "ual/pose", 1),
//velocity_sub_(nh_, "ual/velocity", 1),
//sync_(ApproxTimeSyncPolicy(10), pose_sub_, velocity_sub_)
{
    // Read parameters
    nh_.param("/gauss/monitoring_rate", rate_, 0.5);
    ros::param::param<int>("~uav_id", uav_id_, 0);
    std::string ns_prefix;
    ros::param::param<std::string>("~ns_prefix", ns_prefix, "uav_");

    // Initialization
    //sync_.registerCallback(boost::bind(&USPUALBridge::callbackSyncPoseVelocity, this, _1, _2));

    // Publish
    //rpacommands_pub_ = nh_.advertise<gauss_msgs::Notification>("/gauss/commands",1);  //TBD message to UAVs
    rpa_state_info_pub_ = nh_.advertise<gauss_msgs_mqtt::RPAStateInfo>("/gauss/rpa_state", 1);
    adsb_surveillance_pub_ = nh_.advertise<gauss_msgs_mqtt::ADSBSurveillance>("/gauss/adsb", 1);

    // Subscribe
    //notification_sub_ = nh_.subscribe<gauss_msgs::Notification>("/gauss/notification",1,&USPManager::notificationCB,this);
    state_sub_ = nh_.subscribe("/" + ns_prefix + std::to_string(uav_id_) + "/ual/state", 1, &USPUALBridge::callbackState, this);
    pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/" + ns_prefix + std::to_string(uav_id_) + "/ual/pose", 1, &USPUALBridge::callbackPose, this);
    velocity_sub_ = nh_.subscribe<geometry_msgs::TwistStamped>("/" + ns_prefix + std::to_string(uav_id_) + "/ual/velocity", 1, &USPUALBridge::callbackVelocity, this);
    alternative_flight_plan_sub_ = nh_.subscribe("/gauss/alternative_flight_plan", 1, &USPUALBridge::callbackAlternativeFlightPlan, this);

    // Service client
    ros::ServiceClient read_operation_client = nh_.serviceClient<gauss_msgs::ReadOperation>("/gauss/read_operation");

    // Wait to MAVROS, UAL and UPAT be ready
    ROS_WARN("[UPAT] UAL %d and MAVROS not ready!", uav_id_);
    while (!ros::service::exists("/" + ns_prefix + std::to_string(uav_id_) + "/mavros/param/get", false) || latest_state_msgs_.state == uav_abstraction_layer::State::UNINITIALIZED) {
        ros::spinOnce();
        sleep(1.0);
    }
    while (!ual_communication_.setPX4Param("MPC_XY_VEL_MAX", ual_communication_.max_vxy_)) sleep(1.0);
    while (!ual_communication_.setPX4Param("MPC_Z_VEL_MAX_UP", ual_communication_.max_vz_up_)) sleep(1.0);
    while (!ual_communication_.setPX4Param("MPC_Z_VEL_MAX_DN", ual_communication_.max_vz_dn_)) sleep(1.0);
    ROS_INFO("[UPAT] UAL %d and MAVROS ready!", uav_id_);
    // Get initial flight plan for uav_id
    gauss_msgs::ReadOperation operation_msg;
    operation_msg.request.uav_ids.push_back(uav_id_);
    if (!read_operation_client.call(operation_msg) || !operation_msg.response.success) {
        ROS_ERROR("Failed to read a flight plan");
    } else {
        icao_address_ = operation_msg.response.operation.front().icao_address;
        nav_msgs::Path res_path;
        res_path.header.frame_id = "map";  // world
        std::vector<double> res_times, alternative_times;
        // Translate it to nav_msgs::Path
        for (int i = 0; i < operation_msg.response.operation.front().flight_plan.waypoints.size(); i++) {
            geometry_msgs::PoseStamped temp_pose;
            temp_pose.pose.position.x = operation_msg.response.operation.front().flight_plan.waypoints.at(i).x;
            temp_pose.pose.position.y = operation_msg.response.operation.front().flight_plan.waypoints.at(i).y;
            temp_pose.pose.position.z = operation_msg.response.operation.front().flight_plan.waypoints.at(i).z;
            res_path.poses.push_back(temp_pose);
            res_times.push_back(operation_msg.response.operation.front().flight_plan.waypoints.at(i).stamp.toSec());
        }
        // Rewritte init trajectory for upat_follower
        ual_communication_.flag_redo_ = true;
        ual_communication_.init_path_.poses.clear();
        ual_communication_.init_path_.poses = res_path.poses;
        ual_communication_.init_path_.header = res_path.header;
        ual_communication_.times_.clear();
        ual_communication_.times_ = res_times;
    }

    ROS_INFO("Started usp_ual_bridge node!");
}

/*
void USPUALBridge::callbackSyncPoseVelocity(const geometry_msgs::PoseStampedConstPtr& pose_ptr, const geometry_msgs::TwistStampedConstPtr&  velocity_ptr)
{
    std::cout << "Pose stamp " << pose_ptr->header.stamp << std::endl;
    std::cout << "Velocity stamp " << velocity_ptr->header.stamp << std::endl;
}*/

void USPUALBridge::callbackAlternativeFlightPlan(const gauss_msgs_mqtt::UTMAlternativeFlightPlan& alt_flight_plan){
    if (alt_flight_plan.icao == icao_address_){
        std::string flight_plan_string = alt_flight_plan.new_flight_plan;
        // Replace characters
        std::string replace_str = "],[";
        for (int i = flight_plan_string.find(replace_str); i >= 0; i = flight_plan_string.find(replace_str))
            flight_plan_string.replace(i, replace_str.size(), " ");
        // Remove useless characters
        flight_plan_string.erase(std::remove(flight_plan_string.begin(), flight_plan_string.end(), ','),  flight_plan_string.end());
        flight_plan_string.erase(std::remove(flight_plan_string.begin(), flight_plan_string.end(), '['),  flight_plan_string.end());
        flight_plan_string.erase(std::remove(flight_plan_string.begin(), flight_plan_string.end(), ']'),  flight_plan_string.end());
        flight_plan_string.erase(std::remove(flight_plan_string.begin(), flight_plan_string.end(), '\n'), flight_plan_string.end());
        flight_plan_string.erase(std::remove(flight_plan_string.begin(), flight_plan_string.end(), '\r'), flight_plan_string.end());
        // Clear follower input
        ual_communication_.times_.clear();
        ual_communication_.flag_redo_ = true;
        ual_communication_.init_path_.poses.clear();
        ual_communication_.init_path_.header.frame_id = "map";
        // Save the value and substract it from the string
        std::string::size_type sz;
        while (flight_plan_string.size()>0){
            geometry_msgs::PoseStamped new_poses;
            double longitude, latitude, altitude;
            longitude = (double)std::stod(flight_plan_string, &sz);
            flight_plan_string = flight_plan_string.substr(sz);
            latitude = (double)std::stod(flight_plan_string, &sz);
            flight_plan_string = flight_plan_string.substr(sz);
            altitude = (double)std::stod(flight_plan_string, &sz);
            flight_plan_string = flight_plan_string.substr(sz);
            proj_.Forward(latitude, longitude, altitude, new_poses.pose.position.x, new_poses.pose.position.y, new_poses.pose.position.z);
            ual_communication_.init_path_.poses.push_back(new_poses);
            ual_communication_.times_.push_back((double)std::stod(flight_plan_string, &sz));
            flight_plan_string = flight_plan_string.substr(sz);
        }
    }    
}

void USPUALBridge::callbackPose(const geometry_msgs::PoseStampedConstPtr& pose_ptr) {
    // if (available_velocity_msg_) {
    gauss_msgs_mqtt::RPAStateInfo rpa_state_info_msg;

    double lat, lon, h;
    proj_.Reverse(pose_ptr->pose.position.x, pose_ptr->pose.position.y, pose_ptr->pose.position.z, lat, lon, h);

    rpa_state_info_msg.altitude = h;
    rpa_state_info_msg.latitude = lat;
    rpa_state_info_msg.longitude = lon;
    rpa_state_info_msg.timestamp = pose_ptr->header.stamp.toNSec() / 1000000;

    // TODO: Fill remaining fields of messages
    rpa_state_info_msg.icao = atoi(icao_address_.c_str());    

    rpa_state_info_pub_.publish(rpa_state_info_msg);
    // gauss_msgs_mqtt::ADSBSurveillance adsb_surveillance_msg;
    // adsb_surveillance_msg.altitude = h;
    // adsb_surveillance_msg.latitude = lat;
    // adsb_surveillance_msg.longitude = lon;
    // adsb_surveillance_pub_.publish(adsb_surveillance_msg);
    // }
}

void USPUALBridge::callbackState(const uav_abstraction_layer::State& state_ptr) {
    latest_state_msgs_ = state_ptr;
}

void USPUALBridge::callbackVelocity(const geometry_msgs::TwistStampedConstPtr& velocity_ptr) {
    latest_velocity_msg_ptr_ = velocity_ptr;
    available_velocity_msg_ = true;
}

// MAIN function
int main(int argc, char* argv[]) {
    ros::init(argc, argv, "usp_ual_bridge");

    // Create a USPManager object
    USPUALBridge* usp_ual_bridge = new USPUALBridge();

    ros::Rate rate(30.0);
    while (ros::ok()) {
        usp_ual_bridge->ual_communication_.runMission();
        ros::spinOnce();
        rate.sleep();
    }

    delete usp_ual_bridge;
}