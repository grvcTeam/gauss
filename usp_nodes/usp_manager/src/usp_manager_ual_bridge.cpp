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
#include <sys/stat.h>
#include <upat_follower/ual_communication.h>

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <boost/bind.hpp>
#include <fstream>

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
    void callbackVelocity(const geometry_msgs::TwistStamped& velocity);
    void callbackAlternativeFlightPlan(const gauss_msgs_mqtt::UTMAlternativeFlightPlan& alt_flight_plan);
    // Auxiliary methods
    void csvStore(const gauss_msgs_mqtt::RPAStateInfo& rpa_data);
    // Auxiliary variables
    int uav_id_;
    double rate_;
    bool store_data_;
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
    geometry_msgs::TwistStamped latest_velocity_msg_;
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
    ros::param::param<bool>("~store_data", store_data_, false);

    // Initialization
    //sync_.registerCallback(boost::bind(&USPUALBridge::callbackSyncPoseVelocity, this, _1, _2));

    // Publish
    //rpacommands_pub_ = nh_.advertise<gauss_msgs::Notification>("/gauss/commands",1);  //TBD message to UAVs
    rpa_state_info_pub_ = nh_.advertise<gauss_msgs_mqtt::RPAStateInfo>("/gauss/rpa_state", 1);
    adsb_surveillance_pub_ = nh_.advertise<gauss_msgs_mqtt::ADSBSurveillance>("/gauss/adsb", 1);

    // Subscribe
    //notification_sub_ = nh_.subscribe<gauss_msgs::Notification>("/gauss/notification",1,&USPManager::notificationCB,this);
    state_sub_ = nh_.subscribe("/" + ns_prefix + std::to_string(uav_id_) + "/ual/state", 1, &USPUALBridge::callbackState, this);
    velocity_sub_ = nh_.subscribe("/" + ns_prefix + std::to_string(uav_id_) + "/ual/velocity", 1, &USPUALBridge::callbackVelocity, this);
    alternative_flight_plan_sub_ = nh_.subscribe("/gauss/alternative_flight_plan", 1, &USPUALBridge::callbackAlternativeFlightPlan, this);
    pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/" + ns_prefix + std::to_string(uav_id_) + "/ual/pose", 1, &USPUALBridge::callbackPose, this);

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

void USPUALBridge::callbackAlternativeFlightPlan(const gauss_msgs_mqtt::UTMAlternativeFlightPlan& alt_flight_plan) {
    std::cout << "Alternative flight plan received for icao_address: " << alt_flight_plan.icao << "\n";
    std::cout << "Expected icao address: " << icao_address_ << "\n"; 
    if (alt_flight_plan.icao == std::atoi(icao_address_.c_str())) {
        std::string flight_plan_string = alt_flight_plan.new_flight_plan;
        std::cout << "Received alternative flight plan: \n";
        std::cout << alt_flight_plan.new_flight_plan << "\n";
        // Replace characters
        std::string replace_str = "],[";
        for (int i = flight_plan_string.find(replace_str); i >= 0; i = flight_plan_string.find(replace_str))
            flight_plan_string.replace(i, replace_str.size(), " ");
        // Remove useless characters
        flight_plan_string.erase(std::remove(flight_plan_string.begin(), flight_plan_string.end(), ','), flight_plan_string.end());
        flight_plan_string.erase(std::remove(flight_plan_string.begin(), flight_plan_string.end(), '['), flight_plan_string.end());
        flight_plan_string.erase(std::remove(flight_plan_string.begin(), flight_plan_string.end(), ']'), flight_plan_string.end());
        flight_plan_string.erase(std::remove(flight_plan_string.begin(), flight_plan_string.end(), '\n'), flight_plan_string.end());
        flight_plan_string.erase(std::remove(flight_plan_string.begin(), flight_plan_string.end(), '\r'), flight_plan_string.end());
        // Clear follower input
        ual_communication_.times_.clear();
        ual_communication_.flag_redo_ = true;
        ual_communication_.init_path_.poses.clear();
        ual_communication_.init_path_.header.frame_id = "map";
        // Save the value and substract it from the string
        std::string::size_type sz;
        while (flight_plan_string.size() > 0) {
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
    if (latest_state_msgs_.state == uav_abstraction_layer::State::FLYING_AUTO) {
        gauss_msgs_mqtt::RPAStateInfo rpa_state_info_msg;

        double lat, lon, h;
        proj_.Reverse(pose_ptr->pose.position.x, pose_ptr->pose.position.y, pose_ptr->pose.position.z, lat, lon, h);

        rpa_state_info_msg.altitude = h;
        rpa_state_info_msg.latitude = lat;
        rpa_state_info_msg.longitude = lon;
        rpa_state_info_msg.timestamp = pose_ptr->header.stamp.toNSec() / 1000000;

        rpa_state_info_msg.groundspeed = sqrt(pow(latest_velocity_msg_.twist.linear.x, 2) +
                                              pow(latest_velocity_msg_.twist.linear.y, 2) +
                                              pow(latest_velocity_msg_.twist.linear.z, 2));

        // --- Check this with real data ---
        Eigen::Quaternionf orientation_quaternion(pose_ptr->pose.orientation.w, pose_ptr->pose.orientation.x, pose_ptr->pose.orientation.y, pose_ptr->pose.orientation.z);
        Eigen::Vector3f orientation_euler = Eigen::Matrix3f(orientation_quaternion).eulerAngles(0, 1, 2);
        rpa_state_info_msg.roll = orientation_euler.x();
        rpa_state_info_msg.pitch = orientation_euler.y();
        rpa_state_info_msg.yaw = orientation_euler.z();
        // std::cout << "quaternion: " << orientation_quaternion.w() << " " << orientation_quaternion.x() << " " << orientation_quaternion.y() << " " << orientation_quaternion.z() << "\n";
        // std::cout << "euler:      " << orientation_euler.x() << " " << orientation_euler.y() << " " << orientation_euler.z() << "\n";
        // --------------------------------
        rpa_state_info_msg.icao = atoi(icao_address_.c_str());

        rpa_state_info_pub_.publish(rpa_state_info_msg);
        if (store_data_) csvStore(rpa_state_info_msg);
    }
}

void USPUALBridge::callbackState(const uav_abstraction_layer::State& state_ptr) {
    latest_state_msgs_ = state_ptr;
}

void USPUALBridge::callbackVelocity(const geometry_msgs::TwistStamped& velocity) {
    latest_velocity_msg_ = velocity;
    available_velocity_msg_ = true;
}

void USPUALBridge::csvStore(const gauss_msgs_mqtt::RPAStateInfo& rpa_data) {
    static bool do_once = true;
    static std::ofstream csv_file;
    static double store_time = ros::Time::now().toSec();
    static double wait_time = 1.0;
    if (do_once) {
        auto t = std::time(nullptr);
        auto tm = *std::localtime(&t);
        std::ostringstream oss;
        oss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
        std::string pkg_name_path = ros::package::getPath("usp_manager");
        std::string folder_data_name = pkg_name_path + "/log/";
        // if (mkdir((folder_data_name).c_str(), 0777) == -1) ROS_ERROR("Directory creation failed");
        csv_file.open(folder_data_name + oss.str() + "_RPA" + std::to_string(uav_id_) + "StateInfo.csv");
        csv_file << std::fixed << std::setprecision(8);
        csv_file << "ICAO, latitude, longitude, altitude, yaw, pitch, roll, groundspeed, timestamp\n";
        do_once = false;
    } else if ((ros::Time::now().toSec() - store_time) > wait_time) {
        store_time = ros::Time::now().toSec();
        csv_file << rpa_data.icao << ", " << rpa_data.latitude << ", " << rpa_data.longitude << ", " << rpa_data.altitude << ", "
                 << rpa_data.yaw << ", " << rpa_data.pitch << ", " << rpa_data.roll << ", " << rpa_data.groundspeed << ", "
                 << rpa_data.timestamp << "\n";
    }
}

// MAIN function
int main(int argc, char* argv[]) {
    ros::init(argc, argv, "usp_ual_bridge");

    // Create a USPManager object
    USPUALBridge* usp_ual_bridge = new USPUALBridge();

    ros::Rate rate(30);
    while (ros::ok()) {
        usp_ual_bridge->ual_communication_.runMission();
        ros::spinOnce();
        rate.sleep();
    }

    delete usp_ual_bridge;
}