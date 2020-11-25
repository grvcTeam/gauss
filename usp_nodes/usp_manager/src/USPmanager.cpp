#include <ros/ros.h>

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geographic_msgs/GeoPoint.h>
#include <usp_manager/geographic_to_cartesian.hpp>

#include <gauss_msgs_mqtt/ADSBSurveillance.h>
#include <gauss_msgs_mqtt/RPAStateInfo.h>
#include <gauss_msgs_mqtt/UTMAlert.h>
#include <gauss_msgs_mqtt/UTMAlternativeFlightPlan.h>
#include <gauss_msgs_mqtt/RPSFlightPlanAccept.h>
#include <gauss_msgs_mqtt/RPSChangeFlightStatus.h>

#include <gauss_msgs/PositionReport.h>
#include <gauss_msgs/Notification.h>
// #include <gauss_msgs/Alert.h>
#include <gauss_msgs/Operation.h>
#include <gauss_msgs/Waypoint.h>
#include <gauss_msgs/WaypointList.h>
#include <gauss_msgs/ReadOperation.h>
#include <gauss_msgs/WriteOperation.h>
#include <gauss_msgs/Threats.h>
#include <gauss_msgs/Threat.h>
#include <gauss_msgs/ReadIcao.h>
#include <gauss_msgs/ReadIcaoRequest.h>
#include <gauss_msgs/ReadIcaoResponse.h>

#define ARENOSILLO_LATITUDE 37.101973
#define ARENOSILLO_LONGITUDE -6.736522

#define REFERENCE_LATITUDE ARENOSILLO_LATITUDE
#define REFERENCE_LONGITUDE ARENOSILLO_LONGITUDE


// Class definition
class USPManager
{
public:
    USPManager();

private:
    // Topic Callbacks
    void notificationCB(const gauss_msgs::Notification::ConstPtr& msg); // UTM -> RPS
    void RPAStateCB(const gauss_msgs_mqtt::RPAStateInfo::ConstPtr& msg); // RPS -> UTM
    void ADSBSurveillanceCB(const gauss_msgs_mqtt::ADSBSurveillance::ConstPtr& msg); // RPS -> UTM

    // TODO: Think how to implement this
    void RPSFlightPlanAcceptCB(const gauss_msgs_mqtt::RPSFlightPlanAccept::ConstPtr& msg); // RPS -> UTM

    bool sendThreats(std::vector<int8_t> uav_ids, std::vector<gauss_msgs::Threat> threats);
    bool initializeICAOIDMap();
    bool initializeIDOperationMap();

    // Timer Callbacks
    void timerCallback(const ros::TimerEvent&);

    // Auxilary variables
    double rate;
    ros::NodeHandle nh_;

    std::map<uint8_t, std::string> id_icao_map_;
    std::map<std::string, uint8_t> icao_id_map_;

    std::map<uint8_t, gauss_msgs::Operation> id_operation_map_;

    std::vector<int8_t> initial_uav_ids_;

    // Timer
    ros::Timer timer_sub_;

    // Subscribers
    ros::Subscriber notification_sub_;
    ros::Subscriber rpaState_sub_;
    ros::Subscriber adsb_sub_;

    // Clients
    //ros::ServiceClient alert_client_;
    ros::ServiceClient read_operation_client_;
    ros::ServiceClient write_operation_client_;
    ros::ServiceClient threats_client_;
    ros::ServiceClient read_icao_client_;

    // Publisher
    ros::Publisher rpacommands_pub_;       //TBD message to UAVs
    ros::Publisher position_report_pub_;
    ros::Publisher alternative_flight_plan_pub_;
    ros::Publisher alert_pub_;


    // Variables for geographic to cartesian conversion
    geographic_msgs::GeoPoint origin_geo_;

};

// USPManager Constructor
USPManager::USPManager()
{
    // Read parameters
    nh_.param("/gauss/monitoring_rate",rate,0.5);

    // Initialization


    // Publish
    rpacommands_pub_ = nh_.advertise<gauss_msgs::Notification>("/gauss/commands",1);  //TBD message to UAVs
    position_report_pub_ = nh_.advertise<gauss_msgs::PositionReport>("/gauss/position_report", 1);
    alternative_flight_plan_pub_ = nh_.advertise<gauss_msgs_mqtt::UTMAlternativeFlightPlan>("/gauss/alternative_flight_plan", 1);
    alert_pub_ = nh_.advertise<gauss_msgs_mqtt::UTMAlert>("/gauss/alert", 1);

    // Subscribe
    notification_sub_ = nh_.subscribe<gauss_msgs::Notification>("/gauss/notification",1,&USPManager::notificationCB,this);
    rpaState_sub_= nh_.subscribe<gauss_msgs_mqtt::RPAStateInfo>("/gauss/rpa_state",10,&USPManager::RPAStateCB,this);
    adsb_sub_ = nh_.subscribe<gauss_msgs_mqtt::ADSBSurveillance>("/gauss/adsb", 10, &USPManager::ADSBSurveillanceCB, this);

    // Client
    // alert_client_ = nh_.serviceClient<gauss_msgs::Alert>("/gauss/alert");
    write_operation_client_ = nh_.serviceClient<gauss_msgs::WriteOperation>("/gauss/write_operation");
    threats_client_ = nh_.serviceClient<gauss_msgs::Threats>("/gauss/threats");
    read_icao_client_ = nh_.serviceClient<gauss_msgs::ReadIcao>("/gauss/read_icao");
    read_operation_client_ = nh_.serviceClient<gauss_msgs::ReadOperation>("/gauss/read_operation");

    // Timer
    timer_sub_=nh_.createTimer(ros::Duration(1.0/rate),&USPManager::timerCallback,this);

    ROS_INFO("Started USPManager node!");

    this->initializeICAOIDMap();
}

// Notification callback
void USPManager::notificationCB(const gauss_msgs::Notification::ConstPtr& msg)
{
    gauss_msgs_mqtt::UTMAlert utm_alert_msg;

    utm_alert_msg.alert_message = msg->description;
    //utm_alert_msg.alert_title = msg->


    

    alert_pub_.publish(utm_alert_msg);
}

void USPManager::RPSFlightPlanAcceptCB(const gauss_msgs_mqtt::RPSFlightPlanAccept::ConstPtr& msg)
{
    gauss_msgs::WriteOperation write_operation_msg;

    // TODO: Lookup flight plan on d

    if (!write_operation_client_.call(write_operation_msg) || !write_operation_msg.response.success)
    {
        ROS_WARN("Failed to write operation on database after receiving alternative flight plan acknowledge from pilot");
    }
    else
    {
        ROS_INFO_STREAM(write_operation_msg.response.message);
    }
    
    
}

// RPAStatus Callback
void USPManager::RPAStateCB(const gauss_msgs_mqtt::RPAStateInfo::ConstPtr& msg)
{
    gauss_msgs::PositionReport position_report_msg;

    position_report_msg.header.stamp = ros::Time().fromSec(msg->timestamp/1000.0);
    position_report_msg.icao_address = std::to_string(msg->icao);
    // TODO: Get uav_id from db_manager knowing its icao address
    // Consider the posibility that an RPAStateInfo message could be received from an UAV which hasn't a registered flight plan yet 
    // position_report_msg.uav_id;
    auto it = icao_id_map_.find(position_report_msg.icao_address);
    if (it != icao_id_map_.end())
    {
        position_report_msg.uav_id = (*it).second;
        position_report_msg.confidence = msg->covariance_h + msg->covariance_v; // TODO: Find a proper method for calculating confidence
        position_report_msg.source = position_report_msg.SOURCE_RPA;
        position_report_msg.position.mandatory = true;
        position_report_msg.position.stamp = position_report_msg.header.stamp;
        position_report_msg.header.frame_id = "map";

        // TODO: Convert latitude, longitude, altitude to x,y,z
        // The origin latitude and longitude must be known

        geographic_msgs::GeoPoint uav_geo_point;
        uav_geo_point.altitude = msg->altitude;
        uav_geo_point.latitude = msg->latitude;
        uav_geo_point.longitude = msg->longitude;

        geometry_msgs::Point32 cartesian_translation;

        cartesian_translation = geographic_to_cartesian(uav_geo_point, origin_geo_);

        //float uav_heading = msg->yaw;
        //tf2::Quaternion aux_quaternion_tf2;
        //aux_quaternion_tf2.setRPY(0,0,(90-uav_heading)/180*M_PI);


        position_report_msg.position.x = cartesian_translation.x;
        position_report_msg.position.y = cartesian_translation.y;
        position_report_msg.position.z = cartesian_translation.z;

        position_report_msg.heading = msg->yaw;
        position_report_msg.speed = msg->groundspeed;

        position_report_pub_.publish(position_report_msg);
    }
    else
    {
        ROS_WARN("USP Manager can not find the uav id associated with icao address %s", position_report_msg.icao_address.c_str());
    }
    
}

void USPManager::ADSBSurveillanceCB(const gauss_msgs_mqtt::ADSBSurveillance::ConstPtr& msg)
{
    gauss_msgs::PositionReport position_report_msg;

    position_report_msg.header.stamp = ros::Time().fromSec(msg->timestamp/1000.0);
    position_report_msg.icao_address = std::to_string(msg->icao);

    // TODO: Fill uav_id field

    position_report_msg.confidence = 1.0; // TODO: Find proper method for calc confidence
    position_report_msg.position.mandatory = true;
    position_report_msg.position.stamp = position_report_msg.header.stamp;
    position_report_msg.source = position_report_msg.SOURCE_ADSB;

    // TODO: Convert latitude, longitude, altitude to x,y,z
    // The origin latitude and longitude must be known

    geographic_msgs::GeoPoint uav_geo_point;
    uav_geo_point.altitude = msg->altitude;
    uav_geo_point.latitude = msg->latitude;
    uav_geo_point.longitude = msg->longitude;

    geometry_msgs::Point32 cartesian_translation;

    cartesian_translation = geographic_to_cartesian(uav_geo_point, origin_geo_);

    //float uav_heading = msg->heading;
    //tf2::Quaternion aux_quaternion_tf2;
    //aux_quaternion_tf2.setRPY(0,0,(90-uav_heading)/180*M_PI);

    position_report_msg.position.x = cartesian_translation.x;
    position_report_msg.position.y = cartesian_translation.y;
    position_report_msg.position.z = cartesian_translation.z;

    position_report_msg.heading = msg->heading;
    position_report_msg.speed = msg->speed;

    position_report_pub_.publish(position_report_msg);
}

// Timer Callback
void USPManager::timerCallback(const ros::TimerEvent &)
{
    ROS_INFO("GAUSS USP running");


}

bool USPManager::initializeICAOIDMap()
{
    gauss_msgs::ReadIcaoRequest read_icao_req;
    gauss_msgs::ReadIcaoResponse read_icao_res;

    ros::service::waitForService("/gauss/read_icao");

    bool result = read_icao_client_.call(read_icao_req, read_icao_res);

    result = result && read_icao_res.success;

    if (!read_icao_res.uav_id.empty() && !read_icao_res.icao_address.empty() && result)
    {
        auto it_icao = read_icao_res.icao_address.begin();
        for(auto it_id = read_icao_res.uav_id.begin(); it_id != read_icao_res.uav_id.end(); it_id++, it_icao++)
        {
            id_icao_map_[*it_id] = *it_icao;
            icao_id_map_[*it_icao] = *it_id;
            std::cout << "ID: " << (int) *it_id << " ICAO: " << *it_icao << std::endl;
        }
    }

    return result;
}

bool USPManager::initializeIDOperationMap()
{
    gauss_msgs::ReadOperationRequest read_operation_req;
    gauss_msgs::ReadOperationResponse read_operation_res;

    read_operation_req.uav_ids = initial_uav_ids_;

    ros::service::waitForService("/gauss_read_operation");

    bool result = read_operation_client_.call(read_operation_req, read_operation_res);

    result = result && read_operation_res.success;

    if (result)
    {
        for (auto it = read_operation_res.operation.begin(); it != read_operation_res.operation.end(); it++)
        {
            id_operation_map_[(*it).uav_id] = *it;
        }
    }

    return result;
}

bool USPManager::sendThreats(std::vector<int8_t> uav_ids, std::vector<gauss_msgs::Threat> threats)
{
    gauss_msgs::Threats threats_msg;

    gauss_msgs::Threat::ALERT_WARNING;
    gauss_msgs::Threat::TECHNICAL_FAILURE;
    gauss_msgs::Threat::COMMUNICATION_FAILURE;
    gauss_msgs::Threat::LACK_OF_BATTERY;
    gauss_msgs::Threat::JAMMING_ATTACK;
    gauss_msgs::Threat::SPOOFING_ATTACK;
    gauss_msgs::Threat::GNSS_DEGRADATION;

    threats_msg.request.uav_ids = uav_ids;
    threats_msg.request.threats = threats;

    bool result = threats_client_.call(threats_msg);
    return (result && threats_msg.response.success);
}

// MAIN function
int main(int argc, char *argv[])
{
    ros::init(argc,argv,"USPManager");

    // Create a USPManager object
    USPManager *usp_manager = new USPManager();

    ros::spin();
}
