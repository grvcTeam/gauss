#include <ros/ros.h>

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <gauss_msgs_mqtt/ADSBSurveillance.h>
#include <gauss_msgs_mqtt/RPAStateInfo.h>
#include <gauss_msgs_mqtt/UTMAlert.h>
#include <gauss_msgs_mqtt/UTMAlternativeFlightPlan.h>
#include <gauss_msgs_mqtt/RPSFlightPlanAccept.h>
#include <gauss_msgs_mqtt/RPSChangeFlightStatus.h>

#include <gauss_msgs/PositionReport.h>
#include <gauss_msgs/Notifications.h>
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
#include <gauss_msgs/PilotAnswer.h>
#include <gauss_msgs/WritePlans.h>

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

#include <sstream>

#define ARENOSILLO_LATITUDE 37.101973
#define ARENOSILLO_LONGITUDE -6.736522

#define REFERENCE_LATITUDE ARENOSILLO_LATITUDE
#define REFERENCE_LONGITUDE ARENOSILLO_LONGITUDE

#define YES std::string("yes")
#define NO std::string("no")

struct ThreatFlightPlan {
   uint8_t threat_id;
   gauss_msgs::WaypointList new_flight_plan; 
};

// Class definition
class USPManager
{
public:
    USPManager();

private:
    // Topic Callbacks
    void RPAStateCB(const gauss_msgs_mqtt::RPAStateInfo::ConstPtr& msg); // RPS -> UTM
    void ADSBSurveillanceCB(const gauss_msgs_mqtt::ADSBSurveillance::ConstPtr& msg); // RPS -> UTM
    // Server Callbacks
    bool notificationsCB(gauss_msgs::Notifications::Request &req, gauss_msgs::Notifications::Response &res); // UTM -> RPS
    void RPSChangeFlightStatusCB(const gauss_msgs_mqtt::RPSChangeFlightStatus::ConstPtr& msg); // RPS -> UTM

    // TODO: Think how to implement this
    void RPSFlightPlanAcceptCB(const gauss_msgs_mqtt::RPSFlightPlanAccept::ConstPtr& msg); // RPS -> UTM

    bool sendThreats(std::vector<int8_t> uav_ids, std::vector<gauss_msgs::Threat> threats);
    bool initializeICAOIDMap();
    bool initializeIDOperationMap();

    /*
    // Timer Callbacks
    void timerCallback(const ros::TimerEvent&);
    */

    // Auxilary variables
    double rate;
    ros::NodeHandle nh_;

    std::map<uint8_t, uint32_t> id_icao_map_;
    std::map<uint32_t, uint8_t> icao_id_map_;

    std::map<uint8_t, gauss_msgs::Operation> id_operation_map_;

    std::vector<int8_t> initial_uav_ids_;

    std::map<uint8_t, std::vector<ThreatFlightPlan>> id_threat_flight_plan_map_;

    // Timer
    //ros::Timer timer_sub_;

    // Subscribers
    ros::Subscriber rpaState_sub_;
    ros::Subscriber adsb_sub_;
    ros::Subscriber flight_plan_accept_sub_;
    ros::Subscriber flight_status_sub_;
    ros::Subscriber rpa_flight_acceptance_sub_;

    // Server 
    ros::ServiceServer notification_server_;

    // Clients
    //ros::ServiceClient alert_client_;
    ros::ServiceClient read_operation_client_;
    ros::ServiceClient write_operation_client_;
    ros::ServiceClient threats_client_;
    ros::ServiceClient read_icao_client_;
    ros::ServiceClient send_pilot_answer_client_;
    ros::ServiceClient write_plans_client_;

    // Publisher
    ros::Publisher rpacommands_pub_;       //TBD message to UAVs
    ros::Publisher position_report_pub_;
    ros::Publisher alternative_flight_plan_pub_;
    ros::Publisher alert_pub_;


    // Variables for geographic to cartesian conversion
    double lat0_, lon0_;
    GeographicLib::Geocentric earth_;
    GeographicLib::LocalCartesian proj_;
};

// USPManager Constructor
USPManager::USPManager():
lat0_(REFERENCE_LATITUDE),
lon0_(REFERENCE_LONGITUDE),
earth_(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f()),
proj_(lat0_, lon0_, 0, earth_)
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
    rpaState_sub_= nh_.subscribe<gauss_msgs_mqtt::RPAStateInfo>("/gauss/rpa_state",10,&USPManager::RPAStateCB,this);
    adsb_sub_ = nh_.subscribe<gauss_msgs_mqtt::ADSBSurveillance>("/gauss/adsb", 10, &USPManager::ADSBSurveillanceCB, this);
    rpa_flight_acceptance_sub_ = nh_.subscribe<gauss_msgs_mqtt::RPSFlightPlanAccept>("/gauss/flightacceptance", 1, &USPManager::RPSFlightPlanAcceptCB, this);

    // Server 
    notification_server_ = nh_.advertiseService("/gauss/notifications", &USPManager::notificationsCB, this);
    // Subscribe
    rpaState_sub_= nh_.subscribe<gauss_msgs_mqtt::RPAStateInfo>("/gauss/rpa_state",1,&USPManager::RPAStateCB,this);
    adsb_sub_ = nh_.subscribe<gauss_msgs_mqtt::ADSBSurveillance>("/gauss/adsb", 1, &USPManager::ADSBSurveillanceCB, this);
    flight_plan_accept_sub_ = nh_.subscribe<gauss_msgs_mqtt::RPSFlightPlanAccept>("/gauss/flightacceptance", 1, &USPManager::RPSFlightPlanAcceptCB, this);
    flight_status_sub_ = nh_.subscribe<gauss_msgs_mqtt::RPSChangeFlightStatus>("/gauss/flight", 1, &USPManager::RPSChangeFlightStatusCB, this);

    // Client
    // alert_client_ = nh_.serviceClient<gauss_msgs::Alert>("/gauss/alert");
    write_operation_client_ = nh_.serviceClient<gauss_msgs::WriteOperation>("/gauss/write_operation");
    threats_client_ = nh_.serviceClient<gauss_msgs::Threats>("/gauss/threats");
    read_icao_client_ = nh_.serviceClient<gauss_msgs::ReadIcao>("/gauss/read_icao");
    read_operation_client_ = nh_.serviceClient<gauss_msgs::ReadOperation>("/gauss/read_operation");
    send_pilot_answer_client_ = nh_.serviceClient<gauss_msgs::PilotAnswer>("/gauss/pilotanswer");
    // send_pilot_answer_client_ = nh_.serviceClient<gauss_msgs::PilotAnswer>("/gauss/send_pilot_answer");
    write_plans_client_ = nh_.serviceClient<gauss_msgs::WritePlans>("/gauss/update_flight_plans");

    // Timer
    //timer_sub_=nh_.createTimer(ros::Duration(1.0/rate),&USPManager::timerCallback,this);

    ROS_INFO("Started USPManager node!");

    this->initializeICAOIDMap();
}

// Notification callback
bool USPManager::notificationsCB(gauss_msgs::Notifications::Request &req, gauss_msgs::Notifications::Response &res)
{
    /*
    gauss_msgs_mqtt::UTMAlert utm_alert_msg;
    utm_alert_msg.alert_message = msg->description;
    //utm_alert_msg.alert_title = msg->
    alert_pub_.publish(utm_alert_msg);
    */

    for (auto msg : req.notifications){
        gauss_msgs_mqtt::UTMAlternativeFlightPlan alternative_flight_plan_msg;
        std::string uav_id_string = std::to_string(msg.uav_id);
        int n_zeros = 0;
        if(uav_id_string.length() == 1)
            n_zeros = 1;
        alternative_flight_plan_msg.flight_plan_id = std::string("MISSION") + std::string(n_zeros, '0') + uav_id_string;
        alternative_flight_plan_msg.icao = id_icao_map_[msg.uav_id];

        std::ostringstream new_flight_plan_ss;

        new_flight_plan_ss << "[";

        for(auto it=msg.new_flight_plan.waypoints.begin(); it!=msg.new_flight_plan.waypoints.end(); it++)
        {
            new_flight_plan_ss << "[";
            double lat,lon,h;
            proj_.Reverse(it->x,it->y,it->z,lat,lon,h);
            new_flight_plan_ss << lon <<", " << lat << ", " << h << ", " << it->stamp.toSec();
            new_flight_plan_ss << "]";
            if(it != (msg.new_flight_plan.waypoints.end()-1))
            {
                new_flight_plan_ss << ",";
            }
        }
        alternative_flight_plan_msg.new_flight_plan = new_flight_plan_ss.str();

        alternative_flight_plan_pub_.publish(alternative_flight_plan_msg);

        ThreatFlightPlan threat_flight_plan;
        threat_flight_plan.threat_id = msg.threat.threat_id;
        threat_flight_plan.new_flight_plan = msg.new_flight_plan;
        if(id_threat_flight_plan_map_.find(msg.uav_id) != id_threat_flight_plan_map_.end())
        {
            id_threat_flight_plan_map_[msg.uav_id].push_back(threat_flight_plan);
        }
        else
        {
            id_threat_flight_plan_map_[msg.uav_id] = std::vector<ThreatFlightPlan>();
            id_threat_flight_plan_map_[msg.uav_id].push_back(threat_flight_plan);
        }
    }
    res.success = true;
    return res.success;
}

void USPManager::RPSFlightPlanAcceptCB(const gauss_msgs_mqtt::RPSFlightPlanAccept::ConstPtr& msg)
{
    gauss_msgs::WritePlans write_plans_msg;
    ThreatFlightPlan threat_flight_plan;
    std::string flight_plan_id_aux = msg->flight_plan_id;
    uint8_t flight_plan_id = std::atoi(flight_plan_id_aux.erase((size_t)0,(size_t)7).c_str());
    if(id_threat_flight_plan_map_.find(flight_plan_id) != id_threat_flight_plan_map_.end())
    {
        threat_flight_plan = id_threat_flight_plan_map_[flight_plan_id].front();
        id_threat_flight_plan_map_[flight_plan_id].erase(id_threat_flight_plan_map_[flight_plan_id].begin());
        auto it = icao_id_map_.find(msg->icao);
        if (it != icao_id_map_.end()){
            write_plans_msg.request.flight_plans.push_back(threat_flight_plan.new_flight_plan);
            write_plans_msg.request.uav_ids.push_back((*it).second);
        } else {
            ROS_WARN("USP Manager can not find the uav id associated with icao address %06x", msg->icao);
        }
    }
    else
        return;
    
    if (!write_plans_client_.call(write_plans_msg) || !write_plans_msg.response.success)
    {
        ROS_WARN("Failed to send updated flight plan to tracking after receiving alternative flight plan acknowledge from pilot");
    }
    else
    {
        ROS_INFO_STREAM(write_plans_msg.response.message);
    }
    
    // Send PilotAnswer to Emergency Management
    gauss_msgs::PilotAnswer pilot_answer_msg;
    if(msg->accept == 0)
        pilot_answer_msg.request.pilot_answers.push_back(NO);
    else if(msg->accept == 1)
        pilot_answer_msg.request.pilot_answers.push_back(YES);

    pilot_answer_msg.request.threat_ids.push_back(threat_flight_plan.threat_id);
    if (!send_pilot_answer_client_.call(pilot_answer_msg) || !pilot_answer_msg.response.success)
    {
        ROS_WARN("Failed to send pilot answer to Emergency Management");
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
    auto it = icao_id_map_.find(msg->icao);
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

        geometry_msgs::Point cartesian_translation;

        proj_.Forward(msg->latitude, msg->longitude, msg->altitude, cartesian_translation.x, cartesian_translation.y, cartesian_translation.z);

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

    geometry_msgs::Point cartesian_translation;

    proj_.Forward(msg->latitude, msg->longitude, msg->altitude, cartesian_translation.x, cartesian_translation.y, cartesian_translation.z);

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

void USPManager::RPSChangeFlightStatusCB(const gauss_msgs_mqtt::RPSChangeFlightStatus::ConstPtr& msg)
{
    std::cout << "Executing callback\n";
    ROS_INFO_STREAM("Received RPSChangeFlightStatus message. Flight Plan ID: " << msg->flight_plan_id << " ICAO: " << msg->icao << " STATUS: " << msg->status);
}

/*
// Timer Callback
void USPManager::timerCallback(const ros::TimerEvent &)
{
    ROS_INFO("GAUSS USP running");


}
*/

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
            id_icao_map_[*it_id] = std::atoi((*it_icao).c_str());
            icao_id_map_[std::atoi((*it_icao).c_str())] = *it_id;
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
