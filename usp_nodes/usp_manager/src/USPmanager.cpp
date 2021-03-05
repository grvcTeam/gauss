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
#include <gauss_msgs_mqtt/Waypoint.h>

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
#include <gauss_msgs/ChangeFlightStatus.h>

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

#include <sstream>

#define ARENOSILLO_LATITUDE 37.094784
#define ARENOSILLO_LONGITUDE -6.735478
#define ARENOSILLO_ELLIPSOIDAL_HEIGHT 0 // TODO: MEASURE IT 

#define LORING_LATITUDE 40.65974645776713
#define LORING_LONGITUDE -3.59689676974471
#define LORING_ELLIPSOIDAL_HEIGHT 675

#define REFERENCE_LATITUDE ARENOSILLO_LATITUDE
#define REFERENCE_LONGITUDE ARENOSILLO_LONGITUDE
#define REFERENCE_ELLIPSOIDAL_HEIGHT ARENOSILLO_ELLIPSOIDAL_HEIGHT

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
    USPManager(double origin_latitude, double origin_longitude, double origin_ellipsoidal_height);

private:
    // Topic Callbacks
    void RPAStateCB(const gauss_msgs_mqtt::RPAStateInfo::ConstPtr& msg); // RPS -> UTM
    void ADSBSurveillanceCB(const gauss_msgs_mqtt::ADSBSurveillance::ConstPtr& msg); // RPS -> UTM
    // Server Callbacks
    bool notificationsCB(gauss_msgs::Notifications::Request &req, gauss_msgs::Notifications::Response &res); // UTM -> RPS
    void RPSChangeFlightStatusCB(const gauss_msgs_mqtt::RPSChangeFlightStatus::ConstPtr& msg); // RPS -> UTM

    // TODO: Think how to implement this
    void RPSFlightPlanAcceptCB(const gauss_msgs_mqtt::RPSFlightPlanAccept::ConstPtr& msg); // RPS -> UTM

    // Auxiliary methods
    bool checkRPAHealth(const gauss_msgs_mqtt::RPAStateInfo::ConstPtr &rpa_state, gauss_msgs::Threat &threat, const gauss_msgs::PositionReport &pos_report);
    gauss_msgs::Threats manageThreatList(const bool &_flag_new_threat, gauss_msgs::Threat &_in_threat);

    bool initializeICAOIDMap();
    bool initializeIDOperationMap();

    /*
    // Timer Callbacks
    void timerCallback(const ros::TimerEvent&);
    */

    // Auxilary variables
    ros::NodeHandle nh_;

    std::map<uint8_t, uint32_t> id_icao_map_;
    std::map<uint32_t, uint8_t> icao_id_map_;

    std::map<uint8_t, gauss_msgs::Operation> id_operation_map_;

    std::vector<int8_t> initial_uav_ids_;

    std::map<uint8_t, std::vector<ThreatFlightPlan>> id_threat_flight_plan_map_;

    std::vector<gauss_msgs::Threat> threat_list_;

    // Timer
    //ros::Timer timer_sub_;

    // Subscribers
    ros::Subscriber rpaState_sub_;
    ros::Subscriber adsb_sub_;
    ros::Subscriber flight_plan_accept_sub_;
    ros::Subscriber flight_status_sub_;

    // Server 
    ros::ServiceServer notification_server_;

    // Clients
    ros::ServiceClient read_operation_client_;
    ros::ServiceClient write_operation_client_;
    ros::ServiceClient threats_client_;
    ros::ServiceClient read_icao_client_;
    ros::ServiceClient send_pilot_answer_client_;
    ros::ServiceClient write_plans_client_;
    ros::ServiceClient change_flight_status_client_;

    // Publisher
    ros::Publisher rpacommands_pub_;       //TBD message to UAVs
    ros::Publisher position_report_pub_;
    ros::Publisher alternative_flight_plan_pub_;
    ros::Publisher alert_pub_;


    // Variables for geographic to cartesian conversion
    double lat0_, lon0_, ellipsoidal_height_;
    GeographicLib::Geocentric earth_;
    GeographicLib::LocalCartesian proj_;
};

// USPManager Constructor
USPManager::USPManager(double origin_latitude, double origin_longitude, double origin_ellipsoidal_height):
lat0_(origin_latitude),
lon0_(origin_longitude),
ellipsoidal_height_(origin_ellipsoidal_height),
earth_(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f()),
proj_(lat0_, lon0_, ellipsoidal_height_, earth_)
{
    // Initialization

    // Publish
    rpacommands_pub_ = nh_.advertise<gauss_msgs::Notification>("/gauss/commands",1);  //TBD message to UAVs
    position_report_pub_ = nh_.advertise<gauss_msgs::PositionReport>("/gauss/position_report", 10);
    alternative_flight_plan_pub_ = nh_.advertise<gauss_msgs_mqtt::UTMAlternativeFlightPlan>("/gauss/alternative_flight_plan", 1);
    alert_pub_ = nh_.advertise<gauss_msgs_mqtt::UTMAlert>("/gauss/alert", 1);

    rpaState_sub_= nh_.subscribe<gauss_msgs_mqtt::RPAStateInfo>("/gauss/rpastateinfo",10,&USPManager::RPAStateCB,this);
    adsb_sub_ = nh_.subscribe<gauss_msgs_mqtt::ADSBSurveillance>("/gauss/adsb", 10, &USPManager::ADSBSurveillanceCB, this);
    flight_plan_accept_sub_ = nh_.subscribe<gauss_msgs_mqtt::RPSFlightPlanAccept>("/gauss/flightacceptance", 10, &USPManager::RPSFlightPlanAcceptCB, this);
    flight_status_sub_ = nh_.subscribe<gauss_msgs_mqtt::RPSChangeFlightStatus>("/flight_status", 10, &USPManager::RPSChangeFlightStatusCB, this);
    // Server 
    notification_server_ = nh_.advertiseService("/gauss/notifications", &USPManager::notificationsCB, this);

    // Client
    write_operation_client_ = nh_.serviceClient<gauss_msgs::WriteOperation>("/gauss/write_operation");
    threats_client_ = nh_.serviceClient<gauss_msgs::Threats>("/gauss/threats");
    read_icao_client_ = nh_.serviceClient<gauss_msgs::ReadIcao>("/gauss/read_icao");
    read_operation_client_ = nh_.serviceClient<gauss_msgs::ReadOperation>("/gauss/read_operation");
    send_pilot_answer_client_ = nh_.serviceClient<gauss_msgs::PilotAnswer>("/gauss/pilotanswer");
    write_plans_client_ = nh_.serviceClient<gauss_msgs::WritePlans>("/gauss/update_flight_plans");
    change_flight_status_client_ = nh_.serviceClient<gauss_msgs::ChangeFlightStatus>("/gauss/change_flight_status");

    ROS_INFO("[USPM] Started USPManager node!");
    ROS_INFO_STREAM("Origin (Latitude, Longitude): (" << lat0_ << "," << lon0_ << ")");

    this->initializeICAOIDMap();
}

// Notification callback
bool USPManager::notificationsCB(gauss_msgs::Notifications::Request &req, gauss_msgs::Notifications::Response &res)
{
    for (auto msg : req.notifications){
        // TODO: Treat all possible notifications properly
        if(msg.threat.threat_type == gauss_msgs::Threat::LOSS_OF_SEPARATION || msg.threat.threat_type == gauss_msgs::Threat::GEOFENCE_CONFLICT)
        {
            gauss_msgs_mqtt::UTMAlternativeFlightPlan alternative_flight_plan_msg;
            std::string uav_id_string = std::to_string(msg.uav_id);
            int n_zeros = 0;
            if(uav_id_string.length() == 1)
                n_zeros = 1;
            alternative_flight_plan_msg.flight_plan_id = std::string("MISSION") + std::string(n_zeros, '0') + uav_id_string;
            alternative_flight_plan_msg.icao = id_icao_map_[msg.uav_id];

            for(auto it=msg.new_flight_plan.waypoints.begin(); it!=msg.new_flight_plan.waypoints.end(); it++)
            {
                gauss_msgs_mqtt::Waypoint waypoint_mqtt;
                double lat,lon,h;
                proj_.Reverse(it->x,it->y,it->z,lat,lon,h);
                waypoint_mqtt.waypoint_elements[0] = lon;
                waypoint_mqtt.waypoint_elements[1] = lat;
                waypoint_mqtt.waypoint_elements[2] = h;
                waypoint_mqtt.waypoint_elements[3] = it->stamp.toSec();
                alternative_flight_plan_msg.new_flight_plan.push_back(waypoint_mqtt);
            }

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
        else if(msg.threat.threat_type == gauss_msgs::Threat::JAMMING_ATTACK)
        {
            // TODO: Set properly alert message and title
            gauss_msgs_mqtt::UTMAlert utm_alert_msg;
            utm_alert_msg.alert_message = msg.description;
            utm_alert_msg.alert_title = "JAMMING ATTACK";
            utm_alert_msg.alert_id = "JAMMING ATTACK";
            alert_pub_.publish(utm_alert_msg);   
            // Finish threatened operation
            gauss_msgs::ChangeFlightStatus change_flight_status_msg;
            change_flight_status_msg.request.icao = id_icao_map_[msg.uav_id];
            change_flight_status_msg.request.is_started = false;
            change_flight_status_client_.call(change_flight_status_msg);
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
    flight_plan_id_aux.erase((size_t)0,(size_t)7);
    uint8_t flight_plan_id = std::atoi(flight_plan_id_aux.c_str());
    if(msg->accept)
    {
        if(id_threat_flight_plan_map_.find(flight_plan_id) != id_threat_flight_plan_map_.end())
        {
            if(!id_threat_flight_plan_map_[flight_plan_id].empty())
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
        }
        else
            return;

        if (!write_plans_client_.call(write_plans_msg) || !write_plans_msg.response.success)
        {
            ROS_WARN("Failed to send updated flight plan to tracking after receiving alternative flight plan acknowledge from pilot");
        }
        else
        {
            // ROS_INFO_STREAM(write_plans_msg.response.message);
        }
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

        // std::cout << "Position report icao: " << msg->icao << "\n";
        // std::cout << "x: " << position_report_msg.position.x << "\n";  
        // std::cout << "y: " << position_report_msg.position.y << "\n";
        // std::cout << "z: " << position_report_msg.position.z << "\n";
        position_report_pub_.publish(position_report_msg);

        gauss_msgs::Threat temp_threat;
        bool flag_new_threat = checkRPAHealth(msg, temp_threat, position_report_msg);
        gauss_msgs::Threats new_threats_msgs = manageThreatList(flag_new_threat, temp_threat);
        if (flag_new_threat) threats_client_.call(new_threats_msgs);
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
    gauss_msgs::ChangeFlightStatus change_flight_status_msg;
    ROS_INFO_STREAM("[USPM] Received RPSChangeFlightStatus message. Flight Plan ID: " << msg->flight_plan_id << " ICAO: " << msg->icao << " STATUS: " << msg->status);
    change_flight_status_msg.request.icao = msg->icao;
    if(msg->status == "start")
        change_flight_status_msg.request.is_started = true;
    else if(msg->status == "stop")
        change_flight_status_msg.request.is_started = false;
    change_flight_status_client_.call(change_flight_status_msg);
}

bool USPManager::checkRPAHealth(const gauss_msgs_mqtt::RPAStateInfo::ConstPtr &rpa_state, gauss_msgs::Threat &threat, const gauss_msgs::PositionReport &pos_report){
    bool result = false;
    // Define threshold
    static double threshold_jamming = 0.5;
    static double threshold_spoofing = 0.5;
    // TODO: Complete checks with all necessary fields
    auto it = icao_id_map_.find(rpa_state->icao);
    threat.uav_ids.push_back(it->second);
    threat.times.push_back(ros::Time::now());
    threat.location = pos_report.position;
    if (rpa_state->jamming >= threshold_jamming) {
        threat.threat_type = gauss_msgs::Threat::JAMMING_ATTACK;
        result = true;
    }
    if (rpa_state->spoofing >= threshold_spoofing) {
        threat.threat_type = gauss_msgs::Threat::SPOOFING_ATTACK;
        result = true;
    }

    return result;
}

gauss_msgs::Threats USPManager::manageThreatList(const bool &_flag_new_threat, gauss_msgs::Threat &_in_threat){
    gauss_msgs::Threats out_threats;
    static int threat_list_id_ = 0;
    if (_flag_new_threat){
        if (threat_list_.size() == 0) {
            _in_threat.times.front() = ros::Time::now();
            threat_list_.push_back(_in_threat);
            threat_list_.front().threat_id = threat_list_id_;
            out_threats.request.threats.push_back(threat_list_.front());
            out_threats.request.uav_ids.push_back(threat_list_.front().uav_ids.front());
            threat_list_id_++;
        }
        if (threat_list_.size() > 0){
            bool save_threat = false;
            // Using lambda, check if threat_type of in_threat is in threat_list
            std::vector<gauss_msgs::Threat>::iterator it = std::find_if(threat_list_.begin(), threat_list_.end(), 
                                                                        [_in_threat](gauss_msgs::Threat threat){return (threat.threat_type == _in_threat.threat_type);});
            if (it != threat_list_.end()){
                // Type found!
                if (_in_threat.uav_ids.front() != it->uav_ids.front()){
                        save_threat = true;
                } else {
                    // ID Found! Update time
                    it->times.front() = ros::Time::now();
                }
            } else {
                // Type not found!
                save_threat = true;
            }
            if (save_threat){
                _in_threat.threat_id = threat_list_id_;
                _in_threat.times.front() = ros::Time::now();
                threat_list_.push_back(_in_threat);
                out_threats.request.threats.push_back(_in_threat);
                out_threats.request.uav_ids.push_back(_in_threat.uav_ids.front());
                threat_list_id_++;
            }
        } 
    }
    // Delete non-updated threats from threat_list
    for (auto saved_threat = threat_list_.begin(); saved_threat != threat_list_.end();){
        std::vector<gauss_msgs::Threat>::iterator it = std::find_if(threat_list_.begin(), threat_list_.end(), 
                                                                    [](gauss_msgs::Threat threat){
                                                                        double time_to_delete = 2.0;
                                                                        return (ros::Time::now().toSec() - threat.times.front().toSec()) >= time_to_delete;
                                                                        });
        if (it != threat_list_.end()){
            // Time difference too big, the threat should be deleted. 
            saved_threat = threat_list_.erase(saved_threat);
        } else {
            // Threat should not be deleted, it has been updated 
            saved_threat++;
        }
    }

    std::string cout_threats;
    for (auto i : out_threats.request.threats) cout_threats = cout_threats + " [" + std::to_string(i.threat_id) +
                                                              ", " + std::to_string(i.threat_type) + "]";
    ROS_INFO_STREAM_COND(out_threats.request.threats.size() > 0, "[USPM] New threats detected: (id, type) " + cout_threats);

    return out_threats;
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

// MAIN function
int main(int argc, char *argv[])
{
    ros::init(argc,argv,"USPManager");
    ros::NodeHandle nh;
    double origin_latitude, origin_longitude, origin_ellipsoidal_height;

    origin_latitude = nh.param<double>("origin_latitude", ARENOSILLO_LATITUDE);
    origin_longitude = nh.param<double>("origin_longitude", ARENOSILLO_LONGITUDE);
    origin_ellipsoidal_height = nh.param<double>("origin_ellipsoidal_height", ARENOSILLO_ELLIPSOIDAL_HEIGHT);
    // Create a USPManager object
    USPManager *usp_manager = new USPManager(origin_latitude, origin_longitude, origin_ellipsoidal_height);

    ros::spin();
}
