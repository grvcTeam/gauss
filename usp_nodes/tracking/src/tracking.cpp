#include <ros/ros.h>
#include <gauss_msgs/PositionReport.h>
#include <gauss_msgs/ReadOperation.h>
#include <gauss_msgs/WriteOperation.h>
#include <gauss_msgs/WriteTracking.h>
#include <gauss_msgs/WritePlans.h>
#include <gauss_msgs/ReadIcao.h>
#include <gauss_msgs/ReadIcaoRequest.h>
#include <gauss_msgs/ReadIcaoResponse.h>
#include <boost/thread/mutex.hpp>
#include <map>
#include <vector>
#include <tracking/target_tracker.h>
#include <limits>

#include <tracking/geographic_to_cartesian.hpp>

#include <gauss_msgs/Operation.h>

//#define DEBUG

#define COV_SPEED_XY 0.0
#define VAR_SPEED 1.0

// TODO: Maybe a conversion from geographical coordinates will be needed in the future
//#define GEO_ORIGIN_LATITUDE 36.536291
//#define GEO_ORIGIN_LONGITUDE -6.282740

#define pair_uav_id_icao std::pair<uint8_t, std::string>
#define pair_icao_uav_id std::pair<std::string, uint8_t>
#define pair_wp_index std::pair<int,int>

inline double dotProduct(Eigen::Vector3d vector_1, Eigen::Vector3d vector_2);
inline double distanceBetweenWaypoints(gauss_msgs::Waypoint &waypoint_a, gauss_msgs::Waypoint &waypoint_b);
inline double distanceFromPointToLine(Eigen::Vector3d point, Eigen::Vector3d inline_point, Eigen::Vector3d line_vector);

// Class definition
class Tracking
{
public:
    Tracking();
    ~Tracking();
    void main();

private:
    // Topic Callbacks
    void positionReportCB(const gauss_msgs::PositionReport::ConstPtr& msg);

    // Service Callbacks
    bool updateFlightPlansCB(gauss_msgs::WritePlans::Request &req, gauss_msgs::WritePlans::Response &res); 

    // Auxilary methods
    void predict(ros::Time &now);
	bool update(std::vector<Candidate*> &cand_list);
	int getNumTargets();
	bool getTargetInfo(int target_id, double &x, double &y, double &z);
	void printTargetsInfo();
    bool checkTargetAlreadyExist(std::string icao_address);
    bool checkTargetAlreadyExist(uint8_t uav_id);
    bool checkCooperativeOperationAlreadyExist(uint8_t uav_id);
    bool writeTrackingInfoToDatabase();
    uint32_t findClosestWaypointIndex(gauss_msgs::WaypointList &waypoint_list, gauss_msgs::Waypoint &current_waypoint, 
                                      double &distance_to_waypoint);
    void findSegmentWaypointsIndices(gauss_msgs::Waypoint &current_position, gauss_msgs::WaypointList &flight_plan, int &a_index, int &b_index,
                                     double &distance_to_segment, double &distance_to_point_a, double &distance_to_point_b);                                  
    double distanceBetweenWaypoints(gauss_msgs::Waypoint &waypoint_1, gauss_msgs::Waypoint &waypoint_2);
    void fillTrackingWaypointList();
    void estimateTrajectory();
    void fillFlightPlanUpdated();

    // Auxilary variables
    ros::NodeHandle nh_;
    //gauss_msgs::ReadOperation read_operation_msg_;
    gauss_msgs::WriteOperation write_operation_msg_;
    gauss_msgs::WriteTracking write_tracking_msg_;
    double distance_wp_threshold_margin_; // Threshold from which we'll consider an UAV is not following its flight plan

    // Subscribers
    ros::Subscriber pos_report_sub_;

    // Publisher
    ros::Publisher new_operation_pub_; //TODO: publish modified operation for debugging purposes

    // Timer

    // Server
    ros::ServiceServer update_fligh_plan_server_;

    // Client
    ros::ServiceClient read_operation_client_;
    ros::ServiceClient write_operation_client_;
    ros::ServiceClient write_tracking_client_;
    ros::ServiceClient read_icao_client_;

    // Estimator
    std::vector<Candidate *> candidates_;

    geographic_msgs::GeoPoint origin_geo_;
    double origin_frame_longitude_;
    double origin_frame_latitude_;

    std::map<uint8_t, TargetTracker *> cooperative_targets_; /// Map with cooperative targets
    std::map<uint8_t, std::string> uav_id_icao_address_map_; // Map relating uav_id and icao_address
    std::map<std::string, uint8_t> icao_address_uav_id_map_; // Inverse map
    std::map<uint8_t, gauss_msgs::Operation> cooperative_operations_;
    std::map<uint8_t, pair_wp_index> cooperative_operations_flight_plan_segment_wp_indices_;
    std::map<uint8_t, bool> already_tracked_cooperative_operations_;
    std::map<uint8_t, bool> modified_cooperative_operations_flags_;
    std::map<uint8_t, ros::Time> uav_id_last_time_position_update_map_;
    std::map<std::string, ros::Time> icao_last_time_position_update_map_;
    std::map<uint8_t,gauss_msgs::WaypointList> uav_id_update_flight_plan_map_;
    std::map<uint8_t,bool> updated_flight_plan_flag_map_;

    // Params
    bool use_position_report_;
    bool use_adsb_;
    bool use_speed_info_;

    // Mutex
    boost::mutex candidates_list_mutex_;
    boost::mutex updated_flight_plans_mutex_;

};

// tracking Constructor
Tracking::Tracking()
{
    // Read parameters
    //nh_.param("desired_altitude",desired_altitude,0.5);

    // Initialization
    use_position_report_ = true;
    use_adsb_ = false;
    use_speed_info_ = false;

    // Publish

    // Subscribe
    pos_report_sub_= nh_.subscribe<gauss_msgs::PositionReport>("/gauss/position_report",1,&Tracking::positionReportCB,this);

    // Server
    update_fligh_plan_server_ = nh_.advertiseService("/gauss/update_flight_plans", &Tracking::updateFlightPlansCB, this);

    // Client
    read_operation_client_ = nh_.serviceClient<gauss_msgs::ReadOperation>("/gauss/read_operation");
    write_operation_client_ = nh_.serviceClient<gauss_msgs::WriteOperation>("/gauss/write_operation");
    write_tracking_client_ = nh_.serviceClient<gauss_msgs::WriteTracking>("/gauss/write_tracking");
    read_icao_client_ = nh_.serviceClient<gauss_msgs::ReadIcao>("/gauss/read_icao");

    // Modified Operation publisher TODO: Only for debugging purposes
    new_operation_pub_ = nh_.advertise<gauss_msgs::Operation>("gauss_test/updated_operation", 5);


    // Params
    nh_.param<bool>("use_position_report", use_position_report_, true);
    nh_.param<bool>("use_adsb_", use_adsb_, true);
    nh_.param<double>("wp_distance_threshold_margin", distance_wp_threshold_margin_, 5.0);
    nh_.param<bool>("use_speed_info", use_speed_info_, false);

    read_icao_client_.waitForExistence();
    gauss_msgs::ReadIcao read_icao;

    if ( read_icao_client_.call(read_icao) && read_icao.response.success )
    {
        int index_icao = 0;
        std::vector<std::string> &icao_address = read_icao.response.icao_address;
        for(auto it=read_icao.response.uav_id.begin(); it < read_icao.response.uav_id.end(); it++)
        {
            uav_id_icao_address_map_.insert( std::make_pair((*it),icao_address[index_icao]) );
            icao_address_uav_id_map_.insert( std::make_pair(icao_address[index_icao], (*it)) );
            index_icao++;
        }
        ROS_INFO("Started Tracking node!");
    }
    else
    {
        ROS_ERROR("Failed reading ICAO addresses list from Database");
        ROS_ERROR("Shutting down Tracking node");
        ros::shutdown();
    }

    if (ros::ok())
    {
        // Read all the operations currently registered in the database
        gauss_msgs::ReadOperation read_operation_msg;
        for(auto it=uav_id_icao_address_map_.begin(); it!=uav_id_icao_address_map_.end(); it++)
        {
            read_operation_msg.request.uav_ids.push_back((*it).first);
        }
        if ( read_operation_client_.call(read_operation_msg) && read_operation_msg.response.success )
        {
            for(auto it=read_operation_msg.response.operation.begin(); it!=read_operation_msg.response.operation.end(); it++)
            {
                cooperative_operations_[(*it).uav_id] = (*it);
                cooperative_operations_[(*it).uav_id].track.waypoints.clear();
                cooperative_operations_[(*it).uav_id].estimated_trajectory.waypoints.clear();
                cooperative_operations_[(*it).uav_id].time_tracked = 0;
                already_tracked_cooperative_operations_[(*it).uav_id] = false; // A TargetTracker for this operation hasn't been created yet
                modified_cooperative_operations_flags_[(*it).uav_id] = false;
                updated_flight_plan_flag_map_[(*it).uav_id] = false;
            }
            ROS_INFO("Operations read from database");
        }
        else
        {
            ROS_ERROR("Failed reading operations from database");
            ros::shutdown();
        }
    }
    
    /* Fill uav_ids array with one element, that will be overwritten
       previously to each one of the calls to readOperation service
       By doing this, unnecessary memory reallocation is avoided in each succesive
       execution of positionReportCB callback
    */
    //read_operation_msg_.request.uav_ids.push_back(0);
    //write_operation_msg_.request.uav_ids.push_back(0);
}

Tracking::~Tracking()
{
    for(int i = 0; i < candidates_.size(); i++)
		delete candidates_[i];
    
    candidates_.clear();
}

// Auxilary methods
void Tracking::predict(ros::Time &prediction_time)
{
    for(auto it = cooperative_targets_.begin(); it != cooperative_targets_.end(); ++it)
	{
		(it->second)->predict(prediction_time);
	}
}

bool Tracking::update(std::vector<Candidate*> &cand_list)
{
    candidates_list_mutex_.lock();
    // Traverse candidate list, updating those TargetTracker's that already exist,
    // and initializing those that don't
    for(auto candidates_it=cand_list.begin(); candidates_it!=cand_list.end(); ++candidates_it)
    {
        // Check if Candidate information comes from a non cooperative uav, in that case the info is discarded
        if ((*candidates_it)->uav_id != std::numeric_limits<uint8_t>::max() )
        {
            auto it_target_tracker = cooperative_targets_.find((*candidates_it)->uav_id);
            if(it_target_tracker != cooperative_targets_.end())
                it_target_tracker->second->update(*candidates_it);
            else
            {
                cooperative_targets_[(*candidates_it)->uav_id] = new TargetTracker((*candidates_it)->uav_id);
                cooperative_targets_[(*candidates_it)->uav_id]->initialize(*candidates_it);
                already_tracked_cooperative_operations_[(*candidates_it)->uav_id] = true;
            }
        }

        delete *candidates_it;
    }
    cand_list.clear();
    candidates_list_mutex_.unlock();
    return true;
}

int Tracking::getNumTargets()
{
    return cooperative_targets_.size();
}

bool Tracking::getTargetInfo(int target_id, double &x, double &y, double &z)
{
	bool found = false;

	auto it = cooperative_targets_.find(target_id);
	if(it != cooperative_targets_.end())
	{
		found = true;
		cooperative_targets_[target_id]->getPose(x, y, z);
	}
	
	return found;
}

void Tracking::printTargetsInfo()
{

}

bool Tracking::updateFlightPlansCB(gauss_msgs::WritePlans::Request &req, gauss_msgs::WritePlans::Response &res)
{
    bool result = true;
    updated_flight_plans_mutex_.lock();
    for(int i=0; i<req.uav_ids.size(); i++)
    {
        uav_id_update_flight_plan_map_[i] = req.flight_plans[i];
    }
    updated_flight_plans_mutex_.unlock();

    return result;
}

// PositionReport callback
void Tracking::positionReportCB(const gauss_msgs::PositionReport::ConstPtr &msg)
{
    gauss_msgs::ReadOperation read_operation_msg;

    /*
    if (msg->source == msg->SOURCE_RPA)
        ROS_INFO_STREAM("Received position report from uav_id: " << (int)msg->uav_id);
    else
        ROS_INFO_STREAM("Received ADSB message");
    */
    
    if (msg->header.stamp != ros::Time(0))
    {
        bool create_candidate = false;
        if (use_position_report_ && msg->source == msg->SOURCE_RPA)
        {
            // Reduce the rate at which the candidates are produced
            if (uav_id_last_time_position_update_map_.find(msg->uav_id) == uav_id_last_time_position_update_map_.end())
            {
                uav_id_last_time_position_update_map_.insert(std::make_pair(msg->uav_id, msg->header.stamp));
                create_candidate = true;
            }
            else
            {
                ros::Duration time_delta = msg->header.stamp - uav_id_last_time_position_update_map_[msg->uav_id];
                if (time_delta.toSec() > 0.2)
                {
                    create_candidate = true;
                    uav_id_last_time_position_update_map_[msg->uav_id] = msg->header.stamp;
                }
            }
            if (create_candidate)
            {
                Candidate *candidate_aux_ptr = new Candidate;
                candidate_aux_ptr->uav_id = msg->uav_id;
                candidate_aux_ptr->icao_address = msg->icao_address;
                candidate_aux_ptr->location(0) = msg->position.x;
                candidate_aux_ptr->location(1) = msg->position.y;
                candidate_aux_ptr->location(2) = msg->position.z;
                // TODO: Fill covariances properly
                candidate_aux_ptr->location_covariance(0,0) = 1.0;//msg->confidence;
	            candidate_aux_ptr->location_covariance(0,1) = 0.0;//msg->confidence;
	            candidate_aux_ptr->location_covariance(0,2) = 0.0;//msg->confidence;       	        
                candidate_aux_ptr->location_covariance(1,0) = 0.0;//msg->confidence;
	            candidate_aux_ptr->location_covariance(1,1) = 1.0;//msg->confidence;
	            candidate_aux_ptr->location_covariance(1,2) = 0.0;//msg->confidence;      	        
                candidate_aux_ptr->location_covariance(2,0) = 0.0;//msg->confidence;
	            candidate_aux_ptr->location_covariance(2,1) = 0.0;//msg->confidence;
	            candidate_aux_ptr->location_covariance(2,2) = 1.0;//msg->confidence;
	            candidate_aux_ptr->speed_covariance(0,0) = VAR_SPEED;//msg->confidence;
                candidate_aux_ptr->speed_covariance(0,1) = COV_SPEED_XY;
                candidate_aux_ptr->speed_covariance(0,2) = COV_SPEED_XY;
                candidate_aux_ptr->speed_covariance(1,0) = COV_SPEED_XY;
	            candidate_aux_ptr->speed_covariance(1,1) = VAR_SPEED;//msg->confidence;
                candidate_aux_ptr->speed_covariance(1,2) = COV_SPEED_XY;
                candidate_aux_ptr->speed_covariance(2,0) = COV_SPEED_XY;
                candidate_aux_ptr->speed_covariance(2,1) = COV_SPEED_XY;
	            candidate_aux_ptr->speed_covariance(2,2) = VAR_SPEED;//msg->confidence;
                candidate_aux_ptr->source = Candidate::POSITIONREPORT;
                candidate_aux_ptr->speed_available = use_speed_info_; // TODO:
                // TODO: Fill those speed values with real info from uav
                candidate_aux_ptr->speed(0) = msg->speed * cos(M_PI_2-(msg->heading)*M_PI/180);
                candidate_aux_ptr->speed(1) = msg->speed * sin(M_PI_2-(msg->heading)*M_PI/180);
                candidate_aux_ptr->speed(2) = 0.0;
                candidate_aux_ptr->timestamp = msg->header.stamp;

                candidate_aux_ptr->source = candidate_aux_ptr->POSITIONREPORT;
                candidates_list_mutex_.lock(); // Necessary because callbacks are going to be executed in separate threads concurrently
                candidates_.push_back(candidate_aux_ptr);
                candidates_list_mutex_.unlock();
            }
        }
        if (use_adsb_ && msg->source == msg->SOURCE_ADSB)
        {
            // Reduce the rate at which the candidates are produced
            if (icao_last_time_position_update_map_.find(msg->icao_address) == icao_last_time_position_update_map_.end())
            {
                icao_last_time_position_update_map_.insert(std::make_pair(msg->icao_address, msg->header.stamp));
                create_candidate = true;
            }
            else
            {
                ros::Duration time_delta = msg->header.stamp - icao_last_time_position_update_map_[msg->icao_address];
                if (time_delta.toSec() > 0.2)
                {
                    create_candidate = true;
                    icao_last_time_position_update_map_[msg->icao_address] = msg->header.stamp;
                }
            }
            if (create_candidate)
            {
                Candidate *candidate_aux_ptr = new Candidate;
                // Try to find the associated uav_id of the received icao_address
                auto it = icao_address_uav_id_map_.find(msg->icao_address);
                if (it != icao_address_uav_id_map_.end()) 
                {
                    candidate_aux_ptr->uav_id = (*it).second; // The uav_id is known
                }
                else
                {
                    ROS_INFO("Received position report from UNKNOWN ICAO address");
                    candidate_aux_ptr->uav_id = std::numeric_limits<uint8_t>::max(); // The uav_id is not known, could be a non cooperative one
                }
                candidate_aux_ptr->icao_address = msg->icao_address;
                candidate_aux_ptr->location(0) = msg->position.x;
                candidate_aux_ptr->location(1) = msg->position.y;
                candidate_aux_ptr->location(2) = msg->position.z;
                // TODO: Fill covariances properly
                candidate_aux_ptr->location_covariance(0,0) = 1.0;//msg->confidence;
	            candidate_aux_ptr->location_covariance(0,1) = 0.0;//msg->confidence;
	            candidate_aux_ptr->location_covariance(0,2) = 0.0;//msg->confidence;       	        
                candidate_aux_ptr->location_covariance(1,0) = 0.0;//msg->confidence;
	            candidate_aux_ptr->location_covariance(1,1) = 1.0;//msg->confidence;
	            candidate_aux_ptr->location_covariance(1,2) = 0.0;//msg->confidence;      	        
                candidate_aux_ptr->location_covariance(2,0) = 0.0;//msg->confidence;
	            candidate_aux_ptr->location_covariance(2,1) = 0.0;//msg->confidence;
	            candidate_aux_ptr->location_covariance(2,2) = 1.0;//msg->confidence;
	            candidate_aux_ptr->speed_covariance(0,0) = VAR_SPEED;//msg->confidence;
                candidate_aux_ptr->speed_covariance(0,1) = COV_SPEED_XY;
                candidate_aux_ptr->speed_covariance(0,2) = COV_SPEED_XY;
                candidate_aux_ptr->speed_covariance(1,0) = COV_SPEED_XY;
	            candidate_aux_ptr->speed_covariance(1,1) = VAR_SPEED;//msg->confidence;
                candidate_aux_ptr->speed_covariance(1,2) = COV_SPEED_XY;
                candidate_aux_ptr->speed_covariance(2,0) = COV_SPEED_XY;
                candidate_aux_ptr->speed_covariance(2,1) = COV_SPEED_XY;
	            candidate_aux_ptr->speed_covariance(2,2) = VAR_SPEED;//msg->confidence;
                candidate_aux_ptr->source = Candidate::POSITIONREPORT;
                candidate_aux_ptr->speed_available = use_speed_info_; // TODO:
                // TODO: Fill those speed values with real info from uav
                candidate_aux_ptr->speed(0) = msg->speed * cos(M_PI_2-(msg->heading)*M_PI/180);
                candidate_aux_ptr->speed(1) = msg->speed * sin(M_PI_2-(msg->heading)*M_PI/180);
                candidate_aux_ptr->speed(2) = 0.0;
                candidate_aux_ptr->timestamp = msg->header.stamp;

                candidate_aux_ptr->source = candidate_aux_ptr->ADSB;
                candidates_list_mutex_.lock(); // Necessary because callbacks are going to be executed in separate threads concurrently
                candidates_.push_back(candidate_aux_ptr);
                candidates_list_mutex_.unlock();
            }
        }
    }
    else
    {
        ROS_ERROR("PositionReport timestamp == 0");
    }
    
}

bool Tracking::checkTargetAlreadyExist(std::string icao_address)
{
    bool found = false;

    auto it = icao_address_uav_id_map_.find(icao_address);
    if(it != icao_address_uav_id_map_.end())
        found = true;
    
    return found;
}

bool Tracking::checkTargetAlreadyExist(uint8_t uav_id)
{
    bool found = false;

	auto it = cooperative_targets_.find(uav_id);
	if(it != cooperative_targets_.end())
		found = true;
	
	return found;
}

bool Tracking::checkCooperativeOperationAlreadyExist(uint8_t uav_id)
{
    bool found = false;

    auto it = cooperative_operations_.find(uav_id);
    if(it != cooperative_operations_.end())
        found = true;
    
    return found;
}

bool Tracking::writeTrackingInfoToDatabase()
{
    bool result = true;

    // Write cooperative uavs operations
    for(auto it = cooperative_operations_.begin(); it != cooperative_operations_.end(); ++it)
	{
        if(updated_flight_plan_flag_map_[it->first])
        {
            write_operation_msg_.request.uav_ids.push_back(it->first);
            write_operation_msg_.request.operation.push_back(it->second);
            new_operation_pub_.publish(it->second);
        }
        else if(modified_cooperative_operations_flags_[it->first] == true)
        {
            #ifdef DEBUG
            std::cout << "Writing UAV ID " << (int)it->first << " operation on database\n";
            std::cout << "Estimated trajectory waypoint count: " << it->second.estimated_trajectory.waypoints.size() << std::endl;
            #endif
            
            write_tracking_msg_.request.uav_ids.push_back(it->first);
            write_tracking_msg_.request.current_wps.push_back(it->second.current_wp);
            write_tracking_msg_.request.estimated_trajectories.push_back(it->second.estimated_trajectory);
            write_tracking_msg_.request.times_tracked.push_back(it->second.time_tracked);
            write_tracking_msg_.request.tracks.push_back(it->second.track);
            write_tracking_msg_.request.flight_plans_updated.push_back(it->second.flight_plan_updated);
            //std::cout << "Estimated trajectory size: " << it->second.estimated_trajectory.waypoints.size() << "\n";
            //std::cout << "First waypoint of estimated trajectory\n";
            //std::cout << it->second.estimated_trajectory.waypoints[0] << "\n";
            
            /*
            if(it->first == 0)
            {
                std::cout << "Current waypoint: " << it->second.current_wp << "\n";
                std::cout << it->second.flight_plan.waypoints[it->second.current_wp] << "\n";
                std::cout << "Track size: " << it->second.track.waypoints.size() << "\n";
                std::cout << "Current position:\n"; 
                std::cout << it->second.track.waypoints.back();
            } */
            
        }
        /*
        if(it->first == 0)
        {
            if(updated_flight_plan_flag_map_[it->first])
            {
                std::cout << "RECEIVED FLIGHT PLAN UPDATE\n";
                if(it->second.track.waypoints.size() != 0)
                {
                    std::cout << "Current position\n";
                    std::cout << it->second.track.waypoints.back() << "\n";
                }
                std::cout << "First waypoint of flight plan after tracking update of uav 0" << std::endl;
                std::cout << it->second.flight_plan.waypoints[0] << "\n";
                std::cout << "Estimated trajectory size: " << it->second.estimated_trajectory.waypoints.size() << "\n";
                std::cout << "Estimated trajectory (first 3 waypoints): \n";
                std::cout << it->second.estimated_trajectory.waypoints[0] << "\n";
                std::cout << it->second.estimated_trajectory.waypoints[1] << "\n";
                std::cout << it->second.estimated_trajectory.waypoints[2] << "\n";
            }
        }
        */

        updated_flight_plan_flag_map_[it->first] = false;
        modified_cooperative_operations_flags_[it->first] = false;
	}
    write_tracking_msg_.response.message.clear();
    if(!write_tracking_msg_.request.uav_ids.empty())
    {
        if ( !write_tracking_client_.call(write_tracking_msg_) || !write_tracking_msg_.response.success )
        {
            // TODO: A more complex error handling mechanism must be implemented.
            // This mechanism should retry the call to the service in case it has failed
            ROS_ERROR("Failed writing tracking info to database");
            ROS_ERROR_STREAM("Message: " << write_tracking_msg_.response.message);
            result = false;
        }
        else
        {
            ROS_INFO("Succesful writing operation to database");
            result = true;
        }
    }
    if(!write_operation_msg_.request.uav_ids.empty())
    {
        if ( !write_operation_client_.call(write_operation_msg_) || !write_operation_msg_.response.success )
        {
            // TODO: A more complex error handling mechanism must be implemented.
            // This mechanism should retry the call to the service in case it has failed
            ROS_ERROR("Failed writing tracking info and updated flight plans to database");
            ROS_ERROR_STREAM("Message: " << write_operation_msg_.response.message);
            result = false;
        }
        else
        {
            ROS_INFO("Succesful writing operation to database");
            result = true;
        }
    }    

    write_tracking_msg_.request.uav_ids.clear();
    write_tracking_msg_.request.current_wps.clear();
    write_tracking_msg_.request.estimated_trajectories.clear();
    write_tracking_msg_.request.times_tracked.clear();
    write_tracking_msg_.request.tracks.clear();
    write_tracking_msg_.request.flight_plans_updated.clear();

    write_operation_msg_.request.uav_ids.clear();
    write_operation_msg_.request.operation.clear();

    return result;
}

uint32_t Tracking::findClosestWaypointIndex(gauss_msgs::WaypointList &waypoint_list, gauss_msgs::Waypoint &current_waypoint, double &distance_to_waypoint)
{
    uint32_t index = -1;
    double min_distance = std::numeric_limits<double>::infinity();

    for(int i=0; i != waypoint_list.waypoints.size(); ++i)
    {
        double distance = this->distanceBetweenWaypoints(current_waypoint, waypoint_list.waypoints[i]);
        if (distance < min_distance)
        {
            min_distance = distance;
            index = i;
        }
    }

    distance_to_waypoint = min_distance;
    return index;
}

double Tracking::distanceBetweenWaypoints(gauss_msgs::Waypoint &waypoint_1, gauss_msgs::Waypoint &waypoint_2)
{
    double delta_x = waypoint_1.x - waypoint_2.x;
    double delta_y = waypoint_1.y - waypoint_2.y;
    double delta_z = waypoint_1.z - waypoint_2.z;

    return sqrt(pow(delta_x,2)+pow(delta_y,2)+pow(delta_z,2));
}

void Tracking::fillTrackingWaypointList()
{
    // Tracking waypoint list must be updated for every tracked uav
    
    // First we fill tracking waypoint list of cooperative UAVs
    for(auto it=cooperative_targets_.begin(); it!=cooperative_targets_.end(); ++it)
    {
        gauss_msgs::Waypoint waypoint_aux;
        waypoint_aux.stamp = it->second->currentPositionTimestamp();
        it->second->getPose(waypoint_aux.x,waypoint_aux.y,waypoint_aux.z);
        cooperative_operations_[it->first].track.waypoints.push_back(waypoint_aux); 

        if (cooperative_operations_[it->first].track.waypoints.size() > 1)
        {
            auto it_last_element = cooperative_operations_[it->first].track.waypoints.rbegin();
            auto it_first_element = cooperative_operations_[it->first].track.waypoints.begin();
            cooperative_operations_[it->first].time_tracked = (it_last_element)->stamp.toSec() - (it_first_element)->stamp.toSec();
        }
        else
        {
            cooperative_operations_[it->first].time_tracked = 0;
        }
        modified_cooperative_operations_flags_[it->first] = true;
    }
}

void Tracking::fillFlightPlanUpdated()
{
    /*
    // TODO: Fill the updated flight plan of every uav
    // That flight plan consist in the previously traversed waypoints in addition to the estimated trajectory without time horizon, i.e. until the end of the operation
    for (auto it=cooperative_operations_.begin(); it != cooperative_operations_.end(); it++)
    {
        gauss_msgs::Waypoint waypoint_aux;
        (*it).second.flight_plan_updated.waypoints.clear();

        std::copy((*it).second.estimated_trajectory.waypoints.begin(), (*it).second.estimated_trajectory.waypoints.end(), std::back_inserter((*it).second.flight_plan_updated.waypoints));

        (*it).second.flight_plan_updated.waypoints.push_back(waypoint_aux);
    }
    */
}

void Tracking::estimateTrajectory()
{
    for(auto it=cooperative_operations_.begin(); it!=cooperative_operations_.end(); ++it)
    {
        uint8_t uav_id = it->first;
        if ( already_tracked_cooperative_operations_[uav_id] == true )
        {
            #ifdef DEBUG
            std::cout << "##################################" << std::endl;
            std::cout << "ESTIMATING TRAJECTORY FOR UAV_ID: " << int(uav_id) << std::endl;
            #endif
            gauss_msgs::Operation &operation_aux = it->second;
            gauss_msgs::Waypoint current_position;
            gauss_msgs::WaypointList &flight_plan_ref = operation_aux.flight_plan;
            gauss_msgs::WaypointList &flight_plan_updated = operation_aux.flight_plan_updated;
            gauss_msgs::WaypointList &estimated_trajectory = operation_aux.estimated_trajectory;
            int flight_plan_waypoint_count = flight_plan_ref.waypoints.size();
            // TODO: An efficiency improvent will be to not clear completely the estimated trajectory, but remove
            // some elements and add others. That will require additional state control mechanism.
            estimated_trajectory.waypoints.clear();
            flight_plan_updated.waypoints.clear();

            current_position.stamp = cooperative_targets_[uav_id]->currentPositionTimestamp();
            cooperative_targets_[uav_id]->getPose(current_position.x, current_position.y, current_position.z);

            int flight_plan_current_wp_index = 0;
            int a_waypoint_index = 0;
            int b_waypoint_index = 0;
            double distance_to_segment = 0;
            double distance_to_point_a = 0;
            double distance_to_point_b = 0;
            // For estimating the trajectory we must take into account that waypoints could be time-separated by a non fixed time interval
            findSegmentWaypointsIndices(current_position, flight_plan_ref, a_waypoint_index, b_waypoint_index, distance_to_segment, distance_to_point_a, distance_to_point_b);

            flight_plan_current_wp_index = b_waypoint_index;
            // Trajectory estimation for cooperative uav
            operation_aux.current_wp = b_waypoint_index;
            // If this distance is smaller than a threshold, the estimated trajectory will be exactly the next waypoints of the flight plan
            double dt = operation_aux.dT;
            // dt = 5
            // time_horizon = 90
            uint32_t number_estimated_wps = operation_aux.time_horizon/dt; // 18

            #ifdef DEBUG
            std::cout << "a_waypoint_index: " << a_waypoint_index << "\n";
            std::cout << "b_waypoint_index: " << b_waypoint_index << "\n";
            std::cout << "n_wp: " << number_estimated_wps << "\n";
            std::cout << "Current position: " << current_position.x << "," << current_position.y << "," << current_position.z << "\n";
            std::cout << "Flight plan current waypoint index: " << b_waypoint_index << "\n";
            std::cout << "Flight plan waypoint: " << flight_plan_ref.waypoints[b_waypoint_index].x << ",";
            std::cout << flight_plan_ref.waypoints[b_waypoint_index].y << ",";
            std::cout << flight_plan_ref.waypoints[b_waypoint_index].z << "\n";
            #endif

            int remaining_flight_plan_wps = flight_plan_ref.waypoints.size() - b_waypoint_index;
    
            #ifdef DEBUG
            std::cout << "Remaining waypoints count: " << remaining_flight_plan_wps << "\n"; 
            std::cout << "time_horizon: " << operation_aux.time_horizon << "\n";
            std::cout << "distance to segment: " << distance_to_segment << "\n";
            std::cout << "distance to point a: " << distance_to_point_a << "\n";
            std::cout << "distance to point b: " << distance_to_point_b << std::endl;
            #endif

            if (distance_to_segment <= (operation_aux.operational_volume + distance_wp_threshold_margin_))
            {
                bool closer_than_dt_flag = false;
                // If distance from current estimated position is close enough to flight plan, the estimated trajectory
                // will be composed of the immediately following waypoints.
                int estimated_wp_count = 0;
                gauss_msgs::Waypoint previous_waypoint;
                gauss_msgs::Waypoint wp_aux;
                Eigen::Vector3d speed_vector;
                int wp_count_aux = 0;
                
                ros::Time last_stamp = current_position.stamp;
                double time_to_next_waypoint = 0;
                double distance_between_waypoints = 0;
                double time_between_waypoints = 0;
                double distance_from_current_pos_to_next_wp = distanceBetweenWaypoints(current_position, flight_plan_ref.waypoints[b_waypoint_index]);
                if(a_waypoint_index != b_waypoint_index)
                {
                    distance_between_waypoints = distanceBetweenWaypoints(flight_plan_ref.waypoints[a_waypoint_index], flight_plan_ref.waypoints[b_waypoint_index]);
                    time_between_waypoints = (flight_plan_ref.waypoints[b_waypoint_index].stamp - flight_plan_ref.waypoints[a_waypoint_index].stamp).toSec();
                }
                else
                {
                    if(b_waypoint_index != 0)
                    {
                        distance_between_waypoints = distanceBetweenWaypoints(flight_plan_ref.waypoints[b_waypoint_index], flight_plan_ref.waypoints[b_waypoint_index-1]);
                        time_between_waypoints = (flight_plan_ref.waypoints[b_waypoint_index].stamp - flight_plan_ref.waypoints[b_waypoint_index-1].stamp).toSec();
                    }
                    else
                    {
                        distance_between_waypoints = distanceBetweenWaypoints(flight_plan_ref.waypoints[b_waypoint_index], flight_plan_ref.waypoints[b_waypoint_index+1]);
                        time_between_waypoints = (flight_plan_ref.waypoints[b_waypoint_index+1].stamp - flight_plan_ref.waypoints[b_waypoint_index].stamp).toSec();
                    }
                }
                time_to_next_waypoint = distance_from_current_pos_to_next_wp/distance_between_waypoints * time_between_waypoints;

                //std::cout << "Time to next waypoint " << time_to_next_waypoint << "\n";

                if(time_to_next_waypoint > dt)
                {
                    // Estimate intermediate waypoints between current position and next waypoint
                    wp_aux = current_position;

                    speed_vector.x() = (flight_plan_ref.waypoints[b_waypoint_index].x - current_position.x)/time_to_next_waypoint;
                    speed_vector.y() = (flight_plan_ref.waypoints[b_waypoint_index].y - current_position.y)/time_to_next_waypoint;
                    speed_vector.z() = (flight_plan_ref.waypoints[b_waypoint_index].z - current_position.z)/time_to_next_waypoint;

                    double epsilon = time_to_next_waypoint - (int) time_to_next_waypoint;

                    if (epsilon > 0)
                    {
                        wp_aux.x = wp_aux.x + speed_vector.x()*epsilon;
                        wp_aux.y = wp_aux.y + speed_vector.y()*epsilon;
                        wp_aux.z = wp_aux.z + speed_vector.z()*epsilon;
                        last_stamp = last_stamp + ros::Duration(epsilon);
                        wp_aux.stamp = last_stamp;
                        estimated_trajectory.waypoints.push_back(wp_aux);
                        flight_plan_updated.waypoints.push_back(wp_aux);
                        estimated_wp_count++;
                    }

                    wp_count_aux = time_to_next_waypoint/dt;
                    
                    #ifdef DEBUG
                    std::cout << "Estimating intermediate points between current position and next waypoint" << "\n";
                    std::cout << "Number of points to estimate: " << (wp_count_aux-1) << std::endl;
                    #endif
                    for(int index_1 = 0; index_1 < (wp_count_aux - 1); index_1++)
                    {
                        wp_aux.x = wp_aux.x + speed_vector.x()*dt;
                        wp_aux.y = wp_aux.y + speed_vector.y()*dt;
                        wp_aux.z = wp_aux.z + speed_vector.z()*dt;
                        last_stamp = last_stamp + ros::Duration(dt);
                        wp_aux.stamp = last_stamp;
                        estimated_trajectory.waypoints.push_back(wp_aux);
                        flight_plan_updated.waypoints.push_back(wp_aux);
                        estimated_wp_count++;
                    }
                }
                else if(time_to_next_waypoint <= dt)
                {
                    closer_than_dt_flag = true;
                }                

                // Add next waypoint to estimated trajectory (if estimated waypoint count is less than the max number of estimated waypoints)
                if (estimated_wp_count < number_estimated_wps)
                {
                    #ifdef DEBUG
                    std::cout << "Adding next waypoint to estimated trajectory" << std::endl;
                    #endif
                    wp_aux = flight_plan_ref.waypoints[b_waypoint_index];
                    if(closer_than_dt_flag)
                        last_stamp = last_stamp + ros::Duration(time_to_next_waypoint);
                    else
                        last_stamp = last_stamp + ros::Duration(dt);
                    wp_aux.stamp = last_stamp;
                    estimated_trajectory.waypoints.push_back(wp_aux);
                    flight_plan_updated.waypoints.push_back(wp_aux);
                    estimated_wp_count++;
                }

                int flight_plan_wp_index = flight_plan_current_wp_index + 1;

                while(estimated_wp_count < number_estimated_wps && flight_plan_wp_index < flight_plan_ref.waypoints.size())
                {
                    int aux_index = 0;

                    if(estimated_wp_count < number_estimated_wps)
                    {
                        #ifdef DEBUG
                        std::cout << "Adding flight plan second segment waypoint to estimated trajectory" << std::endl;
                        #endif
                        wp_aux = flight_plan_ref.waypoints[flight_plan_wp_index];
                        last_stamp = last_stamp + ros::Duration(dt);
                        wp_aux.stamp = last_stamp;
                        estimated_trajectory.waypoints.push_back(wp_aux);
                        flight_plan_updated.waypoints.push_back(wp_aux);
                        estimated_wp_count++;
                        flight_plan_wp_index++;
                    }
                }

                while(flight_plan_wp_index < flight_plan_ref.waypoints.size())
                {
                    wp_aux = flight_plan_ref.waypoints[flight_plan_wp_index];
                    last_stamp = last_stamp + ros::Duration(dt);
                    wp_aux.stamp = last_stamp;
                    flight_plan_updated.waypoints.push_back(wp_aux);
                    flight_plan_wp_index++;
                }
                #ifdef DEBUG
                std::cout << "Estimated trajectory size " << estimated_trajectory.waypoints.size() << std::endl;
                #endif
                modified_cooperative_operations_flags_[uav_id] = true;
            }
            else
            {
                #ifdef DEBUG
                std::cout << "Using n prediction steps" << std::endl;
                #endif
                // If distance from current estimated position is too far from the flight plan, the estimated trajectory
                // will be composed of waypoints predicted based on the current estimated speed and position of the uav
                cooperative_targets_[uav_id]->predictNTimes(number_estimated_wps, dt, estimated_trajectory);
                // Signal that the operation has been modified to write it on the database
                modified_cooperative_operations_flags_[uav_id] = true;
            }

        }
    }
}

void Tracking::findSegmentWaypointsIndices(gauss_msgs::Waypoint &current_position, gauss_msgs::WaypointList &flight_plan, int &a_index, int &b_index, double &distance_to_segment, double &distance_to_point_a, double &distance_to_point_b)
{
    int first_index = 0;
    int second_index = 0;
    distance_to_segment = std::numeric_limits<double>::max();
    double distance_to_waypoint_a = std::numeric_limits<double>::max();
    bool flag_distance_to_waypoint_a = false;
    bool flag_distance_to_waypoint_b = false;

    /*
    std::cout << "###################\n";
    std::cout << "Finding current wp\n";
    std::cout << "Current position\n";
    std::cout << current_position << "\n";
    */
    for (int i=0; i < flight_plan.waypoints.size()-1; i++)
    {
        gauss_msgs::Waypoint &waypoint_a = flight_plan.waypoints[i];
        gauss_msgs::Waypoint &waypoint_b = flight_plan.waypoints[i+1];
        /*
        std::cout << "--------\n";
        std::cout << "Segment " << i << "\n";
        std::cout << "waypoint " << i << "\n";
        std::cout << waypoint_a << "\n";
        std::cout << "waypoint " << i+1 << "\n";
        std::cout << waypoint_b << "\n";
        */
        Eigen::Vector3d vector_u;
        flag_distance_to_waypoint_a = false;
        flag_distance_to_waypoint_b = false;
        vector_u.x() = waypoint_b.x - waypoint_a.x;
        vector_u.y() = waypoint_b.y - waypoint_a.y;
        vector_u.z() = waypoint_b.z - waypoint_a.z;
        /*
        std::cout << "Waypoint " << i <<"\n";
        std::cout << waypoint_a << "\n";
        */
        /*

            Solve this equation

            |alpha_1|     | 1    0    0    u_1|^-1   | a_1 - x_0|
            |alpha_2|     | 0    1    0    u_2|      | a_2 - y_0|
            |alpha_3|  =  | 0    0    1    u_3|   *  | a_3 - z_0|
            |    t  |     | u_1  u_2  u_3  0  |      |     0    |

            b = A^-1*x

        */

        Eigen::Matrix4d A;
        Eigen::Vector4d b;
        Eigen::Vector4d x;

        A = Eigen::Matrix4d::Identity();
        A(3,3) = 0;
        A(0,3) = vector_u.x();
        A(1,3) = vector_u.y();
        A(2,3) = vector_u.z();
        A(3,0) = vector_u.x();
        A(3,1) = vector_u.y();
        A(3,2) = vector_u.z();

        x.x() = current_position.x - waypoint_a.x;
        x.y() = current_position.y - waypoint_a.y;
        x.z() = current_position.z - waypoint_a.z;

        b = A.inverse()*x;

        double distance;
        if (b[3] >= 0 && b[3] <= 1)
        {
            distance = sqrt(b[0]*b[0] + b[1]*b[1] + b[2]*b[2]);
            //std::cout << "Distance to segment " << distance << "\n";
        }
        else
        {
            double distance_to_waypoint_a = distanceBetweenWaypoints(current_position, waypoint_a);
            double distance_to_waypoint_b = distanceBetweenWaypoints(current_position, waypoint_b);

            if (distance_to_waypoint_a < distance_to_waypoint_b)
            {
                distance = distance_to_waypoint_a;
                flag_distance_to_waypoint_a = true;
                //std::cout << "Distance to waypoint_a " << distance << "\n";
            }
            else
            {
                distance = distance_to_waypoint_b;
                flag_distance_to_waypoint_b = true;
                //std::cout << "Distance to waypoint_b " << distance << "\n";
            }
        }
        
        // Then check the distance to the line that contains the segment
        // double distance = distanceFromPointToLine(current_position_eigen, point_a, vector_2);
        if (distance <= distance_to_segment)
        {
            distance_to_segment = distance;
            if(flag_distance_to_waypoint_a)
            {
                first_index = i;
                second_index = i;
            }
            else if(flag_distance_to_waypoint_b)
            {
                first_index = i+1;
                second_index = i+1;                
            }
            else
            {
                first_index = i;
                second_index = i+1;
            }
        }
    }

    a_index = first_index;
    b_index = second_index;
    /*
    std::cout << "Current waypoint index: " << b_index << "\n";
    */

    gauss_msgs::Waypoint &waypoint_a = flight_plan.waypoints[first_index];
    gauss_msgs::Waypoint &waypoint_b = flight_plan.waypoints[second_index];

    distance_to_point_a = distanceBetweenWaypoints(waypoint_a, current_position);
    distance_to_point_b = distanceBetweenWaypoints(waypoint_b, current_position);
}

inline double dotProduct(Eigen::Vector3d vector_1, Eigen::Vector3d vector_2)
{
    return vector_1.dot(vector_2);
}

inline double distanceFromPointToLine(Eigen::Vector3d point, Eigen::Vector3d inline_point, Eigen::Vector3d line_vector)
{
    // See https://onlinemschool.com/math/library/analytic_geometry/p_line/
    double distance = 0;

    Eigen::Vector3d M0M1;
    M0M1 << inline_point.x() - point.x(), inline_point.y() - point.y(), inline_point.z() - inline_point.z();

    distance = M0M1.cross(line_vector).norm() / line_vector.norm();

    return distance;
}

inline double distanceBetweenWaypoints(gauss_msgs::Waypoint &waypoint_a, gauss_msgs::Waypoint &waypoint_b)
{
    return sqrt(pow(waypoint_a.x - waypoint_b.x,2) + pow(waypoint_a.y - waypoint_b.y, 2) + pow(waypoint_a.z - waypoint_b.z, 2));
}

void Tracking::main()
{
    double estimator_rate;
    ros::NodeHandle pnh("~");
    pnh.param<double>("estimator_rate", estimator_rate, 1); // Each second the estimator gets updated

    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::Rate rate(estimator_rate);
    ros::Rate sleep_rate(5);

    while(ros::Time::now() == ros::Time(0)) // Wait until /clock messages are published if in simulation
    {
        sleep_rate.sleep();
    }
    int counter = 0;
    while(pnh.ok())
    {
        candidates_list_mutex_.lock();
        bool new_operations_to_download = false;
        for(auto it=candidates_.begin(); it!=candidates_.end(); ++it)
        {
            //std::cout << "Candidates list traversing on MAIN LOOP" << std::endl;
            if ( (*it)->source == Candidate::POSITIONREPORT )
            {
                // The received candidate data comes from a cooperative uav
                // Check if we have the associated operation in memory for each uav_id from which position reports have been received
                if ( !this->checkCooperativeOperationAlreadyExist((*it)->uav_id))
                {
                    // TODO: Delete candidate from candidates list
                }
                else
                {
                    //std::cout << "uav_id : " << (int)(*it)->uav_id << " already tracked" << std::endl;
                }
            }
            else if ( (*it)->source == Candidate::ADSB )
            {
                // Check if we have the associated operation in memory for each uav_id from which position reports have been received
                if( (*it)->uav_id == std::numeric_limits<uint8_t>::max() )
                {
                    // The position info is from a non cooperative uav
                }
            }
        }
        candidates_list_mutex_.unlock();

        //std::cout << "Operations size start of the loop: " << operations_.size() << std::endl;

        ros::Time now(ros::Time::now());
        this->predict(now); // Predict the position
        this->update(candidates_);

        //std::cout << "Operations size after update: " << operations_.size() << std::endl;

        this->fillTrackingWaypointList();

        updated_flight_plans_mutex_.lock();
        for(auto it=cooperative_operations_.begin(); it!=cooperative_operations_.end(); it++)
        {
            if(uav_id_update_flight_plan_map_.find((*it).first) != uav_id_update_flight_plan_map_.end())
            {
                cooperative_operations_[(*it).first].flight_plan = uav_id_update_flight_plan_map_[(*it).first];
                uav_id_update_flight_plan_map_.erase((*it).first);
                updated_flight_plan_flag_map_[(*it).first] = true;
            }
        }
        updated_flight_plans_mutex_.unlock();

        //std::cout << "Operations size after fillTracking: " << operations_.size() << std::endl;
        this->estimateTrajectory();
        // TODO: Fill the field flight_plan_updated of every operation
        // 
        //this->fillFlightPlanUpdated();

        for(int i = 0; i < candidates_.size(); i++)
			delete candidates_[i];

        candidates_.clear();

        counter++;
        // Write on database each five iterations of the filter to avoid unnecessary overload
        if (counter == 5)
        {
            counter = 0;
            this->writeTrackingInfoToDatabase();
        }
        rate.sleep();
        //std::cout << "Cycle time: " << rate.cycleTime() << std::endl;
    }
}

// MAIN function
int main(int argc, char *argv[])
{
    ros::init(argc,argv,"tracking");

    // Create a Tracking object
    Tracking *tracking = new Tracking();
    tracking->main();
    ros::waitForShutdown();
}
