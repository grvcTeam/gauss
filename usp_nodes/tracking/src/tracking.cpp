#include <ros/ros.h>
#include <gauss_msgs/PositionReport.h>
#include <gauss_msgs/ReadOperation.h>
#include <gauss_msgs/WriteOperation.h>
#include <boost/thread/mutex.hpp>
#include <map>
#include <vector>
#include <tracking/target_tracker.h>
#include <limits>

#include <tracking/geographic_to_cartesian.hpp>

#include <gauss_msgs/Operation.h>

// TODO: Maybe a conversion from geographical coordinates will be needed in the future
//#define GEO_ORIGIN_LATITUDE 36.536291
//#define GEO_ORIGIN_LONGITUDE -6.282740

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

    // Auxilary methods
    void predict(ros::Time &now);
	bool update(std::vector<Candidate*> &cand_list);
	int getNumTargets();
	bool getTargetInfo(int target_id, double &x, double &y, double &z);
	void printTargetsInfo();
    bool checkTargetAlreadyExist(std::string icao_address);
    bool checkTargetAlreadyExist(uint8_t uav_id);
    bool writeOperationsToDatabase();
    uint32_t findClosestWaypointIndex(gauss_msgs::WaypointList &waypoint_list, gauss_msgs::Waypoint &current_waypoint, 
                                      double &distance_to_waypoint);
    double distanceBetweenWaypoints(gauss_msgs::Waypoint &waypoint_1, gauss_msgs::Waypoint &waypoint_2);
    void fillTrackingWaypointList();
    void estimateTrajectory();

    // Auxilary variables
    ros::NodeHandle nh_;
    //gauss_msgs::ReadOperation read_operation_msg_;
    gauss_msgs::WriteOperation write_operation_msg_;

    // Subscribers
    ros::Subscriber pos_report_sub_;

    // Publisher
    ros::Publisher new_operation_pub_; //TODO: publish modified operation for debugging purposes

    // Timer

    // Server

    // Client
    ros::ServiceClient read_operation_client_;
    ros::ServiceClient write_operation_client_;

    // Estimator
    std::vector<Candidate *> candidates_;

    geographic_msgs::GeoPoint origin_geo_;
    double origin_frame_longitude_;
    double origin_frame_latitude_;

    std::map<uint8_t, TargetTracker *> targets_;	/// Map with targets
    std::map<uint8_t, std::string> uav_id_icao_address_map_; // Map relating uav_id and icao_address TODO: How to fill that map
    std::map<uint8_t, gauss_msgs::Operation> operations_;
    std::map<uint8_t, bool> modified_operations_flags_;
    std::map<uint8_t, bool> cooperative_uavs_;

    // Params
    bool use_only_position_report_;
    bool use_only_adsb_;
    bool use_position_report_and_adsb_;

    // Mutex
    boost::mutex candidates_list_mutex_;
};

// tracking Constructor
Tracking::Tracking()
{
    // Read parameters
    //nh_.param("desired_altitude",desired_altitude,0.5);

    // Initialization
    // TODO: Get values from rosparams and reduce variables to only one
    use_only_position_report_ = true;
    use_only_adsb_ = false;
    use_position_report_and_adsb_ = false;

    // Publish

    // Subscribe
    pos_report_sub_= nh_.subscribe<gauss_msgs::PositionReport>("/gauss/position_report",1,&Tracking::positionReportCB,this);

    // Server

    // Client
    read_operation_client_ = nh_.serviceClient<gauss_msgs::ReadOperation>("/gauss/read_operation");
    write_operation_client_ = nh_.serviceClient<gauss_msgs::WriteOperation>("/gauss/write_operation");

    // Modified Operation publisher TODO: Only for debugging purposes
    new_operation_pub_ = nh_.advertise<gauss_msgs::Operation>("gauss_test/updated_operation", 5);

    /* Fill uav_ids array with one element, that will be overwritten
       previously to each one of the calls to readOperation service
       By doing this, unnecessary memory reallocation is avoided in each succesive
       execution of positionReportCB callback
    */
    //read_operation_msg_.request.uav_ids.push_back(0);
    //write_operation_msg_.request.uav_ids.push_back(0);

    ROS_INFO("Started Tracking node!");

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
    for(auto it = targets_.begin(); it != targets_.end(); ++it)
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
        auto it_target_tracker = targets_.find((*candidates_it)->uav_id);
        if(it_target_tracker != targets_.end())
            it_target_tracker->second->update(*candidates_it);
        else
        {
            targets_[(*candidates_it)->uav_id] = new TargetTracker((*candidates_it)->uav_id);
            targets_[(*candidates_it)->uav_id]->initialize(*candidates_it);
        }
        delete *candidates_it;
    }
    cand_list.clear();
    candidates_list_mutex_.unlock();
    return true;
}

int Tracking::getNumTargets()
{
    return targets_.size();
}

bool Tracking::getTargetInfo(int target_id, double &x, double &y, double &z)
{
	bool found = false;

	auto it = targets_.find(target_id);
	if(it != targets_.end())
	{
		found = true;
		targets_[target_id]->getPose(x, y, z);
	}
	
	return found;
}

void Tracking::printTargetsInfo()
{

}

/*
// PositionReport callback
void Tracking::positionReportCB(const gauss_msgs::PositionReport::ConstPtr &msg)
{
    gauss_msgs::Operation *operation_ptr;

    // TODO: if the source of the position report is not the main UAV and instead is provided by
    // ADSB surveillance, the UAV_id field should not be used to lookup an operation in the database,
    // but instead, the ICAO address must be used in this case

    int id = msg->uav_id;
    double confidence = msg->confidence;
    gauss_msgs::Waypoint position = msg->position;
    int source=msg->source;
    //gauss_msgs::WaypointList track;

    // Read Operation from Database with a matching UAV id
    read_operation_msg_.request.uav_ids[0] = id;
    if (!read_operation_client_.call(read_operation_msg_) || !read_operation_msg_.response.success)
    {
        // TODO: A more complex error handling mechanism must be implemented.
        // This mechanism should retry the call to the service in case it has failed
        ROS_ERROR("Failed reading operation from database");
        return;
    }
    else
    {
        operation_ptr = &(read_operation_msg_.response.operation[0]);
        ROS_INFO("Succesful reading operation from database");
    }
    
    // Add received position to tracking list of waypoints
    operation_ptr->track.waypoints.push_back(msg->position);

    // Estimate UAV trajectory for an upcoming time_horizon
    // and with a time separation between waypoints of dT
    // Assume time_horizon is multiple of dT
    int num_estimated_waypoints = (int)(operation_ptr->time_horizon/operation_ptr->dT);

    // Delete previous estimation
    operation_ptr->estimated_trajectory.waypoints.clear();
    gauss_msgs::WaypointList new_estimated_trajectory;

    // Two options possible (for an oversimplified tracking): 
    //   - If there is not enough tracking history, we must estimate trajectory using flight plan
    //   - If there is enough tracking history, we can estimate trajectory only using previous tracking data
    
    // If enough tracking history
    int track_len = operation_ptr->track.waypoints.size();
    gauss_msgs::Waypoint &aux_waypoint_1 = operation_ptr->track.waypoints[track_len - 2];
    gauss_msgs::Waypoint &aux_waypoint_2 = operation_ptr->track.waypoints[track_len - 1];  

    double estimated_speed_x = (aux_waypoint_2.x - aux_waypoint_1.x)/((aux_waypoint_2.stamp - aux_waypoint_1.stamp).toSec());
    double estimated_speed_y = (aux_waypoint_2.y - aux_waypoint_1.y)/((aux_waypoint_2.stamp - aux_waypoint_1.stamp).toSec());
    double estimated_speed_z = (aux_waypoint_2.z - aux_waypoint_1.z)/((aux_waypoint_2.stamp - aux_waypoint_1.stamp).toSec());

    gauss_msgs::Waypoint previous_waypoint;
    previous_waypoint = aux_waypoint_2;
    double &dT = operation_ptr->dT;
    for (int i=0; i < num_estimated_waypoints; i++)
    {
        gauss_msgs::Waypoint waypoint;
        waypoint.x = previous_waypoint.x + estimated_speed_x * dT;
        waypoint.y = previous_waypoint.y + estimated_speed_y * dT;
        waypoint.z = previous_waypoint.z + estimated_speed_z * dT;
        new_estimated_trajectory.waypoints.push_back(waypoint);
    }

    operation_ptr->estimated_trajectory.waypoints = new_estimated_trajectory.waypoints;

    // Write the modified operation in the database
    write_operation_msg_.request.operation.push_back(*operation_ptr);

    // TODO: Publish modified operation only for debugging purposes
    new_operation_pub_.publish(*operation_ptr);

    if ( write_operation_client_.call(write_operation_msg_) || !write_operation_msg_.response.success )
    {
        // TODO: A more complex error handling mechanism must be implemented.
        // This mechanism should retry the call to the service in case it has failed
        ROS_ERROR("Failed writing operation to database");
        write_operation_msg_.request.operation.clear();
        return;
    }
    else
    {
        ROS_INFO("Succesful writing operation to database");
        write_operation_msg_.request.operation.clear();
    }
}
*/

// PositionReport callback
void Tracking::positionReportCB(const gauss_msgs::PositionReport::ConstPtr &msg)
{
    gauss_msgs::ReadOperation read_operation_msg;
    ROS_INFO_STREAM("Received position report from uav_id: " << (int)msg->uav_id);
    // TODO: At first we only consider messages whose source is the RPAStateInfo and not those coming from ADSB
    if (use_only_position_report_ && msg->source == msg->SOURCE_RPA)
    {
        ROS_INFO("Creating candidate for target tracking");
        Candidate *candidate_aux_ptr = new Candidate;
        candidate_aux_ptr->uav_id = msg->uav_id;
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
	    candidate_aux_ptr->speed_covariance(0,0) = 0.001;//msg->confidence;
        candidate_aux_ptr->speed_covariance(0,1) = 0.0;
        candidate_aux_ptr->speed_covariance(0,2) = 0.0;
        candidate_aux_ptr->speed_covariance(1,0) = 0.0;
	    candidate_aux_ptr->speed_covariance(1,1) = 0.001;//msg->confidence;
        candidate_aux_ptr->speed_covariance(1,2) = 0.0;
        candidate_aux_ptr->speed_covariance(2,0) = 0.0;
        candidate_aux_ptr->speed_covariance(2,1) = 0.0;
	    candidate_aux_ptr->speed_covariance(2,2) = 0.001;//msg->confidence;
        candidate_aux_ptr->source = Candidate::POSITIONREPORT;
        candidate_aux_ptr->speed_available = false;
        // TODO: Fill those speed values with real info from uav
        candidate_aux_ptr->speed(0) = 0.0;
        candidate_aux_ptr->speed(1) = 0.0;
        candidate_aux_ptr->speed(2) = 0.0;
        candidate_aux_ptr->timestamp = msg->header.stamp;
        candidates_list_mutex_.lock(); // Necessary because callbacks are going to be executed in separate threads concurrently
        candidates_.push_back(candidate_aux_ptr);
        candidates_list_mutex_.unlock();
    }
}

bool Tracking::checkTargetAlreadyExist(std::string icao_address)
{
    //TODO:
    return true;
}

bool Tracking::checkTargetAlreadyExist(uint8_t uav_id)
{
    bool found = false;

	auto it = targets_.find(uav_id);
	if(it != targets_.end())
		found = true;
	
	return found;
}

bool Tracking::writeOperationsToDatabase()
{
    bool result = true;

    for(auto it = operations_.begin(); it != operations_.end(); ++it)
	{
        if(modified_operations_flags_[it->first] == true)
        {
            write_operation_msg_.request.uav_ids.push_back(it->first);
            write_operation_msg_.request.operation.push_back(it->second);
            std::cout << "Writing UAV ID " << (int)it->first << " operation on database\n";
            std::cout << "Estimated trajectory waypoint count: " << it->second.estimated_trajectory.waypoints.size() << std::endl;
            modified_operations_flags_[it->first] = false;
        }
	}
    
    if(!write_operation_msg_.request.uav_ids.empty())
    {
        if ( !write_operation_client_.call(write_operation_msg_) || !write_operation_msg_.response.success )
        {
            // TODO: A more complex error handling mechanism must be implemented.
            // This mechanism should retry the call to the service in case it has failed
            ROS_ERROR("Failed writing operation to database");
            ROS_ERROR_STREAM("Message: " << write_operation_msg_.response.message);
            result = false;
        }
        else
        {
            ROS_INFO("Succesful writing operation to database");
            result = true;
        }
    }
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
    // TODO: 
    for(auto it=targets_.begin(); it!=targets_.end(); ++it)
    {
        gauss_msgs::Waypoint waypoint_aux;
        waypoint_aux.stamp = it->second->currentPositionTimestamp();
        it->second->getPose(waypoint_aux.x,waypoint_aux.y,waypoint_aux.z);
        operations_[it->first].track.waypoints.push_back(waypoint_aux); // If the target has no associated operation already
        // downloaded because it was not registered in the database, this sentence will create an empty operation for that uav_id
        // which should already be considered a non cooperative one.
        operations_[it->first].uav_id = it->first; // Equal operation uav_id to target uav_id. Only needed for the empty operation
        // created for non-cooperative uav
        if (operations_[it->first].track.waypoints.size() > 1)
        {
            auto it_last_element = operations_[it->first].track.waypoints.rbegin();
            auto it_first_element = operations_[it->first].track.waypoints.begin();
            operations_[it->first].time_tracked = (it_last_element)->stamp.toSec() - (it_first_element)->stamp.toSec();
        }
        else
        {
            operations_[it->first].time_tracked = 0;
        }
        modified_operations_flags_[it->first] = true;
    }
}

void Tracking::estimateTrajectory()
{
    // For estimation of trajectory, first we must check if the operation have an associated flight plan i.e. the uav 
    // is a cooperative one. For now, we consider every uav is cooperative.
    for(auto it=operations_.begin(); it!=operations_.end(); ++it)
    {
        uint8_t uav_id = it->first;
        std::cout << "Estimating trajectory for uav_id: " << int(uav_id) << std::endl;
        gauss_msgs::Operation &operation_aux = it->second;
        gauss_msgs::Waypoint current_waypoint;
        gauss_msgs::WaypointList &flight_plan_ref = operation_aux.flight_plan;
        gauss_msgs::WaypointList &estimated_trajectory = operation_aux.estimated_trajectory;
        // TODO: An efficiency improvent will be to not clear completely the estimated trajectory, but remove
        // some elements and add others. That will require additional state control mechanism.
        estimated_trajectory.waypoints.clear();
        current_waypoint.stamp = targets_[uav_id]->currentPositionTimestamp();
        targets_[uav_id]->getPose(current_waypoint.x, current_waypoint.y, current_waypoint.z);
        double distance_to_wp;
        // For each tracked uav, find the closest waypoint of its flight plan to its current estimated position
        uint32_t flight_plan_wp_index = this->findClosestWaypointIndex(flight_plan_ref, current_waypoint, distance_to_wp);

        // If the operation correspond to an untracked uav_id, the returned waypoint index will be -1
        if (flight_plan_wp_index == -1)
        {
            // Trajectory estimation for non-cooperative uav
            std::cout << "Trajectory estimation for non-cooperative uav\n";
            operation_aux.dT = 1.0;
            operation_aux.time_horizon = 100;
            std::cout << "Using n prediction steps" << std::endl;
            uint32_t number_estimated_wps = operation_aux.time_horizon/operation_aux.dT;
            std::cout << "n_wp: " << number_estimated_wps << "\n";
            targets_[uav_id]->predictNTimes(number_estimated_wps, operation_aux.dT, estimated_trajectory);
        }
        else
        {
            // Trajectory estimation for cooperative uav
            operation_aux.current_wp = flight_plan_wp_index;
            std::cout << "Trajectory estimation for cooperative uav\n";
            // If this distance is smaller than a threshold, the estimated trajectory will be exactly the next waypoints of the flight plan
            double distance_wp_threshold = 10.0; // TODO: as parameter
            double dt = operation_aux.dT;
            if (dt == 0) 
            {
                operation_aux.dT = 1.0; 
                dt = 1.0;
            }
            if (operation_aux.time_horizon == 0)
            {
                ROS_WARN("Trajectory estimation time horizon is 0");
                operation_aux.time_horizon = 100;
            }
            uint32_t number_estimated_wps = operation_aux.time_horizon/dt;
            
            std::cout << "n_wp: " << number_estimated_wps << "\n";
            std::cout << "Current waypoint: " << current_waypoint.x << "," << current_waypoint.y << "," << current_waypoint.z << "\n";
            std::cout << "Flight plan waypoint index: " << flight_plan_wp_index << "\n";
            std::cout << "Flight plan waypoint: " << flight_plan_ref.waypoints[flight_plan_wp_index].x << ",";
            std::cout << flight_plan_ref.waypoints[flight_plan_wp_index].y << ",";
            std::cout << flight_plan_ref.waypoints[flight_plan_wp_index].z << "\n";
            std::cout << "Distance to waypoint: " << distance_to_wp << "\n";

            int remaining_flight_plan_wps = (flight_plan_ref.waypoints.size()-1) - flight_plan_wp_index;
        
            std::cout << "Remaining waypoints count: " << remaining_flight_plan_wps << "\n"; 

            std::cout << "time_horizon: " << operation_aux.time_horizon << std::endl;

            if (remaining_flight_plan_wps < number_estimated_wps)
            {
                std::cout << "Remaining flight plan waypoints are less than desired estimated trajectory waypoints number" << std::endl;
                // If flight plan remaining waypoints span a period of time shorter than the estimated trajectory time_horizon
                // equal that time horizon to the timespan of the remaining flight plan
                number_estimated_wps = remaining_flight_plan_wps;
                operation_aux.time_horizon = number_estimated_wps*dt;
            }
            if (distance_to_wp <= distance_wp_threshold)
            {
                std::cout << "Using flight plan" << std::endl;
                std::cout << "flight plan size: " << flight_plan_ref.waypoints.size() << std::endl;
                // If distance from current estimated position is close enough to flight plan, the estimated trajectory
                // will be composed of the immediately following waypoints.
                gauss_msgs::Waypoint aux_waypoint;
                for(int i=0; i < number_estimated_wps; ++i)
                {
                    aux_waypoint = flight_plan_ref.waypoints[flight_plan_wp_index+1+i];
                    ros::Duration aux_duration;
                    aux_waypoint.stamp = current_waypoint.stamp + aux_duration.fromSec((1+i)*dt);
                    estimated_trajectory.waypoints.push_back(aux_waypoint);
                }
            }
            else
            {
                std::cout << "Using n prediction steps" << std::endl;
                // If distance from current estimated position is too far from the flight plan, the estimated trajectory
                // will be composed of waypoints predicted based on the current estimated speed and position of the uav
                targets_[uav_id]->predictNTimes(number_estimated_wps, dt, estimated_trajectory);
            }
        }
    }
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
        gauss_msgs::ReadOperation read_operation_msg;
        // Retrieve all operations that are not already downloaded from the database
        // We must iterate over the candidates list to see if we already have an operation registered for each uav_id
        candidates_list_mutex_.lock();
        bool new_targets = false;
        for(auto it=candidates_.begin(); it!=candidates_.end(); ++it)
        {
            std::cout << "Traversing candidates list" << std::endl;
            // Check if we have the associated operation in memory for each uav_id from which position reports have been received
            if ( !this->checkTargetAlreadyExist((*it)->uav_id))
            {
                // If there isn't already a TargetTracker and an operation for this uav,
                // the associated operation must be pulled from the database
                // If there is no associated operation, the uav should be marked as a non cooperative one

                std::cout << "uav_id : " << (int)(*it)->uav_id << " not already tracked" << std::endl;
                // Read Operation from Database with a matching UAV id
                read_operation_msg.request.uav_ids.push_back((*it)->uav_id);
                new_targets = true;
            }
            else
            {
                std::cout << "uav_id : " << (int)(*it)->uav_id << " already tracked" << std::endl;
            }
            
        }
        candidates_list_mutex_.unlock();
        if (new_targets)
        {
            if (!read_operation_client_.call(read_operation_msg))
            {
                // TODO: A more complex error handling mechanism must be implemented.
                // This mechanism should retry the call to the service in case it has failed
                ROS_ERROR("Failed reading operations from database");
            }
            // This case should not happen because all uavs that report its position through RPAStateInfo should have 
            // a registered operation. This case could happen only for ADSB received positions
            else if (!read_operation_msg.response.success)
            {
                // TODO: That may indicate that the uav has no associated operation an as such, should be treated as non cooperative one
                // TODO: Database manager should return a boolean array for indicating individually if each operation has been retrieved
                // succesfully
                //cooperative_uavs_[msg->uav_id] = false;
                //operations_[msg->uav_id] = gauss_msgs::Operation();
                // TODO: Create operation for this UAV that is non cooperative

                // TODO: Delete candidates_ that are non cooperative until implementation of new service message
                std::cout << read_operation_msg.response.message << std::endl;
            }
            else
            {
                std::vector<gauss_msgs::Operation> &r_operation_vector = read_operation_msg.response.operation;
                for(auto it=r_operation_vector.begin(); it!=r_operation_vector.end(); ++it)
                {
                    cooperative_uavs_[it->uav_id] = true;
                    ROS_INFO_STREAM("Succesful reading operation of uav " << it->uav_id << "from database");
                    operations_[it->uav_id] = (*it);
                    std::cout << "Time horizon after reading: " << operations_[it->uav_id].time_horizon << std::endl;
                    operations_[it->uav_id].track.waypoints.clear(); // TODO: Clear track although it should be empty at initialization
                    modified_operations_flags_[it->uav_id] = false;
                }
            }
        }

        std::cout << "Operations size start of the loop: " << operations_.size() << std::endl;

        ros::Time now(ros::Time::now());
        this->predict(now); // Predict the position
        this->update(candidates_);

        std::cout << "Operations size after update: " << operations_.size() << std::endl;

        this->fillTrackingWaypointList();

        std::cout << "Operations size after fillTracking: " << operations_.size() << std::endl;
        this->estimateTrajectory();
        for(int i = 0; i < candidates_.size(); i++)
			delete candidates_[i];

        candidates_.clear();

        counter++;
        // Write on database each five iterations of the filter to avoid unnecessary overload
        if (counter == 5)
        {
            counter = 0;
            this->writeOperationsToDatabase();
        }
        rate.sleep();
        std::cout << "Cycle time: " << rate.cycleTime() << std::endl;
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
