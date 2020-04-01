#include <ros/ros.h>
#include <gauss_msgs/PositionReport.h>
#include <gauss_msgs/ReadOperation.h>
#include <gauss_msgs/WriteOperation.h>

#include <gauss_msgs/Operation.h>

// Class definition
class Tracking
{
public:
    Tracking();

private:
    // Topic Callbacks
    void positionReportCB(const gauss_msgs::PositionReport::ConstPtr& msg);

    // Service Callbacks

    // Auxilary methods

    // Auxilary variables
    ros::NodeHandle nh_;
    gauss_msgs::ReadOperation read_operation_msg_;
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

};

// tracking Constructor
Tracking::Tracking()
{
    // Read parameters
    //nh_.param("desired_altitude",desired_altitude,0.5);


    // Initialization


    // Publish

    // Subscribe
    pos_report_sub_= nh_.subscribe<gauss_msgs::PositionReport>("/gauss/position_report",1,&Tracking::positionReportCB,this);

    // Server

    // Client
    read_operation_client_ = nh_.serviceClient<gauss_msgs::ReadOperation>("gauss/readOperation");
    write_operation_client_ = nh_.serviceClient<gauss_msgs::WriteOperation>("gauss/writeOperation");

    // Modified Operation publisher TODO: Only for debugging purposes
    new_operation_pub_ = nh_.advertise<gauss_msgs::Operation>("gauss_test/updated_operation", 5);

    /* Fill uav_ids array with one element, that will be overwritten
       previously to each one of the calls to readOperation service
       By doing this, unnecessary memory reallocation is avoided in each succesive
       execution of positionReportCB callback
    */
    read_operation_msg_.request.uav_ids.push_back(0);
    write_operation_msg_.request.UAV_ids.push_back(0);

    ROS_INFO("Started Tracking node!");
}


// Auxilary methods



// PositionReport callback
void Tracking::positionReportCB(const gauss_msgs::PositionReport::ConstPtr &msg)
{
    gauss_msgs::Operation *operation_ptr;

    /* TODO: if the source of the position report is not the main UAV and instead is provided by
       ADSB surveillance, the UAV_id field should not be used to lookup an operation in the database,
       but instead, the ICAO address must be used in this case
    */
    int id = msg->UAV_id;
    double confidence = msg->confidence;
    gauss_msgs::Waypoint position = msg->position;
    int source=msg->source;
    //gauss_msgs::WaypointList track;

    // Read Operation from Database with a matching UAV id
    read_operation_msg_.request.uav_ids.push_back(id);
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
    write_operation_msg_.request.UAV_ids[0] = id;
    write_operation_msg_.request.operation.push_back(*operation_ptr);
    write_operation_msg_.request.operation.clear();

    // TODO: Publish modified operation only for debugging purposes
    new_operation_pub_.publish(*operation_ptr);

    if ( write_operation_client_.call(write_operation_msg_) || !write_operation_msg_.response.success )
    {
        // TODO: A more complex error handling mechanism must be implemented.
        // This mechanism should retry the call to the service in case it has failed
        ROS_ERROR("Failed writing operation to database");
        return;
    }
    else
    {
        ROS_INFO("Succesful writing operation to database");
    }
    
    /*if (source==msg->SOURCE_RPA)
    {
        read_track_msg.request.UAV_ids[0]=id;
        read_plan_msg.request.id[0]=id;
        read_plan_client_.call(read_plan_msg);
        read_track_client_.call(read_track_msg);

        track=read_track_msg.response.track;
        updateTrack(&track,read_plan_msg.response.plan,*msg);

        write_track_msg.request.id=id;
        write_track_msg.request.track=track;
        if(!write_track_client_.call(write_track_msg) || !write_track_msg.response.success)
            ROS_ERROR("Failed writting track in database");
        else
            ROS_INFO("Tracks Database updated with a new position report");
    }
    else
    {
        // TBD
        // Si source es ADSB, tenemos solo la ICAO address, ¿base de datos que relacione ICAO address con id?
    }*/
}



// MAIN function
int main(int argc, char *argv[])
{
    ros::init(argc,argv,"tracking");

    // Create a Tracking object
    Tracking *tracking = new Tracking();

    ros::spin();
}
