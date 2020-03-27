#include <ros/ros.h>
#include <gauss_msgs/PositionReport.h>
#include <gauss_msgs/ReadTracks.h>
#include <gauss_msgs/ReadFlightPlan.h>
#include <gauss_msgs/WriteTracks.h>
#include <gauss_msgs/WriteOperation.h>
#include <gauss_msgs/Operation.h>
#include <gauss_msgs/ReadOperation.h>



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


    // Subscribers
    ros::Subscriber pos_report_sub_;

    // Publisher
    ros::Publisher publicador;

    // Timer

    // Server

    // Client
    ros::ServiceClient read_track_client_;
    ros::ServiceClient write_track_client_;
    ros::ServiceClient read_plan_client_;
    ros::ServiceClient write_operation_client_;
    ros::ServiceClient read_operation_client_;

};

// tracking Constructor
Tracking::Tracking()
{
    // Read parameters
    //nh_.param("desired_altitude",desired_altitude,0.5);


    // Initialization


    // Publish
    publicador=nh_.advertise<gauss_msgs::Operation>("/gauss/operation",1);  //TBD message to UAVs


    // Subscribe
    pos_report_sub_= nh_.subscribe<gauss_msgs::PositionReport>("/gauss/position_report",1,&Tracking::positionReportCB,this);

    // Server

    // Client
    read_track_client_ = nh_.serviceClient<gauss_msgs::ReadTracks>("/gauss/ReadTracks");
    write_track_client_ = nh_.serviceClient<gauss_msgs::WriteTracks>("/gauss_msgs/writeTracks");
    read_plan_client_ = nh_.serviceClient<gauss_msgs::ReadFlightPlan>("/gauss/readFlightPlan");
    write_operation_client_ = nh_.serviceClient<gauss_msgs::WriteOperation>("/gauss/writeOperation");
    read_operation_client_ = nh_.serviceClient<gauss_msgs::ReadOperation>("/gauss/readOperation");


    ROS_INFO("Started Tracking node!");
}


// Auxilary methods



// PositionReport callback
void Tracking::positionReportCB(const gauss_msgs::PositionReport::ConstPtr &msg)
{
    int id = msg->UAV_id;
    double confidence = msg->confidence;
    gauss_msgs::Waypoint position = msg->position;
    int source=msg->source;
    gauss_msgs::WaypointList track;

    gauss_msgs::ReadTracks read_track_msg;
    gauss_msgs::ReadFlightPlan read_plan_msg;
    gauss_msgs::WriteTracks write_track_msg;
    gauss_msgs::WriteOperation write_operation_msg;

    write_operation_msg.request.UAV_ids.push_back(0);


    gauss_msgs::Operation operation;
    operation.UAV_id = 0;
    operation.ICAO_address= "pepe";
    operation.frame=operation.FRAME_ROTOR;
    operation.priority=1;
    operation.current_wp=0;
    gauss_msgs::Waypoint wp;
    wp.x=0;
    wp.y=1;
    wp.z=2;
    wp.stamp=ros::Time(10.0);
    operation.flight_plan.waypoints.push_back(wp);
    wp.stamp=ros::Time(30.0);
    wp.x=10;
    operation.flight_plan.waypoints.push_back(wp);
    operation.dT=10.0;
    operation.time_tracked=0;
    operation.time_horizon=600;
    operation.flight_geometry=10;
    operation.contingency_volume=20;
    operation.conop="nada";
    operation.estimated_trajectory=operation.flight_plan;

    write_operation_msg.request.operation.push_back(operation);

    if (!write_operation_client_.call(write_operation_msg) || !write_operation_msg.response.success)
    {
        ROS_ERROR("error");
    }

    std::getchar();

    gauss_msgs::ReadOperation operation_msg;
    operation_msg.request.uav_ids.push_back(0);
    if (!read_operation_client_.call(operation_msg) || !operation_msg.response.success)
    {
        ROS_ERROR("error reading");
    }
    else
    {
        publicador.publish(operation_msg.response.operation.at(0));

    }

    std::getchar();



    write_operation_msg.request.operation.pop_back();

    operation.UAV_id = 0;
    operation.ICAO_address= "pepa";
    operation.frame=operation.FRAME_ROTOR;
    operation.priority=1;
    operation.current_wp=0;
    wp.x=0;
    wp.y=4;
    wp.z=2;
    wp.stamp=ros::Time(10.0);
    operation.flight_plan.waypoints.push_back(wp);
    wp.stamp=ros::Time(30.0);
    wp.x=10;
    operation.flight_plan.waypoints.push_back(wp);
    operation.dT=10.0;
    operation.time_tracked=0;
    operation.time_horizon=600;
    operation.flight_geometry=10;
    operation.contingency_volume=20;
    operation.conop="nada";
    operation.estimated_trajectory=operation.flight_plan;

    write_operation_msg.request.operation.push_back(operation);

    if (!write_operation_client_.call(write_operation_msg) || !write_operation_msg.response.success)
    {
        ROS_ERROR("error");
    }

    std::getchar();

    operation_msg.request.uav_ids.push_back(0);
    if (!read_operation_client_.call(operation_msg) || !operation_msg.response.success)
    {
        ROS_ERROR("error reading");
    }
    else
    {
        publicador.publish(operation_msg.response.operation.at(0));

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
