#include <ros/ros.h>
#include <gauss_msgs/PositionReport.h>
#include <gauss_msgs/ReadTracks.h>
#include <gauss_msgs/ReadFlightPlan.h>
#include <gauss_msgs/WriteTracks.h>
#include <gauss_msgs/WriteOperation.h>
#include <gauss_msgs/Operation.h>
#include <gauss_msgs/ReadOperation.h>
#include <gauss_msgs/Geofence.h>
#include <gauss_msgs/WriteGeofences.h>
#include <gauss_msgs/ReadGeofences.h>
#include <gauss_msgs/Deconfliction.h>



// Class definition
class Tracking
{
public:
    Tracking();
    bool writeDB();
    ros::ServiceClient write_deconfliction_client_;

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
    ros::ServiceClient write_geofence_client_;
    ros::ServiceClient read_geofence_client_;
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
    read_track_client_ = nh_.serviceClient<gauss_msgs::ReadTracks>("/gauss/read_tracks");
    write_track_client_ = nh_.serviceClient<gauss_msgs::WriteTracks>("/gauss/write_tracks");
    read_plan_client_ = nh_.serviceClient<gauss_msgs::ReadFlightPlan>("/gauss/read_flight_plan");
    write_operation_client_ = nh_.serviceClient<gauss_msgs::WriteOperation>("/gauss/write_operation");
    read_operation_client_ = nh_.serviceClient<gauss_msgs::ReadOperation>("/gauss/read_operation");
    write_geofence_client_ = nh_.serviceClient<gauss_msgs::WriteGeofences>("/gauss/write_geofences");
    write_deconfliction_client_ = nh_.serviceClient<gauss_msgs::Deconfliction>("/gauss/tactical_deconfliction");

    ROS_INFO("Started Tracking node!");
}


// Auxilary methods
bool Tracking::writeDB(){
    gauss_msgs::Operation operation;
    gauss_msgs::Waypoint wp;
    gauss_msgs::WaypointList wp_list;
    wp.x = 1.0;
    wp.y = 4.0;
    wp.z = 1.0;
    wp.stamp = ros::Time(0.0);
    wp_list.waypoints.push_back(wp);
    wp.x = 3.0;
    wp.y = 4.0;
    wp.z = 1.0;
    wp.stamp = ros::Time(5.0);
    wp_list.waypoints.push_back(wp);
    wp.x = 5.0;
    wp.y = 4.0;
    wp.z = 1.0;
    wp.stamp = ros::Time(10.0);
    wp_list.waypoints.push_back(wp);
    wp.x = 7.0;
    wp.y = 4.0;
    wp.z = 1.0;
    wp.stamp = ros::Time(15.0);
    wp_list.waypoints.push_back(wp);
    wp.x = 9.0;
    wp.y = 4.0;
    wp.z = 1.0;
    wp.stamp = ros::Time(20.0);
    wp_list.waypoints.push_back(wp);
    wp.x = 10.0;
    wp.y = 4.0;
    wp.z = 1.0;
    wp.stamp = ros::Time(25.0);
    wp_list.waypoints.push_back(wp);
    operation.flight_plan = wp_list;
    operation.autonomy = 0.0;
    operation.conop = "conop";
    operation.operational_volume = 0.0;
    operation.current_wp = 0;
    operation.dT = 0.0;
    operation.estimated_trajectory = operation.flight_plan;
    operation.flight_geometry = 0.0;
    operation.flight_plan = operation.estimated_trajectory;
    operation.frame = operation.FRAME_ROTOR;
    operation.icao_address = "icaoaddress";
    operation.priority = 0;
    operation.time_horizon = 1000.0;
    operation.time_tracked = 0.0;
    operation.track = operation.estimated_trajectory;
    operation.uav_id = 0;
    gauss_msgs::WriteOperation write_operation;
    write_operation.request.operation.push_back(operation);
    write_operation.request.uav_ids.push_back(0);
    if (!write_operation_client_.call(write_operation) || !write_operation.response.success)
    {
        ROS_ERROR("Call write operation error");
        return false;
    }

    gauss_msgs::Geofence geofence;
    geofence.cylinder_shape = 0;
    geofence.end_time = ros::Time(900.0);
    geofence.id = 0;
    geofence.max_altitude = 10.0;
    geofence.min_altitude = 0.0;
    geofence.polygon.x.push_back(4.0);
    geofence.polygon.y.push_back(6.0);
    geofence.polygon.x.push_back(4.0);
    geofence.polygon.y.push_back(3.0);
    geofence.polygon.x.push_back(8.0);
    geofence.polygon.y.push_back(3.0);
    geofence.polygon.x.push_back(8.0);
    geofence.polygon.y.push_back(6.0);
    geofence.start_time = ros::Time(0.0);
    geofence.static_geofence = 1;
    gauss_msgs::WriteGeofences write_geofence;
    write_geofence.request.geofences.push_back(geofence);
    write_geofence.request.geofence_ids.push_back(0);
    if (!write_geofence_client_.call(write_geofence) || !write_geofence.response.success)
    {
        ROS_ERROR("Call write geofence error");
        return false;
    }

    return true;
}


// PositionReport callback
void Tracking::positionReportCB(const gauss_msgs::PositionReport::ConstPtr &msg)
{
    int id = msg->uav_id;
    double confidence = msg->confidence;
    gauss_msgs::Waypoint position = msg->position;
    int source=msg->source;
    gauss_msgs::WaypointList track;

    gauss_msgs::ReadTracks read_track_msg;
    gauss_msgs::ReadFlightPlan read_plan_msg;
    gauss_msgs::WriteTracks write_track_msg;
    gauss_msgs::WriteOperation write_operation_msg;

    write_operation_msg.request.uav_ids.push_back(0);


    gauss_msgs::Operation operation;
    operation.uav_id = 0;
    operation.icao_address= "pepe";
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
    operation.operational_volume=20;
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

    operation.uav_id = 0;
    operation.icao_address= "pepa";
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
    operation.operational_volume=20;
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
        read_track_msg.request.uas_ids[0]=id;
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
    // Tracking *tracking = new Tracking();
    
    Tracking tracking;
    // tracking.writeDB();
    gauss_msgs::Deconfliction deconfliction;
    deconfliction.request.threat.geofence_ids.push_back(0);
    deconfliction.request.threat.threat_id = deconfliction.request.threat.GEOFENCE_CONFLICT;
    deconfliction.request.threat.times.push_back(ros::Time(0.0));
    deconfliction.request.threat.times.push_back(ros::Time(900.0));
    deconfliction.request.threat.uas_ids.push_back(0);
    deconfliction.request.tactical = true;
    if (!tracking.write_deconfliction_client_.call(deconfliction) || !deconfliction.response.success)
    {
        ROS_ERROR("Call write deconfliction error");
        return false;
    } else {
        for (auto i : deconfliction.response.deconflicted_plans.front().waypoints){
            std::cout << i << std::endl;
        }
    }

    ros::spin();
}
