#include <ros/ros.h>
#include <gauss_msgs/PositionReport.h>
#include <gauss_msgs/ReadTracks.h>
#include <gauss_msgs/ReadFlightPlan.h>
#include <gauss_msgs/WriteTracks.h>



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

    // Timer

    // Server

    // Client
    ros::ServiceClient read_track_client_;
    ros::ServiceClient write_track_client_;
    ros::ServiceClient read_plan_client_;

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
    read_track_client_ = nh_.serviceClient<gauss_msgs::ReadTracks>("/gauss/ReadTracks");
    write_track_client_ = nh_.serviceClient<gauss_msgs::WriteTracks>("/gauss_msgs/writeTracks");
    read_plan_client_ = nh_.serviceClient<gauss_msgs::ReadFlightPlan>("/gauss/readFlightPlan");


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
