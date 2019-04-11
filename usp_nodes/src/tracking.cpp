#include <ros/ros.h>
#include <gauss_msgs/PositionReport.h>
#include <gauss_msgs/Track.h>
#include <gauss_msgs/Plan.h>
#include <gauss_msgs/ReadTrack.h>
#include <gauss_msgs/WriteTrack.h>
#include <gauss_msgs/ReadPlan.h>
#include <gauss_msgs/GetTrack.h>


// Class definition
class Tracking
{
public:
    Tracking();

private:
    // Topic Callbacks
    void positionReportCB(const gauss_msgs::PositionReport::ConstPtr& msg);

    // Service Callbacks
    bool getTrackCB(gauss_msgs::GetTrack::Request &req, gauss_msgs::GetTrack::Response &res);

    // Auxilary methods
    void updateTrack(gauss_msgs::Track *track, gauss_msgs::Plan plan, gauss_msgs::PositionReport position);

    // Auxilary variables

    ros::NodeHandle nh_;


    // Subscribers
    ros::Subscriber pos_report_sub_;

    // Publisher

    // Timer

    // Server
    ros::ServiceServer get_track_server_;

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
    get_track_server_= nh_.advertiseService("/gauss/get_track",&Tracking::getTrackCB,this);

    // Client
    read_track_client_ = nh_.serviceClient<gauss_msgs::ReadTrack>("/gauss/readTrack");
    write_track_client_ = nh_.serviceClient<gauss_msgs::WriteTrack>("/gauss_msgs/writeTrack");
    read_plan_client_ = nh_.serviceClient<gauss_msgs::ReadPlan>("/gauss/readPlan");


    ROS_INFO("Started Tracking node!");
}


// Auxilary methods
void Tracking::updateTrack(gauss_msgs::Track *track, gauss_msgs::Plan plan, gauss_msgs::PositionReport position)
{
    // Filtro para actualizar track
    // Considerar casos en los que no tengamos track o plan de entrada
}


// PositionReport callback
void Tracking::positionReportCB(const gauss_msgs::PositionReport::ConstPtr &msg)
{
    int id=msg->rpa_state.id;
    gauss_msgs::Track track;

    gauss_msgs::ReadTrack read_track_msg;
    gauss_msgs::ReadPlan read_plan_msg;
    gauss_msgs::WriteTrack write_track_msg;

    if (msg->source==msg->SOURCE_RPA)
    {
        read_track_msg.request.id=id;
        read_plan_msg.request.id=id;
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
    }
}

// GetTracks callback
bool Tracking::getTrackCB(gauss_msgs::GetTrack::Request &req, gauss_msgs::GetTrack::Response &res)
{
    int id = req.id;

    gauss_msgs::ReadTrack read_msg;

    read_msg.request.id=id;

    if (!read_track_client_.call(read_msg) || !read_msg.response.success)
    {
        ROS_ERROR("Failed reading track from database");
        res.success=false;
    }
    else
    {
        res.track=read_msg.response.track;
        res.success=true;
    }
    return res.success;
}



// MAIN function
int main(int argc, char *argv[])
{
    ros::init(argc,argv,"tracking");

    // Create a Tracking object
    Tracking *tracking = new Tracking();

    ros::spin();
}
