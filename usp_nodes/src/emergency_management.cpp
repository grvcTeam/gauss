#include <ros/ros.h>
#include <gauss_msgs/PositionReport.h>
#include <gauss_msgs/trajectory.h>
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
    bool getPositionCB(gauss_msgs::GetTrack::Request &req, gauss_msgs::GetTrack::Response &res);

    // Auxilary variables
    gauss_msgs::PositionReport position;
    gauss_msgs::trajectory track_in;
    gauss_msgs::trajectory track_out;


    ros::NodeHandle nh_;


    // Subscribers
    ros::Subscriber pos_report_sub_;

    // Publisher

    // Timer

    // Server
    ros::ServiceServer position_server_;

};

// positionReporting Constructor
Tracking::Tracking()
{
    // Read parameters
    //nh_.param("desired_altitude",desired_altitude,0.5);


    // Initialization


    // Publish

    // Subscribe
    pos_report_sub_=nh_.subscribe<gauss_msgs::PositionReport>("/gauss/position_report",1,&Tracking::positionReportCB,this);

    // Server
    position_server_=nh_.advertiseService("/gauss/get_tracks",&Tracking::getPositionCB,this);

    ROS_INFO("Started Tracking node!");
}



// PositionReport callback
void Tracking::positionReportCB(const gauss_msgs::PositionReport::ConstPtr &msg)
{
    int id=msg->id;
    position=*msg;

    // si no hay id, buscar en DB de flight plans cual puede ser
    // si no puede ser ninguno, guardar tracks como desconocido id=-1
    // si hay id
    // leer de DB flight plans de id
    // leer de DB tracks de id y pasarlo a tracks_in
    // insertar position en tracks in (filtro)
    // subir de nuevo tracks_in a DB según id
}

// GetTracks callback
bool Tracking::getPositionCB(gauss_msgs::GetTrack::Request &req, gauss_msgs::GetTrack::Response &res)
{
    int id = req.id;

    // Search tracks from id in DB --> track_out

    if (true)  // si hay un track para id en la DB
    {
        res.track=track_out;
        return true;
    }
    return false;
}



// MAIN function
int main(int argc, char *argv[])
{
    ros::init(argc,argv,"tracking");

    // Create a Tracking object
    Tracking *tracking = new Tracking();

    ros::spin();
}
