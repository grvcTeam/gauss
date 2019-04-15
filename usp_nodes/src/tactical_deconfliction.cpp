#include <ros/ros.h>
#include <gauss_msgs/Track.h>
#include <gauss_msgs/ReadTrack.h>
#include <gauss_msgs/Deconflict.h>
#include <gauss_msgs/RPICaction.h>
#include <gauss_msgs/Position4D.h>


// Class definition
class TacticalDeconfliction
{
public:
    TacticalDeconfliction();

private:
    // Topic Callbacks

    // Service Callbacks
    bool deconflictCB(gauss_msgs::Deconflict::Request &req, gauss_msgs::Deconflict::Response &res);

    // Auxilary methods
    void Deconfliction(gauss_msgs::Track *tracks, int size);

    // Auxilary variables
    gauss_msgs::RPICaction action;
    gauss_msgs::Position4D **positions;

    ros::NodeHandle nh_;


    // Subscribers

    // Publisher
    ros::Publisher action_pub_;

    // Timer

    // Server
    ros::ServiceServer deconflict_server_;

    // Clients
    ros::ServiceClient read_track_client_;

};

// TacticalDeconfliction Constructor
TacticalDeconfliction::TacticalDeconfliction()
{
    // Read parameters
    //nh_.param("desired_altitude",desired_altitude,0.5);


    // Initialization


    // Publish
    action_pub_ = nh_.advertise<gauss_msgs::RPICaction>("/gauss/rpic_action",1);

    // Subscribe

    // Server
    deconflict_server_=nh_.advertiseService("/gauss/tactical_deconfliction",&TacticalDeconfliction::deconflictCB,this);

    // Cient
    read_track_client_ = nh_.serviceClient<gauss_msgs::ReadTrack>("/gauss/readTrack");


    ROS_INFO("Started TacticalDeconfliction node!");
}

// Auxilary methods
void TacticalDeconfliction::Deconfliction(gauss_msgs::Track *tracks, int size)
{
    // Calcula trayectorias deconflicteadas, usando grid... y guardala en positions (matriz de 2 dimensiones)

}


// deconflictCB callback
bool TacticalDeconfliction::deconflictCB(gauss_msgs::Deconflict::Request &req, gauss_msgs::Deconflict::Response &res)
{
    int size=req.ids.size();
    gauss_msgs::ReadTrack track_msg;
    gauss_msgs::Track tracks[size];

    for (int i=0; i<size; i++)
    {
        track_msg.request.id=req.ids[i];
        if (!read_track_client_.call(track_msg) || !track_msg.response.success)
        {
            ROS_ERROR("[TacticalDeconfliction]: Error reading track.");
            return false;
        }
        else
        {
            ROS_INFO("[TacticalDeconfliction]: Track read.");
            tracks[i]=track_msg.response.track;
        }
    }

    Deconfliction(tracks,size);

    action.header.stamp=ros::Time::now();
    action.action=5; // TBC Avoid drone flght track
    action.action_description="Tactical deconfliction";
    for (int i=0;i<size;i++)
    {
        action.id=req.ids[i];
        //action.positions=positions[i]; rellena action.positions con positions
        action_pub_.publish(action);
    }

    return true;
}



// MAIN function
int main(int argc, char *argv[])
{
    ros::init(argc,argv,"tactical_deconfliction");

    // Create a TacticalDeconfliction object
    TacticalDeconfliction *tactical_deconfliction = new TacticalDeconfliction();

    ros::spin();
}
