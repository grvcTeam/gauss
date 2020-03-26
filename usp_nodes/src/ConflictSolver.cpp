#include <ros/ros.h>
#include <gauss_msgs/Deconfliction.h>
#include <gauss_msgs/Conflict.h>
#include <gauss_msgs/Waypoint.h>


// Class definition
class ConflictSolver
{
public:
    ConflictSolver();

private:
    // Topic Callbacks

    // Service Callbacks
    bool deconflictCB(gauss_msgs::Deconfliction::Request &req, gauss_msgs::Deconfliction::Response &res);

    // Auxilary methods
    gauss_msgs::Conflict check_conflicts(gauss_msgs::Conflict conflict, gauss_msgs::Waypoint waypoints[]);

    // Auxilary variables

    ros::NodeHandle nh_;

    // Subscribers

    // Publisher

    // Timer

    // Server
    ros::ServiceServer deconflict_server_;

    // Clients

};

// TacticalDeconfliction Constructor
ConflictSolver::ConflictSolver()
{
    // Read parameters
    //nh_.param("desired_altitude",desired_altitude,0.5);

    // Initialization


    // Publish

    // Subscribe

    // Server
    deconflict_server_=nh_.advertiseService("/gauss/tactical_deconfliction",&ConflictSolver::deconflictCB,this);

    // Cient


    ROS_INFO("Started ConflictSolver node!");
}


// auxilary methods
gauss_msgs::Conflict check_conflicts(gauss_msgs::Conflict conflict, gauss_msgs::Waypoint waypoints[])
{

}


// deconflictCB callback
bool ConflictSolver::deconflictCB(gauss_msgs::Deconfliction::Request &req, gauss_msgs::Deconfliction::Response &res)
{
    int num_tracks;
    int num_geofences;
    int *UAV_ids;
    bool tactical=req.tactical;
    gauss_msgs::WaypointList *tracks;
    gauss_msgs::WaypointList *geofences;

    //Deconfliction
    if (req.tactical)
    {
        if (req.conflict.threat_id==req.conflict.LOSS_OF_SEPARATION)
        {

        }
        else if (req.conflict.threat_id==req.conflict.GEOFENCE_CONFLICT)
        {

        }
    }

    return true;
}



// MAIN function
int main(int argc, char *argv[])
{
    ros::init(argc,argv,"conflict_solver");

    // Create a ConflictSolver object
    ConflictSolver *conflict_solver = new ConflictSolver();

    ros::spin();
}
