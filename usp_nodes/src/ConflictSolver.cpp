#include <ros/ros.h>
#include <gauss_msgs/Deconfliction.h>


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
    deconflict_server_=nh_.advertiseService("/gauss/conflict_solver",&ConflictSolver::deconflictCB,this);

    // Cient


    ROS_INFO("Started ConflictSolver node!");
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
