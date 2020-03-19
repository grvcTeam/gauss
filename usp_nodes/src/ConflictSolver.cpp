#include <ros/ros.h>
#include <gauss_msgs/Deconfliction.h>
#include <gauss_msgs/Conflict.h>
#include <gauss_msgs/Waypoint.h>
#include <gauss_msgs/CheckConflicts.h>
#include <gauss_msgs/ReadTraj.h>
#include <gauss_msgs/ReadFlightPlan.h>
#include <usp_nodes/path_finder.h>


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
    double rate;
    double dX,dY,dZ,dT;
    ros::NodeHandle nh_;

    // Subscribers

    // Publisher

    // Timer

    // Server
    ros::ServiceServer deconflict_server_;

    // Clients
    ros::ServiceClient check_client_;
    ros::ServiceClient read_trajectory_client_;
    ros::ServiceClient read_flightplan_client_;

};

// TacticalDeconfliction Constructor
ConflictSolver::ConflictSolver()
{
    // Read parameters
    nh_.param("/gauss/deltaX",dX,10.0);
    nh_.param("/gauss/deltaY",dY,10.0);
    nh_.param("/gauss/deltaZ",dZ,10.0);
    nh_.param("/gauss/monitoring_rate",rate,0.5);


    // Initialization
    dT=1.0/rate;

    // Publish

    // Subscribe

    // Server
    deconflict_server_=nh_.advertiseService("/gauss/tactical_deconfliction",&ConflictSolver::deconflictCB,this);

    // Cient
    check_client_ = nh_.serviceClient<gauss_msgs::CheckConflicts>("/gauss/checkConflicts");
    read_trajectory_client_ = nh_.serviceClient<gauss_msgs::ReadTraj>("/gauss/readEstimatedTrajectory");
    read_flightplan_client_ = nh_.serviceClient<gauss_msgs::ReadFlightPlan>("/gauss/readFlightPlan");

    ROS_INFO("Started ConflictSolver node!");
}



// deconflictCB callback
bool ConflictSolver::deconflictCB(gauss_msgs::Deconfliction::Request &req, gauss_msgs::Deconfliction::Response &res)
{

    //Deconfliction
    if (req.tactical)
    {
        gauss_msgs::Conflict conflict;
        if (req.conflict.threat_id==req.conflict.LOSS_OF_SEPARATION)
        {
            gauss_msgs::Waypoint newwp1,newwp2;
            conflict=req.conflict;


            int num_conflicts=1;
            while (num_conflicts>0)
            {
                bool included1=false;
                bool included2=false;
                for (int i=0;i<res.UAV_ids.size();i++)
                {
                    if (conflict.UAV_ids.at(0)==res.UAV_ids.at(i))
                        included1=true;
                    if (conflict.UAV_ids.at(1)==res.UAV_ids.at(i))
                        included2=true;
                }
                if (included1=false)
                    res.UAV_ids.push_back(conflict.UAV_ids.at(0));
                if (included2=false)
                    res.UAV_ids.push_back(conflict.UAV_ids.at(1));

                num_conflicts=0;
                gauss_msgs::ReadTraj traj_msg;
                traj_msg.request.UAV_ids.push_back(conflict.UAV_ids.at(0));
                traj_msg.request.UAV_ids.push_back(conflict.UAV_ids.at(1));
                if (!read_trajectory_client_.call(traj_msg) || !traj_msg.response.success)
                {
                    ROS_ERROR("Failed to read a trajectory");
                    res.success=false;
                    return false;
                }
                int j=0;
                gauss_msgs::Waypoint wp1,wp2;
                gauss_msgs::WaypointList traj1=traj_msg.response.tracks.at(0);
                gauss_msgs::WaypointList traj2=traj_msg.response.tracks.at(1);
                int UAV1=req.conflict.UAV_ids.at(0);
                int UAV2=req.conflict.UAV_ids.at(1);

                while (abs(traj1.waypoints.at(j).stamp.toSec()-conflict.times.at(0).toSec())>dT);
                wp1=traj1.waypoints.at(j);
                j=0;
                while (abs(traj2.waypoints.at(j).stamp.toSec()-conflict.times.at(1).toSec())>dT);
                wp2=traj2.waypoints.at(j);

                double distance=sqrt(pow(wp2.x-wp1.x,2)+pow(wp2.y-wp1.y,2)+pow(wp2.z-wp1.z,2));
                double separation=0;
                double mod1=sqrt(pow(wp1.x,2)+pow(wp1.y,2)+pow(wp1.z,2));
                double mod2=sqrt(pow(wp2.x,2)+pow(wp2.y,2)+pow(wp2.z,2));
                if (distance<dX)
                    separation=(dX-distance)/2;
                newwp1.x=wp1.x+separation*(wp1.x-wp2.x)/distance;
                newwp1.y=wp1.y+separation*(wp1.y-wp2.y)/distance;
                newwp1.z=wp1.z+separation*(wp1.z-wp2.z)/distance;
                newwp1.stamp=wp1.stamp;
                newwp2.x=wp2.x-separation*(wp1.x-wp2.x)/distance;
                newwp2.y=wp2.y-separation*(wp1.y-wp2.y)/distance;
                newwp2.z=wp2.z-separation*(wp1.z-wp2.z)/distance;
                newwp2.stamp=wp2.stamp;

                gauss_msgs::CheckConflicts check_msg;
                check_msg.request.deconflicted_wp.push_back(newwp1);
                check_msg.request.deconflicted_wp.push_back(newwp2);
                check_msg.request.conflict.threat_id=check_msg.request.conflict.LOSS_OF_SEPARATION;
                check_msg.request.conflict.UAV_ids.push_back(UAV1);
                check_msg.request.conflict.UAV_ids.push_back(UAV2);
                check_msg.request.conflict.times.push_back(newwp1.stamp);
                check_msg.request.conflict.times.push_back(newwp2.stamp);
                if (!check_client_.call(check_msg) || !check_msg.response.success)
                {
                    ROS_ERROR("Failed checking new conflicts");
                    res.success=false;
                    return false;
                }

                num_conflicts=check_msg.response.conflicts.size();

                if (num_conflicts>0)
                    conflict=check_msg.response.conflicts.at(0);
            }
            // Leer flight plans, modificarlo (tiempo y posicion) segun los newwp1 y newwp2
            // incluir nuevos flight plans en

            gauss_msgs::ReadFlightPlan plan_msg;
            for (int i=0;i<res.UAV_ids.size();i++)
                plan_msg.request.uav_ids.push_back(res.UAV_ids.at(i));
            if (!read_flightplan_client_.call(plan_msg) || !plan_msg.response.success)
            {
                ROS_ERROR("Failed to read a flight plan");
                res.success=false;
                return false;
            }

            res.deconflicted_plans.at(0);
            res.deconflicted_plans.at(1);

            res.success=true;

        }
        else if (req.conflict.threat_id==req.conflict.GEOFENCE_CONFLICT)
        {
            // Empty variables
            nav_msgs::Path init_path_;
            geometry_msgs::Point init_astar_point, goal_astar_point, min_grid_point, max_grid_point;
            geometry_msgs::Polygon polygon;
            PathFinder path_finder(init_path_, init_astar_point, goal_astar_point, polygon, min_grid_point, max_grid_point);
            // nav_msgs::Path a_star_path_res = path_finder.findNewPath();
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
