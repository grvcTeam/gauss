#include <ros/ros.h>
#include <gauss_msgs/Deconfliction.h>
#include <gauss_msgs/Conflict.h>
#include <gauss_msgs/Waypoint.h>
#include <gauss_msgs/CheckConflicts.h>
#include <gauss_msgs/ReadTraj.h>
#include <gauss_msgs/ReadFlightPlan.h>
#include <gauss_msgs/ReadGeofences.h>
#include <usp_nodes/path_finder.h>
#include <Eigen/Eigen>

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
    geometry_msgs::Point findInitAStarPoint(geometry_msgs::Polygon &_polygon, nav_msgs::Path &_path, int &_init_astar_pos);
    geometry_msgs::Point findGoalAStarPoint(geometry_msgs::Polygon &_polygon, nav_msgs::Path &_path, int &_goal_astar_pos);
    int pnpoly(int nvert, std::vector<float> &vertx, std::vector<float> &verty, float testx, float testy);
    std::vector<double> findGridBorders(geometry_msgs::Polygon &_polygon, nav_msgs::Path &_path, geometry_msgs::Point _init_point, geometry_msgs::Point _goal_point);
    geometry_msgs::Polygon circleToPolygon(float _x, float _y, float _radius, float _nVertices = 8);

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
    ros::ServiceClient read_geofence_client_;

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
    read_geofence_client_ = nh_.serviceClient<gauss_msgs::ReadGeofences>("/gauss/readGeofences");

    ROS_INFO("Started ConflictSolver node!");
}

int ConflictSolver::pnpoly(int nvert, std::vector<float> &vertx, std::vector<float> &verty, float testx, float testy) {
    int i, j, c = 0;
    for (i = 0, j = nvert - 1; i < nvert; j = i++) {
        if (((verty.at(i) > testy) != (verty.at(j) > testy)) &&
            (testx < (vertx.at(j) - vertx.at(i)) * (testy - verty.at(i)) / (verty.at(j) - verty.at(i)) + vertx.at(i)))
            c = !c;
    }
    return c;
}

geometry_msgs::Point ConflictSolver::findInitAStarPoint(geometry_msgs::Polygon &_polygon, nav_msgs::Path &_path, int &_init_astar_pos) {
    geometry_msgs::Point out_point;
    std::vector<float> vert_x, vert_y;
    for (int i = 0; i < _polygon.points.size(); i++) {
        vert_x.push_back(_polygon.points.at(i).x);
        vert_y.push_back(_polygon.points.at(i).y);
    }
    vert_x.push_back(_polygon.points.front().x);
    vert_y.push_back(_polygon.points.front().y);
    for (int i = 0; i < _path.poses.size(); i++) {
        if (pnpoly(vert_x.size(), vert_x, vert_y, _path.poses.at(i).pose.position.x, _path.poses.at(i).pose.position.y)) {
            out_point.x = _path.poses.at(i - 1).pose.position.x;
            out_point.y = _path.poses.at(i - 1).pose.position.y;
            out_point.z = _path.poses.at(i - 1).pose.position.z;
            _init_astar_pos = i - 1;
            break;
        }
    }
    return out_point;
}

geometry_msgs::Point ConflictSolver::findGoalAStarPoint(geometry_msgs::Polygon &_polygon, nav_msgs::Path &_path, int &_goal_astar_pos) {
    bool flag1 = false;
    bool flag2 = false;
    geometry_msgs::Point out_point;
    std::vector<float> vert_x, vert_y;
    for (int i = 0; i < _polygon.points.size(); i++) {
        vert_x.push_back(_polygon.points.at(i).x);
        vert_y.push_back(_polygon.points.at(i).y);
    }
    vert_x.push_back(_polygon.points.front().x);
    vert_y.push_back(_polygon.points.front().y);
    for (int i = 0; i < _path.poses.size(); i++) {
        bool in_obstacle = pnpoly(vert_x.size(), vert_x, vert_y, _path.poses.at(i).pose.position.x, _path.poses.at(i).pose.position.y);
        if (in_obstacle && !flag1 && !flag2) flag1 = true;
        if (!in_obstacle && flag1 && !flag2) flag2 = true;
        if (!in_obstacle && flag1 && flag2) {
            out_point.x = _path.poses.at(i).pose.position.x;
            out_point.y = _path.poses.at(i).pose.position.y;
            out_point.z = _path.poses.at(i).pose.position.z;
            _goal_astar_pos = i;
            break;
        }
    }

    if (out_point.x == 0.0 || out_point.y == 0.0) {
        out_point.x = _path.poses.back().pose.position.x;
        out_point.y = _path.poses.back().pose.position.y;
        out_point.z = _path.poses.back().pose.position.z;
    }

    return out_point;
}

std::vector<double> ConflictSolver::findGridBorders(geometry_msgs::Polygon &_polygon, nav_msgs::Path &_path, geometry_msgs::Point _init_point, geometry_msgs::Point _goal_point) {
    geometry_msgs::Point obs_min, obs_max, out_point;
    std::vector<float> vert_x, vert_y;
    for (int i = 0; i < _polygon.points.size(); i++) {
        vert_x.push_back(_polygon.points.at(i).x);
        vert_y.push_back(_polygon.points.at(i).y);
    }
    vert_x.push_back(_polygon.points.front().x);
    vert_y.push_back(_polygon.points.front().y);
    obs_min.x = *std::min_element(vert_x.begin(), vert_x.end());
    obs_min.y = *std::min_element(vert_y.begin(), vert_y.end());
    obs_max.x = *std::max_element(vert_x.begin(), vert_x.end());
    obs_max.y = *std::max_element(vert_y.begin(), vert_y.end());

    std::vector<double> out_grid_borders, temp_x, temp_y;
    temp_x.push_back(_init_point.x);
    temp_x.push_back(_goal_point.x);
    temp_y.push_back(_init_point.y);
    temp_y.push_back(_goal_point.y);
    double min_x = *std::min_element(temp_x.begin(), temp_x.end());
    double min_y = *std::min_element(temp_y.begin(), temp_y.end());
    double max_x = *std::max_element(temp_x.begin(), temp_x.end());
    double max_y = *std::max_element(temp_y.begin(), temp_y.end());

    while (min_x >= obs_min.x || min_y >= obs_min.y || max_x <= obs_max.x || max_y <= obs_max.y) {
        if (min_x >= obs_min.x) min_x = min_x - 1.0;
        if (min_y >= obs_min.y) min_y = min_y - 1.0;
        if (max_x <= obs_max.x) max_x = max_x + 1.0;
        if (max_y <= obs_max.y) max_y = max_y + 1.0;
    }

    out_grid_borders.push_back(min_x);
    out_grid_borders.push_back(min_y);
    out_grid_borders.push_back(max_x);
    out_grid_borders.push_back(max_y);

    return out_grid_borders;
}

geometry_msgs::Polygon ConflictSolver::circleToPolygon(float _x, float _y, float _radius, float _nVertices){
    geometry_msgs::Polygon out_polygon;
    Eigen::Vector2d centerToVertex(_radius, 0.0), centerToVertexTemp;
    for (int i = 0; i < _nVertices; i++) {
        double theta = i * 2 * M_PI / (_nVertices - 1);
        Eigen::Rotation2D<double> rot2d(theta);
        centerToVertexTemp = rot2d.toRotationMatrix() * centerToVertex;
        geometry_msgs::Point32 temp_point;
        temp_point.x = _x + centerToVertexTemp[0];
        temp_point.y = _y + centerToVertexTemp[1];
        out_polygon.points.push_back(temp_point);
    }

    return out_polygon;
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
            gauss_msgs::ReadFlightPlan plan_msg;
            plan_msg.request.uav_ids.push_back(req.conflict.UAV_ids.front());
            if (!read_flightplan_client_.call(plan_msg) || !plan_msg.response.success)
            {
                ROS_ERROR("Failed to read a flight plan");
                res.success=false;
                return false;
            }
            nav_msgs::Path res_path;
            std::vector<double> res_times;
            for (int i = 0; i < plan_msg.response.plans.front().waypoints.size(); i++){
                geometry_msgs::PoseStamped temp_pose;
                temp_pose.pose.position.x = plan_msg.response.plans.front().waypoints.at(i).x;
                temp_pose.pose.position.y = plan_msg.response.plans.front().waypoints.at(i).y;
                temp_pose.pose.position.z = plan_msg.response.plans.front().waypoints.at(i).z;
                res_path.poses.push_back(temp_pose);
                res_times.push_back(plan_msg.response.plans.front().waypoints.at(i).stamp.toSec());
            }
            gauss_msgs::ReadGeofences geofence_msg;
            geofence_msg.request.geofences_ids.push_back(req.conflict.geofence_ids.front());
            if (!read_geofence_client_.call(geofence_msg) || !geofence_msg.response.success)
            {
                ROS_ERROR("Failed to read a geofence");
                res.success=false;
                return false;
            }
            geometry_msgs::Polygon res_polygon;
            if (geofence_msg.response.geofences.front().cylinder_shape){
                res_polygon = circleToPolygon(geofence_msg.response.geofences.front().circle.x_center, 
                                              geofence_msg.response.geofences.front().circle.y_center,
                                              geofence_msg.response.geofences.front().circle.radius);
            } else {
                for (int i = 0; i < geofence_msg.response.geofences.front().polygon.x.size(); i++){
                    geometry_msgs::Point32 temp_points;
                    temp_points.x = geofence_msg.response.geofences.front().polygon.x.at(i);
                    temp_points.y = geofence_msg.response.geofences.front().polygon.y.at(i);
                    res_polygon.points.push_back(temp_points);
                }
            }
            geometry_msgs::Point init_astar_point, goal_astar_point, min_grid_point, max_grid_point;
            int init_astar_pos, goal_astar_pos;
            init_astar_point = findInitAStarPoint(res_polygon, res_path, init_astar_pos);
            goal_astar_point = findGoalAStarPoint(res_polygon, res_path, goal_astar_pos);
            std::vector<double> grid_borders = findGridBorders(res_polygon, res_path, init_astar_point, goal_astar_point);
            min_grid_point.x = grid_borders[0];
            min_grid_point.y = grid_borders[1];
            max_grid_point.x = grid_borders[2];
            max_grid_point.y = grid_borders[3];
            PathFinder path_finder(res_path, init_astar_point, goal_astar_point, res_polygon, min_grid_point, max_grid_point);
            nav_msgs::Path a_star_path_res = path_finder.findNewPath();
            static upat_follower::Generator generator(1.0, 1.0, 1.0);
            std::vector<double> interp_times, a_star_times_res;
            interp_times.push_back(res_times.at(init_astar_pos));
            interp_times.push_back(res_times.at(goal_astar_pos));
            a_star_times_res = generator.interpWaypointList(interp_times, a_star_path_res.poses.size()-1);
            a_star_times_res.push_back(res_times.at(goal_astar_pos));
            // Solutions of conflict solver are a_star_path_res and a_star_times_res
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
