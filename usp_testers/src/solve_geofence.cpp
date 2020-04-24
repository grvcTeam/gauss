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
#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/Path.h>



// Class definition
class Tester
{
public:
    Tester();
    bool writeDB();
    ros::ServiceClient write_deconfliction_client_, read_plan_client_, read_geofence_client_;
    ros::Publisher pub_geofence_, pub_flight_plan_, pub_astar_plan_;

private:
    // Topic Callbacks
    void positionReportCB(const gauss_msgs::PositionReport::ConstPtr& msg);
    // Service Callbacks
    // Auxilary methods
    // Auxilary variables
    ros::NodeHandle nh_;
    // Subscribers
    // Publisher
    // Timer
    // Server
    // Client
};

// Tester Constructor
Tester::Tester()
{
    // Read parameters
    // Initialization
    // Publish
    pub_geofence_ = nh_.advertise<geometry_msgs::PolygonStamped>("/gauss/tester/geofence", 1);
    pub_flight_plan_ = nh_.advertise<nav_msgs::Path>("/gauss/tester/flight_plan", 1);
    pub_astar_plan_ = nh_.advertise<nav_msgs::Path>("/gauss/tester/astar_plan", 1);
    // Subscribe
    // Server
    // Client
    write_deconfliction_client_ = nh_.serviceClient<gauss_msgs::Deconfliction>("/gauss/tactical_deconfliction");
    read_plan_client_ = nh_.serviceClient<gauss_msgs::ReadFlightPlan>("/gauss/read_flight_plan");
    read_geofence_client_ = nh_.serviceClient<gauss_msgs::ReadGeofences>("/gauss/read_geofences");

    ROS_INFO("Started Tester node!");
}

// MAIN function
int main(int argc, char *argv[])
{
    ros::init(argc,argv,"Tester");
    Tester tester;
    ros::Rate rate(30);

    gauss_msgs::ReadFlightPlan plan_msg;
    plan_msg.request.uav_ids.push_back(0);
    if (!tester.read_plan_client_.call(plan_msg) || !plan_msg.response.success)
    {
        ROS_ERROR("Failed to read a flight plan");
        return false;
    }
    nav_msgs::Path res_path;
    res_path.header.frame_id = "world";
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
    geofence_msg.request.geofences_ids.push_back(0);
    if (!tester.read_geofence_client_.call(geofence_msg) || !geofence_msg.response.success)
    {
        ROS_ERROR("Failed to read a geofence");
        return false;
    }
    geometry_msgs::PolygonStamped res_polygon;
    res_polygon.header.frame_id = "world";
    if (geofence_msg.response.geofences.front().cylinder_shape){
        // res_polygon = circleToPolygon(geofence_msg.response.geofences.front().circle.x_center, 
        //                                 geofence_msg.response.geofences.front().circle.y_center,
        //                                 geofence_msg.response.geofences.front().circle.radius);
    } else {
        for (int i = 0; i < geofence_msg.response.geofences.front().polygon.x.size(); i++){
            geometry_msgs::Point32 temp_points;
            temp_points.x = geofence_msg.response.geofences.front().polygon.x.at(i);
            temp_points.y = geofence_msg.response.geofences.front().polygon.y.at(i);
            res_polygon.polygon.points.push_back(temp_points);
        }
    }

    gauss_msgs::Deconfliction deconfliction;
    deconfliction.request.threat.geofence_ids.push_back(0);
    deconfliction.request.threat.threat_id = deconfliction.request.threat.GEOFENCE_INTRUSION;
    deconfliction.request.threat.times.push_back(ros::Time(0.0));
    deconfliction.request.threat.times.push_back(ros::Time(900.0));
    deconfliction.request.threat.uav_ids.push_back(0);
    deconfliction.request.tactical = true;
    if (!tester.write_deconfliction_client_.call(deconfliction) || !deconfliction.response.success)
    {
        ROS_ERROR("Call write deconfliction error");
        return false;
    } else {
        for (auto plan : deconfliction.response.deconflicted_plans){
            // std::cout << plan << std::endl;
        }
    }

    nav_msgs::Path astar_path;
    astar_path.header.frame_id = "world";
    for (auto wps : deconfliction.response.deconflicted_plans.front().waypoints){
        geometry_msgs::PoseStamped temp_pose;
        temp_pose.pose.position.x = wps.x;
        temp_pose.pose.position.y = wps.y;
        temp_pose.pose.position.z = wps.z;
        astar_path.poses.push_back(temp_pose);
    }

    while(ros::ok()){
        tester.pub_flight_plan_.publish(res_path);
        tester.pub_geofence_.publish(res_polygon);
        tester.pub_astar_plan_.publish(astar_path);
        rate.sleep();
        ros::spinOnce();
    }


}
