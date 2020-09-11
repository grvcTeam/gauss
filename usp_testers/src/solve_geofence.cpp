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
#include <gauss_msgs/Notification.h>
#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/Path.h>
#include <Eigen/Eigen>



// Class definition
class Tester
{
public:
    Tester();
    bool writeDB();
    ros::ServiceClient write_deconfliction_client_, read_plan_client_, read_geofence_client_;
    ros::Publisher pub_geofence_, pub_flight_plan_1_, pub_flight_plan_2_, pub_flight_plan_3_, pub_astar_plan_1_, pub_astar_plan_2_, pub_astar_plan_3_, pub_notification_;
    geometry_msgs::PolygonStamped circleToPolygon(float _x, float _y, float _radius, float _nVertices = 9);
    std::vector<nav_msgs::Path> convertFlightPlans(const std::vector<gauss_msgs::WaypointList> &_plans);

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
    pub_flight_plan_1_ = nh_.advertise<nav_msgs::Path>("/gauss/tester/flight_plan_1", 1);
    pub_flight_plan_2_ = nh_.advertise<nav_msgs::Path>("/gauss/tester/flight_plan_2", 1);
    pub_flight_plan_3_ = nh_.advertise<nav_msgs::Path>("/gauss/tester/flight_plan_3", 1);
    pub_astar_plan_1_ = nh_.advertise<nav_msgs::Path>("/gauss/tester/astar_plan_1", 1);
    pub_astar_plan_2_ = nh_.advertise<nav_msgs::Path>("/gauss/tester/astar_plan_2", 1);
    pub_astar_plan_3_ = nh_.advertise<nav_msgs::Path>("/gauss/tester/astar_plan_3", 1);
    pub_notification_ = nh_.advertise<gauss_msgs::Notification>("notification", 1);
    // Subscribe
    // Server
    // Client
    write_deconfliction_client_ = nh_.serviceClient<gauss_msgs::Deconfliction>("/gauss/tactical_deconfliction");
    read_plan_client_ = nh_.serviceClient<gauss_msgs::ReadFlightPlan>("/gauss/read_flight_plan");
    read_geofence_client_ = nh_.serviceClient<gauss_msgs::ReadGeofences>("/gauss/read_geofences");

    ROS_INFO("Started Tester node!");
}

geometry_msgs::PolygonStamped Tester::circleToPolygon(float _x, float _y, float _radius, float _nVertices){
    geometry_msgs::PolygonStamped out_polygon;
    out_polygon.header.frame_id = "world";
    Eigen::Vector2d centerToVertex(_radius, 0.0), centerToVertexTemp;
    for (int i = 0; i < _nVertices; i++) {
        double theta = i * 2 * M_PI / (_nVertices - 1);
        Eigen::Rotation2D<double> rot2d(theta);
        centerToVertexTemp = rot2d.toRotationMatrix() * centerToVertex;
        geometry_msgs::Point32 temp_point;
        temp_point.x = _x + centerToVertexTemp[0];
        temp_point.y = _y + centerToVertexTemp[1];
        out_polygon.polygon.points.push_back(temp_point);
    }

    return out_polygon;
}

std::vector<nav_msgs::Path> Tester::convertFlightPlans(const std::vector<gauss_msgs::WaypointList> &_plans){
    std::vector<nav_msgs::Path> out_paths;
    
    for (auto plan : _plans){
        nav_msgs::Path temp_path;
        temp_path.header.frame_id = "world";
        for (auto wp : plan.waypoints){
            geometry_msgs::PoseStamped temp_pose;
            temp_pose.pose.position.x = wp.x;
            temp_pose.pose.position.y = wp.y;
            temp_pose.pose.position.z = wp.z;
            temp_pose.header.stamp = wp.stamp; 
            temp_path.poses.push_back(temp_pose);
        }
        out_paths.push_back(temp_path);
    }


    return out_paths;
}

// MAIN function
int main(int argc, char *argv[])
{
    ros::init(argc,argv,"Tester");
    Tester tester;
    ros::Rate rate(30);
    int test_geofence_id = 0;
    int test_uav_id = 0;
    gauss_msgs::ReadFlightPlan plan_msg;
    plan_msg.request.uav_ids.push_back(0);
    plan_msg.request.uav_ids.push_back(1);
    plan_msg.request.uav_ids.push_back(2);
    if (!tester.read_plan_client_.call(plan_msg) || !plan_msg.response.success)
    {
        ROS_ERROR("Failed to read a flight plan");
        return false;
    }
    plan_msg.response.plans;
    std::vector<nav_msgs::Path> res_paths = tester.convertFlightPlans(plan_msg.response.plans);

    gauss_msgs::ReadGeofences geofence_msg;
    geofence_msg.request.geofences_ids.push_back(test_geofence_id);
    if (!tester.read_geofence_client_.call(geofence_msg) || !geofence_msg.response.success)
    {
        ROS_ERROR("Failed to read a geofence");
        return false;
    }
    geometry_msgs::PolygonStamped res_polygon;
    res_polygon.header.frame_id = "world";
    if (geofence_msg.response.geofences.front().cylinder_shape){
        res_polygon = tester.circleToPolygon(geofence_msg.response.geofences.front().circle.x_center, 
                                             geofence_msg.response.geofences.front().circle.y_center,
                                             geofence_msg.response.geofences.front().circle.radius);
    } else {
        for (int i = 0; i < geofence_msg.response.geofences.front().polygon.x.size(); i++){
            geometry_msgs::Point32 temp_points;
            temp_points.x = geofence_msg.response.geofences.front().polygon.x.at(i);
            temp_points.y = geofence_msg.response.geofences.front().polygon.y.at(i);
            res_polygon.polygon.points.push_back(temp_points);
        }
    }

    gauss_msgs::Deconfliction deconfliction;
    // deconfliction.request.threat.geofence_ids.push_back(test_geofence_id); // Comment for -> Loss of separation
    deconfliction.request.threat.threat_id = deconfliction.request.threat.LOSS_OF_SEPARATION;
    deconfliction.request.threat.priority_ops.push_back(1); // Uncomment for -> Loss of separation
    deconfliction.request.threat.priority_ops.push_back(1); // Uncomment for -> Loss of separation
    deconfliction.request.threat.times.push_back(ros::Time(15.0));
    deconfliction.request.threat.times.push_back(ros::Time(15.0));
    deconfliction.request.threat.uav_ids.push_back(0);
    deconfliction.request.threat.uav_ids.push_back(2); // Uncomment for -> Loss of separation
    deconfliction.request.tactical = true;

    if (!tester.write_deconfliction_client_.call(deconfliction) || !deconfliction.response.success)
    {
        ROS_ERROR("Call write deconfliction error");
        return false;
    } else {
        for (auto plan : deconfliction.response.deconfliction_plans){
            std::cout << plan << std::endl;
        }
    }

    int change_path_to_pub = 1;
    nav_msgs::Path astar_path_1, astar_path_2, astar_path_3;
    astar_path_1.header.frame_id = astar_path_2.header.frame_id = astar_path_3.header.frame_id = "world";
    for (auto plans : deconfliction.response.deconfliction_plans){
        for (auto wps : plans.waypoint_list){
            geometry_msgs::PoseStamped temp_pose;
            temp_pose.pose.position.x = wps.x;
            temp_pose.pose.position.y = wps.y;
            temp_pose.pose.position.z = wps.z;
            temp_pose.header.stamp = wps.stamp;
            if (change_path_to_pub == 3){
                astar_path_3.poses.push_back(temp_pose);
            } else if (change_path_to_pub == 2){
                astar_path_2.poses.push_back(temp_pose);
            } else if (change_path_to_pub == 1){
                astar_path_1.poses.push_back(temp_pose);
            }
        }
        change_path_to_pub++;
    }

    gauss_msgs::Notification notification;
    notification.uav_id = 0;
    notification.threat.threat_id = notification.threat.LOSS_OF_SEPARATION;
    notification.maneuver_type = 1;
    
    // TO TEST. DELETE THIS
    // gauss_msgs::Waypoint wp;
    // wp.x = 5.0;
    // wp.y = 4.0;
    // wp.z = 1.0;
    // wp.stamp = ros::Time(15.0);
    // notification.waypoints.push_back(wp);
    // TO TEST. DELETE THIS
    
    for (auto i : astar_path_1.poses){
        gauss_msgs::Waypoint wp;
        wp.x = i.pose.position.x;
        wp.y = i.pose.position.y;
        wp.z = i.pose.position.z;
        wp.stamp = i.header.stamp;
        notification.waypoints.push_back(wp);
    }

    while(ros::ok()){
        tester.pub_flight_plan_1_.publish(res_paths.at(0));
        tester.pub_flight_plan_2_.publish(res_paths.at(1));
        tester.pub_flight_plan_3_.publish(res_paths.at(2));
        tester.pub_geofence_.publish(res_polygon);
        tester.pub_astar_plan_1_.publish(astar_path_1);
        tester.pub_astar_plan_2_.publish(astar_path_2);
        tester.pub_astar_plan_3_.publish(astar_path_3);
        if (std::cin.get() == '\n'){
            tester.pub_notification_.publish(notification);
        }        
        rate.sleep();
        ros::spinOnce();
    }


}
