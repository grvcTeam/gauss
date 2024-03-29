#include <ros/ros.h>
#include <gauss_msgs/PositionReport.h>
#include <gauss_msgs/WriteOperation.h>
#include <gauss_msgs/Operation.h>
#include <gauss_msgs/ReadOperation.h>
#include <gauss_msgs/Geofence.h>
#include <gauss_msgs/WriteGeofences.h>
#include <gauss_msgs/ReadGeofences.h>
#include <gauss_msgs/Deconfliction.h>
#include <gauss_msgs/Notification.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <Eigen/Eigen>



// Class definition
class Tester
{
public:
    Tester();
    bool writeDB();
    ros::ServiceClient write_deconfliction_client_, write_operation_client_, read_operation_client_, read_geofence_client_;
    ros::Publisher pub_geofence_, pub_flight_plan_1_, pub_flight_plan_2_, pub_flight_plan_3_, pub_astar_plan_1_, pub_astar_plan_2_, pub_astar_plan_3_, pub_notification_;
    ros::Subscriber sub_ual_0_pose_;
    geometry_msgs::PoseStamped ual_0_pose_;
    geometry_msgs::PolygonStamped circleToPolygon(float _x, float _y, float _radius, float _nVertices = 9);
    std::vector<nav_msgs::Path> convertFlightPlans(const std::vector<gauss_msgs::WaypointList> &_plans);
    bool checkDistanceLessThan(const geometry_msgs::Pose &_p1, const geometry_msgs::Pose &_p2, double &_distance);

private:
    // Topic Callbacks
    void ualPoseCB(const geometry_msgs::PoseStamped::ConstPtr &_ual_pose);
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
    sub_ual_0_pose_ = nh_.subscribe("/uav_0/ual/pose", 0, &Tester::ualPoseCB, this);
    // Server
    // Client
    write_deconfliction_client_ = nh_.serviceClient<gauss_msgs::Deconfliction>("/gauss/tactical_deconfliction");
    write_operation_client_ = nh_.serviceClient<gauss_msgs::WriteOperation>("/gauss/write_operation");
    read_operation_client_ = nh_.serviceClient<gauss_msgs::ReadOperation>("/gauss/read_operation");
    read_geofence_client_ = nh_.serviceClient<gauss_msgs::ReadGeofences>("/gauss/read_geofences");

    ROS_INFO("Started Tester node!");
}

void Tester::ualPoseCB(const geometry_msgs::PoseStamped::ConstPtr &_ual_pose) {
    ual_0_pose_ = *_ual_pose;
}

geometry_msgs::PolygonStamped Tester::circleToPolygon(float _x, float _y, float _radius, float _nVertices){
    geometry_msgs::PolygonStamped out_polygon;
    out_polygon.header.frame_id = "map";
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
        temp_path.header.frame_id = "map";
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

bool Tester::checkDistanceLessThan(const geometry_msgs::Pose &_p1, const geometry_msgs::Pose &_p2, double &_check_distance){
    Eigen::Vector3f p1 = Eigen::Vector3f(_p1.position.x, _p1.position.y, _p1.position.z);
    Eigen::Vector3f p2 = Eigen::Vector3f(_p2.position.x, _p2.position.y, _p2.position.z);
    double distance = (p2 - p1).norm();
    if (distance > _check_distance){
        return false;
    } else {
        return true;
    }
}

// MAIN function
int main(int argc, char *argv[])
{
    ros::init(argc,argv,"Tester");
    Tester tester;
    ros::Rate rate(30);
    int test_geofence_id = 0;
    int test_uav_id = 0;
    gauss_msgs::ReadOperation operation_msg;
    operation_msg.request.uav_ids.push_back(0);
    operation_msg.request.uav_ids.push_back(1);
    operation_msg.request.uav_ids.push_back(2);
    if (!tester.read_operation_client_.call(operation_msg) || !operation_msg.response.success)
    {
        ROS_ERROR("Failed to read a flight plan");
        return false;
    }
    std::vector<gauss_msgs::WaypointList> all_flight_plans;
    for (auto i : operation_msg.response.operation) all_flight_plans.push_back(i.flight_plan);
    std::vector<nav_msgs::Path> res_paths = tester.convertFlightPlans(all_flight_plans);

    gauss_msgs::ReadGeofences geofence_msg;
    geofence_msg.request.geofences_ids.push_back(test_geofence_id);
    if (!tester.read_geofence_client_.call(geofence_msg) || !geofence_msg.response.success)
    {
        ROS_ERROR("Failed to read a geofence");
        return false;
    }
    geometry_msgs::PolygonStamped res_polygon;
    res_polygon.header.frame_id = "map";
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
    deconfliction.request.threat.threat_type = deconfliction.request.threat.LOSS_OF_SEPARATION;
    deconfliction.request.threat.priority_ops.push_back(0); // Uncomment for -> Loss of separation
    deconfliction.request.threat.priority_ops.push_back(0); // Uncomment for -> Loss of separation
    deconfliction.request.threat.times.push_back(ros::Time(15.0));
    deconfliction.request.threat.times.push_back(ros::Time(15.0));
    deconfliction.request.threat.uav_ids.push_back(0);
    deconfliction.request.threat.uav_ids.push_back(1); // Uncomment for -> Loss of separation
    deconfliction.request.tactical = true;

    if (!tester.write_deconfliction_client_.call(deconfliction) || !deconfliction.response.success)
    {
        ROS_ERROR("Call write deconfliction error 1");
        return false;
    } 

    int change_path_to_pub = 1;
    nav_msgs::Path astar_path_1, astar_path_2, astar_path_3;
    astar_path_1.header.frame_id = astar_path_2.header.frame_id = astar_path_3.header.frame_id = "map";
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

    gauss_msgs::Notification notification, notification2;
    notification.uav_id = notification2.uav_id = 0;
    notification.threat.threat_type = notification2.threat.threat_type = notification.threat.LOSS_OF_SEPARATION;
    notification.maneuver_type = notification2.maneuver_type = 1;
        
    for (auto i : astar_path_2.poses){
        gauss_msgs::Waypoint wp;
        wp.x = i.pose.position.x;
        wp.y = i.pose.position.y;
        wp.z = i.pose.position.z;
        wp.stamp = i.header.stamp;
        notification.waypoints.push_back(wp);
    }

    double check_distance_send_conflict = 2.0;
    int pos_on_flight_plan_send_conflict = 1;
    geometry_msgs::Pose wp_send_conflict, wp_send_conflict2;
    wp_send_conflict.position.x = operation_msg.response.operation.front().flight_plan.waypoints.at(pos_on_flight_plan_send_conflict).x;
    wp_send_conflict.position.y = operation_msg.response.operation.front().flight_plan.waypoints.at(pos_on_flight_plan_send_conflict).y;
    wp_send_conflict.position.z = operation_msg.response.operation.front().flight_plan.waypoints.at(pos_on_flight_plan_send_conflict).z;
    pos_on_flight_plan_send_conflict = 3;
    wp_send_conflict2.position.x = operation_msg.response.operation.front().flight_plan.waypoints.at(pos_on_flight_plan_send_conflict).x;
    wp_send_conflict2.position.y = operation_msg.response.operation.front().flight_plan.waypoints.at(pos_on_flight_plan_send_conflict).y;
    wp_send_conflict2.position.z = operation_msg.response.operation.front().flight_plan.waypoints.at(pos_on_flight_plan_send_conflict).z;

    bool send_conflict_once = true; // Send a conflict while flying for UAV 0 and UAV 1
    bool send_conflict_once2 = false; // Send a conflict while flying for UAV 0 and UAV 2

    while(ros::ok()){
        tester.pub_flight_plan_1_.publish(res_paths.at(0));
        tester.pub_flight_plan_2_.publish(res_paths.at(1));
        tester.pub_flight_plan_3_.publish(res_paths.at(2));
        tester.pub_geofence_.publish(res_polygon);
        tester.pub_astar_plan_1_.publish(astar_path_1);
        tester.pub_astar_plan_2_.publish(astar_path_2);
        tester.pub_astar_plan_3_.publish(astar_path_3);
        if (tester.checkDistanceLessThan(tester.ual_0_pose_.pose, wp_send_conflict, check_distance_send_conflict) && send_conflict_once){
            tester.pub_notification_.publish(notification);
            send_conflict_once = false;
        }
        if (tester.checkDistanceLessThan(tester.ual_0_pose_.pose, wp_send_conflict2, check_distance_send_conflict) && send_conflict_once2){
            gauss_msgs::ReadOperation operation_read;
            operation_read.request.uav_ids.push_back(0);
            if (!tester.read_operation_client_.call(operation_read) || !operation_read.response.success)
            {
                ROS_ERROR("Failed to read an operation");
                return false;
            }
            gauss_msgs::WriteOperation operation_write;
            operation_write.request.uav_ids.push_back(0);
            operation_write.request.operation.push_back(operation_read.response.operation.front());
            operation_write.request.operation.front().current_wp = 3;
            operation_write.request.operation.front().estimated_trajectory.waypoints.clear();
            for (int i = 0; i < 3; i++){
                gauss_msgs::Waypoint temp_wp;
                temp_wp.x = operation_write.request.operation.front().flight_plan.waypoints.at(operation_write.request.operation.front().current_wp + i).x;
                temp_wp.y = operation_write.request.operation.front().flight_plan.waypoints.at(operation_write.request.operation.front().current_wp + i).y;
                temp_wp.z = operation_write.request.operation.front().flight_plan.waypoints.at(operation_write.request.operation.front().current_wp + i).z;
                temp_wp.stamp = operation_write.request.operation.front().flight_plan.waypoints.at(operation_write.request.operation.front().current_wp + i).stamp;
                operation_write.request.operation.front().estimated_trajectory.waypoints.push_back(temp_wp);
            }
            operation_write.request.operation.front().estimated_trajectory.waypoints.at(0).stamp = ros::Time(30.0); // Uncomment if using 3D path follower 
            operation_write.request.operation.front().estimated_trajectory.waypoints.at(1).stamp = ros::Time(37.5); // Uncomment if using 3D path follower
            operation_write.request.operation.front().estimated_trajectory.waypoints.at(2).stamp = ros::Time(45.0); // Uncomment if using 3D path follower
            if(!tester.write_operation_client_.call(operation_write) || !operation_write.response.success){
                ROS_ERROR("Failed to write an operation");
                return false;
            }
            gauss_msgs::Deconfliction deconfliction2;
            deconfliction2.request.threat.threat_type = deconfliction.request.threat.LOSS_OF_SEPARATION;
            deconfliction2.request.threat.priority_ops.push_back(0); // Uncomment for -> Loss of separation
            deconfliction2.request.threat.priority_ops.push_back(1); // Uncomment for -> Loss of separation
            deconfliction2.request.threat.times.push_back(ros::Time(37.5)); // If using 3D path follower 37.5 else 30.0
            deconfliction2.request.threat.times.push_back(ros::Time(37.5));
            deconfliction2.request.threat.uav_ids.push_back(0);
            deconfliction2.request.threat.uav_ids.push_back(2); // Uncomment for -> Loss of separation
            deconfliction2.request.tactical = true;
            if (!tester.write_deconfliction_client_.call(deconfliction2) || !deconfliction2.response.success)
            {
                ROS_ERROR("Call write deconfliction error 2");
                return false;
            }
            for (auto i : deconfliction2.response.deconfliction_plans.front().waypoint_list){
                gauss_msgs::Waypoint wp;
                wp.x = i.x;
                wp.y = i.y;
                wp.z = i.z;
                wp.stamp = i.stamp;
                notification2.waypoints.push_back(wp);
            }
            tester.pub_notification_.publish(notification2);
            send_conflict_once2 = false;
        }
        
        rate.sleep();
        ros::spinOnce();
    }


}
