#include <ros/ros.h>
#include <gauss_msgs/ReadIcao.h>
#include <gauss_msgs/ReadOperation.h>

struct Segment {
    Segment() = default;
    Segment(gauss_msgs::Waypoint a, gauss_msgs::Waypoint b): point_a(a), point_b(b) {}
    gauss_msgs::Waypoint point_a;
    gauss_msgs::Waypoint point_b;
};

void checkSegments(const std::pair<Segment, Segment>& segments) {
    printf("Checking:\n");
    std::cout << segments.first.point_a << "_____________\n" << segments.first.point_b << '\n';
    printf("Against:\n");
    std::cout << segments.second.point_a << "_____________\n" << segments.second.point_b << '\n';
    printf("Done!\n\n");
}

void checkTrajectories(const std::pair<gauss_msgs::WaypointList, gauss_msgs::WaypointList>& trajectories) {
    if (trajectories.first.waypoints.size() < 2) {
        ROS_ERROR("[Monitoring]: trajectory must contain more than 2 points, [%ld] found in first argument", trajectories.first.waypoints.size());
        return;
    }
    if (trajectories.second.waypoints.size() < 2) {
        ROS_ERROR("[Monitoring]: trajectory must contain more than 2 points, [%ld] found in second argument", trajectories.second.waypoints.size());
        return;
    }

    for (int i = 0; i < trajectories.first.waypoints.size() - 1; i++) {
        std::pair<Segment, Segment> segments;
        printf("First segment, i = %d\n", i);
        segments.first = Segment(trajectories.first.waypoints[i], trajectories.first.waypoints[i+1]);
        // std::cout << segments.first.point_a << "_____________\n" << segments.first.point_b << '\n';
        for (int j = 0; j < trajectories.second.waypoints.size() - 1; j++) {
            printf("Second segment, j = %d\n", j);
            segments.second = Segment(trajectories.second.waypoints[j], trajectories.second.waypoints[j+1]);
            // std::cout << segments.second.point_a << "_____________\n" << segments.second.point_b << '\n';
            checkSegments(segments);
        }
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "continuous_monitoring");


    ros::NodeHandle n;
    // ros::NodeHandle np("~");
    ROS_INFO("[Monitoring] Started monitoring node!");

    auto read_icao_srv_url = "/gauss/read_icao";
    auto read_operation_srv_url = "/gauss/read_operation";

    ros::ServiceClient icao_client = n.serviceClient<gauss_msgs::ReadIcao>(read_icao_srv_url);
    ros::ServiceClient operation_client = n.serviceClient<gauss_msgs::ReadOperation>(read_operation_srv_url);

    ROS_INFO("[Monitoring] Waiting for required services...");
    ros::service::waitForService(read_icao_srv_url, -1);
    ROS_INFO("[Monitoring] %s: ok", read_icao_srv_url);
    ros::service::waitForService(read_operation_srv_url, -1);
    ROS_INFO("[Monitoring] %s: ok", read_operation_srv_url);

    ros::Rate rate(1);  // [Hz]
    while (ros::ok()) {

        gauss_msgs::ReadIcao read_icao;
        if (icao_client.call(read_icao)) {
            // ROS_INFO("[Monitoring] Read icao addresses... ok");
            // std::cout << read_icao.response << '\n';
        } else {
            ROS_ERROR("[Monitoring] Failed to call service: [%s]", read_icao_srv_url);
            return 1;
        }

        gauss_msgs::ReadOperation read_operation;
        read_operation.request.uav_ids = read_icao.response.uav_id;
        if (operation_client.call(read_operation)) {
            // ROS_INFO("[Monitoring] Read operations... ok");
            // std::cout << read_operation.response << '\n';
        } else {
            ROS_ERROR("[Monitoring] Failed to call service: [%s]", read_operation_srv_url);
            return 1;
        }

        std::map<std::string, int> icao_to_index_map;
        std::vector<gauss_msgs::WaypointList> estimated_trajectories;
        for (auto operation: read_operation.response.operation) {
            // std::cout << operation << '\n';
            icao_to_index_map[operation.icao_address] = estimated_trajectories.size();
            estimated_trajectories.push_back(operation.estimated_trajectory);
        }

        auto trajectories_count = estimated_trajectories.size();
        if (trajectories_count < 2) { continue; }
        for (int i = 0; i < trajectories_count - 1; i++) {
            std::pair<gauss_msgs::WaypointList, gauss_msgs::WaypointList> trajectories;
            trajectories.first = estimated_trajectories[i];
            for (int j = i + 1; j < trajectories_count; j++){
                printf("[%d, %d]\n", i, j);
                trajectories.second = estimated_trajectories[j];
                checkTrajectories(trajectories);
            }
        }

        ros::spinOnce();
        rate.sleep();
    }

    ros::spin();
    return 0;
}
