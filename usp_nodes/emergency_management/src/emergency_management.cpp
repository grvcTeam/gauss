#include <ros/ros.h>
// #include <gauss_msgs/Alert.h>
#include <gauss_msgs/Deconfliction.h>
#include <gauss_msgs/WriteGeofences.h>
#include <gauss_msgs/ReadGeofences.h>
#include <gauss_msgs/Notification.h>


// Class definition
class EmergencyManagement
{
public:
    EmergencyManagement();

private:
    // Topic Callbacks

    // Service Callbacks
    // bool alertEmergencyCB(gauss_msgs::Alert::Request &req, gauss_msgs::Alert::Response &res);

    // Auxilary methods


    // Auxilary variables

    ros::NodeHandle nh_;


    // Subscribers

    // Publisher
    ros::Publisher notification_pub_;

    // Timer

    // Server
    ros::ServiceServer alert_server_;

    // Client
    ros::ServiceClient read_geofence_client_;     
    ros::ServiceClient write_geofence_client_;
    ros::ServiceClient deconfliction_client;
};

// EmergencyManagement Constructor
EmergencyManagement::EmergencyManagement()
{
    // Read parameters
    //nh_.param("desired_altitude",desired_altitude,0.5);


    // Initialization


    // Publish
    notification_pub_ = nh_.advertise<gauss_msgs::Notification>("/gauss/notification",1);

    // Subscribe

    // Server
    // alert_server_=nh_.advertiseService("/gauss/alert",&EmergencyManagement::alertEmergencyCB,this);

    // Clients
    read_geofence_client_ = nh_.serviceClient<gauss_msgs::ReadGeofences>("/gauss_msgs/readGeofences");
    write_geofence_client_ = nh_.serviceClient<gauss_msgs::WriteGeofences>("/gauss_msgs/writeGeofences");
    deconfliction_client = nh_.serviceClient<gauss_msgs::Deconfliction>("/gauss/conflict_solver");

    ROS_INFO("Started EmergencyManagement node!");
}

// Auxilary methods


// Alert callback
// bool EmergencyManagement::alertEmergencyCB(gauss_msgs::Alert::Request &req, gauss_msgs::Alert::Response &res)
// {
//     int alert_type=req.code;
//     int alert_level=req.level;
//     int *UAV_ids;
//     int number_of_plans=req.plans.size();
//     gauss_msgs::WaypointList *plans;


    /*switch (alert_type) {
    case 1: // Plan deviation TBC
        track_msg.request.id=req.emergency.ids[0];
        plan_msg.request.id=req.emergency.ids[0];
        if (!read_track_client_.call(track_msg) || !track_msg.response.success)
            ROS_ERROR("[EmergencyManagement]: Failed reading track.");
        else
            ROS_INFO("[EmergencyManagement]:Track read.");
        if (!read_plan_client_.call(plan_msg) || !plan_msg.response.success)
            ROS_ERROR("[EmergencyManagement]: Failed reading plan.");
        else
            ROS_INFO("[EmergencyManagement]:Plan read.");
        action.header.stamp=ros::Time::now();
        action.id=req.emergency.ids[0];;
        action.action_description="Return to predefined flight plan";
        action.action=1; //TBC returnToPlan
        returnToPlan(track_msg.response.track,plan_msg.response.plan);
        action_pub_.publish(action);
        break;
    case 2: // Static Geofence Violation TBC
        track_msg.request.id=req.emergency.ids[0];
        geofence_msg.request.type=0; // Static geofences
        geofence_msg.request.id=req.emergency.geofences[0];
        if (!read_track_client_.call(track_msg) || !track_msg.response.success)
            ROS_ERROR("[EmergencyManagement]: Failed reading track.");
        else
            ROS_INFO("[EmergencyManagement]:Track read.");
        if (!read_geofence_client_.call(geofence_msg) || !geofence_msg.response.success)
            ROS_ERROR("[EmergencyManagement]: Failed reading static geofence.");
        else
            ROS_INFO("[EmergencyManagement]: Static Geofence read.");
        action.header.stamp=ros::Time::now();
        action.id=req.emergency.ids[0];
        action.action_description="Leave static geofence.";
        action.action=2; //TBC Leave static geofence
        leaveGeofence(track_msg.response.track,geofence_msg.response.geofence);
        action_pub_.publish(action);
        break;
    case 3: // Temporary Geofence Violation TBC
        track_msg.request.id=req.emergency.ids[0];
        geofence_msg.request.type=1; // Temporary geofences
        geofence_msg.request.id=req.emergency.geofences[0];
        if (!read_track_client_.call(track_msg) || !track_msg.response.success)
            ROS_ERROR("[EmergencyManagement]: Failed reading track.");
        else
            ROS_INFO("[EmergencyManagement]:Track read.");
        if (!read_geofence_client_.call(geofence_msg) || !geofence_msg.response.success)
            ROS_ERROR("[EmergencyManagement]: Failed reading temporary geofence.");
        else
            ROS_INFO("[EmergencyManagement]:Temporary Geofence read.");
        action.header.stamp=ros::Time::now();
        action.id=req.emergency.ids[0];
        action.action_description="Leave temporary geofence.";
        action.action=3; //TBC leave temporary geofence
        leaveGeofence(track_msg.response.track,geofence_msg.response.geofence);
        action_pub_.publish(action);
        break;
    case 4: // Unexpected obstacle TBC  --> escribimos un nuevo geofence... TBD
        ageofence_msg.request.type=0; // Static geofences
        if (!read_all_geofence_client_.call(ageofence_msg) || !ageofence_msg.response.success)
            ROS_ERROR("[EmergencyManagement]: Failed reading Static geofence.");
        else
        {
            ROS_INFO("[EmergencyManagement]:Static Geofence read.");
            if (!checkGeofence(req.emergency.obstacle,&ageofence_msg.response.geofences[0]))
            {
                ageofence_msg.request.type=1; // Temporary geofences
                if (!read_all_geofence_client_.call(ageofence_msg) || !ageofence_msg.response.success)
                    ROS_ERROR("[EmergencyManagement]: Failed reading Temporary geofence.");
                else
                {
                    ROS_INFO("[EmergencyManagement]:Temporary Geofence read.");
                    if (!checkGeofence(req.emergency.obstacle,&ageofence_msg.response.geofences[0]))
                    {
                        wgeofence_msg.request.id=ageofence_msg.response.size;
                        wgeofence_msg.request.type=1; // Temporary geofences
                        wgeofence_msg.request.geofence=req.emergency.obstacle;
                        if (!write_geofence_client_.call(wgeofence_msg) || !wgeofence_msg.response.success)
                            ROS_ERROR("[EmergencyManagement]: Failed writting Temporary geofence.");
                        else
                            ROS_INFO("[EmergencyManagement]:Temporary Geofence writted.");
                    }
                }
            }
        }
        break;
    case 5: // Jamming detection
        // Update jamming map
        req.emergency.jamming;
        break;
    default:
        break;
    }*/

//     return true;
// }



// MAIN function
int main(int argc, char *argv[])
{
    ros::init(argc,argv,"emergency_management");

    // Create a EmergencyManagement object
    EmergencyManagement *emergency_management = new EmergencyManagement();

    ros::spin();
}
