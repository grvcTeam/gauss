#include <ros/ros.h>
#include <gauss_msgs/Track.h>
#include <gauss_msgs/ReadTrack.h>
#include <gauss_msgs/ReadGeofence.h>
#include <gauss_msgs/ReadPlan.h>
#include <gauss_msgs/Alert.h>
#include <gauss_msgs/Deconflict.h>
#include <gauss_msgs/Emergency.h>
#include <gauss_msgs/Geofence.h>
#include <gauss_msgs/Traffic.h>
#include <gauss_msgs/Plan.h>
#include <gauss_msgs/ReadAllGeofence.h>
#include <gauss_msgs/ReadAllTrack.h>


#define SPACE_SIZE 100
#define TIME_SIZE 10
#define DB_SIZE 100

// Class definition
class Monitoring
{
public:
    Monitoring();

private:
    // Topic Callbacks

    // Service Callbacks

    // Timer Callbacks
    void timerCallback(const ros::TimerEvent&);

    // Auxilary methods
    void planDeviation();
    void geofenceApproaching();
    void conflictDetection();


    // Auxilary variables
    int id;
    double rate;
    gauss_msgs::Plan plan;
    gauss_msgs::Track track;
    gauss_msgs::Track tracks[DB_SIZE];
    gauss_msgs::ReadTrack track_msg;
    gauss_msgs::ReadAllTrack all_tracks_msg;
    gauss_msgs::ReadAllGeofence geofence_msg;
    gauss_msgs::Alert emergency_msg;
    gauss_msgs::Deconflict deconflict_msg;
    gauss_msgs::Geofence geofences[DB_SIZE];
    gauss_msgs::Traffic traffic;
    int grid[SPACE_SIZE][SPACE_SIZE][SPACE_SIZE][TIME_SIZE][2];
    bool plan_read;
    int geofence_size;
    int track_size;

    ros::NodeHandle nh_;

    // Subscribers


    // Publisher
    ros::Publisher traffic_pub_;

    // Timer
    ros::Timer timer_sub_;

    // Server

    // Client
    ros::ServiceClient read_track_client_;
    ros::ServiceClient read_geofence_client_;
    ros::ServiceClient read_all_geofence_client_;
    ros::ServiceClient read_plan_client_;
    ros::ServiceClient alert_client_;
    ros::ServiceClient deconflict_client_;
};

// Monitoring Constructor
Monitoring::Monitoring()
{
    // Read parameters
    nh_.param("/gauss/monitoring_rate",rate,0.5);
    nh_.param("/gauss/monitoring_id",id,1);


    // Initialization
    plan_read=false;
    track_msg.request.id=id;
    geofence_size=track_size=0;

    // Publish
    traffic_pub_ = nh_.advertise<gauss_msgs::Traffic>("/gauss/traffic",1);

    // Subscribe

    // Server

    // Client
    read_track_client_ = nh_.serviceClient<gauss_msgs::ReadTrack>("/gauss/readTrack");
    read_geofence_client_ = nh_.serviceClient<gauss_msgs::ReadGeofence>("/gauss_msgs/readGeofence");
    read_all_geofence_client_ =  nh_.serviceClient<gauss_msgs::ReadAllGeofence>("/gauss/readAllGeofences");
    read_plan_client_ = nh_.serviceClient<gauss_msgs::ReadPlan>("/gauss/readPlan");
    alert_client_ = nh_.serviceClient<gauss_msgs::Alert>("/gauss/alert_emergency");
    deconflict_client_ = nh_.serviceClient<gauss_msgs::Deconflict>("/gauss/tactical_deconfliction");

    // Timer
    timer_sub_=nh_.createTimer(ros::Duration(1.0/rate),&Monitoring::timerCallback,this);

    ROS_INFO("Started Monitoring node!");
}

// Auxilary methods
void Monitoring::planDeviation()
{
    if (!read_track_client_.call(track_msg) || !track_msg.response.success)
        ROS_ERROR("[Monitoring]: Failed reading track.");
    else
    {
        track=track_msg.response.track;
        // Rellena grid con track

        traffic.track=track;
        // Comprueba que track no se desvía del plan
        if (false) // Si track se desvía del plan
        {
            traffic.warnings_id.push_back(1); // TBD 1: Plan deviation
            traffic.warnings_description.push_back("TBD");

            if (false) // Si desviación es muy grande
            {
                emergency_msg.request.id=id;
                emergency_msg.request.emergency.emergency_id=1; // Plan deviation TBD
                emergency_msg.request.emergency.emergency_description="TBD";
                emergency_msg.request.emergency.ids.push_back(id);
                if (!alert_client_.call(emergency_msg) || !emergency_msg.response.success)
                    ROS_ERROR("[Monitoring]: Failed calling Emergency management service.");
                else
                    ROS_INFO("[Monitoring]: Emergency management service called.");
            }

        }
    }
}

void Monitoring::geofenceApproaching()
{
    // Rellena geofences
    geofence_msg.request.type=0;  // Estáticas
    if (!read_geofence_client_.call(geofence_msg) || !geofence_msg.response.success)
        ROS_ERROR("[Monitoring]: Failed reading static geofences.");
    else
    {
        for (int i=0; i<geofence_msg.response.size;i++)
            geofences[i]=geofence_msg.response.geofences[i];
        geofence_size=geofence_msg.response.size;
    }
    geofence_msg.request.type=1;   // Temporales
    if (!read_geofence_client_.call(geofence_msg) || !geofence_msg.response.success)
        ROS_ERROR("[Monitoring]: Failed reading temporary geofences.");
    else
    {
        for (int i=0; i<geofence_msg.response.size;i++)
            geofences[i+geofence_size]=geofence_msg.response.geofences[i];
        geofence_size+=geofence_msg.response.size;
    }
    // Rellena grid con geofences

    // Comprueba si track se mete en algún geofence, usa geofences y track
    if (false) // Si track se acerca a algún geofence
    {
        traffic.warnings_id.push_back(2); // TBD 2: Static Geofence approach or 3: Temporary Geofences approach
        traffic.warnings_description.push_back("TBD");

        // Comprueba si está dentro de un geofence
        if (false)  // Si UAV está dentro de un geofence
        {
            emergency_msg.request.id=id;
            emergency_msg.request.emergency.emergency_id=2; // 2: Static  or 3: Temporary Geofence violation TBD
            emergency_msg.request.emergency.emergency_description="TBD";
            emergency_msg.request.emergency.geofences.push_back(geofences[1].id); // se incluye el id del geofence
            if (!alert_client_.call(emergency_msg) || !emergency_msg.response.success)
                ROS_ERROR("[Monitoring]: Failed calling Emergency management service.");
            else
                ROS_INFO("[Monitoring]: Emergency management service called.");
        }
    }
}

void Monitoring::conflictDetection()
{
    // Rellena tracks
    if (!read_track_client_.call(track_msg) || !track_msg.response.success)
        ROS_ERROR("[Monitoring]: Failed reading tracks.");
    else
        for (int i=0; i<all_tracks_msg.response.size;i++)
            tracks[i]=all_tracks_msg.response.tracks[i];

    //Rellena grid con tracks (menos el track propio)
    // Comprueba si hay conflicto
    if (false)  // Si hay conflicto
    {
        deconflict_msg.request.ids.push_back(id);
        deconflict_msg.request.ids.push_back(1);   // Ponemos todos los id en conflicto
        // ...
        deconflict_msg.request.tracks.push_back(track);
        deconflict_msg.request.tracks.push_back(tracks[1]);
        //...
        if (!deconflict_client_.call(deconflict_msg) || !deconflict_msg.response.success)
            ROS_ERROR("[Monitoring]: Failed calling Tactical Deconfliction service.");
        else
            ROS_INFO("[Monitoring]: Tactical Deconfliction service called.");
    }
}



// Timer Callback
void Monitoring::timerCallback(const ros::TimerEvent &)
{
    if (!plan_read)
    {
        gauss_msgs::ReadPlan plan_msg;
        plan_msg.request.id=id;
        if (!read_plan_client_.call(plan_msg) || !plan_msg.response.success)
            ROS_ERROR("[Monitoring]: Failed reading flight plan.");
        else
        {
            plan_read=true;
            plan=plan_msg.response.plan;
            ROS_INFO("[Monitoring]: Flight plan read");
        }
    }

    if (plan_read)
        planDeviation();
    geofenceApproaching();
    conflictDetection();


    traffic_pub_.publish(traffic); // Publishing traffic information
}



// MAIN function
int main(int argc, char *argv[])
{
    ros::init(argc,argv,"monitoring");

    // Create a Monitoring object
    Monitoring *monitoring = new Monitoring();

    ros::spin();
}
