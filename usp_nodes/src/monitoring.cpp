#include <ros/ros.h>
#include <gauss_msgs/Alert.h>
#include <gauss_msgs/ReadGeofences.h>
#include <gauss_msgs/ReadTraj.h>
#include <gauss_msgs/DB_size.h>
#include <list>
#include <gauss_msgs/WaypointList.h>
#include <gauss_msgs/Conflict.h>

// Alert srv constatns
#define CONFLICT 0

// Conflict msg constants
#define UAVLOSSOFSEPARATION 0
#define GEOFENCEVIOLATION 1

using namespace std;

struct cell {
  list<int> traj;
  list<int> wp;
};


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


    // Auxilary variables
    double rate;
    int X,Y,Z,T;
    double dX,dY,dZ,dT;

    ros::NodeHandle nh_;

    // Subscribers


    // Publisher

    // Timer
    ros::Timer timer_sub_;

    // Server

    // Client
    ros::ServiceClient alert_client_;
    ros::ServiceClient read_geofence_client_;
    ros::ServiceClient read_traj_client_;
    ros::ServiceClient dbsize_cilent_;
};

// Monitoring Constructor
Monitoring::Monitoring()
{
    // Read parameters
    nh_.param("/gauss/monitoring_rate",rate,0.5);
    nh_.param("/gauss/latitude",X,50);
    nh_.param("/gauss/longitude",Y,50);
    nh_.param("/gauss/altitude",Z,5);
    nh_.param("/gauss/horizon",T,30);
    nh_.param("/gauss/deltaX",dX,10.0);
    nh_.param("/gauss/deltaY",dY,10.0);
    nh_.param("/gauss/deltaZ",dZ,10.0);


    // Initialization    
    dT=1.0/rate;

    // Publish

    // Subscribe

    // Server

    // Client
    read_traj_client_ = nh_.serviceClient<gauss_msgs::ReadTraj>("/gauss/readTraj");
    read_geofence_client_ = nh_.serviceClient<gauss_msgs::ReadGeofences>("/gauss/readGeofence");
    alert_client_ = nh_.serviceClient<gauss_msgs::Alert>("/gauss/alert");
    dbsize_cilent_ = nh_.serviceClient<gauss_msgs::DB_size>("/gauss/db_size");

    // Timer
    timer_sub_=nh_.createTimer(ros::Duration(dT),&Monitoring::timerCallback,this);

    ROS_INFO("Started Monitoring node!");
}

// Auxilary methods

// Timer Callback
void Monitoring::timerCallback(const ros::TimerEvent &)
{
    ROS_INFO_ONCE("Monitoring");

    int missions;
    int geofeces;

    // Ask for number of missions and geofences

    gauss_msgs::DB_size msg_size;
    if (!(dbsize_cilent_.call(msg_size)) || !(msg_size.response.success))
    {
        ROS_ERROR("Failed to ask for number of missions and geofences");
        return;
    }
    else
    {
        missions=msg_size.response.missions;
        geofeces=msg_size.response.geofences;
    }

    // Include geofences in the 4D-grid
    cell grid[X][Y][Z][T];
    gauss_msgs::Alert alert;
    alert.request.code=CONFLICT;
    //alert.request.level //definir este level en función de la severidad de los conflictos

    for (int i=0; i<geofeces; i++)
    {
        gauss_msgs::ReadGeofences msg_geo;
        msg_geo.request.geofences_ids[0]=i;
        if(!(read_geofence_client_.call(msg_geo)) || !(msg_geo.response.success))
        {
            ROS_ERROR("Failed to read a geofence");
            return;
        }
        msg_geo.response.geofences;
        // función que rellene el grid segun el geofence (el mensaje anterior contiene una lista de waypoints que delimita el geofence, el geofence esta definido entre el tiempo del primer elemento y el tiempo del ultimo)
    }

    // Rellena grid con waypoints de las missiones
    for (int i=0; i<missions; i++)
    {
        gauss_msgs::ReadTraj msg_traj;
        msg_traj.request.UAV_ids[0]=i;
        if(!(read_traj_client_.call(msg_traj)) || !(msg_traj.response.success))
        {
            ROS_ERROR("Failed to read a trajectory");
            return;
        }

        gauss_msgs::WaypointList trajectory = msg_traj.response.tracks[0];

        int waypoints = trajectory.waypoints.size();

        for (int j=1; j<waypoints; j++)
        {
            int posx = floor(trajectory.waypoints.at(j).latitude/dX);
            int posy = floor(trajectory.waypoints.at(j).longitude/dX);
            int posz = floor(trajectory.waypoints.at(j).altitude/dX);
            int post = floor(trajectory.waypoints.at(j).stamp.sec/dT);

            grid[posx][posy][posz][post].traj.push_back(i);
            grid[posx][posy][posz][post].wp.push_back(j);

            
            for (int m=max(0,posx-1); m<min(X,posx+1); m++)
                for (int n=max(0,posy-1); n<min(Y,posy+1); n++)
                    for (int p=max(0,posz-1); p<min(Z,posz+1); p++)
                        for (int t=max(0,post-1); t<min(T,post+1); t++)
                        {
                            if (grid[m][n][p][t].traj.size()>0)
                            {
                                list<int>::iterator it = grid[m][n][p][t].traj.begin();
                                list<int>::iterator it_wp = grid[m][n][p][t].wp.begin();
                                while (it != grid[m][n][p][t].traj.end())
                                {
                                    if (*it != i && *it<1000)  // los id superiores a 1000 pertencen a geofences
                                    {
                                        gauss_msgs::ReadTraj msg_traj2;
                                        msg_traj2.request.UAV_ids[0]=*it;
                                        if(!(read_traj_client_.call(msg_traj2)) || !(msg_traj2.response.success))
                                        {
                                            ROS_ERROR("Failed to read a trajectory");
                                            return;
                                        }
                                        gauss_msgs::WaypointList trajectory2 = msg_traj.response.tracks[0];

                                        if (sqrt(pow(trajectory.waypoints.at(j).latitude-trajectory2.waypoints.at(*it_wp).latitude,2)+
                                                 pow(trajectory.waypoints.at(j).longitude-trajectory2.waypoints.at(*it_wp).longitude,2)+
                                                 pow(trajectory.waypoints.at(j).altitude-trajectory2.waypoints.at(*it_wp).altitude,2))<dX &&
                                                abs(trajectory.waypoints.at(j).stamp.sec-trajectory2.waypoints.at(*it_wp).stamp.sec)<dT)
                                        {
                                            gauss_msgs::Conflict conflict;
                                            conflict.UAV_ids.push_back(i);
                                            conflict.UAV_ids.push_back(*it);
                                            conflict.wp_ids.push_back(j);
                                            conflict.wp_ids.push_back(*it_wp);
                                            conflict.header.stamp=ros::Time::now();
                                            conflict.type=UAVLOSSOFSEPARATION;
                                            alert.request.conflicts.push_back(conflict);
                                        }
                                    }
                                    else if (*it>=1000)  // Conflicto con geofence
                                    {
                                        gauss_msgs::Conflict conflict;
                                        conflict.UAV_ids.push_back(i);
                                        conflict.wp_ids.push_back(j);
                                        conflict.geofence_ids.push_back(*it-1000);
                                        conflict.header.stamp=ros::Time::now();
                                        conflict.type=GEOFENCEVIOLATION;
                                        alert.request.conflicts.push_back(conflict);
                                    }
                                    it++;
                                    it_wp++;
                                }
                            }
                        }
        }
    }

    // LLamar al servicio alerta
    if (alert.request.conflicts.size()>0)
    {
        alert.request.description="New conflicts detected";
        if(!(alert_client_.call(alert)) || !(alert.response.success))
        {
            ROS_ERROR("Failed to send alert message");
            return;
        }
    }
    else
        ROS_INFO("%s",alert.response.message);
}



// MAIN function
int main(int argc, char *argv[])
{
    ros::init(argc,argv,"monitoring");

    // Create a Monitoring object
    Monitoring *monitoring = new Monitoring();

    ros::spin();
}
