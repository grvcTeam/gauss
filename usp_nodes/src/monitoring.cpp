#include <ros/ros.h>
#include <gauss_msgs/Threats.h>
#include <gauss_msgs/Threat.h>
#include <gauss_msgs/ReadGeofences.h>
#include <gauss_msgs/ReadOperation.h>
#include <gauss_msgs/ReadTraj.h>
#include <gauss_msgs/DB_size.h>
#include <list>
#include <gauss_msgs/Conflict.h>
#include <geometry_msgs/Point.h>
#include <gauss_msgs/Waypoint.h>

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
    int checkGeofences(gauss_msgs::Waypoint position4D, int geofence_size);


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
    ros::ServiceClient threats_client_;
    ros::ServiceClient read_geofence_client_;
    ros::ServiceClient read_operation_client_;
    ros::ServiceClient read_trajectory_client_;
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
    read_operation_client_ = nh_.serviceClient<gauss_msgs::ReadOperation>("/gauss/readOperation");
    read_trajectory_client_ = nh_.serviceClient<gauss_msgs::ReadTraj>("/gauss/readEstimatedTrajectory");
    read_geofence_client_ = nh_.serviceClient<gauss_msgs::ReadGeofences>("/gauss/readGeofence");
    threats_client_ = nh_.serviceClient<gauss_msgs::Threats>("/gauss/threats");
    dbsize_cilent_ = nh_.serviceClient<gauss_msgs::DB_size>("/gauss/db_size");

    // Timer
    timer_sub_=nh_.createTimer(ros::Duration(dT),&Monitoring::timerCallback,this);

    ROS_INFO("Started Monitoring node!");
}

// Auxilary methods

int Monitoring::checkGeofences(gauss_msgs::Waypoint position4D, int geofence_size)
{
    for (int i=0; i<geofence_size; i++)
    {
        gauss_msgs::ReadGeofences msg_geo;
        msg_geo.request.geofences_ids[0]=i;
        if(!(read_geofence_client_.call(msg_geo)) || !(msg_geo.response.success))
        {
            ROS_ERROR("Failed to read a geofence");
            return -1;
        }
        gauss_msgs::Geofence geofence= msg_geo.response.geofences[0];

        if (position4D.stamp.nsec>=geofence.start_time.nsec && position4D.stamp.nsec<=geofence.end_time.nsec
                && position4D.altitude>=geofence.min_altitude && position4D.altitude<=geofence.max_altitude)
        {
            if (geofence.cylinder_shape)
            {
                if (sqrt((position4D.latitude-geofence.circle.x_center)*(position4D.latitude-geofence.circle.x_center)+
                         (position4D.longitude-geofence.circle.y_center)*(position4D.longitude-geofence.circle.y_center))<=geofence.circle.radius)
                    return i;
            }
            else
            {
                int vertexes=geofence.polygon.x.size();
                double angle_sum=0.0;
                for (int i=1; i<vertexes; i++)
                {
                    geometry_msgs::Point v1, v2;
                    v2.x=geofence.polygon.x[i]-position4D.latitude;
                    v2.y=geofence.polygon.y[i]-position4D.longitude;
                    v1.x=geofence.polygon.x[i-1]-position4D.latitude;
                    v1.y=geofence.polygon.y[i-1]-position4D.longitude;
                    angle_sum+=acos((v2.x*v1.x+v2.y*v1.y)/(sqrt(v1.x*v1.x+v1.y*v1.y)*sqrt(v2.x*v2.x+v1.y*v1.y)));
                }
                geometry_msgs::Point v1, v2;
                v2.x=geofence.polygon.x[0]-position4D.latitude;
                v2.y=geofence.polygon.y[0]-position4D.longitude;
                v1.x=geofence.polygon.x[vertexes-1]-position4D.latitude;
                v1.y=geofence.polygon.y[vertexes-1]-position4D.longitude;
                angle_sum+=acos((v2.x*v1.x+v2.y*v1.y)/(sqrt(v1.x*v1.x+v1.y*v1.y)*sqrt(v2.x*v2.x+v1.y*v1.y)));

                if (abs(angle_sum)>6)  // cercano a 2PI (algoritmo radial para deterinar si un punto está dentro de un polígono)
                    return i;
            }
        }
    }
    return -1;
}


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
        geofeces=msg_size.response.geofences;
        missions=msg_size.response.operations;
    }

    // Include geofences in the 4D-grid
    cell grid[X][Y][Z][T];
    gauss_msgs::Threats threats_msg;

    // Rellena grid con waypoints de las missiones
    for (int i=0; i<missions; i++)
    {
        gauss_msgs::ReadOperation msg_op;
        msg_op.request.uav_ids[0]=i;
        if(!(read_operation_client_.call(msg_op)) || !(msg_op.response.success))
        {
            ROS_ERROR("Failed to read a trajectory");
            return;
        }

        gauss_msgs::Operation operation = msg_op.response.operation[0];
        gauss_msgs::WaypointList trajectory = operation.estimated_trajectory;
        gauss_msgs::WaypointList plan = operation.flight_plan;

        geometry_msgs::Point vd;
        geometry_msgs::Point pq;
        geometry_msgs::Point pv;
        if (operation.current_wp >= plan.waypoints.size())
        {
            vd.x=plan.waypoints.at(operation.current_wp).latitude-plan.waypoints.at(operation.current_wp-1).latitude;
            vd.y=plan.waypoints.at(operation.current_wp).longitude-plan.waypoints.at(operation.current_wp-1).longitude;
            vd.z=plan.waypoints.at(operation.current_wp).altitude-plan.waypoints.at(operation.current_wp-1).altitude;
        }
        else
        {
            vd.x=plan.waypoints.at(operation.current_wp+1).latitude-plan.waypoints.at(operation.current_wp).latitude;
            vd.y=plan.waypoints.at(operation.current_wp+1).longitude-plan.waypoints.at(operation.current_wp).longitude;
            vd.z=plan.waypoints.at(operation.current_wp+1).altitude-plan.waypoints.at(operation.current_wp).altitude;
        }
        pq.x=trajectory.waypoints.at(0).latitude-plan.waypoints.at(operation.current_wp).latitude;
        pq.y=trajectory.waypoints.at(0).longitude-plan.waypoints.at(operation.current_wp).longitude;
        pq.z=trajectory.waypoints.at(0).altitude-plan.waypoints.at(operation.current_wp).altitude;

        pv.x=vd.y*pq.z-vd.z*pq.y;
        pv.y=vd.x*pq.z-vd.z*pq.x;
        pv.z=vd.x*pq.y-vd.y*pq.x;

        // distancia = |pq x vd| / |vd|: distancia de punto q a recta cuyo vevtor director es vd y que contiene al punto p

        double distance=sqrt(pv.x*pv.x+pv.y*pv.y+pv.z*pv.z)/sqrt(vd.x*vd.x+vd.y*vd.y+vd.z*vd.z);

        if (distance>operation.flight_geometry)
        {
            gauss_msgs::Threat threat;
            threat.header.stamp=ros::Time::now();
            threat.uas_ids.push_back(i);
            threat.times.push_back(ros::Time::now());
            if (distance<operation.contingency_volume)
                threat.threat_id=threat.UAS_IN_CV;
            else
                threat.threat_id=threat.UAS_OUT_CV;
            threats_msg.request.uas_ids.push_back(i);
            threats_msg.request.threats.push_back(threat);
        }

        int waypoints = trajectory.waypoints.size();

        for (int j=1; j<waypoints; j++)
        {
            // para la trayectoria estimada comprobar que no estas dentro de un GEOFENCE
            int geofence_intrusion = checkGeofences(trajectory.waypoints.at(j),geofeces);
            if (geofence_intrusion>=0)
            {
                gauss_msgs::Threat threat;
                threat.header.stamp=ros::Time::now();
                threat.uas_ids.push_back(i);
                threat.times.push_back(ros::Time::now());
                if (j==0)
                    threat.threat_id=threat.GEOFENCE_INTRUSION;
                else
                    threat.threat_id=threat.GEOFENCE_CONFLICT;
                threats_msg.request.uas_ids.push_back(i);
                threats_msg.request.threats.push_back(threat);
            }

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
                                    if (*it != i)
                                    {
                                        gauss_msgs::ReadTraj msg_traj2;
                                        msg_traj2.request.UAV_ids[0]=*it;
                                        if(!(read_trajectory_client_.call(msg_traj2)) || !(msg_traj2.response.success))
                                        {
                                            ROS_ERROR("Failed to read a trajectory");
                                            return;
                                        }
                                        gauss_msgs::WaypointList trajectory2 = msg_traj2.response.tracks[0];

                                        if (sqrt(pow(trajectory.waypoints.at(j).latitude-trajectory2.waypoints.at(*it_wp).latitude,2)+
                                                 pow(trajectory.waypoints.at(j).longitude-trajectory2.waypoints.at(*it_wp).longitude,2)+
                                                 pow(trajectory.waypoints.at(j).altitude-trajectory2.waypoints.at(*it_wp).altitude,2))<dX &&
                                                abs(trajectory.waypoints.at(j).stamp.sec-trajectory2.waypoints.at(*it_wp).stamp.sec)<dT)
                                        {
                                            gauss_msgs::Threat threat;
                                            threat.header.stamp=ros::Time::now();
                                            threat.uas_ids.push_back(i);
                                            threat.uas_ids.push_back(*it);
                                            threat.times.push_back(trajectory.waypoints.at(j).stamp);
                                            threat.times.push_back(trajectory2.waypoints.at(*it_wp).stamp);
                                            threat.threat_id=threat.LOSS_OF_SEPARATION;

                                            threats_msg.request.uas_ids.push_back(i);
                                            threats_msg.request.threats.push_back(threat);
                                        }
                                    }
                                    it++;
                                    it_wp++;
                                }
                            }
                        }
        }
    }

    // LLamar al servicio alerta
    if (threats_msg.request.threats.size()>0)
    {
        if(!(threats_client_.call(threats_msg)) || !(threats_msg.response.success))
        {
            ROS_ERROR("Failed to send alert message");
            return;
        }
    }
    else
        ROS_INFO("%s",threats_msg.response.message);
}



// MAIN function
int main(int argc, char *argv[])
{
    ros::init(argc,argv,"monitoring");

    // Create a Monitoring object
    Monitoring *monitoring = new Monitoring();

    ros::spin();
}
