#include <ros/ros.h>
#include <gauss_msgs/Threats.h>
#include <gauss_msgs/Threat.h>
#include <gauss_msgs/ReadGeofences.h>
#include <gauss_msgs/ReadOperation.h>
#include <gauss_msgs/DB_size.h>
#include <list>
#include <geometry_msgs/Point.h>
#include <gauss_msgs/Waypoint.h>
#include <gauss_msgs/CheckConflicts.h>
#include <mutex>

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
    bool checkConflictsCB(gauss_msgs::CheckConflicts::Request &req, gauss_msgs::CheckConflicts::Response &res);

    // Timer Callbacks
    void timerCallback(const ros::TimerEvent&);

    // Auxilary methods
    int checkGeofences(gauss_msgs::Waypoint position4D, int geofence_size, double safety_distance);
    gauss_msgs::Threats manageThreatList(const gauss_msgs::Threats &_in_threats, const double &_start_time);
    void deleteOldThreats(const double &_start_time, const double &_time_check);

    // Auxilary variables
    double rate;
    int X,Y,Z,T;
    double dX,dY,dZ,dT;
    double minX,maxX,minY,maxY,minZ,maxZ,maxT;
    double minDist;
    int threat_list_id_ = 0;
    map <int, pair<gauss_msgs::Threat, double>> threat_list_;

    cell ****grid;

    bool locker;
    std::mutex mutex_lock_;

    ros::Time current_stamp;

    ros::NodeHandle nh_;

    // Subscribers


    // Publisher

    // Timer
    ros::Timer timer_sub_;

    // Server
    ros::ServiceServer check_conflicts_server_;

    // Client
    ros::ServiceClient threats_client_;
    ros::ServiceClient read_geofence_client_;
    ros::ServiceClient read_operation_client_;
    ros::ServiceClient dbsize_cilent_;
};

// Monitoring Constructor
Monitoring::Monitoring()
{
    // Read
    nh_.param("/gauss/monitoring_rate",rate,0.2);
    nh_.param("/gauss/minX",minX,-200.0);
    nh_.param("/gauss/minY",minY,-200.0);
    nh_.param("/gauss/minZ",minZ,0.0);
    nh_.param("/gauss/maxX",maxX,200.0);
    nh_.param("/gauss/maxY",maxY,200.0);
    nh_.param("/gauss/maxZ",maxZ,30.0);
    nh_.param("/gauss/time_horizon",maxT,90.0);
    nh_.param("/gauss/deltaX",dX,10.0);
    nh_.param("/gauss/deltaY",dY,10.0);
    nh_.param("/gauss/deltaZ",dZ,10.0);
    nh_.param("/gauss/safetyDistance",minDist,10.0);


    // Initialization    
    dT=1.0/rate;
    X=ceil((maxX-minX)/dX);
    Y=ceil((maxY-minY)/dY);
    Z=ceil((maxZ-minZ)/dZ);
    T=ceil(maxT/dT);

    grid=NULL;
    //locker=true;
    mutex_lock_.lock();

    grid= new cell***[X];
    for (int i=0;i<X;i++)
    {
        grid[i]=new cell**[Y];
        for (int j=0;j<Y;j++)
        {
            grid[i][j]=new cell*[Z];
            for (int k=0;k<Z;k++)
                grid[i][j][k]=new cell[T];
        }
    }
    mutex_lock_.unlock();

    // Publish

    // Subscribe

    // Server
    check_conflicts_server_=nh_.advertiseService("/gauss/check_conflicts",&Monitoring::checkConflictsCB,this);

    // Client
    read_operation_client_ = nh_.serviceClient<gauss_msgs::ReadOperation>("/gauss/read_operation");
    read_geofence_client_ = nh_.serviceClient<gauss_msgs::ReadGeofences>("/gauss/read_geofences");
    threats_client_ = nh_.serviceClient<gauss_msgs::Threats>("/gauss/threats");
    dbsize_cilent_ = nh_.serviceClient<gauss_msgs::DB_size>("/gauss/db_size");

    // Timer
    timer_sub_=nh_.createTimer(ros::Duration(dT),&Monitoring::timerCallback,this);

    ROS_INFO("[Monitoring] Started Monitoring node!");
}

// Auxilary methods
void Monitoring::deleteOldThreats(const double &_start_time, const double &_time_check){
    std::vector<int> delete_id;
    for (map<int, pair<gauss_msgs::Threat, double>>::const_iterator it = threat_list_.begin(); it != threat_list_.end(); it++){
        if (ros::Time::now().toSec() - _start_time - it->second.second > _time_check) {
            delete_id.push_back(it->first);
        }
    }
    for (int i = 0; i < delete_id.size(); i++){
        map<int, pair<gauss_msgs::Threat, double>>::const_iterator it;
        it=threat_list_.find(delete_id.at(i));
        threat_list_.erase(it);
    }
}


gauss_msgs::Threats Monitoring::manageThreatList(const gauss_msgs::Threats &_in_threats, const double &_start_time){
    gauss_msgs::Threats out_threats;
    if (threat_list_.size() == 0) {
        threat_list_[threat_list_id_] = make_pair(_in_threats.request.threats.front(), ros::Time::now().toSec() - _start_time);
        threat_list_[threat_list_id_].first.threat_id = threat_list_id_;
        out_threats.request.threats.push_back(threat_list_[threat_list_id_].first);
        out_threats.request.uav_ids.push_back(threat_list_[threat_list_id_].first.uav_ids.front());
        if (threat_list_[threat_list_id_].first.threat_type == threat_list_[threat_list_id_].first.LOSS_OF_SEPARATION) 
            out_threats.request.uav_ids.push_back(threat_list_[threat_list_id_].first.uav_ids.back());
        threat_list_id_++;
    }
    if (threat_list_.size() > 0){
        for (int i = 0; i < _in_threats.request.threats.size(); i++){
            static bool save_threat = false;
            gauss_msgs::Threat threat_to_save;
            for (map<int, pair<gauss_msgs::Threat, double>>::const_iterator it = threat_list_.begin(); it != threat_list_.end(); it++){
                save_threat = false;
                // If loss of separation, check 2 uav ids else check 1 uav id. If a geofence is involved, check geofence id.
                if (_in_threats.request.threats.at(i).threat_type == it->second.first.threat_type){
                    if (_in_threats.request.threats.at(i).threat_type == _in_threats.request.threats.at(i).LOSS_OF_SEPARATION){
                        if (_in_threats.request.threats.at(i).uav_ids.front() == it->second.first.uav_ids.front() ||
                            _in_threats.request.threats.at(i).uav_ids.end() == it->second.first.uav_ids.end()){
                            break;
                        }
                    } else if (_in_threats.request.threats.at(i).threat_type == _in_threats.request.threats.at(i).GEOFENCE_CONFLICT){
                        if (_in_threats.request.threats.at(i).uav_ids.front() == it->second.first.uav_ids.front() ||
                            _in_threats.request.threats.at(i).geofence_ids.front() == it->second.first.geofence_ids.front()){
                            break;
                        }
                    } else if (_in_threats.request.threats.at(i).threat_type == _in_threats.request.threats.at(i).GEOFENCE_INTRUSION){
                        if (_in_threats.request.threats.at(i).uav_ids.front() == it->second.first.uav_ids.front() ||
                            _in_threats.request.threats.at(i).geofence_ids.front() == it->second.first.geofence_ids.front()){
                            break;
                        }
                    } else {
                        if (_in_threats.request.threats.at(i).uav_ids.front() == it->second.first.uav_ids.front()){
                            break;
                        }
                    }
                }
                threat_to_save = _in_threats.request.threats.at(i);
                save_threat = true;
            }
            if(save_threat){
                threat_list_[threat_list_id_] = make_pair(threat_to_save, ros::Time::now().toSec() - _start_time);
                threat_list_[threat_list_id_].first.threat_id = threat_list_id_;
                out_threats.request.threats.push_back(threat_list_[threat_list_id_].first);
                out_threats.request.uav_ids.push_back(threat_list_[threat_list_id_].first.uav_ids.front());
                if (threat_list_[threat_list_id_].first.threat_type == threat_list_[threat_list_id_].first.LOSS_OF_SEPARATION){
                    out_threats.request.uav_ids.push_back(threat_list_[threat_list_id_].first.uav_ids.back());
                }
                threat_list_id_++;
            }
        }
    } 
    //Uncomment to check list
    // for (map<int, pair<gauss_msgs::Threat, double>>::const_iterator it = threat_list_.begin(); it != threat_list_.end(); it++){
    //     std::cout << "[" << it->first << "] Time: " << it->second.second << 
    //     " Thread: " << it->second.first <<
    //      "\n";
    //     std::cout << " -------------------------------------------------------- \n";
    // }

    std::string cout_threats;
    for (auto i : out_threats.request.threats) cout_threats = cout_threats + " [" + std::to_string(i.threat_id) +
                                                              ", " + std::to_string(i.threat_type) + "]";
    ROS_INFO_STREAM_COND(out_threats.request.threats.size() > 0, "[Monitoring] New threats detected: (id, type) " + cout_threats);

    return out_threats;
}

int Monitoring::checkGeofences(gauss_msgs::Waypoint position4D, int geofence_size, double safety_distance)
{
    for (int i=0; i<geofence_size; i++)
    {
        gauss_msgs::ReadGeofences msg_geo;
        msg_geo.request.geofences_ids.push_back(i);
        if(!(read_geofence_client_.call(msg_geo)) || !(msg_geo.response.success))
        {
            ROS_ERROR("Failed to read a geofence");
            return -2;
        }
        gauss_msgs::Geofence geofence= msg_geo.response.geofences[0];

        if (position4D.stamp.toNSec()>=geofence.start_time.toNSec() && position4D.stamp.toNSec()<=geofence.end_time.toNSec()
                && position4D.z>=geofence.min_altitude && position4D.z<=geofence.max_altitude)
        {
            if (geofence.cylinder_shape)
            {
                if (sqrt((position4D.x-geofence.circle.x_center)*(position4D.x-geofence.circle.x_center)+
                         (position4D.y-geofence.circle.y_center)*(position4D.y-geofence.circle.y_center))<=geofence.circle.radius+safety_distance)
                    return i;
            }
            else
            {
                double distance=100000;
                int id_min=-1;
                int vertexes=geofence.polygon.x.size();
                double angle_sum=0.0;
                if (sqrt((geofence.polygon.x[0]-position4D.x)*(geofence.polygon.x[0]-position4D.x)+(geofence.polygon.y[0]-position4D.y)*(geofence.polygon.y[0]-position4D.y))<distance)
                {
                    id_min=i;
                    distance=sqrt((geofence.polygon.x[0]-position4D.x)*(geofence.polygon.x[0]-position4D.x)+(geofence.polygon.y[0]-position4D.y)*(geofence.polygon.y[0]-position4D.y));
                }
                for (int i=1; i<vertexes; i++)
                {
                    geometry_msgs::Point v1, v2;
                    v2.x=geofence.polygon.x[i]-position4D.x;
                    v2.y=geofence.polygon.y[i]-position4D.y;
                    v1.x=geofence.polygon.x[i-1]-position4D.x;
                    v1.y=geofence.polygon.y[i-1]-position4D.y;
                    angle_sum+=acos((v2.x*v1.x+v2.y*v1.y)/(sqrt(v1.x*v1.x+v1.y*v1.y)*sqrt(v2.x*v2.x+v2.y*v2.y)));
                    if (sqrt((geofence.polygon.x[i]-position4D.x)*(geofence.polygon.x[i]-position4D.x)+(geofence.polygon.y[i]-position4D.y)*(geofence.polygon.y[i]-position4D.y))<distance)
                    {
                        id_min=i;
                        distance=sqrt((geofence.polygon.x[i]-position4D.x)*(geofence.polygon.x[i]-position4D.x)+(geofence.polygon.y[i]-position4D.y)*(geofence.polygon.y[i]-position4D.y));
                    }
                }
                geometry_msgs::Point v1, v2, v3;
                v2.x=geofence.polygon.x[0]-position4D.x;
                v2.y=geofence.polygon.y[0]-position4D.y;
                v1.x=geofence.polygon.x[vertexes-1]-position4D.x;
                v1.y=geofence.polygon.y[vertexes-1]-position4D.y;
                angle_sum+=acos((v2.x*v1.x+v2.y*v1.y)/(sqrt(v1.x*v1.x+v1.y*v1.y)*sqrt(v2.x*v2.x+v2.y*v2.y)));

                if (abs(angle_sum)>6)  // cercano a 2PI (algoritmo radial para deterinar si un punto está dentro de un polígono)
                    return i;
                else   //calculating distance to no cylindrical geofence
                {
                    v1.x=position4D.x-geofence.polygon.x[id_min];
                    v1.y=position4D.y-geofence.polygon.y[id_min];
                    double ang1,ang2;

                    if (id_min==0)
                    {
                        v2.x=geofence.polygon.x[id_min+1]-geofence.polygon.x[id_min];
                        v2.y=geofence.polygon.y[id_min+1]-geofence.polygon.x[id_min];
                        v3.x=geofence.polygon.x[vertexes]-geofence.polygon.x[id_min];
                        v3.y=geofence.polygon.y[vertexes]-geofence.polygon.x[id_min];

                    }
                    else if (id_min==vertexes)
                    {
                        v2.x=geofence.polygon.x[0]-geofence.polygon.x[id_min];
                        v2.y=geofence.polygon.y[0]-geofence.polygon.x[id_min];
                        v3.x=geofence.polygon.x[id_min-1]-geofence.polygon.x[id_min];
                        v3.y=geofence.polygon.y[id_min-1]-geofence.polygon.x[id_min];

                    }
                    else
                    {
                        v2.x=geofence.polygon.x[id_min+1]-geofence.polygon.x[id_min];
                        v2.y=geofence.polygon.y[id_min+1]-geofence.polygon.x[id_min];
                        v3.x=geofence.polygon.x[id_min-1]-geofence.polygon.x[id_min];
                        v3.y=geofence.polygon.y[id_min-1]-geofence.polygon.x[id_min];
                    }
                    ang1=acos((v2.x*v1.x+v2.y*v1.y)/(sqrt(v1.x*v1.x+v1.y*v1.y)*sqrt(v2.x*v2.x+v2.y*v2.y)));
                    ang2=acos((v3.x*v1.x+v3.y*v1.y)/(sqrt(v1.x*v1.x+v1.y*v1.y)*sqrt(v3.x*v3.x+v3.y*v3.y)));

                    if (ang1<1.57)
                        distance=distance*sin(ang1);
                    else if (ang2<1.57)
                        distance=distance*sin(ang2);

                    if (distance<safety_distance)
                        return i;
                }
            }
        }
    }
    return -1;
}

// Service Callbacks
bool Monitoring::checkConflictsCB(gauss_msgs::CheckConflicts::Request &req, gauss_msgs::CheckConflicts::Response &res)
{
    int tam = req.deconflicted_wp.size();
    gauss_msgs::DB_size msg_size;
    int geofences;
    res.success=false;
    if (!(dbsize_cilent_.call(msg_size)) || !(msg_size.response.success))
    {
        ROS_ERROR("Failed to ask for number of missions and geofences");
        return false;
    }
    else
        geofences=msg_size.response.geofences;

    gauss_msgs::ReadOperation msg_op;
    msg_op.request.uav_ids.push_back(req.uav_id);
    if(!(read_operation_client_.call(msg_op)) || !(msg_op.response.success))
    {
        ROS_ERROR("Failed to read a trajectory");
        return false;
    }


    for (int i=0;i<tam;i++)
    {
        int geofence_intrusion = checkGeofences(req.deconflicted_wp.at(i),geofences,max(minDist,msg_op.response.operation.at(0).operational_volume));
        if (geofence_intrusion>=0)
        {
            gauss_msgs::Threat threat;
            threat.header.stamp=ros::Time::now();
            threat.uav_ids.push_back(req.uav_id);
            threat.geofence_ids.push_back(geofence_intrusion);
            threat.times.push_back(req.deconflicted_wp.at(i).stamp);
            threat.threat_type=threat.GEOFENCE_CONFLICT;
            res.threats.push_back(threat);
        }
        else if (geofence_intrusion==-2)
            return false;
    }

    if (mutex_lock_.try_lock())
    {
        int wps = req.deconflicted_wp.size();

        for (int i=0; i<wps; i++)
        {
            int posx = floor((req.deconflicted_wp.at(i).x-minX)/dX);
            int posy = floor((req.deconflicted_wp.at(i).y-minY)/dY);
            int posz = floor((req.deconflicted_wp.at(i).z-minZ)/dZ);
            int post = floor(req.deconflicted_wp.at(i).stamp.toSec()/dT-current_stamp.toSec()/dT);

            for (int m=max(0,posx-1); m<min(X,posx+2); m++)
                for (int n=max(0,posy-1); n<min(Y,posy+2); n++)
                    for (int p=max(0,posz-1); p<min(Z,posz+2); p++)
                        for (int t=max(0,post-1); t<min(T,post+2); t++)
                        {
                            if (grid[m][n][p][t].traj.size()>0)
                            {

                                list<int>::iterator it = grid[m][n][p][t].traj.begin();
                                list<int>::iterator it_wp = grid[m][n][p][t].wp.begin();
                                while (it != grid[m][n][p][t].traj.end())
                                {
                                    if (*it != req.uav_id)
                                    {
                                        gauss_msgs::WaypointList trajectory2 = msg_op.response.operation.at(*it).track;

                                        if (sqrt(pow(req.deconflicted_wp.at(i).x-trajectory2.waypoints.at(*it_wp).x,2)+
                                                 pow(req.deconflicted_wp.at(i).y-trajectory2.waypoints.at(*it_wp).y,2)+
                                                 pow(req.deconflicted_wp.at(i).z-trajectory2.waypoints.at(*it_wp).z,2))<minDist &&
                                                abs(req.deconflicted_wp.at(i).stamp.toSec()-trajectory2.waypoints.at(*it_wp).stamp.toSec())<dT)
                                        {
                                            gauss_msgs::Threat threat;
                                            threat.header.stamp=ros::Time::now();
                                            threat.threat_type = threat.LOSS_OF_SEPARATION;
                                            threat.uav_ids.push_back(req.uav_id);
                                            threat.uav_ids.push_back(*it);
                                            threat.times.push_back(req.deconflicted_wp.at(i).stamp);
                                            threat.times.push_back(trajectory2.waypoints.at(*it_wp).stamp);
                                            threat.priority_ops.push_back(msg_op.response.operation.at(i).priority);
                                            threat.priority_ops.push_back(msg_op.response.operation.at(*it).priority);
                                            res.threats.push_back(threat);
                                        }
                                    }
                                    it++;
                                    it_wp++;
                                }
                            }
                        }

        }
        mutex_lock_.unlock();
    }

    res.success=true;
    return true;
}

// Timer Callback
void Monitoring::timerCallback(const ros::TimerEvent &)
{
    int missions;
    int geofeces;

    // Set a start time
    static double start_time = ros::Time::now().toSec();

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

    gauss_msgs::Threats threats_msg;

    //locker=true;
    mutex_lock_.lock();

    //Clear previous grid
    for(int m=0;m<X;m++)
        for (int n=0;n<Y;n++)
            for (int p=0;p<Z;p++)
                for (int t=0;t<T;t++)
                {
                    grid[m][n][p][t].traj.clear();
                    grid[m][n][p][t].wp.clear();
                }

    gauss_msgs::ReadOperation msg_op;
    for (int i = 0; i < missions; i++) msg_op.request.uav_ids.push_back(i);
    if(!(read_operation_client_.call(msg_op)) || !(msg_op.response.success))
    {
        ROS_ERROR("Failed to read a trajectory");
        return;
    }
    // Rellena grid con waypoints de las missiones
    for (int i=0; i<missions; i++)
    {

        gauss_msgs::Operation operation = msg_op.response.operation[i];
        gauss_msgs::WaypointList trajectory = operation.estimated_trajectory;
        gauss_msgs::WaypointList plan = operation.flight_plan;

        geometry_msgs::Point vd;
        geometry_msgs::Point pq;
        geometry_msgs::Point pv;
        /*
        if (operation.current_wp >= plan.waypoints.size())
        {
            vd.x=plan.waypoints.at(operation.current_wp).x-plan.waypoints.at(operation.current_wp-1).x;
            vd.y=plan.waypoints.at(operation.current_wp).y-plan.waypoints.at(operation.current_wp-1).y;
            vd.z=plan.waypoints.at(operation.current_wp).z-plan.waypoints.at(operation.current_wp-1).z;
        }
        else
        {
            vd.x=plan.waypoints.at(operation.current_wp+1).x-plan.waypoints.at(operation.current_wp).x;
            vd.y=plan.waypoints.at(operation.current_wp+1).y-plan.waypoints.at(operation.current_wp).y;
            vd.z=plan.waypoints.at(operation.current_wp+1).z-plan.waypoints.at(operation.current_wp).z;
        }
        */

        // TODO: Next sentences fail when current_wp is 0 (that could happen if database is initialized with current_wp=0 and tracking hasn't updated it before monitoring reads)
        vd.x=plan.waypoints.at(operation.current_wp).x-plan.waypoints.at(operation.current_wp-1).x;
        vd.y=plan.waypoints.at(operation.current_wp).y-plan.waypoints.at(operation.current_wp-1).y;
        vd.z=plan.waypoints.at(operation.current_wp).z-plan.waypoints.at(operation.current_wp-1).z;

        pq.x=trajectory.waypoints.at(0).x-plan.waypoints.at(operation.current_wp).x;
        pq.y=trajectory.waypoints.at(0).y-plan.waypoints.at(operation.current_wp).y;
        pq.z=trajectory.waypoints.at(0).z-plan.waypoints.at(operation.current_wp).z;

        pv.x=vd.y*pq.z-vd.z*pq.y;
        pv.y=vd.x*pq.z-vd.z*pq.x;
        pv.z=vd.x*pq.y-vd.y*pq.x;

        // distancia = |pq x vd| / |vd|: distancia de punto q a recta cuyo vevtor director es vd y que contiene al punto p

        double distance=sqrt(pv.x*pv.x+pv.y*pv.y+pv.z*pv.z)/sqrt(vd.x*vd.x+vd.y*vd.y+vd.z*vd.z);

        if (distance>operation.flight_geometry)
        {
            gauss_msgs::Threat threat;
            threat.header.stamp=ros::Time::now();
            threat.uav_ids.push_back(i);
            threat.times.push_back(trajectory.waypoints.at(0).stamp);
            if (distance<operation.operational_volume)
                threat.threat_type=threat.UAS_IN_CV;
            else
                threat.threat_type=threat.UAS_OUT_OV;
            threats_msg.request.uav_ids.push_back(i);
            threats_msg.request.threats.push_back(threat);
        }

        int waypoints = trajectory.waypoints.size();

        for (int j=0; j<waypoints; j++)
        {
            // para la trayectoria estimada comprobar que no estas dentro de un GEOFENCE
            int geofence_intrusion = checkGeofences(trajectory.waypoints.at(j),geofeces,max(minDist,operation.operational_volume));
            if (geofence_intrusion>=0)
            {
                gauss_msgs::Threat threat;
                threat.header.stamp=ros::Time::now();
                threat.uav_ids.push_back(i);
                threat.geofence_ids.push_back(geofence_intrusion);
                threat.times.push_back(trajectory.waypoints.at(j).stamp);
                if (j==0)
                    threat.threat_type=threat.GEOFENCE_INTRUSION;
                else
                    threat.threat_type=threat.GEOFENCE_CONFLICT;
                threats_msg.request.uav_ids.push_back(i);
                threats_msg.request.threats.push_back(threat);
            }
            int posx = floor((trajectory.waypoints.at(j).x-minX)/dX);
            int posy = floor((trajectory.waypoints.at(j).y-minY)/dY);
            int posz = floor((trajectory.waypoints.at(j).z-minZ)/dZ);
            int post = floor(trajectory.waypoints.at(j).stamp.toSec()/dT-trajectory.waypoints.at(0).stamp.toSec()/dT);

            current_stamp=trajectory.waypoints.at(0).stamp;

            grid[posx][posy][posz][post].traj.push_back(i);
            grid[posx][posy][posz][post].wp.push_back(j);

            for (int m=max(0,posx-1); m<min(X,posx+2); m++)
                for (int n=max(0,posy-1); n<min(Y,posy+2); n++)
                    for (int p=max(0,posz-1); p<min(Z,posz+2); p++)
                        for (int t=max(0,post-1); t<min(T,post+2); t++)
                        {

                            if (grid[m][n][p][t].traj.size()>0)
                            {
                                list<int>::iterator it = grid[m][n][p][t].traj.begin();
                                list<int>::iterator it_wp = grid[m][n][p][t].wp.begin();
                                while (it != grid[m][n][p][t].traj.end())
                                {
                                    if (*it != i)
                                    {
                                        gauss_msgs::WaypointList trajectory2 = msg_op.response.operation[*it].estimated_trajectory;

                                        double minDistAux=max(minDist,operation.operational_volume+msg_op.response.operation[*it].operational_volume);


                                        if (sqrt(pow(trajectory.waypoints.at(j).x-trajectory2.waypoints.at(*it_wp).x,2)+
                                                 pow(trajectory.waypoints.at(j).y-trajectory2.waypoints.at(*it_wp).y,2)+
                                                 pow(trajectory.waypoints.at(j).z-trajectory2.waypoints.at(*it_wp).z,2))<minDistAux &&
                                                abs(trajectory.waypoints.at(j).stamp.toSec()-trajectory2.waypoints.at(*it_wp).stamp.toSec())<dT)
                                        {
                                            std::cout << "Threat detected between uav_ids: " << i << " and " << *it << "\n";
                                            std::cout << "waypoints (x,y,z,t): (";
                                            std::cout << trajectory.waypoints.at(j).x << "," << trajectory.waypoints.at(j).y << "," << trajectory.waypoints.at(j).z << "," << trajectory.waypoints.at(j).stamp.toSec() << ")";
                                            std::cout << ",(";
                                            std::cout << trajectory2.waypoints.at(*it_wp).x << "," << trajectory2.waypoints.at(*it_wp).y << "," << trajectory2.waypoints.at(*it_wp).z << "," << trajectory2.waypoints.at(*it_wp).stamp.toSec() << ")";
                                            std::cout << "\n";
                                            gauss_msgs::Threat threat;
                                            threat.header.stamp=ros::Time::now();
                                            threat.uav_ids.push_back(i);
                                            threat.uav_ids.push_back(*it);
                                            threat.times.push_back(trajectory.waypoints.at(j).stamp);
                                            threat.times.push_back(trajectory2.waypoints.at(*it_wp).stamp);
                                            threat.priority_ops.push_back(msg_op.response.operation.at(i).priority);
                                            threat.priority_ops.push_back(msg_op.response.operation.at(*it).priority);
                                            threat.threat_type=threat.LOSS_OF_SEPARATION;

                                            threats_msg.request.uav_ids.push_back(i);
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
    //locker=false;
    mutex_lock_.unlock();

    // LLamar al servicio alerta
    if (threats_msg.request.threats.size() > 0)
    {
        std::cout << "Calling threat service\n"; 
        gauss_msgs::Threats new_threats_msgs = manageThreatList(threats_msg, start_time);
        for (auto i : new_threats_msgs.request.threats) std::cout << i << "\n";
        if (new_threats_msgs.request.threats.size() > 0){
            if(!(threats_client_.call(new_threats_msgs)) || !(new_threats_msgs.response.success))
            {
                ROS_ERROR("Failed to send alert message");
                return;
            }
        }

    }
    double delete_time = 60.0; // seconds
    // Delete old threats on the list
    deleteOldThreats(start_time, delete_time);

}



// MAIN function
int main(int argc, char *argv[])
{
    ros::init(argc,argv,"monitoring");

    // Create a Monitoring object
    Monitoring *monitoring = new Monitoring();

    ros::spin();
}
