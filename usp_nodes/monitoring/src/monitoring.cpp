#include <ros/ros.h>
#include <gauss_msgs/Threats.h>
#include <gauss_msgs/Threat.h>
#include <gauss_msgs/ReadGeofences.h>
#include <gauss_msgs/ReadOperation.h>
#include <gauss_msgs/DB_size.h>
#include <gauss_msgs/ReadIcao.h>
#include <gauss_msgs/UpdateThreats.h>
#include <list>
#include <geometry_msgs/Point.h>
#include <gauss_msgs/Waypoint.h>
#include <gauss_msgs/CheckConflicts.h>
#include <algorithm>
#include <mutex>

using namespace std;

inline bool indexIsInRange(int index, int size) {
    return (index >= 0) && (index < size);
}

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
    bool updateThreatsCB(gauss_msgs::UpdateThreats::Request &req, gauss_msgs::UpdateThreats::Response &res);

    // Timer Callbacks
    void timerCallback(const ros::TimerEvent&);

    // Auxilary methods
    int checkGeofences(std::vector<gauss_msgs::Geofence> &_geofences, gauss_msgs::Waypoint position4D, double safety_distance);
    void cleanThreatList(const std::vector<gauss_msgs::Threat> &_in_threats);
    gauss_msgs::Threats manageThreatList(const gauss_msgs::Threats &_in_threats);
    gauss_msgs::Threats fillConflictiveFields(gauss_msgs::Threats &_in_threats, const gauss_msgs::ReadOperation &_msg_op, const gauss_msgs::ReadGeofences &_msg_geofence);
    bool posIndicesAreInRange(int x, int y, int z, int t);

    // Auxilary variables
    bool just_one_threat;
    double monitoring_timer;
    int X,Y,Z,T;
    double dX,dY,dZ,dT;
    double minX,maxX,minY,maxY,minZ,maxZ,maxT;
    double minDist;
    int threat_list_id_ = 0;
    vector<gauss_msgs::Threat> threat_list_;

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
    ros::ServiceServer check_conflicts_server_, update_threats_server_;

    // Client
    ros::ServiceClient threats_client_, read_geofence_client_, read_operation_client_, dbsize_cilent_, read_icao_client_;
};

// Monitoring Constructor
Monitoring::Monitoring()
{
    // Read
    nh_.param("safetyDistance",minDist,10.0);
    nh_.param("minX",minX,-400.0);
    nh_.param("minY",minY,0.0);
    nh_.param("minZ",minZ,0.0);
    nh_.param("maxX",maxX,0.0);
    nh_.param("maxY",maxY,400.0);
    nh_.param("maxZ",maxZ,300.0);
    nh_.param("time_horizon",maxT,300.0);
    nh_.param("dT",dT,15.0);
    nh_.param("monitoring_timer", monitoring_timer,1.0);  // TODO: Check monitoring_rate from usp_manager_ual_bridge.cpp
    nh_.param("deltaX",dX,10.0);
    nh_.param("deltaY",dY,10.0);
    nh_.param("deltaZ",dZ,10.0);
    nh_.param("just_one_threat",just_one_threat, false);

    double cell_diagonal = sqrt(pow(dX, 2) + pow(dY, 2) + pow(dZ, 2));

    if (minDist > cell_diagonal){
        ROS_ERROR("[Monitoring] Safety distance (%.2f) can not be bigger than the diagonal of the 3D cell (%.2f). Check delta XYZ values and safety distance to fix it.", minDist, cell_diagonal);
        minDist = cell_diagonal;
    }

    // Initialization    
    // dT=1.0/rate;
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
    update_threats_server_ = nh_.advertiseService("/gauss/update_threats",&Monitoring::updateThreatsCB,this);

    // Client
    read_operation_client_ = nh_.serviceClient<gauss_msgs::ReadOperation>("/gauss/read_operation");
    read_geofence_client_ = nh_.serviceClient<gauss_msgs::ReadGeofences>("/gauss/read_geofences");
    threats_client_ = nh_.serviceClient<gauss_msgs::Threats>("/gauss/threats");
    dbsize_cilent_ = nh_.serviceClient<gauss_msgs::DB_size>("/gauss/db_size");
    read_icao_client_ = nh_.serviceClient<gauss_msgs::ReadIcao>("/gauss/read_icao");

    // Timer
    timer_sub_=nh_.createTimer(ros::Duration(monitoring_timer),&Monitoring::timerCallback,this);

    ROS_INFO("[Monitoring] Started Monitoring node!");
}

// Auxilary methods
gauss_msgs::Threats Monitoring::manageThreatList(const gauss_msgs::Threats &_in_threats){
    gauss_msgs::Threats out_threats;
    std::vector<int> threat_list_ids_updated;
    if (threat_list_.size() == 0) {
        threat_list_.push_back(_in_threats.request.threats.front());
        threat_list_.front().threat_id = threat_list_id_;
        threat_list_ids_updated.push_back(threat_list_id_);
        out_threats.request.threats.push_back(threat_list_.front());
        out_threats.request.uav_ids.push_back(threat_list_.front().uav_ids.front());
        if (threat_list_.front().threat_type == threat_list_.front().LOSS_OF_SEPARATION) 
            out_threats.request.uav_ids.push_back(threat_list_.front().uav_ids.back());
        threat_list_id_++;
    }
    if (threat_list_.size() > 0){
        for (auto in_threat : _in_threats.request.threats){
            bool save_threat = false;
            // Using lambda, check if threat_type of in_threat is in threat_list
            std::vector<gauss_msgs::Threat>::iterator it = std::find_if(threat_list_.begin(), threat_list_.end(), 
                                                                        [in_threat](gauss_msgs::Threat threat){return (threat.threat_type == in_threat.threat_type);});
            if (it != threat_list_.end()){
                // Type found!
                // If loss of separation, check 2 uav ids else check 1 uav id. If a geofence is involved, check geofence id.
                if (in_threat.threat_type == in_threat.LOSS_OF_SEPARATION){
                    // TODO: Check all uav ids
                    if ((in_threat.uav_ids.front() != it->uav_ids.front() && in_threat.uav_ids.back() != it->uav_ids.back())) {
                       save_threat = true;
                    }
                } else if (in_threat.threat_type == in_threat.GEOFENCE_CONFLICT ||
                           in_threat.threat_type == in_threat.GEOFENCE_INTRUSION){
                               if (in_threat.uav_ids.front() != it->uav_ids.front() ||
                                   in_threat.geofence_ids.front() != it->geofence_ids.front()){
                                       save_threat = true;
                                }
                } else {
                    if (in_threat.uav_ids.front() != it->uav_ids.front()){
                            save_threat = true;
                    }
                }
            } else {
                // Type not found!
                save_threat = true;
            }
            if (save_threat){
                in_threat.threat_id = threat_list_id_;
                threat_list_.push_back(in_threat);
                threat_list_ids_updated.push_back(threat_list_id_);
                out_threats.request.threats.push_back(in_threat);
                out_threats.request.uav_ids.push_back(in_threat.uav_ids.front());
                if (in_threat.threat_type == in_threat.LOSS_OF_SEPARATION){
                    out_threats.request.uav_ids.push_back(in_threat.uav_ids.back());
                }
                threat_list_id_++;
            }
            if (just_one_threat) break;
        }
    } 

    std::string cout_threats;
    for (auto threat : out_threats.request.threats) {
        cout_threats = cout_threats + " [" + std::to_string(threat.threat_id) + " " + std::to_string(threat.threat_type) + " |";
        for (auto uav_id : threat.uav_ids) cout_threats = cout_threats + " " + std::to_string(uav_id);
        cout_threats = cout_threats + "]";
    } 
    ROS_INFO_STREAM_COND(out_threats.request.threats.size() > 0, "[Monitoring] Threats detected: (id type | uav) " + cout_threats);

    return out_threats;
}

gauss_msgs::Threats Monitoring::fillConflictiveFields(gauss_msgs::Threats &_in_threats, const gauss_msgs::ReadOperation &_msg_op, const gauss_msgs::ReadGeofences &_msg_geofence){
    for (int i = 0; i < _in_threats.request.uav_ids.size(); i++){
        gauss_msgs::ConflictiveOperation conflictive_operation;
        int j = _in_threats.request.uav_ids.at(i);
        conflictive_operation.uav_id = _msg_op.response.operation.at(j).uav_id;
        conflictive_operation.current_wp = _msg_op.response.operation.at(j).current_wp;
        conflictive_operation.flight_plan = _msg_op.response.operation.at(j).flight_plan;
        conflictive_operation.landing_spots = _msg_op.response.operation.at(j).landing_spots;
        conflictive_operation.actual_wp = _msg_op.response.operation.at(j).track.waypoints.back();
        conflictive_operation.operational_volume = _msg_op.response.operation.at(j).operational_volume;
        conflictive_operation.flight_plan_updated = _msg_op.response.operation.at(j).flight_plan_updated;
        conflictive_operation.estimated_trajectory = _msg_op.response.operation.at(j).estimated_trajectory;
        _in_threats.request.operations.push_back(conflictive_operation);
    }
    // Check which geofences are conflictive and do not repeat it
    std::vector<int> aux_geofences_id;
    for (auto threat : _in_threats.request.threats){
        for (auto geofence_id : threat.geofence_ids){
            if (std::find(aux_geofences_id.begin(), aux_geofences_id.end(), geofence_id) == aux_geofences_id.end()){ // Geofence id not found!
                aux_geofences_id.push_back(geofence_id);
            }
        }
    }
    // Fill conflictive geofences
    for (auto id : aux_geofences_id){
        _in_threats.request.geofences.push_back(_msg_geofence.response.geofences.at(id));
    }

    return _in_threats;
}

void Monitoring::cleanThreatList(const std::vector<gauss_msgs::Threat> &_in_threats){
    // Delete non-updated threats from threat_list_
    for (auto saved_threat = threat_list_.begin(); saved_threat != threat_list_.end();){
        std::vector<gauss_msgs::Threat>::const_iterator it = std::find_if(_in_threats.begin(), _in_threats.end(), 
                                                                    [saved_threat](gauss_msgs::Threat threat){
                                                                        bool condition;
                                                                        if (threat.threat_type == saved_threat->threat_type){
                                                                            if (threat.threat_type == threat.LOSS_OF_SEPARATION){
                                                                                condition = (threat.uav_ids.front() == saved_threat->uav_ids.front() &&
                                                                                                threat.uav_ids.back() == saved_threat->uav_ids.back() &&
                                                                                                (abs(threat.times.front().toSec() - saved_threat->times.front().toSec()) <= 2.5 && // TODO: Always check this. 2.5 = dT / 2
                                                                                                abs(threat.times.back().toSec() - saved_threat->times.back().toSec()) <= 2.5));
                                                                                                            
                                                                            } else {
                                                                                condition = (threat.uav_ids.front() == saved_threat->uav_ids.front());
                                                                            }
                                                                        }
                                                                        return condition;});

        if (it != _in_threats.end()){
            // Condition found!
            saved_threat++;
        } else {
            // Condition not found!
            saved_threat = threat_list_.erase(saved_threat);
            if (_in_threats.size() == 0) saved_threat = threat_list_.end();
        }
    }
}

int Monitoring::checkGeofences(std::vector<gauss_msgs::Geofence> &_geofences, gauss_msgs::Waypoint position4D, double safety_distance)
{
    for (int i=0; i<_geofences.size(); i++)
    {
        if (_geofences.at(i).start_time < ros::Time::now() && ros::Time::now() < _geofences.at(i).end_time) {
            if (position4D.stamp.toNSec()>=_geofences.at(i).start_time.toNSec() && position4D.stamp.toNSec()<=_geofences.at(i).end_time.toNSec()
                    && position4D.z>=_geofences.at(i).min_altitude && position4D.z<=_geofences.at(i).max_altitude)
            {
                if (_geofences.at(i).cylinder_shape)
                {
                    if (sqrt((position4D.x-_geofences.at(i).circle.x_center)*(position4D.x-_geofences.at(i).circle.x_center)+
                            (position4D.y-_geofences.at(i).circle.y_center)*(position4D.y-_geofences.at(i).circle.y_center))<=_geofences.at(i).circle.radius+safety_distance)
                        return i;
                }
                else
                {
                    double distance=100000;
                    int id_min=-1;
                    int vertexes=_geofences.at(i).polygon.x.size();
                    double angle_sum=0.0;
                    if (sqrt((_geofences.at(i).polygon.x[0]-position4D.x)*(_geofences.at(i).polygon.x[0]-position4D.x)+(_geofences.at(i).polygon.y[0]-position4D.y)*(_geofences.at(i).polygon.y[0]-position4D.y))<distance)
                    {
                        id_min=i;
                        distance=sqrt((_geofences.at(i).polygon.x[0]-position4D.x)*(_geofences.at(i).polygon.x[0]-position4D.x)+(_geofences.at(i).polygon.y[0]-position4D.y)*(_geofences.at(i).polygon.y[0]-position4D.y));
                    }
                    for (int j=1; j<vertexes; j++)
                    {
                        geometry_msgs::Point v1, v2;
                        v2.x=_geofences.at(i).polygon.x[j]-position4D.x;
                        v2.y=_geofences.at(i).polygon.y[j]-position4D.y;
                        v1.x=_geofences.at(i).polygon.x[j-1]-position4D.x;
                        v1.y=_geofences.at(i).polygon.y[j-1]-position4D.y;
                        angle_sum+=acos((v2.x*v1.x+v2.y*v1.y)/(sqrt(v1.x*v1.x+v1.y*v1.y)*sqrt(v2.x*v2.x+v2.y*v2.y)));
                        if (sqrt((_geofences.at(i).polygon.x[j]-position4D.x)*(_geofences.at(i).polygon.x[j]-position4D.x)+(_geofences.at(i).polygon.y[j]-position4D.y)*(_geofences.at(i).polygon.y[j]-position4D.y))<distance)
                        {
                            id_min=j;
                            distance=sqrt((_geofences.at(i).polygon.x[j]-position4D.x)*(_geofences.at(i).polygon.x[j]-position4D.x)+(_geofences.at(i).polygon.y[j]-position4D.y)*(_geofences.at(i).polygon.y[j]-position4D.y));
                        }
                    }
                    geometry_msgs::Point v1, v2, v3;
                    v2.x=_geofences.at(i).polygon.x[0]-position4D.x;
                    v2.y=_geofences.at(i).polygon.y[0]-position4D.y;
                    v1.x=_geofences.at(i).polygon.x[vertexes-1]-position4D.x;
                    v1.y=_geofences.at(i).polygon.y[vertexes-1]-position4D.y;
                    angle_sum+=acos((v2.x*v1.x+v2.y*v1.y)/(sqrt(v1.x*v1.x+v1.y*v1.y)*sqrt(v2.x*v2.x+v2.y*v2.y)));

                    if (abs(angle_sum)>6)  // cercano a 2PI (algoritmo radial para deterinar si un punto está dentro de un polígono)
                        return i;
                    else   //calculating distance to no cylindrical geofence
                    {
                        v1.x=position4D.x-_geofences.at(i).polygon.x[id_min];
                        v1.y=position4D.y-_geofences.at(i).polygon.y[id_min];
                        double ang1,ang2;

                        if (id_min==0)
                        {
                            v2.x=_geofences.at(i).polygon.x[id_min+1]-_geofences.at(i).polygon.x[id_min];
                            v2.y=_geofences.at(i).polygon.y[id_min+1]-_geofences.at(i).polygon.x[id_min];
                            v3.x=_geofences.at(i).polygon.x[vertexes]-_geofences.at(i).polygon.x[id_min];
                            v3.y=_geofences.at(i).polygon.y[vertexes]-_geofences.at(i).polygon.x[id_min];

                        }
                        else if (id_min==vertexes)
                        {
                            v2.x=_geofences.at(i).polygon.x[0]-_geofences.at(i).polygon.x[id_min];
                            v2.y=_geofences.at(i).polygon.y[0]-_geofences.at(i).polygon.x[id_min];
                            v3.x=_geofences.at(i).polygon.x[id_min-1]-_geofences.at(i).polygon.x[id_min];
                            v3.y=_geofences.at(i).polygon.y[id_min-1]-_geofences.at(i).polygon.x[id_min];

                        }
                        else
                        {
                            v2.x=_geofences.at(i).polygon.x[id_min+1]-_geofences.at(i).polygon.x[id_min];
                            v2.y=_geofences.at(i).polygon.y[id_min+1]-_geofences.at(i).polygon.x[id_min];
                            v3.x=_geofences.at(i).polygon.x[id_min-1]-_geofences.at(i).polygon.x[id_min];
                            v3.y=_geofences.at(i).polygon.y[id_min-1]-_geofences.at(i).polygon.x[id_min];
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
    }
    return -1;
}

// Service Callbacks
bool Monitoring::updateThreatsCB(gauss_msgs::UpdateThreats::Request &req, gauss_msgs::UpdateThreats::Response &res){
    gauss_msgs::Threats threats_msg;
    gauss_msgs::ReadOperation msg_op;
    gauss_msgs::ReadGeofences msg_geo;
    std::string msg_not_found_id;
    
    for (auto threat_id : req.threat_ids){
        std::vector<gauss_msgs::Threat>::iterator it = std::find_if(threat_list_.begin(), threat_list_.end(), 
                                                            [threat_id](gauss_msgs::Threat threat){return (threat.threat_id == threat_id);});
        if (it != threat_list_.end()){ // Threat id found!
            threats_msg.request.threats.push_back(*it);
            for (auto uav_id : it->uav_ids){
                if (std::find(msg_op.request.uav_ids.begin(), msg_op.request.uav_ids.end(), uav_id) == msg_op.request.uav_ids.end()){ // UAV id not found!
                    msg_op.request.uav_ids.push_back(uav_id);
                }
            }
            if (it->threat_type == it->GEOFENCE_CONFLICT || it->threat_type == it->GEOFENCE_INTRUSION){
                for (auto geofence_id : it->geofence_ids){
                    if (std::find(msg_geo.request.geofences_ids.begin(), msg_geo.request.geofences_ids.end(), geofence_id) == msg_geo.request.geofences_ids.end()){ // UAV id not found!
                        msg_geo.request.geofences_ids.push_back(geofence_id);
                    }
                }
            }
        } else { // Threat id not found!
            msg_not_found_id = msg_not_found_id + " " + std::to_string(it->threat_id);
        }
    }

    if (msg_op.request.uav_ids.size() > 0){
        if (!read_operation_client_.call(msg_op) || !msg_op.response.success){
            ROS_ERROR("[Monitoring] Failed to read operations on data base");
        } else {
            res.uav_ids = msg_op.request.uav_ids;
            threats_msg.request.uav_ids = msg_op.request.uav_ids;
        }
    }
    if (msg_geo.request.geofences_ids.size() > 0){
        if (!read_geofence_client_.call(msg_geo) || !msg_geo.response.success){
            ROS_ERROR("[Monitoring] Failed to read geofences on data base");
        } else {
            res.geofences = msg_geo.response.geofences;
        }
    }

    threats_msg = fillConflictiveFields(threats_msg, msg_op, msg_geo);
    res.threats = threats_msg.request.threats;
    res.operations = threats_msg.request.operations;

    if (msg_not_found_id.empty()){
        res.message = "All threats were updated";
        res.success = true;
    } else {
        res.message = "Not found threat ids: " + msg_not_found_id;
    }

    return true;
}

bool Monitoring::checkConflictsCB(gauss_msgs::CheckConflicts::Request &req, gauss_msgs::CheckConflicts::Response &res)
{
    gauss_msgs::ReadIcao msg_ids;
    if (!(read_icao_client_.call(msg_ids)) || !(msg_ids.response.success))
    {
        ROS_ERROR("[Monitoring] Failed to ask for number of missions and geofences");
        return false;
    }

    gauss_msgs::ReadOperation msg_op;
    for (int i = 0; i < msg_ids.response.uav_id.size(); i++) msg_op.request.uav_ids.push_back(i);
    if(!(read_operation_client_.call(msg_op)) || !(msg_op.response.success))
    {
        ROS_ERROR("[Monitoring] Failed to read a trajectory");
        return false;
    }

    gauss_msgs::ReadGeofences msg_geofence;
    if (msg_ids.response.geofence_id.size() > 0){
        for (int i = 0; i < msg_ids.response.geofence_id.size(); i++) msg_geofence.request.geofences_ids.push_back(i);
        if(!(read_geofence_client_.call(msg_geofence)) || !(msg_geofence.response.success))
        {
            ROS_ERROR("[Monitoring] Failed to read a geofence");
            return false;
        }
    }
    // Rellena grid con waypoints de las missiones
    for (int i=0; i<msg_ids.response.uav_id.size(); i++)
    {
        if (msg_op.response.operation[i].current_wp != 0 && msg_op.response.operation[i].is_started){
            for (int j=0; j<msg_op.response.operation[i].estimated_trajectory.waypoints.size() - 1; j++)
            {
                // para la trayectoria estimada comprobar que no estas dentro de un GEOFENCE
                if (msg_ids.response.geofence_id.size()>0){
                    int geofence_intrusion = checkGeofences(msg_geofence.response.geofences, msg_op.response.operation[i].estimated_trajectory.waypoints.at(j),max(minDist,msg_op.response.operation[i].operational_volume));
                    if (geofence_intrusion>=0)
                    {
                        gauss_msgs::Threat threat;
                        threat.header.stamp=ros::Time::now();
                        threat.uav_ids.push_back(i);
                        threat.geofence_ids.push_back(geofence_intrusion);
                        threat.times.push_back(msg_op.response.operation[i].estimated_trajectory.waypoints.at(j).stamp);
                        if (j==0)
                            threat.threat_type=threat.GEOFENCE_INTRUSION;
                        else
                            threat.threat_type=threat.GEOFENCE_CONFLICT;
                        res.threats.push_back(threat);
                    }
                }
                // TODO: Try to get the best value for minDistAux
                double minDistAux = msg_op.response.operation.at(i).operational_volume;

                for (auto traj2_wp : req.deconflicted_wp){
                    if (sqrt(pow(msg_op.response.operation[i].estimated_trajectory.waypoints.at(j).x-traj2_wp.x,2)+
                             pow(msg_op.response.operation[i].estimated_trajectory.waypoints.at(j).y-traj2_wp.y,2)+
                             pow(msg_op.response.operation[i].estimated_trajectory.waypoints.at(j).z-traj2_wp.z,2))<minDistAux &&
                             abs(msg_op.response.operation[i].estimated_trajectory.waypoints.at(j).stamp.toSec()-traj2_wp.stamp.toSec())<dT)
                    {
                        gauss_msgs::Threat threat;
                        threat.header.stamp=ros::Time::now();
                        threat.uav_ids.push_back(i);
                        threat.uav_ids.push_back(req.uav_id);
                        threat.times.push_back(msg_op.response.operation[i].estimated_trajectory.waypoints.at(j).stamp);
                        threat.times.push_back(traj2_wp.stamp);
                        threat.priority_ops.push_back(msg_op.response.operation.at(i).priority);
                        threat.threat_type=threat.LOSS_OF_SEPARATION;
                        res.threats.push_back(threat);
                    }
                }
            }
        }
    }

    res.success=true;
    return true;
}

// Timer Callback
void Monitoring::timerCallback(const ros::TimerEvent &)
{
    int missions;
    int geofeces;

    // Ask for number of missions and geofences
    double start_computational_time = ros::Time::now().toSec();
    gauss_msgs::ReadIcao msg_ids;
    if (!(read_icao_client_.call(msg_ids)) || !(msg_ids.response.success))
    {
        ROS_ERROR("[Monitoring] Failed to ask for number of missions and geofences");
        return;
    }
    else
    {
        geofeces=msg_ids.response.geofence_id.size();
        missions=msg_ids.response.uav_id.size();
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
    for (int i = 0; i < msg_ids.response.uav_id.size(); i++) msg_op.request.uav_ids.push_back(i);
    if(!(read_operation_client_.call(msg_op)) || !(msg_op.response.success))
    {
        ROS_ERROR("[Monitoring] Failed to read a trajectory");
        return;
    }
    gauss_msgs::ReadGeofences msg_geofence;
    if (msg_ids.response.geofence_id.size() > 0){
        for (int i = 0; i < msg_ids.response.geofence_id.size(); i++) msg_geofence.request.geofences_ids.push_back(msg_ids.response.geofence_id.at(i));
        if(!(read_geofence_client_.call(msg_geofence)) || !(msg_geofence.response.success))
        {
            ROS_ERROR_STREAM("[Monitoring] " + msg_geofence.response.message);
            return;
        }
    }
    // Rellena grid con waypoints de las missiones
    for (int i=0; i<missions; i++)
    {
        gauss_msgs::Operation operation = msg_op.response.operation[i];
        if (operation.current_wp != 0 && operation.is_started){
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

            for (int j=0; j<trajectory.waypoints.size() - 1; j++)
            {
                // para la trayectoria estimada comprobar que no estas dentro de un GEOFENCE
                if (msg_ids.response.geofence_id.size()>0){
                    int geofence_intrusion = checkGeofences(msg_geofence.response.geofences, trajectory.waypoints.at(j),max(minDist,operation.operational_volume));
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
                }
                int posx = floor((trajectory.waypoints.at(j).x-minX)/dX);
                int posy = floor((trajectory.waypoints.at(j).y-minY)/dY);
                int posz = floor((trajectory.waypoints.at(j).z-minZ)/dZ);
                int post = floor(trajectory.waypoints.at(j).stamp.toSec()/dT-trajectory.waypoints.at(0).stamp.toSec()/dT);

                current_stamp=trajectory.waypoints.at(0).stamp;

                if (!posIndicesAreInRange(posx, posy, posz, post)) { return; }

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
                                            gauss_msgs::WaypointList trajectory2 = msg_op.response.operation.at(*it).estimated_trajectory;

                                            double minDistAux=max(minDist,operation.operational_volume+msg_op.response.operation.at(*it).operational_volume);


                                            if (sqrt(pow(trajectory.waypoints.at(j).x-trajectory2.waypoints.at(*it_wp).x,2)+
                                                    pow(trajectory.waypoints.at(j).y-trajectory2.waypoints.at(*it_wp).y,2)+
                                                    pow(trajectory.waypoints.at(j).z-trajectory2.waypoints.at(*it_wp).z,2))<minDistAux &&
                                                    abs(trajectory.waypoints.at(j).stamp.toSec()-trajectory2.waypoints.at(*it_wp).stamp.toSec())<dT)
                                            {
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
        } else if (operation.is_started && operation.current_wp == 0) {
            ROS_WARN("[Monitoring] Operation %d current wp is 0", operation.uav_id);
        } else if (!operation.is_started && ros::Time::now().toSec() > operation.flight_plan.waypoints.front().stamp.toSec()) {
            ROS_WARN_ONCE("[Monitoring] Operation %d did not start. First waypoint is at %.f (s)", operation.uav_id, operation.flight_plan.waypoints.front().stamp.toSec());
        }
    }
    //locker=false;
    mutex_lock_.unlock();

    // LLamar al servicio alerta
    if (threats_msg.request.threats.size() > 0)
    {
        // Manage threat list
        gauss_msgs::Threats new_threats_msgs = manageThreatList(threats_msg);
        // Fill conflictive operations
        if (new_threats_msgs.request.threats.size() > 0){
            new_threats_msgs = fillConflictiveFields(new_threats_msgs, msg_op, msg_geofence);
            // Call threats service
            // ROS_INFO("[Monitoring] Computational time: %0.4f", ros::Time::now().toSec() - start_computational_time);
            if(!(threats_client_.call(new_threats_msgs)) || !(new_threats_msgs.response.success))
            {
                ROS_ERROR("[Monitoring] Failed to send alert message");
                return;
            }
        }
    }
    // Delete old saved threats
    // if(threat_list_.size() > 0) cleanThreatList(threats_msg.request.threats);
}

bool Monitoring::posIndicesAreInRange(int x, int y, int z, int t) {
    if (!indexIsInRange(x, X)) {
        ROS_ERROR("[Monitoring] Index x is out of range: %d out of [0, %d)", x, X);
        return false;
    }
    if (!indexIsInRange(y, Y)) {
        ROS_ERROR("[Monitoring] Index y is out of range: %d out of [0, %d)", y, Y);
        return false;
    }
    if (!indexIsInRange(z, Z)) {
        ROS_ERROR("[Monitoring] Index z is out of range: %d out of [0, %d)", z, Z);
        return false;
    }
    if (!indexIsInRange(t, T)) {
        ROS_ERROR("[Monitoring] Index t is out of range: %d out of [0, %d)", t, T);
        return false;
    }
    return true;
}


// MAIN function
int main(int argc, char *argv[])
{
    ros::init(argc,argv,"monitoring");

    // Create a Monitoring object
    Monitoring *monitoring = new Monitoring();

    ros::spin();
}
