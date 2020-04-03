#include <ros/ros.h>
#include <ros/package.h>
#include <gauss_msgs/ReadTracks.h>
#include <gauss_msgs/WriteTracks.h>
#include <gauss_msgs/ReadTraj.h>
#include <gauss_msgs/WriteTraj.h>
#include <gauss_msgs/ReadFlightPlan.h>
#include <gauss_msgs/WriteFlightPlan.h>
#include <gauss_msgs/ReadGeofences.h>
#include <gauss_msgs/WriteGeofences.h>
#include <gauss_msgs/Operation.h>
#include <gauss_msgs/ReadOperation.h>
#include <gauss_msgs/WriteOperation.h>
#include <gauss_msgs/Geofence.h>
#include <gauss_msgs/DB_size.h>
#include <list>
#include <fstream>
#include <db_manager/json.hpp>

using namespace std;

// Class definition
class DataBase
{
public:
    DataBase();

private:
    // Topic Callbacks

    // Service Callbacks
    bool readOperationCB(gauss_msgs::ReadOperation::Request &req, gauss_msgs::ReadOperation::Response &res);
    bool writeOperationCB(gauss_msgs::WriteOperation::Request &req, gauss_msgs::WriteOperation::Response &res);
    bool readGeofenceCB(gauss_msgs::ReadGeofences::Request &req, gauss_msgs::ReadGeofences::Response &res);
    bool writeGeofenceCB(gauss_msgs::WriteGeofences::Request &req, gauss_msgs::WriteGeofences::Response &res);
    bool readPlanCB(gauss_msgs::ReadFlightPlan::Request &req, gauss_msgs::ReadFlightPlan::Response &res);
    bool writePlanCB(gauss_msgs::WriteFlightPlan::Request &req, gauss_msgs::WriteFlightPlan::Response &res);
    bool readTrackCB(gauss_msgs::ReadTracks::Request &req, gauss_msgs::ReadTracks::Response &res);
    bool writeTrackCB(gauss_msgs::WriteTracks::Request &req, gauss_msgs::WriteTracks::Response &res);
    bool readTrajectoryCB(gauss_msgs::ReadTraj::Request &req, gauss_msgs::ReadTraj::Response &res);
    bool writeTrajectoryCB(gauss_msgs::WriteTraj::Request &req, gauss_msgs::WriteTraj::Response &res);
    bool returnDBsizeCB(gauss_msgs::DB_size::Request &req, gauss_msgs::DB_size::Response &res);

    // Auxilary variables
    int size_plans;
    int size_geofences;
    // Auxilary methods
    bool operationsFromJson(std::string _file_name);

    list<gauss_msgs::Operation> operation_db;
    list<gauss_msgs::Geofence> geofence_db;

    ros::NodeHandle nh_;

    // Subscribers

    // Publisher

    // Timer

    // Server
    ros::ServiceServer read_operation_server_;
    ros::ServiceServer write_operation_server_;
    ros::ServiceServer read_geofences_server_;
    ros::ServiceServer write_geofences_server_;
    ros::ServiceServer read_plan_server_;
    ros::ServiceServer write_plan_server_;
    ros::ServiceServer read_track_server_;
    ros::ServiceServer write_track_server_;
    ros::ServiceServer read_traj_server_;
    ros::ServiceServer write_traj_server_;
    ros::ServiceServer dbsize_server_;
};

// DataBase Constructor
DataBase::DataBase()
{
    // Read parameters


    // Initialization
    size_plans=size_geofences=0;

    // Lee archivo de datos para inicializar databases y actualizar valor de size_plans y size_tracks
    std::string file_name = "OPERATIONS.json";
    operationsFromJson(file_name);

    // Publish

    // Subscribe


    // Server
    read_operation_server_=nh_.advertiseService("/gauss/readOperation",&DataBase::readOperationCB,this);
    write_operation_server_=nh_.advertiseService("/gauss/writeOperation",&DataBase::writeOperationCB,this);
    read_geofences_server_=nh_.advertiseService("/gauss/readGeofences",&DataBase::readGeofenceCB,this);
    write_geofences_server_=nh_.advertiseService("/gauss/writeGeofences",&DataBase::writeGeofenceCB,this);
    read_plan_server_=nh_.advertiseService("/gauss/readFlightPlan",&DataBase::readPlanCB,this);
    write_plan_server_=nh_.advertiseService("/gauss/writeFlightPlan",&DataBase::writePlanCB,this);
    read_track_server_=nh_.advertiseService("/gauss/readTracks",&DataBase::readTrackCB,this);
    write_track_server_=nh_.advertiseService("/gauss/writeTracks",&DataBase::writeTrackCB,this);
    read_traj_server_=nh_.advertiseService("/gauss/readEstimatedTrajectory",&DataBase::readTrajectoryCB,this);
    write_traj_server_=nh_.advertiseService("/gauss/writeEstimatedTrajectory",&DataBase::writeTrajectoryCB,this);
    dbsize_server_=nh_.advertiseService("/gauss/db_size",&DataBase::returnDBsizeCB,this);

    ROS_INFO("Started DBManager node!");
}

bool DataBase::operationsFromJson(std::string _file_name)
{
    std::string pkg_path = ros::package::getPath("db_manager");
    std::string file_path = pkg_path + "/config/" + _file_name;
    std::ifstream i(file_path);
    nlohmann::json jsonDB;
    i >> jsonDB;
    gauss_msgs::WriteOperation json_operation;
    for(const auto& item : jsonDB.at("operations").items()){
        gauss_msgs::Operation operation;
        operation.UAV_id = item.value()["UAV_id"].get<double>();
        operation.autonomy = item.value()["autonomy"].get<double>();
        operation.conop = item.value()["conop"].get<std::string>();
        operation.contingency_volume = item.value()["contingency_volume"].get<double>();
        operation.current_wp = item.value()["current_wp"].get<double>();
        operation.dT = item.value()["dT"].get<double>();
        operation.flight_geometry = item.value()["flight_geometry"].get<double>();
        operation.frame = operation.FRAME_ROTOR; // Check this parameter
        operation.ICAO_address = item.value()["ICAO_address"].get<std::string>();
        operation.priority = item.value()["priority"].get<double>();
        operation.time_horizon = item.value()["time_horizon"].get<double>();
        operation.time_tracked = item.value()["time_tracked"].get<double>();
        gauss_msgs::WaypointList wp_list;
        for(const auto& it : item.value()["flight_plan"].front().items()){
            gauss_msgs::Waypoint wp;
            wp.x = it.value()["x"].get<double>();
            wp.y = it.value()["y"].get<double>();
            wp.z = it.value()["z"].get<double>();
            wp.stamp = ros::Time(it.value()["stamp"].get<double>());
            wp.mandatory = it.value()["mandatory"].get<double>();
            wp_list.waypoints.push_back(wp);
        } 
        operation.flight_plan = wp_list;
        wp_list.waypoints.clear();
        for(const auto& it : item.value()["track"].front().items()){
            gauss_msgs::Waypoint wp;
            wp.x = it.value()["x"].get<double>();
            wp.y = it.value()["y"].get<double>();
            wp.z = it.value()["z"].get<double>();
            wp.stamp = ros::Time(it.value()["stamp"].get<double>());
            wp.mandatory = it.value()["mandatory"].get<double>();
            wp_list.waypoints.push_back(wp);
        } 
        operation.track = wp_list;
        wp_list.waypoints.clear();
        for(const auto& it : item.value()["estimated_trajectory"].front().items()){
            gauss_msgs::Waypoint wp;
            wp.x = it.value()["x"].get<double>();
            wp.y = it.value()["y"].get<double>();
            wp.z = it.value()["z"].get<double>();
            wp.stamp = ros::Time(it.value()["stamp"].get<double>());
            wp.mandatory = it.value()["mandatory"].get<double>();
            wp_list.waypoints.push_back(wp);
        } 
        operation.estimated_trajectory = wp_list;
        json_operation.request.operation.push_back(operation);
        json_operation.request.UAV_ids.push_back(operation.UAV_id);
    }
    writeOperationCB(json_operation.request, json_operation.response);
    if (!json_operation.response.success){
        ROS_ERROR("Error initializing DataBase from JSON!");
        return false;
    } else {
        ROS_INFO_STREAM(json_operation.response.message);
    }

    return true;
}

// Callback

bool DataBase::returnDBsizeCB(gauss_msgs::DB_size::Request &req, gauss_msgs::DB_size::Response &res)
{
    res.geofences=size_geofences;
    res.operations=size_plans;

    res.message="Database sizes";
    res.success=true;
}

bool DataBase::readOperationCB(gauss_msgs::ReadOperation::Request &req, gauss_msgs::ReadOperation::Response &res)
{
    int tam=req.uav_ids.size();

    res.success=true;
    for (int i=0; i<tam; i++)
    {
        ROS_INFO("hola %d", req.uav_ids[i]);
         if (req.uav_ids[i]<size_plans)
         {
             ROS_INFO("hola");
             list<gauss_msgs::Operation>::iterator it = operation_db.begin();
              ROS_INFO("aqui");
             while (it->UAV_id!=req.uav_ids[i])
                 it++;
             //res.operation.at(i)=*it;
             res.operation.push_back(*it);
         }
         else
             res.success=false;
    }
    if (!res.success)
        res.message="An operation was not returned";
    else
        res.message="All requested operations was returned";
}

bool DataBase::writeOperationCB(gauss_msgs::WriteOperation::Request &req, gauss_msgs::WriteOperation::Response &res)
{
    int tam = req.UAV_ids.size();
    ROS_INFO("tam %d",tam);

    res.success=true;
    for (int i=0; i<tam; i++)
    {
        if (req.UAV_ids[i]>=size_plans)
        {
            operation_db.push_back(req.operation[i]);
            size_plans++;
        }
        else
        {
            ROS_INFO("escribiendo");
            list<gauss_msgs::Operation>::iterator it = operation_db.begin();
            while (it->UAV_id!=req.UAV_ids[i])
                it++;
            *it=req.operation[i];
        }
        ROS_INFO("operaciones %d",size_plans);
    }
    res.message="All requested operations was written on the DataBase";
}

bool DataBase::readGeofenceCB(gauss_msgs::ReadGeofences::Request &req, gauss_msgs::ReadGeofences::Response &res)
{
    int tam=req.geofences_ids.size();

    res.success=true;
    for (int i=0; i<tam; i++)
    {
         if (req.geofences_ids[i]<size_geofences)
         {
             list<gauss_msgs::Geofence>::iterator it = geofence_db.begin();
             while (it->id!=req.geofences_ids[i])
                 it++;
             res.geofences.push_back(*it);
         }
         else
             res.success=false;
    }
    if (!res.success)
        res.message="A geofence was not returned";
    else
        res.message="All requested geofences was returned";
}

bool DataBase::writeGeofenceCB(gauss_msgs::WriteGeofences::Request &req, gauss_msgs::WriteGeofences::Response &res)
{
    int tam = req.geofence_ids.size();

    res.success=true;
    for (int i=0; i<tam; i++)
    {
        if (req.geofence_ids[i]>=size_geofences)
        {
            geofence_db.push_back(req.geofences[i]);
            size_geofences++;
        }
        else
        {

            list<gauss_msgs::Geofence>::iterator it = geofence_db.begin();
            while (it->id!=req.geofence_ids[i])
                it++;
            *it=req.geofences[i];
        }
    }
    res.message="All requested geofences was written on the DataBase";
}

bool DataBase::readPlanCB(gauss_msgs::ReadFlightPlan::Request &req, gauss_msgs::ReadFlightPlan::Response &res)
{
    int tam=req.uav_ids.size();

    res.success=true;
    for (int i=0; i<tam; i++)
    {
         if (req.uav_ids[i]<size_plans)
         {
             list<gauss_msgs::Operation>::iterator it = operation_db.begin();
             while (it->UAV_id!=req.uav_ids[i])
                 it++;
             res.plans.push_back(it->flight_plan);
         }
         else
             res.success=false;
    }
    if (!res.success)
        res.message="A flight plan was not returned";
    else
        res.message="All requested flight plans was returned";
}

bool DataBase::writePlanCB(gauss_msgs::WriteFlightPlan::Request &req, gauss_msgs::WriteFlightPlan::Response &res)
{
    int tam = req.UAV_ids.size();

    res.success=true;
    for (int i=0; i<tam; i++)
    {
        if (req.UAV_ids[i]>=size_plans)
        {
            res.success=false;
            res.message="A flight plan was not written";
        }
        else
        {
            list<gauss_msgs::Operation>::iterator it = operation_db.begin();
            while (it->UAV_id!=req.UAV_ids[i])
                it++;
            it->flight_plan=req.plans[i];
        }
    }
    res.message="All requested flight plans was written on the DataBase";
}

bool DataBase::readTrackCB(gauss_msgs::ReadTracks::Request &req, gauss_msgs::ReadTracks::Response &res)
{
    int tam=req.UAV_ids.size();

    res.success=true;
    for (int i=0; i<tam; i++)
    {
         if (req.UAV_ids[i]<size_plans)
         {
             list<gauss_msgs::Operation>::iterator it = operation_db.begin();
             while (it->UAV_id!=req.UAV_ids[i])
                 it++;
             res.tracks.push_back(it->track);
         }
         else
             res.success=false;
    }
    if (!res.success)
        res.message="A Track was not returned";
    else
        res.message="All requested tracks was returned";
}

bool DataBase::writeTrackCB(gauss_msgs::WriteTracks::Request &req, gauss_msgs::WriteTracks::Response &res)
{
    int tam = req.UAV_ids.size();

    res.success=true;
    for (int i=0; i<tam; i++)
    {
        if (req.UAV_ids[i]>=size_plans)
        {
            res.success=false;
            res.message="A track was not written";
        }
        else
        {
            list<gauss_msgs::Operation>::iterator it = operation_db.begin();
            while (it->UAV_id!=req.UAV_ids[i])
                it++;
            it->track=req.tracks[i];
        }
    }
    res.message="All requested tracks was written on the DataBase";
}

bool DataBase::readTrajectoryCB(gauss_msgs::ReadTraj::Request &req, gauss_msgs::ReadTraj::Response &res)
{
    int tam=req.UAV_ids.size();

    res.success=true;
    for (int i=0; i<tam; i++)
    {
         if (req.UAV_ids[i]<size_plans)
         {
             list<gauss_msgs::Operation>::iterator it = operation_db.begin();
             while (it->UAV_id!=req.UAV_ids[i])
                 it++;
             res.tracks.push_back(it->estimated_trajectory);
         }
         else
             res.success=false;
    }
    if (!res.success)
        res.message="A trajectory was not returned";
    else
        res.message="All requested trajectories was returned";
}

bool DataBase::writeTrajectoryCB(gauss_msgs::WriteTraj::Request &req, gauss_msgs::WriteTraj::Response &res)
{
    int tam = req.UAV_ids.size();

    res.success=true;
    for (int i=0; i<tam; i++)
    {
        if (req.UAV_ids[i]>=size_plans)
        {
            res.success=false;
            res.message="A trajectory was not written";
        }
        else
        {
            list<gauss_msgs::Operation>::iterator it = operation_db.begin();
            while (it->UAV_id!=req.UAV_ids[i])
                it++;
            it->estimated_trajectory=req.tracks[i];
        }
    }
    res.message="All requested trajectory was written on the DataBase";
}



// MAIN function
int main(int argc, char *argv[])
{
    ros::init(argc,argv,"DBmanager");

    // Create a DataBase object
    DataBase *database = new DataBase();

    ros::spin();
}
