#include <ros/ros.h>
#include <ros/package.h>
#include <gauss_msgs/ReadIcao.h>
#include <gauss_msgs/Geofence.h>
#include <gauss_msgs/ReadGeofences.h>
#include <gauss_msgs/WriteGeofences.h>
#include <gauss_msgs/Operation.h>
#include <gauss_msgs/ReadOperation.h>
#include <gauss_msgs/WriteOperation.h>
#include <gauss_msgs/WriteTracking.h>
#include <gauss_msgs/WritePlans.h>
#include <gauss_msgs/DB_size.h>
#include <gauss_msgs/Polygon.h>
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
    bool readIcaoCB(gauss_msgs::ReadIcao::Request &req, gauss_msgs::ReadIcao::Response &res);
    bool writeOperationCB(gauss_msgs::WriteOperation::Request &req, gauss_msgs::WriteOperation::Response &res);
    bool readGeofenceCB(gauss_msgs::ReadGeofences::Request &req, gauss_msgs::ReadGeofences::Response &res);
    bool writeGeofenceCB(gauss_msgs::WriteGeofences::Request &req, gauss_msgs::WriteGeofences::Response &res);
    bool returnDBsizeCB(gauss_msgs::DB_size::Request &req, gauss_msgs::DB_size::Response &res);
    bool writeTrackingCB(gauss_msgs::WriteTracking::Request &req, gauss_msgs::WriteTracking::Response &res);
    bool writePlansCB(gauss_msgs::WritePlans::Request &req, gauss_msgs::WritePlans::Response &res);

    // Auxilary variables
    int size_plans;
    int size_geofences;
    // Auxilary methods
    bool operationsFromJson(std::string _file_name);
    bool geofencesFromJson(std::string _file_name);
    bool checkNewFlightPlan(const gauss_msgs::WaypointList &_pre_flight_plan, const gauss_msgs::WaypointList &_flight_plan);

    list<gauss_msgs::Operation> operation_db;
    list<gauss_msgs::Geofence> geofence_db;

    ros::NodeHandle nh_, pnh_;

    // Subscribers

    // Publisher

    // Timer

    // Server
    ros::ServiceServer read_operation_server_, write_operation_server_, read_icao_server_, read_geofences_server_, write_geofences_server_, dbsize_server_, write_tracking_server_, write_plan_server_;
};

// DataBase Constructor
DataBase::DataBase() : nh_(), pnh_("~")
{
    // Read parameters
    std::string operations_name = "OPERATIONS";
    std::string geofences_name = "UTM_GEOFENCE_CREATION";
    pnh_.getParam("operations_json", operations_name);
    pnh_.getParam("geofences_json",  geofences_name);
    
    // Initialization
    size_plans=size_geofences=0;

    // Lee archivo de datos para inicializar databases y actualizar valor de size_plans y size_tracks
    operationsFromJson(operations_name + ".json");
    geofencesFromJson(geofences_name + ".json");

    // Publish

    // Subscribe


    // Server
    read_operation_server_=nh_.advertiseService("/gauss/read_operation",&DataBase::readOperationCB,this);
    write_operation_server_=nh_.advertiseService("/gauss/write_operation",&DataBase::writeOperationCB,this);
    read_icao_server_=nh_.advertiseService("/gauss/read_icao",&DataBase::readIcaoCB,this);
    read_geofences_server_=nh_.advertiseService("/gauss/read_geofences",&DataBase::readGeofenceCB,this);
    write_geofences_server_=nh_.advertiseService("/gauss/write_geofences",&DataBase::writeGeofenceCB,this);
    dbsize_server_=nh_.advertiseService("/gauss/db_size",&DataBase::returnDBsizeCB,this);
    write_tracking_server_=nh_.advertiseService("/gauss/write_tracking",&DataBase::writeTrackingCB,this);
    write_plan_server_=nh_.advertiseService("/gauss/write_plans",&DataBase::writePlansCB,this);

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
        operation.uav_id = item.value()["uav_id"].get<double>();
        operation.autonomy = item.value()["autonomy"].get<double>();
        operation.conop = item.value()["conop"].get<std::string>();
        operation.operational_volume = item.value()["operational_volume"].get<double>();
        operation.current_wp = item.value()["current_wp"].get<double>();
        operation.dT = item.value()["dT"].get<double>();
        operation.flight_geometry = item.value()["flight_geometry"].get<double>();
        operation.frame = operation.FRAME_ROTOR; // Check this parameter
        operation.icao_address = item.value()["icao_address"].get<std::string>();
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
        wp_list.waypoints.clear();
        for(const auto& it : item.value()["landing_spots"].front().items()){
            gauss_msgs::Waypoint wp;
            wp.x = it.value()["x"].get<double>();
            wp.y = it.value()["y"].get<double>();
            wp.z = it.value()["z"].get<double>();
            wp.stamp = ros::Time(it.value()["stamp"].get<double>());
            wp.mandatory = it.value()["mandatory"].get<double>();
            wp_list.waypoints.push_back(wp);
        } 
        operation.landing_spots = wp_list;
        json_operation.request.operation.push_back(operation);
        json_operation.request.uav_ids.push_back(operation.uav_id);
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

bool DataBase::geofencesFromJson(std::string _file_name)
{
    std::string pkg_path = ros::package::getPath("db_manager");
    std::string file_path = pkg_path + "/config/" + _file_name;
    std::ifstream i(file_path);
    nlohmann::json jsonDB;
    i >> jsonDB;
    gauss_msgs::WriteGeofences json_geofence;
    for(const auto& item : jsonDB.at("geofences").items()){
        gauss_msgs::Geofence geofence;
        geofence.id = item.value()["id"].get<double>();
        geofence.static_geofence = item.value()["static_geofence"].get<bool>();
        geofence.cylinder_shape = item.value()["cylinder_shape"].get<bool>();
        geofence.min_altitude = item.value()["min_altitude"].get<double>();
        geofence.max_altitude = item.value()["max_altitude"].get<double>();
        geofence.start_time = ros::Time(item.value()["start_time"].get<double>());
        geofence.end_time = ros::Time(item.value()["end_time"].get<double>());
        geofence.circle.x_center = item.value()["circle"]["x_center"].get<double>();
        geofence.circle.y_center = item.value()["circle"]["y_center"].get<double>();
        geofence.circle.radius =   item.value()["circle"]["radius"].get<double>();
        for(const auto& it : item.value()["polygon"].front().items()){
            geofence.polygon.x.push_back(it.value()["x"].get<double>());
            geofence.polygon.y.push_back(it.value()["y"].get<double>());
        }
        json_geofence.request.geofences.push_back(geofence);
        json_geofence.request.geofence_ids.push_back(geofence.id);
    }
    writeGeofenceCB(json_geofence.request, json_geofence.response);
    if (!json_geofence.response.success){
        ROS_ERROR("Error initializing DataBase from JSON!");
        return false;
    } else {
        ROS_INFO_STREAM(json_geofence.response.message);
    }

    return true;
}

bool DataBase::checkNewFlightPlan(const gauss_msgs::WaypointList &_pre_flight_plan, const gauss_msgs::WaypointList &_flight_plan){
    if (_pre_flight_plan.waypoints.size() != _flight_plan.waypoints.size()){
        return true;
    } else {
        for (int i = 0; i < _pre_flight_plan.waypoints.size(); i++){
            if (_pre_flight_plan.waypoints.at(i).x != _flight_plan.waypoints.at(i).x ||
                _pre_flight_plan.waypoints.at(i).y != _flight_plan.waypoints.at(i).y ||
                _pre_flight_plan.waypoints.at(i).z != _flight_plan.waypoints.at(i).z ||
                _pre_flight_plan.waypoints.at(i).stamp != _flight_plan.waypoints.at(i).stamp ||
                _pre_flight_plan.waypoints.at(i).mandatory != _flight_plan.waypoints.at(i).mandatory){
                    return true;
                }
        }
        return false;
    }
}


// Callback

bool DataBase::returnDBsizeCB(gauss_msgs::DB_size::Request &req, gauss_msgs::DB_size::Response &res)
{
    res.geofences=size_geofences;
    res.operations=size_plans;

    res.message="Database sizes";
    res.success=true;
    return true;
}

bool DataBase::readIcaoCB(gauss_msgs::ReadIcao::Request &req, gauss_msgs::ReadIcao::Response &res)
{
    for (list<gauss_msgs::Operation>::iterator it = operation_db.begin(); it != operation_db.end(); it++){
        res.uav_id.push_back(it->uav_id);
        res.icao_address.push_back(it->icao_address);
    }
    res.message = "All ICAO address were returned";
    res.success = true;
    
    return true;
}

bool DataBase::readOperationCB(gauss_msgs::ReadOperation::Request &req, gauss_msgs::ReadOperation::Response &res)
{
    std::string invalid_ids;
    if (req.uav_ids.size() <= operation_db.size()){
        for (int i = 0; i < req.uav_ids.size(); i++){
            res.success = false;
            for(list<gauss_msgs::Operation>::iterator it = operation_db.begin(); it != operation_db.end(); it++){
                if (it->uav_id == req.uav_ids[i]){
                    res.success = true;
                    res.operation.push_back(*it);
                    res.message = "All requested operations were returned";
                    break;
                }
            }
            if(!res.success) invalid_ids = invalid_ids + " " + std::to_string(req.uav_ids[i]); 
        }
        if (!invalid_ids.empty()){
            res.success = false;
            res.operation.clear();
            res.message = "Data base does not contain requested ids:" + invalid_ids;
        }
    } else {
        res.success = false;
        res.message = "Request ids size can not be larger than operation_db size!";
    }
    return true;
}

bool DataBase::writeOperationCB(gauss_msgs::WriteOperation::Request &req, gauss_msgs::WriteOperation::Response &res)
{
    for (int i = 0; i < req.uav_ids.size(); i++)
    {
        res.success=false;
        if (operation_db.empty()){
            req.operation[i].flight_plan_mod_t = ros::Time::now().toSec();
            operation_db.push_back(req.operation[i]);
            res.success = true;
        } else {
            for(list<gauss_msgs::Operation>::iterator it = operation_db.begin(); it != operation_db.end(); it++){
                if (it->uav_id == req.uav_ids[i]){
                    if (checkNewFlightPlan(it->flight_plan, req.operation[i].flight_plan)){
                        req.operation[i].flight_plan_mod_t = ros::Time::now().toSec();
                    }
                    *it = req.operation[i];
                    res.success = true;
                    break;
                }
            }
            if (!res.success){
                req.operation[i].flight_plan_mod_t = ros::Time::now().toSec();
                operation_db.push_back(req.operation[i]);
                res.success = true;
            }
        }
    }
    size_plans = operation_db.size();
    res.message="All requested operations were written on the DataBase";
    return true;
}

bool DataBase::readGeofenceCB(gauss_msgs::ReadGeofences::Request &req, gauss_msgs::ReadGeofences::Response &res)
{
    std::string invalid_ids;
    if (req.geofences_ids.size() <= geofence_db.size()){
        for (int i = 0; i < req.geofences_ids.size(); i++){
            res.success = false;
            for(list<gauss_msgs::Geofence>::iterator it = geofence_db.begin(); it != geofence_db.end(); it++){
                if (it->id == req.geofences_ids[i]){
                    res.success = true;
                    res.geofences.push_back(*it);
                    res.message = "All requested geofences were returned";
                    break;
                }
            }
            if(!res.success) invalid_ids = invalid_ids + " " + std::to_string(req.geofences_ids[i]);
        }
        if (!invalid_ids.empty()){
            res.success = false;
            res.geofences.clear();
            res.message = "Data base does not contain requested ids:" + invalid_ids;
        }
    } else {
        res.success = false;
        res.message = "Request ids size can not be larger than geofence_db size!";
    }
    return true;
}

bool DataBase::writeGeofenceCB(gauss_msgs::WriteGeofences::Request &req, gauss_msgs::WriteGeofences::Response &res)
{
    for (int i = 0; i < req.geofences.size(); i++)
    {
        res.success=false;
        if (geofence_db.empty()){
            geofence_db.push_back(req.geofences[i]);
            res.success = true;
        } else {
            for(list<gauss_msgs::Geofence>::iterator it = geofence_db.begin(); it != geofence_db.end(); it++){
                if (it->id == req.geofence_ids[i]){
                    *it = req.geofences[i];
                    res.success = true;
                    break;
                }
            }
            if (!res.success){
                geofence_db.push_back(req.geofences[i]);
                res.success = true;
            }
        }
    }
    size_geofences = geofence_db.size();
    res.message="All requested geofences were written on the DataBase";
    return true;
}

bool DataBase::writeTrackingCB(gauss_msgs::WriteTracking::Request &req, gauss_msgs::WriteTracking::Response &res)
{
    std::string message = "";
    if (operation_db.empty()){
        res.success = false;
        message = "No operations saved on the DataBase. Can not update requested tracking fields.";
    } else {
        std::vector<int> not_found_ids;
        for (int i = 0; i < req.uav_ids.size(); i++)
        {
            res.success = false;
            for(list<gauss_msgs::Operation>::iterator it = operation_db.begin(); it != operation_db.end(); it++){
                if (it->uav_id == req.uav_ids[i]){
                    it->track = req.tracks[i];
                    it->current_wp = req.current_wps[i];
                    it->time_tracked = req.times_tracked[i];
                    it->estimated_trajectory = req.estimated_trajectories[i];
                    break;
                }
                not_found_ids.push_back(i);
            }
        }
        if (!not_found_ids.empty()){
            message = "Data base does not contain requested ids ["; 
            for (int idx = 0; idx < not_found_ids.size(); idx++) message = message + " " + std::to_string(not_found_ids.at(idx));
            message = message + "].";
            res.success = false;
        } else {
            message = "All requested tracking fields were written on the DataBase.";
            res.success = true;
        }
    }
    res.message = message;
    return true;
}

bool DataBase::writePlansCB(gauss_msgs::WritePlans::Request &req, gauss_msgs::WritePlans::Response &res)
{
    std::string message = "";
    if (operation_db.empty()){
        res.success = false;
        message = "No operations saved on the DataBase. Can not update requested flight plans.";
    } else {
        std::vector<int> not_found_ids;
        for (int i = 0; i < req.uav_ids.size(); i++)
        {
            res.success = false;
            for(list<gauss_msgs::Operation>::iterator it = operation_db.begin(); it != operation_db.end(); it++){
                if (it->uav_id == req.uav_ids[i]){
                    it->flight_plan = req.flight_plans[i];
                    break;
                }
                not_found_ids.push_back(i);
            }
        }
        if (!not_found_ids.empty()){
            message = "Data base does not contain requested ids ["; 
            for (int idx = 0; idx < not_found_ids.size(); idx++) message = message + " " + std::to_string(not_found_ids.at(idx));
            message = message + "].";
            res.success = false;
        } else {
            message = "All requested flight plans were written on the DataBase.";
            res.success = true;
        }
    }
    res.message = message;
    return true;
}


// MAIN function
int main(int argc, char *argv[])
{
    ros::init(argc,argv,"DBmanager");

    // Create a DataBase object
    DataBase *database = new DataBase();

    ros::spin();
}
