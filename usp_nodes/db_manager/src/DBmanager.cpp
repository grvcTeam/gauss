#include <gauss_msgs/DB_size.h>
#include <gauss_msgs/Geofence.h>
#include <gauss_msgs/Operation.h>
#include <gauss_msgs/Polygon.h>
#include <gauss_msgs/ReadGeofences.h>
#include <gauss_msgs/ReadIcao.h>
#include <gauss_msgs/ReadOperation.h>
#include <gauss_msgs/WriteGeofences.h>
#include <gauss_msgs/WriteOperation.h>
#include <gauss_msgs/WritePlans.h>
#include <gauss_msgs/WriteTracking.h>
#include <ros/package.h>
#include <ros/ros.h>

#include <db_manager/json.hpp>
#include <fstream>

using namespace std;

// Class definition
class DataBase {
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
    ros::Time init_time_;
    // Auxilary methods
    bool jsonExists(std::string _file_name);
    bool operationsFromJson(std::string _file_name);
    bool geofencesFromJson(std::string _file_name);
    bool checkNewFlightPlan(const gauss_msgs::WaypointList &_pre_flight_plan, const gauss_msgs::WaypointList &_flight_plan);

    map<int, gauss_msgs::Operation> saved_operations;
    map<int, gauss_msgs::Geofence> saved_geofences;

    ros::NodeHandle nh_, pnh_;

    // Subscribers

    // Publisher

    // Timer

    // Server
    ros::ServiceServer read_operation_server_, write_operation_server_, read_icao_server_, read_geofences_server_, write_geofences_server_, dbsize_server_, write_tracking_server_, write_plan_server_;
};

// DataBase Constructor
DataBase::DataBase() : nh_(), pnh_("~") {
    // Read (public) parameters
    double time_param = 0.0;
    std::string operations_name = "loss_operations";
    std::string geofences_name = "no_geofences";
    nh_.getParam("init_time", time_param);
    nh_.getParam("operations_json", operations_name);
    nh_.getParam("geofences_json", geofences_name);
    std::string pkg_path = ros::package::getPath("db_manager");
    std::string file_path = pkg_path + "/config/";
    bool ok_json_operations = jsonExists(file_path + operations_name + ".json");
    bool ok_json_geofences = jsonExists(file_path + geofences_name + ".json");
    if (ok_json_geofences && ok_json_operations) {
        // Initialization
        size_plans = size_geofences = 0;
        if (time_param == 0.0){
            init_time_ = ros::Time::now();
        } else {
            init_time_ = ros::Time(time_param);
        }
        // Lee archivo de datos para inicializar databases y actualizar valor de size_plans y size_tracks
        ROS_WARN_STREAM(file_path + operations_name + ".json");
        ROS_WARN_STREAM(file_path + geofences_name + ".json");
        operationsFromJson(file_path + operations_name + ".json");
        geofencesFromJson(file_path + geofences_name + ".json");
        // Server
        read_operation_server_ = nh_.advertiseService("/gauss/read_operation", &DataBase::readOperationCB, this);
        write_operation_server_ = nh_.advertiseService("/gauss/write_operation", &DataBase::writeOperationCB, this);
        read_icao_server_ = nh_.advertiseService("/gauss/read_icao", &DataBase::readIcaoCB, this);
        read_geofences_server_ = nh_.advertiseService("/gauss/read_geofences", &DataBase::readGeofenceCB, this);
        write_geofences_server_ = nh_.advertiseService("/gauss/write_geofences", &DataBase::writeGeofenceCB, this);
        dbsize_server_ = nh_.advertiseService("/gauss/db_size", &DataBase::returnDBsizeCB, this);
        write_tracking_server_ = nh_.advertiseService("/gauss/write_tracking", &DataBase::writeTrackingCB, this);
        write_plan_server_ = nh_.advertiseService("/gauss/write_plans", &DataBase::writePlansCB, this);
    } else {
        if (!ok_json_geofences) ROS_ERROR("Geofences JSON does not exist!");
        if (!ok_json_operations) ROS_ERROR("Operations JSON does not exist!");
    }
    ROS_INFO("[DB] Started DBManager node!");
}

bool DataBase::jsonExists(std::string _file_name) {
    std::ifstream i(_file_name);
    return i.good();
}

bool DataBase::operationsFromJson(std::string _file_name) {
    std::ifstream i(_file_name);
    nlohmann::json jsonDB;
    i >> jsonDB;
    gauss_msgs::WriteOperation json_operation;
    if (jsonDB.at("operations").size() == 0) {
        ROS_WARN("No operations on initial JSON!");
    } else {
        for (const auto &item : jsonDB.at("operations").items()) {
            gauss_msgs::Operation operation;
            operation.uav_id = item.value()["uav_id"].get<double>();
            operation.autonomy = item.value()["autonomy"].get<double>();
            operation.conop = item.value()["conop"].get<std::string>();
            operation.is_started = item.value()["is_started"].get<bool>();
            if (item.value()["current_wp"].get<double>() == 0) {
                operation.current_wp = 1;
            } else {
                operation.current_wp = item.value()["current_wp"].get<double>();
            }
            // operation.dT = item.value()["dT"].get<double>();
            operation.operational_volume = item.value()["operational_volume"].get<double>();
            if (item.value()["operational_volume"].get<double>() < item.value()["flight_geometry"].get<double>()) {
                operation.flight_geometry = item.value()["operational_volume"].get<double>() * 0.8;
            } else {
                operation.flight_geometry = item.value()["flight_geometry"].get<double>();
            }
            operation.frame = operation.FRAME_ROTOR;  // Check this parameter
            operation.icao_address = item.value()["icao_address"].get<std::string>();
            operation.priority = item.value()["priority"].get<double>();
            // operation.time_horizon = item.value()["time_horizon"].get<double>();
            operation.time_tracked = item.value()["time_tracked"].get<double>();
            gauss_msgs::WaypointList wp_list;
            if (item.value()["flight_plan"].front().size() == 0) {
                ROS_WARN("Operation %d has empty flight plan on initial JSON!", item.value()["uav_id"].get<int>());
            } else {
                for (const auto &it : item.value()["flight_plan"].front().items()) {
                    gauss_msgs::Waypoint wp;
                    wp.x = it.value()["x"].get<double>();
                    wp.y = it.value()["y"].get<double>();
                    wp.z = it.value()["z"].get<double>();
                    wp.stamp = ros::Time(init_time_.toSec() + it.value()["stamp"].get<double>());
                    wp.mandatory = it.value()["mandatory"].get<double>();
                    wp_list.waypoints.push_back(wp);
                }
                operation.flight_plan = wp_list;
            }
            operation.track.waypoints.push_back(operation.flight_plan.waypoints.front());
            wp_list.waypoints.clear();

            if (item.value()["landing_spots"].front().size() == 0) {
                ROS_WARN("Operation %d has empty landing spots on initial JSON!", item.value()["uav_id"].get<int>());
            } else {
                for (const auto &it : item.value()["landing_spots"].front().items()) {
                    gauss_msgs::Waypoint wp;
                    wp.x = it.value()["x"].get<double>();
                    wp.y = it.value()["y"].get<double>();
                    wp.z = it.value()["z"].get<double>();
                    wp.stamp = ros::Time(it.value()["stamp"].get<double>());
                    wp.mandatory = it.value()["mandatory"].get<double>();
                    wp_list.waypoints.push_back(wp);
                }
                operation.landing_spots = wp_list;
            }
            json_operation.request.operation.push_back(operation);
            json_operation.request.uav_ids.push_back(operation.uav_id);
        }
    }
    if (json_operation.request.uav_ids.size() > 0) {
        writeOperationCB(json_operation.request, json_operation.response);
        if (!json_operation.response.success) {
            ROS_ERROR("Error initializing DataBase from JSON!");
            return false;
        } else {
            ROS_INFO_STREAM(json_operation.response.message);
        }
    }
    return true;
}

bool DataBase::geofencesFromJson(std::string _file_name) {
    std::ifstream i(_file_name);
    nlohmann::json jsonDB;
    i >> jsonDB;
    gauss_msgs::WriteGeofences json_geofence;
    if (jsonDB.at("geofences").size() == 0) {
        ROS_WARN("No geofences on initial JSON!");
    } else {
        for (const auto &item : jsonDB.at("geofences").items()) {
            gauss_msgs::Geofence geofence;
            geofence.id = item.value()["id"].get<double>();
            geofence.static_geofence = item.value()["static_geofence"].get<bool>();
            geofence.cylinder_shape = item.value()["cylinder_shape"].get<bool>();
            geofence.min_altitude = item.value()["min_altitude"].get<double>();
            geofence.max_altitude = item.value()["max_altitude"].get<double>();
            geofence.start_time = ros::Time(init_time_.toSec() + item.value()["start_time"].get<double>());
            geofence.end_time = ros::Time(init_time_.toSec() + item.value()["end_time"].get<double>());
            geofence.circle.x_center = item.value()["circle"]["x_center"].get<double>();
            geofence.circle.y_center = item.value()["circle"]["y_center"].get<double>();
            geofence.circle.radius = item.value()["circle"]["radius"].get<double>();
            if (item.value()["polygon"].front().size() == 0) {
                ROS_WARN("Geofence %d has empty polygon on initial JSON!", item.value()["id"].get<int>());
            } else {
                for (const auto &it : item.value()["polygon"].front().items()) {
                    geofence.polygon.x.push_back(it.value()["x"].get<double>());
                    geofence.polygon.y.push_back(it.value()["y"].get<double>());
                }
            }
            json_geofence.request.geofences.push_back(geofence);
            json_geofence.request.geofence_ids.push_back(geofence.id);
        }
    }
    if (json_geofence.request.geofence_ids.size() > 0) {
        writeGeofenceCB(json_geofence.request, json_geofence.response);
        if (!json_geofence.response.success) {
            ROS_ERROR("Error initializing DataBase from JSON!");
            return false;
        } else {
            ROS_INFO_STREAM(json_geofence.response.message);
        }
    }
    return true;
}

bool DataBase::checkNewFlightPlan(const gauss_msgs::WaypointList &_pre_flight_plan, const gauss_msgs::WaypointList &_flight_plan) {
    if (_pre_flight_plan.waypoints.size() != _flight_plan.waypoints.size()) {
        return true;
    } else {
        for (int i = 0; i < _pre_flight_plan.waypoints.size(); i++) {
            if (_pre_flight_plan.waypoints.at(i).x != _flight_plan.waypoints.at(i).x ||
                _pre_flight_plan.waypoints.at(i).y != _flight_plan.waypoints.at(i).y ||
                _pre_flight_plan.waypoints.at(i).z != _flight_plan.waypoints.at(i).z ||
                _pre_flight_plan.waypoints.at(i).stamp != _flight_plan.waypoints.at(i).stamp ||
                _pre_flight_plan.waypoints.at(i).mandatory != _flight_plan.waypoints.at(i).mandatory) {
                return true;
            }
        }
        return false;
    }
}

// Callback

bool DataBase::returnDBsizeCB(gauss_msgs::DB_size::Request &req, gauss_msgs::DB_size::Response &res) {
    res.geofences = size_geofences;
    res.operations = size_plans;

    res.message = "Database sizes";
    res.success = true;
    return true;
}

bool DataBase::readIcaoCB(gauss_msgs::ReadIcao::Request &req, gauss_msgs::ReadIcao::Response &res) {
    for (map<int, gauss_msgs::Operation>::const_iterator it = saved_operations.begin(); it != saved_operations.end(); it++) {
        res.uav_id.push_back(it->second.uav_id);
        res.icao_address.push_back(it->second.icao_address);
    }
    for (map<int, gauss_msgs::Geofence>::const_iterator it = saved_geofences.begin(); it != saved_geofences.end(); it++) {
        res.geofence_id.push_back(it->second.id);
    }
    res.message = "All ICAO address were returned";
    res.success = true;

    return true;
}

bool DataBase::readOperationCB(gauss_msgs::ReadOperation::Request &req, gauss_msgs::ReadOperation::Response &res) {
    std::string invalid_ids;
    if (req.uav_ids.size() <= saved_operations.size()) {
        for (int i = 0; i < req.uav_ids.size(); i++) {
            map<int, gauss_msgs::Operation>::iterator it = saved_operations.find(req.uav_ids[i]);
            if (it != saved_operations.end()) {
                res.operation.push_back(it->second);
            } else {
                invalid_ids = invalid_ids + " " + std::to_string(req.uav_ids[i]);
            }
        }
        if (!invalid_ids.empty()) {
            res.success = false;
            // res.operation.clear();
            res.message = "Data base does not contain requested operation ids:" + invalid_ids;
        } else {
            res.success = true;
            res.message = "All requested operations were returned";
        }
    } else {
        res.success = false;
        res.message = "Request ids size can not be larger than saved_operations size!";
    }
    return true;
}

bool DataBase::writeOperationCB(gauss_msgs::WriteOperation::Request &req, gauss_msgs::WriteOperation::Response &res) {
    for (int i = 0; i < req.uav_ids.size(); i++) {
        map<int, gauss_msgs::Operation>::iterator it = saved_operations.find(req.uav_ids[i]);
        if (it != saved_operations.end()) {
            if (checkNewFlightPlan(it->second.flight_plan, req.operation[i].flight_plan)) {
                req.operation[i].flight_plan_mod_t = ros::Time::now().toSec();
            }
            it->second = req.operation[i];
        } else {
            req.operation[i].flight_plan_mod_t = ros::Time::now().toSec();
            if (req.operation[i].current_wp == 0) req.operation[i].current_wp = 1;
            if (req.operation[i].track.waypoints.size() == 0) req.operation[i].track.waypoints.push_back(req.operation[i].flight_plan.waypoints.front());
            if (req.operation[i].operational_volume < req.operation[i].flight_geometry) req.operation[i].flight_geometry = req.operation[i].operational_volume * 0.8;
            for (int j = 0; j < std::min((int)req.operation[i].flight_plan.waypoints.size(), 18); j++) req.operation[i].estimated_trajectory.waypoints.push_back(req.operation[i].flight_plan.waypoints.at(j));
            saved_operations.insert(pair<int, gauss_msgs::Operation>(req.operation[i].uav_id, req.operation[i]));
            if (req.operation[i].flight_plan.waypoints.size() == 0) ROS_WARN("Operation %d has empty flight plan!", (int)req.operation[i].uav_id);
            if (req.operation[i].landing_spots.waypoints.size() == 0) ROS_WARN("Operation %d has empty landing spot!", (int)req.operation[i].uav_id);
        }
    }
    res.success = true;
    size_plans = saved_operations.size();
    res.message = "All requested operations were written on the DataBase";
    return true;
}

bool DataBase::readGeofenceCB(gauss_msgs::ReadGeofences::Request &req, gauss_msgs::ReadGeofences::Response &res) {
    std::string invalid_ids;
    if (req.geofences_ids.size() <= saved_geofences.size()) {
        for (int i = 0; i < req.geofences_ids.size(); i++) {
            map<int, gauss_msgs::Geofence>::iterator it = saved_geofences.find(req.geofences_ids[i]);
            if (it != saved_geofences.end()) {
                res.geofences.push_back(it->second);
            } else {
                invalid_ids = invalid_ids + " " + std::to_string(req.geofences_ids[i]);
            }
        }
        if (!invalid_ids.empty()) {
            res.success = false;
            // res.geofences.clear();
            res.message = "Data base does not contain requested geofence ids:" + invalid_ids;
        } else {
            res.success = true;
            res.message = "All requested geofences were returned";
        }
    } else {
        res.success = false;
        res.message = "Request ids size can not be larger than saved_geofences size!";
    }
    return true;
}

bool DataBase::writeGeofenceCB(gauss_msgs::WriteGeofences::Request &req, gauss_msgs::WriteGeofences::Response &res) {
    for (int i = 0; i < req.geofence_ids.size(); i++) {
        if (saved_geofences.empty()) {
            saved_geofences.insert(pair<int, gauss_msgs::Geofence>(req.geofences[i].id, req.geofences[i]));
        } else {
            map<int, gauss_msgs::Geofence>::iterator it = saved_geofences.find(req.geofence_ids[i]);
            if (it != saved_geofences.end()) {
                it->second = req.geofences[i];
            } else {
                if (req.geofences[i].cylinder_shape) {
                    if (req.geofences[i].circle.radius == 0) ROS_WARN("Geofence %d has no radius!", req.geofences[i].id);
                } else {
                    if (req.geofences[i].polygon.x.size() == 0 || req.geofences[i].polygon.y.size() == 0 ) ROS_WARN("Geofence %d has empty polygon!", req.geofences[i].id);
                }
                saved_geofences.insert(pair<int, gauss_msgs::Geofence>(req.geofences[i].id, req.geofences[i]));
            }
        }
    }
    res.success = true;
    size_plans = saved_geofences.size();
    res.message = "All requested geofences were written on the DataBase";
    return true;
}

bool DataBase::writeTrackingCB(gauss_msgs::WriteTracking::Request &req, gauss_msgs::WriteTracking::Response &res) {
    std::string message = "";
    if (saved_operations.empty()) {
        res.success = false;
        message = "No operations saved on the DataBase. Can not update requested tracking fields.";
    } else {
        std::vector<int> not_found_ids;
        for (int i = 0; i < req.uav_ids.size(); i++) {
            map<int, gauss_msgs::Operation>::iterator it = saved_operations.find(req.uav_ids[i]);
            if (it != saved_operations.end()) {
                it->second.track = req.tracks[i];
                it->second.current_wp = req.current_wps[i];
                it->second.time_tracked = req.times_tracked[i];
                it->second.estimated_trajectory = req.estimated_trajectories[i];
                it->second.flight_plan_updated = req.flight_plans_updated[i];
                it->second.is_started = req.is_started[i];
            } else {
                not_found_ids.push_back(i);
            }
        }
        if (!not_found_ids.empty()) {
            message = "Data base does not contain requested operation (tracking) ids [";
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

bool DataBase::writePlansCB(gauss_msgs::WritePlans::Request &req, gauss_msgs::WritePlans::Response &res) {
    std::string message = "";
    if (saved_operations.empty()) {
        res.success = false;
        message = "No operations saved on the DataBase. Can not update requested flight plans.";
    } else {
        std::vector<int> not_found_ids;
        for (int i = 0; i < req.uav_ids.size(); i++) {
            map<int, gauss_msgs::Operation>::iterator it = saved_operations.find(req.uav_ids[i]);
            if (it != saved_operations.end()) {
                it->second.flight_plan = req.flight_plans[i];
                it->second.flight_plan_mod_t = ros::Time::now().toSec();
            } else {
                not_found_ids.push_back(i);
            }
        }
        if (!not_found_ids.empty()) {
            message = "Data base does not contain requested flight plan ids [";
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
int main(int argc, char *argv[]) {
    ros::init(argc, argv, "DBmanager");

    // Create a DataBase object
    DataBase *database = new DataBase();

    ros::spin();
}
