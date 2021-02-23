#include <stdio.h>
#include <ros/ros.h>
#include <cstring>
#include <iostream>
#include <fstream>
#include <vector>
#include <jsoncpp/json/json.h>
#include <pugixml.hpp>

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

#define NOMINAL_SPEED 2.0 // meters per second

inline double distanceBetweenWaypoints(double &x1, double &y1, double &z1, double &x2, double &y2, double &z2);

inline double distanceBetweenWaypoints(double &x1, double &y1, double &z1, double &x2, double &y2, double &z2)
{
    return sqrt(pow(x1-x2,2)+pow(y1-y2,2)+pow(z1-z2,2));
}

// This program reads a kml file with multiple operations, and writes them into a json file with the format expected by db_manager

int main(int argc, char** argv)
{
    if(argc != 5)
    {
        std::cout << "Usage: multiple_flight_plans_kml_to_db_json origin_latitude origin_longitude KML_INPUT_FILE JSON_OUTPUT_FILE \n"; 
        return 1;
    }
    
    uint32_t icao = 123456;
    uint32_t priority = 1;
    std::string conop;
    uint32_t uav_id = 0;

    double lat0,lon0;
    lat0 = atof(argv[1]);
    lon0 = atof(argv[2]);
    GeographicLib::Geocentric earth(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f());
    GeographicLib::LocalCartesian proj(lat0,lon0,0,earth);

    std::cout << "Origin Latitude: " << proj.LatitudeOrigin() << "\n";
    std::cout << "Origin Longitude: " << proj.LongitudeOrigin() << "\n";
    std::cout << "Origin Height: " << proj.HeightOrigin() << "\n";

    std::string kml_filename(argv[3]);
    std::string json_filename(argv[4]);

    Json::Value waypoint;
    waypoint["x"] = 0;
    waypoint["y"] = 0;
    waypoint["z"] = 0;
    waypoint["stamp"] = 0;
    waypoint["mandatory"] = 0;

    Json::Value db_json_root;
    Json::Value operation_dict;
    
    operation_dict["uav_id"] = 0;
    operation_dict["icao_address"] = "";
    operation_dict["frame"] = "";
    operation_dict["autonomy"] = 10;
    operation_dict["priority"] = 0;
    operation_dict["is_started"] = false;
    operation_dict["current_wp"] = 1;
    operation_dict["flight_plan"]["waypoints"] = Json::arrayValue;
    operation_dict["dT"] = 5.0;
    operation_dict["time_tracked"] = 0;
    operation_dict["track"]["waypoints"] = Json::arrayValue;
    operation_dict["time_horizon"] = 90;
    operation_dict["estimated_trajectory"]["waypoints"] = Json::arrayValue;
    operation_dict["landing_spots"]["waypoints"] = Json::arrayValue;
    operation_dict["flight_geometry"] = 4.0;
    operation_dict["operational_volume"] = 2.0;
    operation_dict["conop"] = "";

    // Read kml file
    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_file(kml_filename.c_str());

    std::cout << "Loading file: " << kml_filename << "\n";
    //std::cout << "Result description: " << result.description() << "\n";
    
    double initial_timestamp = 0;
    double current_timestamp;
    double previous_timestamp;

    std::cout << "Doc:\n";
    //std::cout << doc.value() << "\n";
    pugi::xml_node kml_node = doc.document_element();
    //std::cout << kml_node.attribute("xmlns").value() << "\n";
    pugi::xml_node document_node = kml_node.child("Document");
  
    // First, find Folder tag and extract all the operations inside it
    pugi::xml_node folder_node = document_node.child("Folder");
    std::cout << "Folder id: " << folder_node.attribute("id").as_string() << "\n";
    
    bool folder_end = false;
    pugi::xml_node placemark_node = folder_node.child("Placemark");
    while(!folder_end)
    {
        if(!placemark_node.empty())
        {
            Json::Value flight_plan_waypoints;
            pugi::xml_node extended_data_node = placemark_node.child("ExtendedData");
            
            if(!extended_data_node.empty())
            {
                pugi::xml_node icao_node = extended_data_node.find_child_by_attribute("Data","name", "icao");
                pugi::xml_node priority_node = extended_data_node.find_child_by_attribute("Data","name", "priority");
                pugi::xml_node conop_node = extended_data_node.find_child_by_attribute("Data","name", "conop");
                pugi::xml_node icao_value_node = icao_node.child("value");
                pugi::xml_node priority_value_node = priority_node.child("value");
                pugi::xml_node conop_value_node = conop_node.child("value");
                icao = std::atoi(icao_value_node.text().as_string());
                priority = std::atoi(priority_value_node.text().as_string());
                if(!conop_node.empty())
                    conop = std::string(conop_value_node.text().as_string());

                operation_dict["uav_id"] = uav_id;
                operation_dict["icao_address"] = icao;
                operation_dict["priority"] = priority;
                operation_dict["conop"] = conop;
                uav_id++;
            }
            else
            {
                operation_dict["uav_id"] = uav_id;
                operation_dict["priority"] = priority;
                uav_id++;
                priority++;
            }
            
            current_timestamp = initial_timestamp;
            previous_timestamp = current_timestamp;
            pugi::xml_node name_node = placemark_node.child("name");
            std::string operation_name = name_node.text().as_string();
            std::cout << "Processing operation " << operation_name << "\n";
            pugi::xml_node linestring_node = placemark_node.child("LineString");
            if(!linestring_node.empty())
            {
                flight_plan_waypoints["waypoints"] = Json::arrayValue;
                pugi::xml_node coordinates_node = linestring_node.child("coordinates");
                std::string coordinates(coordinates_node.text().as_string());
                std::cout << "Coordinates: " << coordinates << "\n";
                std::vector<std::string> comma_separated_strings;
                comma_separated_strings.push_back("");
                int comma_counter = 0;
                double previous_x, previous_y, previous_z;
                bool first_waypoint = true;
                bool end_triplet = false;
                bool some_data_read = false;
                for(int i=0; i<coordinates.size(); i++)
                {
                    if(coordinates[i] == ' ')
                    {
                        comma_counter = 0;
                        end_triplet = true;
                    }
                    else if(coordinates[i] == ',')
                    {
                        end_triplet = false;
                        comma_separated_strings.push_back("");
                        comma_counter++;
                    } else {
                        end_triplet = false;
                        some_data_read = true;
                        comma_separated_strings.at(comma_counter) += coordinates[i];
                    }
                    if(end_triplet && some_data_read)
                    {
                        // Extract flight plan waypoints
                        double latitude,longitude,altitude;
                        longitude = atof(comma_separated_strings[0].c_str());
                        latitude = atof(comma_separated_strings[1].c_str());
                        altitude = atof(comma_separated_strings[2].c_str());

                        // Convert geographic coordinates to local coordinates
                        double x,y,z;
                        proj.Forward(latitude,longitude,altitude,x,y,z);

                        if(first_waypoint)
                        {
                            current_timestamp = initial_timestamp;
                            previous_timestamp = current_timestamp;
                            first_waypoint = false;
                        }
                        else
                        {
                            double delta_time = distanceBetweenWaypoints(x,y,z,previous_x,previous_y,previous_z)/NOMINAL_SPEED;
                            current_timestamp = previous_timestamp + delta_time;

                            previous_timestamp = current_timestamp;
                        }

                        waypoint["x"] = x;
                        waypoint["y"] = y;
                        waypoint["z"] = z;
                        waypoint["stamp"] = current_timestamp;
                        previous_x = x;
                        previous_y = y;
                        previous_z = z;

                        flight_plan_waypoints["waypoints"].append(waypoint);
                        comma_separated_strings.clear();
                        comma_separated_strings.push_back("");
                    }
                }
            }
            else
            {
                current_timestamp = initial_timestamp;
                previous_timestamp = current_timestamp;
                // Look for Polygon
                pugi::xml_node name_node = placemark_node.child("name");
                std::string operation_name = name_node.text().as_string();
                std::cout << "Processing operation " << operation_name << "\n";
                pugi::xml_node polygon_node = placemark_node.child("Polygon");
                pugi::xml_node outer_boundary_node = polygon_node.child("outerBoundaryIs");
                pugi::xml_node linear_ring_node = outer_boundary_node.child("LinearRing");
                pugi::xml_node coordinates_node = linear_ring_node.child("coordinates");
                flight_plan_waypoints["waypoints"] = Json::arrayValue;
                std::string coordinates(coordinates_node.text().as_string());
                std::cout << "Coordinates: " << coordinates << "\n";
                std::vector<std::string> comma_separated_strings;
                comma_separated_strings.push_back("");
                int comma_counter = 0;
                double previous_x, previous_y, previous_z;
                bool first_waypoint = true;
                bool end_triplet = false;
                bool some_data_read = false;
                for(int i=0; i<coordinates.size(); i++)
                {
                    if(coordinates[i] == ' ')
                    {
                        comma_counter = 0;
                        end_triplet = true;
                    }
                    else if(coordinates[i] == ',')
                    {
                        end_triplet = false;
                        comma_separated_strings.push_back("");
                        comma_counter++;
                    } else {
                        end_triplet = false;
                        some_data_read = true;
                        comma_separated_strings.at(comma_counter) += coordinates[i];
                    }
                    if(end_triplet && some_data_read)
                    {
                        // Extract flight plan waypoints
                        double latitude,longitude,altitude;
                        longitude = atof(comma_separated_strings[0].c_str());
                        latitude = atof(comma_separated_strings[1].c_str());
                        altitude = atof(comma_separated_strings[2].c_str());
                        // Convert geographic coordinates to local coordinates
                        double x,y,z;
                        proj.Forward(latitude,longitude,altitude,x,y,z);
                        if(first_waypoint)
                        {
                            current_timestamp = initial_timestamp;
                            previous_timestamp = current_timestamp;
                            first_waypoint = false;
                        }
                        else
                        {
                            double delta_time = distanceBetweenWaypoints(x,y,z,previous_x,previous_y,previous_z)/NOMINAL_SPEED;
                            current_timestamp = previous_timestamp + delta_time;
                            previous_timestamp = current_timestamp;
                        }
                        waypoint["x"] = x;
                        waypoint["y"] = y;
                        waypoint["z"] = z;
                        waypoint["stamp"] = current_timestamp;
                        previous_x = x;
                        previous_y = y;
                        previous_z = z;
                        flight_plan_waypoints["waypoints"].append(waypoint);
                        comma_separated_strings.clear();
                        comma_separated_strings.push_back("");
                    }
                }
            }
            operation_dict["flight_plan"] = flight_plan_waypoints;
            db_json_root.append(operation_dict);
            placemark_node = placemark_node.next_sibling();
        }
        else
        {
            folder_end = true;
        }
    }

    bool document_end = false;
    placemark_node = document_node.child("Placemark");
    while(!document_end)
    {
        if(!placemark_node.empty())
        {
            Json::Value flight_plan_waypoints;
            pugi::xml_node extended_data_node = placemark_node.child("ExtendedData");
            
            if(!extended_data_node.empty())
            {
                pugi::xml_node icao_node = extended_data_node.find_child_by_attribute("name", "icao");
                pugi::xml_node priority_node = extended_data_node.find_child_by_attribute("name", "priority");
                pugi::xml_node conop_node = extended_data_node.find_child_by_attribute("name", "conop");
                pugi::xml_node icao_value_node = icao_node.child("value");
                pugi::xml_node priority_value_node = priority_node.child("value");
                pugi::xml_node conop_value_node = conop_node.child("value");
                icao = std::atoi(icao_value_node.text().as_string());
                priority = std::atoi(priority_value_node.text().as_string());
                if(!conop_node.empty())
                    conop = std::string(conop_value_node.text().as_string());

                operation_dict["uav_id"] = uav_id;
                operation_dict["icao_address"] = icao;
                operation_dict["priority"] = priority;
                operation_dict["conop"] = conop;
            }
            else
            {
                operation_dict["uav_id"] = uav_id;
                operation_dict["priority"] = priority;
                uav_id++;
                priority++;
            }
            
            current_timestamp = initial_timestamp;
            previous_timestamp = current_timestamp;
            pugi::xml_node name_node = placemark_node.child("name");
            std::string operation_name = name_node.text().as_string();
            std::cout << "Processing operation " << operation_name << "\n";
            pugi::xml_node linestring_node = placemark_node.child("LineString");
            if(!linestring_node.empty())
            {
                flight_plan_waypoints["flight_plan"]["waypoints"] = Json::arrayValue;
                pugi::xml_node coordinates_node = linestring_node.child("coordinates");
                std::string coordinates(coordinates_node.text().as_string());
                std::cout << "Coordinates: " << coordinates << "\n";
                std::vector<std::string> comma_separated_strings;
                comma_separated_strings.push_back("");
                int comma_counter = 0;
                double previous_x, previous_y, previous_z;
                bool first_waypoint = true;
                bool end_triplet = false;
                bool some_data_read = false;
                for(int i=0; i<coordinates.size(); i++)
                {
                    if(coordinates[i] == ' ')
                    {
                        comma_counter = 0;
                        end_triplet = true;
                    }
                    else if(coordinates[i] == ',')
                    {
                        end_triplet = false;
                        comma_separated_strings.push_back("");
                        comma_counter++;
                    } else {
                        end_triplet = false;
                        some_data_read = true;
                        comma_separated_strings.at(comma_counter) += coordinates[i];
                    }
                    if(end_triplet && some_data_read)
                    {
                        // Extract flight plan waypoints
                        double latitude,longitude,altitude;
                        longitude = atof(comma_separated_strings[0].c_str());
                        latitude = atof(comma_separated_strings[1].c_str());
                        altitude = atof(comma_separated_strings[2].c_str());

                        // Convert geographic coordinates to local coordinates
                        double x,y,z;
                        proj.Forward(latitude,longitude,altitude,x,y,z);

                        if(first_waypoint)
                        {
                            current_timestamp = initial_timestamp;
                            previous_timestamp = current_timestamp;
                            first_waypoint = false;
                        }
                        else
                        {
                            double delta_time = distanceBetweenWaypoints(x,y,z,previous_x,previous_y,previous_z)/NOMINAL_SPEED;
                            current_timestamp = previous_timestamp + delta_time;

                            previous_timestamp = current_timestamp;
                        }

                        waypoint["x"] = x;
                        waypoint["y"] = y;
                        waypoint["z"] = z;
                        waypoint["stamp"] = current_timestamp;
                        previous_x = x;
                        previous_y = y;
                        previous_z = z;

                        flight_plan_waypoints["waypoints"].append(waypoint);
                        comma_separated_strings.clear();
                        comma_separated_strings.push_back("");
                    }
                }
            }
            else
            {
                current_timestamp = initial_timestamp;
                previous_timestamp = current_timestamp;
                // Look for Polygon
                pugi::xml_node name_node = placemark_node.child("name");
                std::string operation_name = name_node.text().as_string();
                std::cout << "Processing operation " << operation_name << "\n";
                pugi::xml_node polygon_node = placemark_node.child("Polygon");
                pugi::xml_node outer_boundary_node = polygon_node.child("outerBoundaryIs");
                pugi::xml_node linear_ring_node = outer_boundary_node.child("LinearRing");
                pugi::xml_node coordinates_node = linear_ring_node.child("coordinates");
                flight_plan_waypoints["waypoints"] = Json::arrayValue;
                std::string coordinates(coordinates_node.text().as_string());
                std::cout << "Coordinates: " << coordinates << "\n";
                std::vector<std::string> comma_separated_strings;
                comma_separated_strings.push_back("");
                int comma_counter = 0;
                double previous_x, previous_y, previous_z;
                bool first_waypoint = true;
                bool end_triplet = false;
                bool some_data_read = false;
                for(int i=0; i<coordinates.size(); i++)
                {
                    if(coordinates[i] == ' ')
                    {
                        comma_counter = 0;
                        end_triplet = true;
                    }
                    else if(coordinates[i] == ',')
                    {
                        end_triplet = false;
                        comma_separated_strings.push_back("");
                        comma_counter++;
                    } else {
                        end_triplet = false;
                        some_data_read = true;
                        comma_separated_strings.at(comma_counter) += coordinates[i];
                    }
                    if(end_triplet && some_data_read)
                    {
                        // Extract flight plan waypoints
                        double latitude,longitude,altitude;
                        longitude = atof(comma_separated_strings[0].c_str());
                        latitude = atof(comma_separated_strings[1].c_str());
                        altitude = atof(comma_separated_strings[2].c_str());
                        // Convert geographic coordinates to local coordinates
                        double x,y,z;
                        proj.Forward(latitude,longitude,altitude,x,y,z);
                        if(first_waypoint)
                        {
                            current_timestamp = initial_timestamp;
                            previous_timestamp = current_timestamp;
                            first_waypoint = false;
                        }
                        else
                        {
                            double delta_time = distanceBetweenWaypoints(x,y,z,previous_x,previous_y,previous_z)/NOMINAL_SPEED;
                            current_timestamp = previous_timestamp + delta_time;
                            previous_timestamp = current_timestamp;
                        }
                        waypoint["x"] = x;
                        waypoint["y"] = y;
                        waypoint["z"] = z;
                        waypoint["stamp"] = current_timestamp;
                        previous_x = x;
                        previous_y = y;
                        previous_z = z;
                        flight_plan_waypoints["waypoints"].append(waypoint);
                        comma_separated_strings.clear();
                        comma_separated_strings.push_back("");
                    }
                }
            }
            operation_dict["flight_plan"] = flight_plan_waypoints;
            db_json_root.append(operation_dict);
            placemark_node = placemark_node.next_sibling();
        }
        else
        {
            document_end = true;
        }
    }

    // Write operations JSON to file
    std::ofstream json_filestream(json_filename.c_str());
    if(json_filestream.is_open())
    {
        json_filestream << db_json_root;
    }

    return 0;
}