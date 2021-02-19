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

#define NOMINAL_SPEED 5.0 // meters per second

inline double distanceBetweenWaypoints(double &x1, double &y1, double &z1, double &x2, double &y2, double &z2);

inline double distanceBetweenWaypoints(double &x1, double &y1, double &z1, double &x2, double &y2, double &z2)
{
    return sqrt(pow(x1-x2,2)+pow(y1-y2,2)+pow(z1-z2,2));
}

int main(int argc, char** argv)
{
    if(argc != 5)
    {
        std::cout << "Usage: multiple_flight_plans_kml_to_db_json origin_latitude origin_longitude KML_INPUT_FILE JSON_OUTPUT_FILE_PREFIX \n"; 
        return 1;
    }
    
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
            current_timestamp = initial_timestamp;
            previous_timestamp = current_timestamp;
            pugi::xml_node name_node = placemark_node.child("name");
            std::string operation_name = name_node.text().as_string();
            std::cout << "Processing operation " << operation_name << "\n";
            pugi::xml_node linestring_node = placemark_node.child("LineString");
            if(!linestring_node.empty())
            {
                Json::Value root;
                root["flight_plan"]["waypoints"] = Json::arrayValue;
                pugi::xml_node coordinates_node = linestring_node.child("coordinates");
                Json::Value waypoint_array = Json::arrayValue;
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

                        root["flight_plan"]["waypoints"].append(waypoint);
                        comma_separated_strings.clear();
                        comma_separated_strings.push_back("");
                    }
                }
                // Write flight plan JSON to file
                std::ofstream json_filestream((json_filename+operation_name).c_str());
                if(json_filestream.is_open())
                {
                    json_filestream << root;
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
                Json::Value root;
                root["flight_plan"]["waypoints"] = Json::arrayValue;
                Json::Value waypoint_array = Json::arrayValue;
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
                        root["flight_plan"]["waypoints"].append(waypoint);
                        comma_separated_strings.clear();
                        comma_separated_strings.push_back("");
                    }
                }
                // Write flight plan JSON to file
                std::ofstream json_filestream((json_filename+operation_name).c_str());
                if(json_filestream.is_open())
                {
                    json_filestream << root;
                }
            }
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
            current_timestamp = initial_timestamp;
            previous_timestamp = current_timestamp;
            pugi::xml_node name_node = placemark_node.child("name");
            std::string operation_name = name_node.text().as_string();
            std::cout << "Processing operation " << operation_name << "\n";
            pugi::xml_node linestring_node = placemark_node.child("LineString");
            if(!linestring_node.empty())
            {
                Json::Value root;
                root["flight_plan"]["waypoints"] = Json::arrayValue;
                pugi::xml_node coordinates_node = linestring_node.child("coordinates");
                Json::Value waypoint_array = Json::arrayValue;
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

                        root["flight_plan"]["waypoints"].append(waypoint);
                        comma_separated_strings.clear();
                        comma_separated_strings.push_back("");
                    }
                }
                // Write flight plan JSON to file
                std::ofstream json_filestream((json_filename+operation_name).c_str());
                if(json_filestream.is_open())
                {
                    json_filestream << root;
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
                Json::Value root;
                root["flight_plan"]["waypoints"] = Json::arrayValue;
                Json::Value waypoint_array = Json::arrayValue;
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
                        root["flight_plan"]["waypoints"].append(waypoint);
                        comma_separated_strings.clear();
                        comma_separated_strings.push_back("");
                    }
                }
                // Write flight plan JSON to file
                std::ofstream json_filestream((json_filename+operation_name).c_str());
                if(json_filestream.is_open())
                {
                    json_filestream << root;
                }
            }
            placemark_node = placemark_node.next_sibling();
        }
        else
        {
            document_end = true;
        }
    }


    return 0;
}