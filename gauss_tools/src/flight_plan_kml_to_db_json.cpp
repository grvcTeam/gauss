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
    if(argc != 3)
    {
        std::cout << "Usage: flight_plan_kml_to_db_json KML_filename KML_INPUT_FILE JSON_OUTPUT_FILE\n"; 
        return 1;
    }
    
    double lat0,lon0;
    lat0 = 37.094784;
    lon0 = -6.735478;
    GeographicLib::Geocentric earth(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f());
    GeographicLib::LocalCartesian proj(lat0,lon0,0,earth);

    std::string kml_filename(argv[1]);
    std::string json_filename(argv[2]);

    Json::Value root;

    root["flight_plan"]["waypoints"] = Json::arrayValue;

    Json::Value waypoint;
    waypoint["x"] = 0;
    waypoint["y"] = 0;
    waypoint["z"] = 0;
    waypoint["stamp"] = 0;
    waypoint["mandatory"] = 0;

    // Read kml file
    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_file(kml_filename.c_str());

    //std::cout << "Result description: " << result.description() << "\n";
    
    double initial_timestamp = 0;
    double current_timestamp;
    double previous_timestamp;

    std::cout << "Doc:\n";
    //std::cout << doc.value() << "\n";
    pugi::xml_node kml_node = doc.document_element();
    //std::cout << kml_node.attribute("xmlns").value() << "\n";
    pugi::xml_node document_node = kml_node.child("Document");
    pugi::xml_node placemark_node = document_node.child("Placemark");
    if(placemark_node.empty())
        std::cout << "Placemark node empty\n";
    std::cout << "Placemark id: " << placemark_node.attribute("id").value() << "\n";
    bool kml_end_found = false;
    bool first_waypoint = true;

    double previous_x, previous_y,previous_z;

    while(!kml_end_found)
    {
        pugi::xml_node name_node = placemark_node.child("name");
        if(name_node.empty())
            std::cout << "name node empty\n";
        std::string name_string(name_node.text().as_string());
        std::cout << "Name string: " << name_string << "\n";
        if(name_string[0] == 'W' && name_string[1] == 'P')
        {
            Json::Value waypoint_array = Json::arrayValue;
            pugi::xml_node point_node = placemark_node.child("Point");
            pugi::xml_node coordinates_node = point_node.child("coordinates");
            std::string coordinates(coordinates_node.text().as_string());
            std::cout << "Coordinates: " << coordinates << "\n";
            std::vector<std::string> comma_separated_strings;
            comma_separated_strings.push_back("");
            int comma_counter = 0;
            for(int i=0; i<coordinates.size(); i++)
            {
                if(coordinates[i] == ',')
                {
                    comma_separated_strings.push_back("");
                    comma_counter++;
                } else {
                    comma_separated_strings.at(comma_counter) += coordinates[i];
                }
            }
            // Extract flight plan waypoints
            double latitude,longitude,altitude;
            longitude = atof(comma_separated_strings[0].c_str());
            latitude = atof(comma_separated_strings[1].c_str());
            altitude = atof(comma_separated_strings[2].c_str());

            pugi::xml_node look_at_node = placemark_node.child("LookAt");
            //pugi::xml_node longitude_node = look_at_node.child("longitude");
            //pugi::xml_node latitude_node = look_at_node.child("latitude");
            pugi::xml_node altitude_node = look_at_node.child("altitude");

            //latitude = latitude_node.text().as_double();
            //longitude = longitude_node.text().as_double();
            altitude = altitude_node.text().as_double();

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
        }
        else
        {
            kml_end_found = true;
        }
        
        placemark_node = placemark_node.next_sibling();
        if(placemark_node.empty())
            kml_end_found = true;
    }


    // Write flight plan JSON to file
    std::ofstream json_filestream(json_filename.c_str());
    if(json_filestream.is_open())
    {
        json_filestream << root;
    }

    return 0;
}