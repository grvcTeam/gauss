#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

#include <iostream>

void print_usage()
{
    std::cout << "Usage: \nTo convert from geographical to cartesian use\n";
    std::cout << "coordinate_conversion_tool -f origin_latitude origin_longitude origin_ellipsoidal_height target_latitude target_longitude target_ellipsoidal_height\n";
    std::cout << "To convert from cartesian to geographical use\n";
    std::cout << "coordinate_conversion_tool -r origin_latitude origin_longitude origin_ellipsoidal_height target_x target_y target_altitude\n";
}

int main(int argc, char** argv)
{
    GeographicLib::Geocentric earth(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f());
    
    double origin_latitude, origin_longitude, origin_ellipsoidal_height;
    double target_latitude, target_longitude, target_ellipsoidal_height;
    double target_x, target_y, target_altitude;

    if(argc != 8)
    {
        std::cout << "Wrong number of arguments\n";
        print_usage();
    }
    else
    {
        origin_latitude = atof(argv[2]);
        origin_longitude = atof(argv[3]);
        origin_ellipsoidal_height = atof(argv[4]);

        GeographicLib::LocalCartesian proj(origin_latitude, origin_longitude, origin_ellipsoidal_height, earth);
        if(std::string(argv[1]) == "-f")
        {
            std::cout << "Conversion from geographical to local cartesian\n";
            // Forward conversion
            target_latitude = atof(argv[5]);
            target_longitude = atof(argv[6]);
            target_ellipsoidal_height = atof(argv[7]);

            proj.Forward(target_latitude, target_longitude, target_ellipsoidal_height, target_x, target_y, target_altitude);
            std::cout << "Converted Local Euclidean Coordinates (x,y,altitude): " << target_x << "," << target_y << "," << target_altitude << "\n";
        }
        else if(std::string(argv[1]) == "-r")
        {
            std::cout << "Conversion from local cartesian to geographical\n";
            // Reverse conversion
            target_x = atof(argv[5]);
            target_y = atof(argv[6]);
            target_altitude = atof(argv[7]);

            proj.Reverse(target_x, target_y, target_altitude, target_latitude, target_longitude, target_ellipsoidal_height);
            std::cout << "Converted Geographical Coordinates (lat,lon,ellipsoidal_height): " << target_latitude << "," << target_longitude << "," << target_ellipsoidal_height << "\n";
        }
        else    
        {
            // Bad option
            std::cout << "Bad option\n";
            print_usage();
        }
    }

    return 0;
}