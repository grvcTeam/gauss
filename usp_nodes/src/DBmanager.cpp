#include <ros/ros.h>
#include <gauss_msgs/WaypointList.h>
#include <gauss_msgs/Waypoint.h>
#include <gauss_msgs/ReadTracks.h>
#include <gauss_msgs/WriteTracks.h>
#include <gauss_msgs/ReadFlightPlan.h>
#include <gauss_msgs/WriteFlightPlan.h>
#include <gauss_msgs/ReadGeofences.h>
#include <gauss_msgs/WriteGeofences.h>

#define DB_SIZE 100

// Class definition
class DataBase
{
public:
    DataBase();

private:
    // Topic Callbacks

    // Service Callbacks
    bool readTrackCB(gauss_msgs::ReadTracks::Request &req, gauss_msgs::ReadTracks::Response &res);
    bool writeTrackCB(gauss_msgs::WriteTracks::Request &req, gauss_msgs::WriteTracks::Response &res);
    bool readPlanCB(gauss_msgs::ReadFlightPlan::Request &req, gauss_msgs::ReadFlightPlan::Response &res);
    bool writePlanCB(gauss_msgs::WriteFlightPlan::Request &req, gauss_msgs::WriteFlightPlan::Response &res);
    bool readGeofenceCB(gauss_msgs::ReadGeofences::Request &req, gauss_msgs::ReadGeofences::Response &res);
    bool writeGeofenceCB(gauss_msgs::WriteGeofences::Request &req, gauss_msgs::WriteGeofences::Response &res);

    // Auxilary variables
    int size_plans;
    int size_tracks;


    ros::NodeHandle nh_;

    // Subscribers

    // Publisher

    // Timer

    // Server
    ros::ServiceServer read_track_server_;
    ros::ServiceServer write_track_server_;
    ros::ServiceServer read_plan_server_;
    ros::ServiceServer write_plan_server_;
    ros::ServiceServer read_geofences_server_;
    ros::ServiceServer write_geofences_server_;

};

// positionReporting Constructor
DataBase::DataBase()
{
    // Read parameters
    //nh_.param("desired_altitude",desired_altitude,0.5);


    // Initialization
    size_plans=size_tracks=0;


    // Publish

    // Subscribe


    // Server
    read_track_server_=nh_.advertiseService("/gauss/readTracks",&DataBase::readTrackCB,this);
    write_track_server_=nh_.advertiseService("/gauss/writeTracks",&DataBase::writeTrackCB,this);
    read_plan_server_=nh_.advertiseService("/gauss/readFlightPlan",&DataBase::readPlanCB,this);
    write_plan_server_=nh_.advertiseService("/gauss/writeFlightPlan",&DataBase::writePlanCB,this);
    read_geofences_server_=nh_.advertiseService("/gauss_msgs/readGeofences",&DataBase::readGeofenceCB,this);
    write_geofences_server_=nh_.advertiseService("/gauss/writeGeofences",&DataBase::writeGeofenceCB,this);

    ROS_INFO("Started DBManager node!");
}


// ReadTrack callbacks
bool DataBase::readTrackCB(gauss_msgs::ReadTracks::Request &req, gauss_msgs::ReadTracks::Response &res)
{
   /* int id = req.id;

    if (size_tracks>0)
    {
        for (int i=0; i<size_tracks; i++)
        {
            if (tracks[i].id==id)
            {
                res.track=tracks[i];
                res.success=true;
                return true;
            }
        }
    }
    res.success=false;*/
    return true;
}


// WriteTrack callback
bool DataBase::writeTrackCB(gauss_msgs::WriteTracks::Request &req, gauss_msgs::WriteTracks::Response &res)
{
   /* int id = req.id;
    bool rewritted =false;

    if (size_tracks>0)
    {
        for (int i=0; i<size_tracks; i++)
        {
            if (tracks[i].id==id)
            {
                // Previous checkings TBD
                tracks[i]=req.track;
                res.success=true;
                rewritted=true;
            }
        }
        if (!rewritted)
        {
            tracks[size_tracks]=req.track;
            size_tracks++;
            res.success=true;
        }
    }
    else
    {
        tracks[0]=req.track;
        size_tracks++;
        res.success=true;
        return true;
    }
    res.success=false;*/
    return true;
}

// ReadPlan callbacks
bool DataBase::readPlanCB(gauss_msgs::ReadFlightPlan::Request &req, gauss_msgs::ReadFlightPlan::Response &res)
{
  /*  int id = req.id;

    if (size_plans>0)
    {
        for (int i=0; i<size_plans; i++)
        {
            if (tracks[i].id==id)
            {
                res.plan=plans[i];
                res.success=true;
                return true;
            }
        }
    }
    res.success=false;*/
    return true;
}


// WritePlan callback
bool DataBase::writePlanCB(gauss_msgs::WriteFlightPlan::Request &req, gauss_msgs::WriteFlightPlan::Response &res)
{
   /* int id = req.id;
    bool rewritted =false;

    if (size_plans>0)
    {
        for (int i=0; i<size_plans; i++)
        {
            if (plans[i].id==id)
            {
                // Previous checkings TBD
                plans[i]=req.plan;
                res.success=true;
                rewritted=true;
            }
        }
        if (!rewritted)
        {
            plans[size_plans]=req.plan;
            size_plans++;
            res.success=true;
        }
    }
    else
    {
        plans[0]=req.plan;
        size_plans++;
        res.success=true;
        return true;
    }
    res.success=false;*/
    return true;
}

// ReadGeofence callbacks
bool DataBase::readGeofenceCB(gauss_msgs::ReadGeofences::Request &req, gauss_msgs::ReadGeofences::Response &res)
{
   /* int id = req.id;

    if(req.type==0)
    {
        if (size_static_geofences>0)
        {
            for (int i=0; i<size_static_geofences; i++)
            {
                if (static_geofences[i].id==id)
                {
                    res.geofence=static_geofences[i];
                    res.success=true;
                    return true;
                }
            }
        }
    }
    else if (req.type==1)
    {
        if (size_temp_geofences>0)
        {
            for (int i=0; i<size_temp_geofences; i++)
            {
                if (temp_geofences[i].id==id)
                {
                    res.geofence=temp_geofences[i];
                    res.success=true;
                    return true;
                }
            }
        }
    }

    res.success=false;*/
    return true;
}


// WriteGeofence callback
bool DataBase::writeGeofenceCB(gauss_msgs::WriteGeofences::Request &req, gauss_msgs::WriteGeofences::Response &res)
{
 /*   int id = req.id;
    bool rewritted =false;

    if (req.type==0)
    {
        if (size_static_geofences>0)
        {
            for (int i=0; i<size_static_geofences; i++)
            {
                if (static_geofences[i].id==id)
                {
                    // Previous checkings TBD
                    static_geofences[i]=req.geofence;
                    res.success=true;
                    rewritted=true;
                }
            }
            if (!rewritted)
            {
                static_geofences[size_static_geofences]=req.geofence;
                size_static_geofences++;
                res.success=true;
            }
        }
        else
        {
            static_geofences[0]=req.geofence;
            size_static_geofences++;
            res.success=true;
            return true;
        }
    }
    else if (req.type==1)
    {
        if (size_temp_geofences>0)
        {
            for (int i=0; i<size_temp_geofences; i++)
            {
                if (temp_geofences[i].id==id)
                {
                    // Previous checkings TBD
                    temp_geofences[i]=req.geofence;
                    res.success=true;
                    rewritted=true;
                }
            }
            if (!rewritted)
            {
                temp_geofences[size_temp_geofences]=req.geofence;
                size_temp_geofences++;
                res.success=true;
            }
        }
        else
        {
            temp_geofences[0]=req.geofence;
            size_temp_geofences++;
            res.success=true;
            return true;
        }
    }

    res.success=false;*/
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
