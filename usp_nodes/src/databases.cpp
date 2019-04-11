#include <ros/ros.h>
#include <gauss_msgs/Track.h>
#include <gauss_msgs/Plan.h>
#include <gauss_msgs/ReadTrack.h>
#include <gauss_msgs/WriteTrack.h>
#include <gauss_msgs/ReadPlan.h>
#include <gauss_msgs/WritePlan.h>
#include <gauss_msgs/ReadGeofence.h>
#include <gauss_msgs/WriteGeofence.h>

#define DB_SIZE 100

// Class definition
class DataBase
{
public:
    DataBase();

private:
    // Topic Callbacks

    // Service Callbacks
    bool readTrackCB(gauss_msgs::ReadTrack::Request &req, gauss_msgs::ReadTrack::Response &res);
    bool writeTrackCB(gauss_msgs::WriteTrack::Request &req, gauss_msgs::WriteTrack::Response &res);
    bool readPlanCB(gauss_msgs::ReadPlan::Request &req, gauss_msgs::ReadPlan::Response &res);
    bool writePlanCB(gauss_msgs::WritePlan::Request &req, gauss_msgs::WritePlan::Response &res);
    bool readGeofenceCB(gauss_msgs::ReadGeofence::Request &req, gauss_msgs::ReadGeofence::Response &res);
    bool writeGeofenceCB(gauss_msgs::WriteGeofence::Request &req, gauss_msgs::WriteGeofence::Response &res);

    // Auxilary variables
    gauss_msgs::Track tracks[DB_SIZE];
    gauss_msgs::Plan plans[DB_SIZE];
    gauss_msgs::Geofence static_geofences[DB_SIZE];
    gauss_msgs::Geofence temp_geofences[DB_SIZE];
    int size_tracks;
    int size_plans;
    int size_static_geofences;
    int size_temp_geofences;

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
    read_track_server_=nh_.advertiseService("/gauss/readTrack",&DataBase::readTrackCB,this);
    write_track_server_=nh_.advertiseService("/gauss/writeTrack",&DataBase::writeTrackCB,this);
    read_plan_server_=nh_.advertiseService("/gauss/readPlan",&DataBase::readPlanCB,this);
    write_plan_server_=nh_.advertiseService("/gauss/writePlan",&DataBase::writePlanCB,this);
    read_geofences_server_=nh_.advertiseService("/gauss_msgs/readGeofence",&DataBase::readGeofenceCB,this);
    write_geofences_server_=nh_.advertiseService("/gauss/writeGeofence",&DataBase::writeGeofenceCB,this);

    ROS_INFO("Started DataBase node!");
}


// ReadTrack callbacks
bool DataBase::readTrackCB(gauss_msgs::ReadTrack::Request &req, gauss_msgs::ReadTrack::Response &res)
{
    int id = req.id;

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
    res.success=false;
    return true;
}


// WriteTrack callback
bool DataBase::writeTrackCB(gauss_msgs::WriteTrack::Request &req, gauss_msgs::WriteTrack::Response &res)
{
    int id = req.id;
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
    res.success=false;
    return true;
}

// ReadPlan callbacks
bool DataBase::readPlanCB(gauss_msgs::ReadPlan::Request &req, gauss_msgs::ReadPlan::Response &res)
{
    int id = req.id;

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
    res.success=false;
    return true;
}


// WritePlan callback
bool DataBase::writePlanCB(gauss_msgs::WritePlan::Request &req, gauss_msgs::WritePlan::Response &res)
{
    int id = req.id;
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
    res.success=false;
    return true;
}

// ReadGeofence callbacks
bool DataBase::readGeofenceCB(gauss_msgs::ReadGeofence::Request &req, gauss_msgs::ReadGeofence::Response &res)
{
    int id = req.id;

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

    res.success=false;
    return true;
}


// WriteGeofence callback
bool DataBase::writeGeofenceCB(gauss_msgs::WriteGeofence::Request &req, gauss_msgs::WriteGeofence::Response &res)
{
    int id = req.id;
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

    res.success=false;
    return true;
}




// MAIN function
int main(int argc, char *argv[])
{
    ros::init(argc,argv,"database");

    // Create a DataBase object
    DataBase *database = new DataBase();

    ros::spin();
}
