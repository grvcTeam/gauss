//------------------------------------------------------------------------------
// GRVC
//------------------------------------------------------------------------------
//
// Copyright (c) 2016 GRVC University of Seville
//
//------------------------------------------------------------------------------

#ifndef CANDIDATE_H_
#define CANDIDATE_H_

#include <Eigen/Eigen>
#include <ros/ros.h>

struct Candidate{
    
    enum SOURCE : uint8_t {POSITIONREPORT = 0, ADSB = 1};

    // Source from which data has come
    SOURCE source;

    std::string icao_address;
    uint8_t uav_id;

    // Location and speed
    Eigen::Vector3d location; // Inluding altitude
    Eigen::Vector3d speed; // This speed must be calculated from yaw, pitch, roll and groundspeed

    // Location and speed covariances
    Eigen::Matrix3d location_covariance;
    Eigen::Matrix3d speed_covariance;

    ros::Time timestamp; // Time at which those measures were taken

    // Flag that indicates speed measures are available
    bool speed_available;
};

#endif
