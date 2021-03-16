//------------------------------------------------------------------------------
// GRVC 
// Author Alejandro Braza Barba <alejandrobrazabarba@gmail.com>
//------------------------------------------------------------------------------
//
// Copyright (c) 2016 GRVC University of Seville
//
//------------------------------------------------------------------------------

#ifndef TIMER_H_
#define TIMER_H_

#include <ros/time.h>

/**
 * class Timer
 *
 * A simple timer implementation by using ros::Time
 *
 * @author Alejandro Braza Barba
 */

class Timer
{
public:
    /**
     * Create a new timer and start it
     */
    Timer(const ros::Time &beginning) : beg_(beginning) {}
    /**
     * Reset the timer to 0
     */
    void reset(const ros::Time &beginning) { beg_ = beginning; }
    /**
     * Get the elapsed time in seconds since the creation
     * of the object or the last time reset() method was called
     * @return elapsed: the elapsed time in seconds
     */
    ros::Duration elapsed(const ros::Time &now) const { 
        return ros::Duration
            (now - beg_); }

    ros::Time referenceTime() { return beg_; };
private:
    ros::Time beg_;
};

#endif