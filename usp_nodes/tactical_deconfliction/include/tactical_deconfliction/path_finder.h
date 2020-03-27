//----------------------------------------------------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2018 GRVC University of Seville
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
// documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
// Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
// OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
// OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//----------------------------------------------------------------------------------------------------------------------

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <upat_follower/generator.h>
#include <tactical_deconfliction/path_planner.h>

#include <Eigen/Eigen>

#ifndef PATH_FINDER_H
#define PATH_FINDER_H

class PathFinder {
   public:
    PathFinder();
    PathFinder(nav_msgs::Path &_init_path, geometry_msgs::Point &_init_astar_point, geometry_msgs::Point &_goal_astar_point, geometry_msgs::Polygon &_polygon, geometry_msgs::Point &_min_grid_point, geometry_msgs::Point &_max_grid_point);
    ~PathFinder();

    nav_msgs::Path findNewPath();

   private:
    // Callbacks
    // Methods
    nav_msgs::Path createPathFromPlanner(std::vector<geometry_msgs::Point> &_in_path, geometry_msgs::Point _init_p, double _path_height);
    // Node handlers
    ros::NodeHandle nh_, pnh_;
    // Subscribers
    // Publishers
    // Services
    // Variables
    nav_msgs::Path init_path_, target_path_, a_star_path_;
    geometry_msgs::Point init_astar_point_, goal_astar_point_;
    geometry_msgs::Polygon polygon_;
    double x_min_, y_min_, x_max_, y_max_;
    // Params
};

#endif  // PATH_FINDEr_H