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

#include <usp_nodes/path_finder.h>

PathFinder::PathFinder(nav_msgs::Path &_init_path, geometry_msgs::Point &_init_astar_point, geometry_msgs::Point &_goal_astar_point, geometry_msgs::Polygon &_polygon, geometry_msgs::Point &_min_grid_point, geometry_msgs::Point &_max_grid_point) {
    init_path_ = _init_path;
    static upat_follower::Generator generator(1.0, 1.0, 1.0);
    target_path_ = generator.generatePath(init_path_);
    init_astar_point_ = _init_astar_point;
    goal_astar_point_ = _goal_astar_point;
    polygon_ = _polygon;
    x_min_ = std::min(_min_grid_point.x, _max_grid_point.x);
    x_max_ = std::max(_min_grid_point.x, _max_grid_point.x);
    y_min_ = std::min(_min_grid_point.y, _max_grid_point.y);
    y_max_ = std::max(_min_grid_point.y, _max_grid_point.y);
}

PathFinder::~PathFinder() {
}

nav_msgs::Path PathFinder::createPathFromPlanner(std::vector<geometry_msgs::Point> &_in_path, geometry_msgs::Point _init_p, double _path_height) {
    nav_msgs::Path out_path;
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = _init_p.x;
    pose.pose.position.y = _init_p.y;
    pose.pose.position.z = _path_height;
    out_path.poses.push_back(pose);
    for (int i = 0; i < _in_path.size(); i++) {
        pose.pose.position.x = _in_path.at(i).x;
        pose.pose.position.y = _in_path.at(i).y;
        pose.pose.position.z = _path_height;
        out_path.poses.push_back(pose);
    }

    return out_path;
}

nav_msgs::Path PathFinder::findNewPath() {
    std::vector<geometry_msgs::Polygon> vec_polygons;
    vec_polygons.push_back(polygon_);
    geometry_msgs::Polygon temp_polygon;
    geometry_msgs::Point32 temp_point;
    temp_point.x = x_min_;
    temp_point.y = y_min_;
    temp_polygon.points.push_back(temp_point);
    temp_polygon.points.push_back(temp_point);
    vec_polygons.push_back(temp_polygon);
    temp_polygon.points.clear();
    temp_point.x = x_max_;
    temp_point.y = y_max_;
    temp_polygon.points.push_back(temp_point);
    temp_polygon.points.push_back(temp_point);
    vec_polygons.push_back(temp_polygon);
    // Call path planner A*
    multidrone::PathPlanner path_planner(vec_polygons);
    std::vector<geometry_msgs::Point> a_star_getpath = path_planner.getPath(init_astar_point_, goal_astar_point_, false, false);
    nav_msgs::Path a_star_path_res = createPathFromPlanner(a_star_getpath, init_astar_point_, target_path_.poses.front().pose.position.z);
    // Linear interpolation for Z axis
    std::vector<double> default_z, interp1_z;
    default_z.push_back(init_astar_point_.z);
    default_z.push_back(goal_astar_point_.z);
    static upat_follower::Generator generator(1.0, 1.0, 1.0);
    interp1_z = generator.interpWaypointList(default_z, a_star_path_res.poses.size() - 1);
    for (int i = 0; i < interp1_z.size(); i++) {
        a_star_path_res.poses.at(i).pose.position.z = interp1_z.at(i);
    }
    a_star_path_res.poses.back().pose.position.z = goal_astar_point_.z;

    return a_star_path_res;
}