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

#include <tactical_deconfliction/path_finder.h>

PathFinder::PathFinder(nav_msgs::Path &_init_path, geometry_msgs::Point &_init_astar_point, geometry_msgs::Point &_goal_astar_point, geometry_msgs::Polygon &_polygon, geometry_msgs::Point &_min_grid_point, geometry_msgs::Point &_max_grid_point) {
    init_path_ = _init_path;
    target_path_ = generatePath(init_path_);
    init_astar_point_ = _init_astar_point;
    goal_astar_point_ = _goal_astar_point;
    polygon_ = _polygon;
    x_min_ = std::min(_min_grid_point.x, _max_grid_point.x);
    x_max_ = std::max(_min_grid_point.x, _max_grid_point.x);
    y_min_ = std::min(_min_grid_point.y, _max_grid_point.y);
    y_max_ = std::max(_min_grid_point.y, _max_grid_point.y);
}

PathFinder::PathFinder(){
}

PathFinder::~PathFinder() {
}

std::vector<double> PathFinder::interpWaypointList(std::vector<double> &_list_pose_axis, int _amount_of_points) {
    std::vector<double> aux_axis;
    std::vector<double> new_aux_axis;
    for (int i = 0; i < _list_pose_axis.size(); i++) {
        aux_axis.push_back(i);
    }
    double portion = (aux_axis.back() - aux_axis.front()) / (_amount_of_points);
    double new_pose = aux_axis.front();
    new_aux_axis.push_back(new_pose);
    for (int i = 1; i < _amount_of_points; i++) {
        new_pose = new_pose + portion;
        new_aux_axis.push_back(new_pose);
    }
    auto interp1_path = linealInterp1(aux_axis, _list_pose_axis, new_aux_axis);
    return interp1_path;
}

int PathFinder::nearestNeighbourIndex(std::vector<double> &_x, double &_value) {
    double dist = std::numeric_limits<double>::max();
    double newDist = dist;
    size_t idx = 0;

    for (size_t i = 0; i < _x.size(); ++i) {
        newDist = std::abs(_value - _x[i]);
        if (newDist <= dist) {
            dist = newDist;
            idx = i;
        }
    }

    return idx;
}

std::vector<double> PathFinder::linealInterp1(std::vector<double> &_x, std::vector<double> &_y, std::vector<double> &_x_new) {
    std::vector<double> y_new;
    double dx, dy, m, b;
    size_t x_max_idx = _x.size() - 1;
    size_t x_new_size = _x_new.size();

    y_new.reserve(x_new_size);

    for (size_t i = 0; i < x_new_size; ++i) {
        size_t idx = nearestNeighbourIndex(_x, _x_new[i]);

        if (_x[idx] > _x_new[i]) {
            dx = idx > 0 ? (_x[idx] - _x[idx - 1]) : (_x[idx + 1] - _x[idx]);
            dy = idx > 0 ? (_y[idx] - _y[idx - 1]) : (_y[idx + 1] - _y[idx]);
        } else {
            dx = idx < x_max_idx ? (_x[idx + 1] - _x[idx]) : (_x[idx] - _x[idx - 1]);
            dy = idx < x_max_idx ? (_y[idx + 1] - _y[idx]) : (_y[idx] - _y[idx - 1]);
        }

        m = dy / dx;
        b = _y[idx] - _x[idx] * m;

        y_new.push_back(_x_new[i] * m + b);
    }

    return y_new;
}

nav_msgs::Path PathFinder::generatePath(nav_msgs::Path &_init_path, int _generator_mode, double _d_between_wps) {
    std::vector<double> list_pose_x, list_pose_y, list_pose_z, list_pose_t;
    for (int i = 0; i < _init_path.poses.size(); i++) {
        list_pose_x.push_back(_init_path.poses.at(i).pose.position.x);
        list_pose_y.push_back(_init_path.poses.at(i).pose.position.y);
        list_pose_z.push_back(_init_path.poses.at(i).pose.position.z);
        list_pose_t.push_back(_init_path.poses.at(i).header.stamp.toSec());
    }
    int total_distance = 0;

    for (int i = 0; i < _init_path.poses.size() - 1; i++) {
        Eigen::Vector3f point_1, point_2;
        point_1 = Eigen::Vector3f(list_pose_x[i], list_pose_y[i], list_pose_z[i]);
        point_2 = Eigen::Vector3f(list_pose_x[i + 1], list_pose_y[i + 1], list_pose_z[i + 1]);
        total_distance = total_distance + (point_2 - point_1).norm();
    }
    int interp1_final_size_ = total_distance / _d_between_wps;
    // CreatePathInterp1
    std::vector<double> interp1_list_x, interp1_list_y, interp1_list_z, interp1_list_t;
    interp1_list_x = interpWaypointList(list_pose_x, interp1_final_size_);
    interp1_list_y = interpWaypointList(list_pose_y, interp1_final_size_);
    interp1_list_z = interpWaypointList(list_pose_z, interp1_final_size_);
    interp1_list_t = interpWaypointList(list_pose_t, interp1_final_size_);
    nav_msgs::Path out_path_;
    std::vector<geometry_msgs::PoseStamped> poses(interp1_final_size_);
    for (int i = 0; i < interp1_list_x.size(); i++) {
        poses.at(i).pose.position.x = interp1_list_x[i];
        poses.at(i).pose.position.y = interp1_list_y[i];
        poses.at(i).pose.position.z = interp1_list_z[i];
        poses.at(i).pose.orientation.x = 0;
        poses.at(i).pose.orientation.y = 0;
        poses.at(i).pose.orientation.z = 0;
        poses.at(i).pose.orientation.w = 1;
        poses.at(i).header.stamp.fromSec(interp1_list_t[i]);
    }
    out_path_.poses = poses;

    return out_path_;
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
    std::vector<geometry_msgs::Polygon> vec_obstacles;
    vec_obstacles.push_back(polygon_);
    geometry_msgs::Polygon grid_borders;
    geometry_msgs::Point32 temp_point;
    temp_point.x = x_min_;
    temp_point.y = y_min_;
    grid_borders.points.push_back(temp_point);
    grid_borders.points.push_back(temp_point);
    temp_point.x = x_max_;
    temp_point.y = y_min_;
    grid_borders.points.push_back(temp_point);
    grid_borders.points.push_back(temp_point);
    temp_point.x = x_max_;
    temp_point.y = y_max_;
    grid_borders.points.push_back(temp_point);
    grid_borders.points.push_back(temp_point);
    temp_point.x = x_min_;
    temp_point.y = y_max_;
    grid_borders.points.push_back(temp_point);
    grid_borders.points.push_back(temp_point);
    temp_point.x = x_min_;
    temp_point.y = y_min_;
    grid_borders.points.push_back(temp_point);
    grid_borders.points.push_back(temp_point);
    // Call path planner A*
    int max_grid_side;
    if (x_max_ - x_min_ >= y_max_ - y_min_) max_grid_side = x_max_ - x_min_;
    else max_grid_side = y_max_ - y_min_;
    grvc::PathPlanner path_planner(vec_obstacles, grid_borders, max_grid_side);
    std::vector<geometry_msgs::Point> a_star_getpath = path_planner.getPath(init_astar_point_, goal_astar_point_);
    nav_msgs::Path a_star_path_res = createPathFromPlanner(a_star_getpath, init_astar_point_, target_path_.poses.front().pose.position.z);
    // Linear interpolation for Z axis
    std::vector<double> default_z, interp1_z;
    default_z.push_back(init_astar_point_.z);
    default_z.push_back(goal_astar_point_.z);
    interp1_z = interpWaypointList(default_z, a_star_path_res.poses.size() - 1);
    for (int i = 0; i < interp1_z.size(); i++) {
        a_star_path_res.poses.at(i).pose.position.z = interp1_z.at(i);
    }
    a_star_path_res.poses.back().pose.position.z = goal_astar_point_.z;

    return a_star_path_res;
}