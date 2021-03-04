#include <gauss_msgs/CheckConflicts.h>
#include <gauss_msgs/ConflictiveOperation.h>
#include <gauss_msgs/Deconfliction.h>
#include <gauss_msgs/DeconflictionPlan.h>
#include <gauss_msgs/Threat.h>
#include <gauss_msgs/Waypoint.h>
#include <ros/ros.h>
#include <tactical_deconfliction/path_finder.h>
#include <Eigen/Eigen>

using namespace std;

// Class definition
class ConflictSolver {
   public:
    ConflictSolver();

   private:
    // Topic Callbacks

    // Service Callbacks
    bool deconflictCB(gauss_msgs::Deconfliction::Request &req, gauss_msgs::Deconfliction::Response &res);

    // Auxilary methods
    geometry_msgs::Point findInitAStarPoint(geometry_msgs::Polygon &_polygon, nav_msgs::Path &_path, int &_init_astar_pos);
    geometry_msgs::Point findGoalAStarPoint(geometry_msgs::Polygon &_polygon, nav_msgs::Path &_path, int &_goal_astar_pos);
    int pnpoly(int nvert, std::vector<float> &vertx, std::vector<float> &verty, float testx, float testy);
    std::vector<double> findGridBorders(geometry_msgs::Polygon &_polygon, nav_msgs::Path &_path, geometry_msgs::Point _init_point, geometry_msgs::Point _goal_point);
    geometry_msgs::Polygon circleToPolygon(double &_x, double &_y, double &_radius, double _nVertices = 9);
    gauss_msgs::Waypoint intersectingPoint(gauss_msgs::Waypoint &_p1, gauss_msgs::Waypoint &_p2, geometry_msgs::Polygon &_polygon);
    std::pair<std::vector<double>, double> getCoordinatesAndDistance(double _x0, double _y0, double _x1, double _y1, double _x2, double _y2);
    double pathDistance(gauss_msgs::DeconflictionPlan &_wp_list);
    double pointsDistance(gauss_msgs::Waypoint &_p1, gauss_msgs::Waypoint &_p2);
    double minDistanceToGeofence(std::vector<gauss_msgs::Waypoint> &_wp_list, geometry_msgs::Polygon &_polygon);
    double calculateRiskiness(gauss_msgs::DeconflictionPlan newplan);
    void decreasePolygon(const geometry_msgs::Polygon &p, double thickness, geometry_msgs::Polygon &q);
    double signedArea(const geometry_msgs::Polygon &p);

    // Auxilary variables
    double rate_;
    double minDist_, dT_;
    double minX_, maxX_, minY_, maxY_, minZ_, maxZ_;
    ros::NodeHandle nh_;

    // Subscribers

    // Publisher
    ros::Publisher pub_sol_1_;
    ros::Publisher pub_sol_2_;
    ros::Publisher pub_sol_3_;
    ros::Publisher pub_sol_4_;
    ros::Publisher pub_sol_5_;
    ros::Publisher pub_sol_6_;
    ros::Publisher pub_sol_7_;
    ros::Publisher pub_sol_8_;

    // Timer

    // Server
    ros::ServiceServer deconflict_server_;

    // Clients
    ros::ServiceClient check_client_;
};

// TacticalDeconfliction Constructor
ConflictSolver::ConflictSolver() {
    // Read parameters
    nh_.param("/tactical_deconfliction/safetyDistance", minDist_, 10.0);
    nh_.param("/tactical_deconfliction/dT", dT_, 15.0);
    nh_.param("/tactical_deconfliction/minX", minX_, -400.0);
    nh_.param("/tactical_deconfliction/minY", minY_, 0.0);
    nh_.param("/tactical_deconfliction/minZ", minZ_, 0.0);
    nh_.param("/tactical_deconfliction/maxX", maxX_, 0.0);
    nh_.param("/tactical_deconfliction/maxY", maxY_, 400.0);
    nh_.param("/tactical_deconfliction/maxZ", maxZ_, 300.0);


    // Initialization
    // dT_ = 1.0 / rate_;

    // Publish
    pub_sol_1_ = nh_.advertise<nav_msgs::Path>("/sol1", 1);
    pub_sol_2_ = nh_.advertise<nav_msgs::Path>("/sol2", 1);
    pub_sol_3_ = nh_.advertise<nav_msgs::Path>("/sol3", 1);
    pub_sol_4_ = nh_.advertise<nav_msgs::Path>("/sol4", 1);
    pub_sol_5_ = nh_.advertise<nav_msgs::Path>("/sol5", 1);
    pub_sol_6_ = nh_.advertise<nav_msgs::Path>("/sol6", 1);
    pub_sol_7_ = nh_.advertise<nav_msgs::Path>("/sol7", 1);
    pub_sol_8_ = nh_.advertise<nav_msgs::Path>("/sol8", 1);

    // Subscribe

    // Server
    deconflict_server_ = nh_.advertiseService("/gauss/tactical_deconfliction", &ConflictSolver::deconflictCB, this);

    // Cient
    check_client_ = nh_.serviceClient<gauss_msgs::CheckConflicts>("/gauss/check_conflicts");

    ROS_INFO("[Deconfliction] Started ConflictSolver node!");
}

int ConflictSolver::pnpoly(int nvert, std::vector<float> &vertx, std::vector<float> &verty, float testx, float testy) {
    int i, j, c = 0;
    for (i = 0, j = nvert - 1; i < nvert; j = i++) {
        if (((verty.at(i) > testy) != (verty.at(j) > testy)) &&
            (testx < (vertx.at(j) - vertx.at(i)) * (testy - verty.at(i)) / (verty.at(j) - verty.at(i)) + vertx.at(i)))
            c = !c;
    }
    return c;
}

double ConflictSolver::signedArea(const geometry_msgs::Polygon &p) {
    double A = 0;
    //========================================================//
    // Assumes:                                               //
    //    N+1 vertices:   p[0], p[1], ... , p[N-1], p[N]      //
    //    Closed polygon: p[0] = p[N]                         //
    // Returns:                                               //
    //    Signed area: +ve if anticlockwise, -ve if clockwise //
    //========================================================//
    int N = p.points.size() - 1;
    for (int i = 0; i < N; i++) A += p.points.at(i).x * p.points.at(i + 1).y - p.points.at(i + 1).x * p.points.at(i).y;
    A *= 0.5;
    return A;
}

void ConflictSolver::decreasePolygon(const geometry_msgs::Polygon &p, double thickness, geometry_msgs::Polygon &q) {
    //=====================================================//
    // Assumes:                                            //
    //    N+1 vertices:   p[0], p[1], ... , p[N-1], p[N]   //
    //    Closed polygon: p[0] = p[N]                      //
    //    No zero-length sides                             //
    // Returns (by reference, as a parameter):             //
    //    Internal poly:  q[0], q[1], ... , q[N-1], q[N]   //
    //=====================================================//
    int N = p.points.size() - 1;
    q.points.resize(N + 1);
    double a, b, A, B, d, cross;
    double displacement = thickness;
    if (signedArea(p) < 0) displacement = -displacement;  // Detects clockwise order
    // Unit vector (a,b) along last edge
    a = p.points.at(N).x - p.points.at(N - 1).x;
    b = p.points.at(N).y - p.points.at(N - 1).y;
    d = sqrt(a * a + b * b);
    a /= d;
    b /= d;
    for (int i = 0; i < N; i++) {  // Loop round the polygon, dealing with successive intersections of lines
        // Unit vector (A,B) along previous edge
        A = a;
        B = b;
        // Unit vector (a,b) along next edge
        a = p.points.at(i + 1).x - p.points.at(i).x;
        b = p.points.at(i + 1).y - p.points.at(i).y;
        d = sqrt(a * a + b * b);
        a /= d;
        b /= d;
        // New vertex
        cross = A * b - a * B;
        const double SMALL = 1.0e-10;
        if (abs(cross) < SMALL) {  // Degenerate cases: 0 or 180 degrees at vertex
            q.points.at(i).x = p.points.at(i).x - displacement * b;
            q.points.at(i).y = p.points.at(i).y + displacement * a;
        } else {  // Usual case
            q.points.at(i).x = p.points.at(i).x + displacement * (a - A) / cross;
            q.points.at(i).y = p.points.at(i).y + displacement * (b - B) / cross;
        }
    }
    // Close the inside polygon
    q.points.at(N) = q.points.at(0);
}

geometry_msgs::Point ConflictSolver::findInitAStarPoint(geometry_msgs::Polygon &_polygon, nav_msgs::Path &_path, int &_init_astar_pos) {
    geometry_msgs::Point out_point;
    std::vector<float> vert_x, vert_y;
    for (int i = 0; i < _polygon.points.size(); i++) {
        vert_x.push_back(_polygon.points.at(i).x);
        vert_y.push_back(_polygon.points.at(i).y);
    }
    vert_x.push_back(_polygon.points.front().x);
    vert_y.push_back(_polygon.points.front().y);
    for (int i = 0; i < _path.poses.size(); i++) {
        if (pnpoly(vert_x.size(), vert_x, vert_y, _path.poses.at(i).pose.position.x, _path.poses.at(i).pose.position.y)) {
            out_point.x = _path.poses.at(i - 1).pose.position.x;
            out_point.y = _path.poses.at(i - 1).pose.position.y;
            out_point.z = _path.poses.at(i - 1).pose.position.z;
            _init_astar_pos = i - 1;
            break;
        }
    }
    return out_point;
}

geometry_msgs::Point ConflictSolver::findGoalAStarPoint(geometry_msgs::Polygon &_polygon, nav_msgs::Path &_path, int &_goal_astar_pos) {
    bool flag1 = false;
    bool flag2 = false;
    geometry_msgs::Point out_point;
    std::vector<float> vert_x, vert_y;
    for (int i = 0; i < _polygon.points.size(); i++) {
        vert_x.push_back(_polygon.points.at(i).x);
        vert_y.push_back(_polygon.points.at(i).y);
    }
    vert_x.push_back(_polygon.points.front().x);
    vert_y.push_back(_polygon.points.front().y);
    for (int i = 0; i < _path.poses.size(); i++) {
        bool in_obstacle = pnpoly(vert_x.size(), vert_x, vert_y, _path.poses.at(i).pose.position.x, _path.poses.at(i).pose.position.y);
        if (in_obstacle && !flag1 && !flag2) flag1 = true;
        if (!in_obstacle && flag1 && !flag2) flag2 = true;
        if (!in_obstacle && flag1 && flag2) {
            out_point.x = _path.poses.at(i).pose.position.x;
            out_point.y = _path.poses.at(i).pose.position.y;
            out_point.z = _path.poses.at(i).pose.position.z;
            _goal_astar_pos = i;
            break;
        }
    }

    if (out_point.x == 0.0 && out_point.y == 0.0) {
        out_point.x = _path.poses.back().pose.position.x;
        out_point.y = _path.poses.back().pose.position.y;
        out_point.z = _path.poses.back().pose.position.z;
    }

    return out_point;
}

std::vector<double> ConflictSolver::findGridBorders(geometry_msgs::Polygon &_polygon, nav_msgs::Path &_path, geometry_msgs::Point _init_point, geometry_msgs::Point _goal_point) {
    geometry_msgs::Point obs_min, obs_max, out_point;
    std::vector<float> vert_x, vert_y;
    for (int i = 0; i < _polygon.points.size(); i++) {
        vert_x.push_back(_polygon.points.at(i).x);
        vert_y.push_back(_polygon.points.at(i).y);
    }
    vert_x.push_back(_polygon.points.front().x);
    vert_y.push_back(_polygon.points.front().y);
    obs_min.x = *std::min_element(vert_x.begin(), vert_x.end());
    obs_min.y = *std::min_element(vert_y.begin(), vert_y.end());
    obs_max.x = *std::max_element(vert_x.begin(), vert_x.end());
    obs_max.y = *std::max_element(vert_y.begin(), vert_y.end());

    std::vector<double> out_grid_borders, temp_x, temp_y;
    temp_x.push_back(_init_point.x);
    temp_x.push_back(_goal_point.x);
    temp_y.push_back(_init_point.y);
    temp_y.push_back(_goal_point.y);
    double min_x = *std::min_element(temp_x.begin(), temp_x.end());
    double min_y = *std::min_element(temp_y.begin(), temp_y.end());
    double max_x = *std::max_element(temp_x.begin(), temp_x.end());
    double max_y = *std::max_element(temp_y.begin(), temp_y.end());

    while (min_x >= obs_min.x || min_y >= obs_min.y || max_x <= obs_max.x || max_y <= obs_max.y) {
        if (min_x >= obs_min.x) min_x = min_x - 1.0;
        if (min_y >= obs_min.y) min_y = min_y - 1.0;
        if (max_x <= obs_max.x) max_x = max_x + 1.0;
        if (max_y <= obs_max.y) max_y = max_y + 1.0;
    }

    out_grid_borders.push_back(min_x);
    out_grid_borders.push_back(min_y);
    out_grid_borders.push_back(max_x);
    out_grid_borders.push_back(max_y);

    return out_grid_borders;
}

geometry_msgs::Polygon ConflictSolver::circleToPolygon(double &_x, double &_y, double &_radius, double _nVertices) {
    geometry_msgs::Polygon out_polygon;
    Eigen::Vector2d centerToVertex(_radius, 0.0), centerToVertexTemp;
    for (int i = 0; i < _nVertices; i++) {
        double theta = i * 2 * M_PI / (_nVertices - 1);
        Eigen::Rotation2D<double> rot2d(theta);
        centerToVertexTemp = rot2d.toRotationMatrix() * centerToVertex;
        geometry_msgs::Point32 temp_point;
        temp_point.x = _x + centerToVertexTemp[0];
        temp_point.y = _y + centerToVertexTemp[1];
        out_polygon.points.push_back(temp_point);
    }

    return out_polygon;
}

template <typename KeyType, typename ValueType>
std::pair<KeyType, ValueType> get_min(const std::map<KeyType, ValueType> &x) {
    using pairtype = std::pair<KeyType, ValueType>;
    return *std::min_element(x.begin(), x.end(), [](const pairtype &p1, const pairtype &p2) {
        return p1.second < p2.second;
    });
}

std::pair<std::vector<double>, double> ConflictSolver::getCoordinatesAndDistance(double _x0, double _y0, double _x1, double _y1, double _x2, double _y2) {
    std::vector<double> coordinates;
    double distance = 1000000;
    std::vector<double> point_xs, point_ys;
    point_xs.push_back(_x1);
    point_xs.push_back(_x2);
    point_ys.push_back(_y1);
    point_ys.push_back(_y2);
    // Get abc -> ax + by + c = 0
    double a = _y2 - _y1;
    double b = _x1 - _x2;
    double c = -(a * _x1 + b * _y1);
    // get XY outpoint
    coordinates.push_back((b * (b * _x0 - a * _y0) - a * c) / (a * a + b * b));
    coordinates.push_back((a * (a * _y0 - b * _x0) - b * c) / (a * a + b * b));
    // check if xy is between p1 and p2
    double max_x = *std::max_element(point_xs.begin(), point_xs.end());
    double min_x = *std::min_element(point_xs.begin(), point_xs.end());
    double max_y = *std::max_element(point_ys.begin(), point_ys.end());
    double min_y = *std::min_element(point_ys.begin(), point_ys.end());
    if (max_x >= coordinates.front() && coordinates.front() >= min_x && max_y >= coordinates.back() && coordinates.back() >= min_y) {
        // get distance
        distance = (double)(abs((_y2 - _y1) * _x0 - (_x2 - _x1) * _y0 + _x2 * _y1 - _y2 * _x1) /
                            sqrt(pow(_y2 - _y1, 2) + pow(_x2 - _x1, 2)));
    }
    // if (distance < 1.0) distance = 1000000;
    // std::cout << max_x << ">" << coordinates.front() << ">" << min_x << " | " << max_y << ">" << coordinates.back() << ">" << min_y << " | d = " << distance << std::endl;
    return std::make_pair(coordinates, distance);
}

gauss_msgs::Waypoint ConflictSolver::intersectingPoint(gauss_msgs::Waypoint &_p1, gauss_msgs::Waypoint &_p2, geometry_msgs::Polygon &_polygon) {
    gauss_msgs::Waypoint out_point;
    // std::cout << "p1" << std::endl << _p1 << std::endl << "p2" << std::endl << _p2 << std::endl;
    for (int i = 0; i < _polygon.points.size() - 1; i++) {
        double dx1, dy1, dx2, dy2, m1, c1, m2, c2, temp_x, temp_y;
        dx1 = _p2.x - _p1.x;
        dy1 = _p2.y - _p1.y;
        m1 = dy1 / dx1;
        c1 = _p1.y - m1 * _p1.x;

        dx2 = _polygon.points.at(i + 1).x - _polygon.points.at(i).x;
        dy2 = _polygon.points.at(i + 1).y - _polygon.points.at(i).y;
        m2 = dy2 / dx2;
        c2 = _polygon.points.at(i + 1).y - m2 * _polygon.points.at(i + 1).x;

        std::vector<double> point_xs, point_ys, point_xs1, point_ys1;
        point_xs.push_back(_polygon.points.at(i).x);
        point_xs.push_back(_polygon.points.at(i + 1).x);
        point_ys.push_back(_polygon.points.at(i).y);
        point_ys.push_back(_polygon.points.at(i + 1).y);
        double max_x = *std::max_element(point_xs.begin(), point_xs.end());
        double min_x = *std::min_element(point_xs.begin(), point_xs.end());
        double max_y = *std::max_element(point_ys.begin(), point_ys.end());
        double min_y = *std::min_element(point_ys.begin(), point_ys.end());

        point_xs1.push_back(_p1.x);
        point_xs1.push_back(_p2.x);
        point_ys1.push_back(_p1.y);
        point_ys1.push_back(_p2.y);
        double max_x1 = *std::max_element(point_xs1.begin(), point_xs1.end());
        double min_x1 = *std::min_element(point_xs1.begin(), point_xs1.end());
        double max_y1 = *std::max_element(point_ys1.begin(), point_ys1.end());
        double min_y1 = *std::min_element(point_ys1.begin(), point_ys1.end());

        // std::cout << _polygon.points.at(i) << std::endl << _polygon.points.at(i+1) << std::endl;
        if ((m1 - m2) != 0) {
            temp_x = (c2 - c1) / (m1 - m2);
            temp_y = m1 * temp_x + c1;
            // std::cout << "[1] " << max_x << " > " << temp_x << " > " << min_x << " | " << max_y << " > " << temp_y << " > " << min_y << std::endl;
            if (max_x >= temp_x && temp_x >= min_x && max_y >= temp_y && temp_y >= min_y) {
                // std::cout << "[2] " << max_x1 << " > " << temp_x << " > " << min_x1 << " | " << max_y1 << " > " << temp_y << " > " << min_y1 << std::endl;
                if (max_x1 >= temp_x && temp_x >= min_x1 && max_y1 >= temp_y && temp_y >= min_y1) {
                    out_point.x = temp_x;
                    out_point.y = temp_y;
                    // std::cout << out_point << std::endl;
                    break;
                }
            }
        }
        // std::cout << " -- " << std::endl;
    }
    // std::cout << " -- -- -- " << std::endl;

    return out_point;
}

double ConflictSolver::pathDistance(gauss_msgs::DeconflictionPlan &_wp_list) {
    double distance = 0;
    for (int i = 0; i < _wp_list.waypoint_list.size() - 1; i++) {
        Eigen::Vector3f p1 = Eigen::Vector3f(_wp_list.waypoint_list.at(i).x, _wp_list.waypoint_list.at(i).y, _wp_list.waypoint_list.at(i).z);
        Eigen::Vector3f p2 = Eigen::Vector3f(_wp_list.waypoint_list.at(i + 1).x, _wp_list.waypoint_list.at(i + 1).y, _wp_list.waypoint_list.at(i + 1).z);
        distance = distance + (p2 - p1).norm();
    }

    return distance;
}

double ConflictSolver::pointsDistance(gauss_msgs::Waypoint &_p1, gauss_msgs::Waypoint &_p2) {
    double distance = 0;
    Eigen::Vector2f p1 = Eigen::Vector2f(_p1.x, _p1.y);
    Eigen::Vector2f p2 = Eigen::Vector2f(_p2.x, _p2.y);
    distance = (p2 - p1).norm();

    return distance;
}

double ConflictSolver::minDistanceToGeofence(std::vector<gauss_msgs::Waypoint> &_wp_list, geometry_msgs::Polygon &_polygon) {
    double min_distance = 1000000;
    for (auto wp : _wp_list) {
        Eigen::Vector2f wp_p = Eigen::Vector2f(wp.x, wp.y);
        std::map<std::vector<double>, double> point_and_distance;
        for (int i = 0; i < _polygon.points.size() - 1; i++) {
            Eigen::Vector2f vertex_p = Eigen::Vector2f(_polygon.points.at(i).x, _polygon.points.at(i).y);
            double temp_distance = (vertex_p - wp_p).norm();
            if (temp_distance < min_distance) min_distance = temp_distance;
            point_and_distance.insert(getCoordinatesAndDistance(wp.x, wp.y,
                                                                _polygon.points.at(i).x, _polygon.points.at(i).y,
                                                                _polygon.points.at(i + 1).x, _polygon.points.at(i + 1).y));
        }
        Eigen::Vector2f vertex_p = Eigen::Vector2f(_polygon.points.back().x, _polygon.points.back().y);
        double temp_distance = (vertex_p - wp_p).norm();
        if (temp_distance < min_distance) min_distance = temp_distance;
        point_and_distance.insert(getCoordinatesAndDistance(wp.x, wp.y,
                                                            _polygon.points.front().x, _polygon.points.front().y,
                                                            _polygon.points.back().x, _polygon.points.back().y));

        auto min_distance_coordinate = get_min(point_and_distance);
        // Check != 0 (if 0 the path collide with the geofence)
        if (min_distance_coordinate.second < min_distance && min_distance_coordinate.second != 0) min_distance = min_distance_coordinate.second;
    }

    return min_distance;
}

double ConflictSolver::calculateRiskiness(gauss_msgs::DeconflictionPlan _newplan) {
    nav_msgs::Path temp_path;
    for (auto wp : _newplan.waypoint_list) {
        geometry_msgs::PoseStamped temp_wp_stamped;
        temp_wp_stamped.pose.position.x = wp.x;
        temp_wp_stamped.pose.position.y = wp.y;
        temp_wp_stamped.pose.position.z = wp.z;
        temp_wp_stamped.header.stamp = wp.stamp;
        temp_path.poses.push_back(temp_wp_stamped);
    }

    PathFinder path_finder;
    nav_msgs::Path interp_path = path_finder.generatePath(temp_path, 0.0, 1.0);
    gauss_msgs::CheckConflicts check_conflict;
    check_conflict.request.uav_id = _newplan.uav_id;
    for (auto wp : interp_path.poses) {
        gauss_msgs::Waypoint temp_wp;
        temp_wp.x = wp.pose.position.x;
        temp_wp.y = wp.pose.position.y;
        temp_wp.z = wp.pose.position.z;
        temp_wp.stamp = wp.header.stamp;
        check_conflict.request.deconflicted_wp.push_back(temp_wp);
    }

    if (!check_client_.call(check_conflict) || !check_conflict.response.success)
        ROS_ERROR("Failed checking conflicts");

    return 100 * check_conflict.response.threats.size() / interp_path.poses.size();
}

// deconflictCB callback
bool ConflictSolver::deconflictCB(gauss_msgs::Deconfliction::Request &req, gauss_msgs::Deconfliction::Response &res) {
    ROS_INFO("[Deconfliction] New conflict received! [%d %d]", req.threat.threat_id, req.threat.threat_type);
    //Deconfliction
    if (req.tactical) {
        gauss_msgs::Threat conflict;
        conflict = req.threat;
        std::vector<gauss_msgs::ConflictiveOperation> conflictive_operations;
        for (int i = 0; i < req.threat.uav_ids.size(); i++) {
            for (int j = 0; j < req.operations.size(); j++) {
                if (req.threat.uav_ids.at(i) == req.operations.at(j).uav_id) {
                    conflictive_operations.push_back(req.operations.at(j));
                }
            }
        }

        if (req.threat.threat_type == req.threat.LOSS_OF_SEPARATION) {
            gauss_msgs::WaypointList traj1 = conflictive_operations.at(0).estimated_trajectory;
            gauss_msgs::WaypointList traj2 = conflictive_operations.at(1).estimated_trajectory;
            gauss_msgs::Waypoint wp1, wp2;
            int j = 0;
            while (abs(traj1.waypoints.at(j).stamp.toSec() - conflict.times.at(0).toSec()) >= dT_ / 2)
                j++;
            wp1 = traj1.waypoints.at(j);
            int k = 0;
            while (abs(traj2.waypoints.at(k).stamp.toSec() - conflict.times.at(1).toSec()) >= dT_ / 2)
                k++;
            wp2 = traj2.waypoints.at(k);
            
            double minDistAux = max(minDist_, conflictive_operations.at(0).operational_volume + conflictive_operations.at(1).operational_volume);

            double dist_vert = abs(wp2.z - wp1.z);
            double dist_hor = sqrt(pow(wp2.x - wp1.x, 2) + pow(wp2.y - wp1.y, 2));
            double sep_vert = sqrt(pow(minDistAux, 2) - pow(dist_hor, 2));
            double sep_hor = sqrt(pow(minDistAux, 2) - pow(dist_vert, 2));

            gauss_msgs::DeconflictionPlan newplan;
            gauss_msgs::Waypoint newwp;
            newplan.maneuver_type = 8;  // defined in https://docs.google.com/document/d/1R5jWSw4pyPyplHwwrQCDprIUkMimVz-yR8u_t19ooOA/edit

            if (req.threat.priority_ops.at(0) >= req.threat.priority_ops.at(1)) {
                newplan.uav_id = req.threat.uav_ids.at(1);
                //Above
                if (k > 0)
                    newplan.waypoint_list.push_back(traj2.waypoints.at(k - 1));
                newwp = traj2.waypoints.at(k);
                newwp.z = traj1.waypoints.at(j).z + sep_vert;
                if (newwp.z <= maxZ_ && newwp.z >= minZ_) {
                    newplan.waypoint_list.push_back(newwp);
                    if (k < traj2.waypoints.size()){
                        newplan.waypoint_list.push_back(traj2.waypoints.at(k + 1));
                    }
                    newplan.cost = pathDistance(newplan);
                    newplan.riskiness = calculateRiskiness(newplan);
                    res.deconfliction_plans.push_back(newplan);
                }
                //Below
                newplan.waypoint_list.clear();
                if (k > 0)
                    newplan.waypoint_list.push_back(traj2.waypoints.at(k - 1));
                newwp = traj2.waypoints.at(k);
                newwp.z = traj1.waypoints.at(j).z - sep_vert;
                if (newwp.z <= maxZ_ && newwp.z >= minZ_) {
                    newplan.waypoint_list.push_back(newwp);
                    if (k < traj2.waypoints.size()){
                        newplan.waypoint_list.push_back(traj2.waypoints.at(k + 1));
                    }
                    newplan.cost = pathDistance(newplan);
                    newplan.riskiness = calculateRiskiness(newplan);
                    res.deconfliction_plans.push_back(newplan);
                }
                //On the right
                newplan.waypoint_list.clear();
                if (k > 0)
                    newplan.waypoint_list.push_back(traj2.waypoints.at(k - 1));
                newwp = traj2.waypoints.at(k);
                if (dist_hor == 0) {
                    newwp.x = wp1.x + sep_hor;
                    newwp.y = wp1.y + sep_hor;
                } else {
                    newwp.x = wp1.x + sep_hor * (wp2.x - wp1.x) / dist_hor;
                    newwp.y = wp1.y + sep_hor * (wp2.y - wp1.y) / dist_hor;
                }
                if (newwp.x <= maxX_ && newwp.x >= minX_ && newwp.y <= maxY_ && newwp.y >= minY_) {
                    newplan.waypoint_list.push_back(newwp);
                    if (k < traj2.waypoints.size()){
                        newplan.waypoint_list.push_back(traj2.waypoints.at(k + 1));
                    }
                    newplan.cost = pathDistance(newplan);
                    newplan.riskiness = calculateRiskiness(newplan);
                    res.deconfliction_plans.push_back(newplan);
                }
                //On the left
                newplan.waypoint_list.clear();
                if (k > 0)
                    newplan.waypoint_list.push_back(traj2.waypoints.at(k - 1));
                newwp = traj2.waypoints.at(k);
                if (dist_hor == 0) {
                    newwp.x = wp1.x - sep_hor;
                    newwp.y = wp1.y - sep_hor;
                } else {
                    newwp.x = wp1.x - sep_hor * (wp2.x - wp1.x) / dist_hor;
                    newwp.y = wp1.y - sep_hor * (wp2.y - wp1.y) / dist_hor;
                }
                if (newwp.x <= maxX_ && newwp.x >= minX_ && newwp.y <= maxY_ && newwp.y >= minY_) {
                    newplan.waypoint_list.push_back(newwp);
                    if (k < traj2.waypoints.size()){
                        newplan.waypoint_list.push_back(traj2.waypoints.at(k + 1));
                    }
                    newplan.cost = pathDistance(newplan);
                    newplan.riskiness = calculateRiskiness(newplan);
                    res.deconfliction_plans.push_back(newplan);
                }
            }
            if (req.threat.priority_ops.at(1) >= req.threat.priority_ops.at(0)) {
                newplan.uav_id = req.threat.uav_ids.at(0);
                //Above
                newplan.waypoint_list.clear();
                if (j > 0)
                    newplan.waypoint_list.push_back(traj1.waypoints.at(j - 1));
                newwp = traj1.waypoints.at(j);
                newwp.z = traj2.waypoints.at(k).z + sep_vert;
                if (newwp.z <= maxZ_ && newwp.z >= minZ_) {
                    newplan.waypoint_list.push_back(newwp);
                    if (j < traj1.waypoints.size()){
                        newplan.waypoint_list.push_back(traj1.waypoints.at(j + 1));
                    }
                    newplan.cost = pathDistance(newplan);
                    newplan.riskiness = calculateRiskiness(newplan);
                    res.deconfliction_plans.push_back(newplan);
                }
                //Below
                newplan.waypoint_list.clear();
                if (j > 0)
                    newplan.waypoint_list.push_back(traj1.waypoints.at(j - 1));
                newwp = traj1.waypoints.at(j);
                newwp.z = traj2.waypoints.at(k).z - sep_vert;
                if (newwp.z <= maxZ_ && newwp.z >= minZ_) {
                    newplan.waypoint_list.push_back(newwp);
                    if (j < traj1.waypoints.size()){
                        newplan.waypoint_list.push_back(traj1.waypoints.at(j + 1));
                    }
                    newplan.cost = pathDistance(newplan);
                    newplan.riskiness = calculateRiskiness(newplan);
                    res.deconfliction_plans.push_back(newplan);
                }
                //On the right
                newplan.waypoint_list.clear();
                if (j > 0)
                    newplan.waypoint_list.push_back(traj1.waypoints.at(j - 1));
                newwp = traj1.waypoints.at(j);
                if (dist_hor == 0) {
                    newwp.x = wp2.x + sep_hor;
                    newwp.y = wp2.y + sep_hor;
                } else {
                    newwp.x = wp2.x + sep_hor * (wp1.x - wp2.x) / dist_hor;
                    newwp.y = wp2.y + sep_hor * (wp1.y - wp2.y) / dist_hor;
                }
                if (newwp.x <= maxX_ && newwp.x >= minX_ && newwp.y <= maxY_ && newwp.y >= minY_) {
                    newplan.waypoint_list.push_back(newwp);
                    if (j < traj1.waypoints.size()){
                        newplan.waypoint_list.push_back(traj1.waypoints.at(j + 1));
                    }
                    newplan.cost = pathDistance(newplan);
                    newplan.riskiness = calculateRiskiness(newplan);
                    res.deconfliction_plans.push_back(newplan);
                }
                //On the left
                newplan.waypoint_list.clear();
                if (j > 0)
                    newplan.waypoint_list.push_back(traj1.waypoints.at(j - 1));
                newwp = traj1.waypoints.at(j);
                if (dist_hor == 0) {
                    newwp.x = wp2.x - sep_hor;
                    newwp.y = wp2.y - sep_hor;
                } else {
                    newwp.x = wp2.x - sep_hor * (wp1.x - wp2.x) / dist_hor;
                    newwp.y = wp2.y - sep_hor * (wp1.y - wp2.y) / dist_hor;
                }
                if (newwp.x <= maxX_ && newwp.x >= minX_ && newwp.y <= maxY_ && newwp.y >= minY_) {
                    newplan.waypoint_list.push_back(newwp);
                    if (j < traj1.waypoints.size()){
                        newplan.waypoint_list.push_back(traj1.waypoints.at(j + 1));
                    }
                    newplan.cost = pathDistance(newplan);
                    newplan.riskiness = calculateRiskiness(newplan);
                    res.deconfliction_plans.push_back(newplan);
                }
            }
            res.message = "Conflict solved";
            res.success = true;
        } else if (req.threat.threat_type == req.threat.GEOFENCE_CONFLICT) {
            nav_msgs::Path res_path;
            std::vector<double> res_times;
            for (int i = 0; i < conflictive_operations.front().estimated_trajectory.waypoints.size(); i++) {
                geometry_msgs::PoseStamped temp_pose;
                temp_pose.pose.position.x = conflictive_operations.front().estimated_trajectory.waypoints.at(i).x;
                temp_pose.pose.position.y = conflictive_operations.front().estimated_trajectory.waypoints.at(i).y;
                temp_pose.pose.position.z = conflictive_operations.front().estimated_trajectory.waypoints.at(i).z;
                res_path.poses.push_back(temp_pose);
                res_times.push_back(conflictive_operations.front().estimated_trajectory.waypoints.at(i).stamp.toSec());
            }
            std::vector<gauss_msgs::Geofence> geofences;
            for (int i = 0; i < req.threat.geofence_ids.size(); i++) {
                for (int j = 0; j < req.geofences.size(); j++) {
                    if (req.threat.uav_ids.at(i) == req.operations.at(j).uav_id) {
                        geofences.push_back(req.geofences.at(j));
                    }
                }
            }
            geometry_msgs::Polygon res_polygon;
            if (geofences.front().cylinder_shape) {
                res_polygon = circleToPolygon(geofences.front().circle.x_center,
                                              geofences.front().circle.y_center,
                                              geofences.front().circle.radius);
            } else {
                for (int i = 0; i < geofences.front().polygon.x.size(); i++) {
                    geometry_msgs::Point32 temp_points;
                    temp_points.x = geofences.front().polygon.x.at(i);
                    temp_points.y = geofences.front().polygon.y.at(i);
                    res_polygon.points.push_back(temp_points);
                }
            }
            // [1] Ruta a mi destino evitando una geofence
            geometry_msgs::Point init_astar_point, goal_astar_point, min_grid_point, max_grid_point;
            int init_astar_pos, goal_astar_pos;
            geometry_msgs::Polygon polygon_test_input = res_polygon;
            if (!geofences.front().cylinder_shape) {
                polygon_test_input.points.push_back(polygon_test_input.points.front());
            }
            geometry_msgs::Polygon polygon_test_output;
            decreasePolygon(polygon_test_input, -conflictive_operations.front().operational_volume, polygon_test_output); 
            if (!geofences.front().cylinder_shape) {
                polygon_test_output.points.pop_back();
            }
            init_astar_point = findInitAStarPoint(polygon_test_output, res_path, init_astar_pos);
            goal_astar_point = findGoalAStarPoint(polygon_test_output, res_path, goal_astar_pos);
            std::vector<double> grid_borders = findGridBorders(polygon_test_output, res_path, init_astar_point, goal_astar_point);
            min_grid_point.x = minX_;
            min_grid_point.y = minY_;
            max_grid_point.x = maxX_;
            max_grid_point.y = maxY_;
            PathFinder path_finder(res_path, init_astar_point, goal_astar_point, polygon_test_output, min_grid_point, max_grid_point);
            nav_msgs::Path a_star_path_res = path_finder.findNewPath();
            std::vector<double> interp_times, a_star_times_res;
            interp_times.push_back(res_times.at(init_astar_pos));
            interp_times.push_back(res_times.at(goal_astar_pos));
            a_star_times_res = path_finder.interpWaypointList(interp_times, a_star_path_res.poses.size() - 1);
            a_star_times_res.push_back(res_times.at(goal_astar_pos));
            // Solutions of conflict solver are a_star_path_res and a_star_times_res
            gauss_msgs::Waypoint temp_wp;
            gauss_msgs::DeconflictionPlan temp_wp_list;
            for (int i = 0; i < a_star_path_res.poses.size(); i++) {
                temp_wp.x = a_star_path_res.poses.at(i).pose.position.x;
                temp_wp.y = a_star_path_res.poses.at(i).pose.position.y;
                temp_wp.z = a_star_path_res.poses.at(i).pose.position.z;
                temp_wp.stamp = ros::Time(a_star_times_res.at(i));
                temp_wp_list.waypoint_list.push_back(temp_wp);
            }
            temp_wp_list.maneuver_type = 1;
            temp_wp_list.cost = pathDistance(temp_wp_list);
            temp_wp_list.riskiness = minDistanceToGeofence(temp_wp_list.waypoint_list, res_polygon);
            temp_wp_list.uav_id = req.threat.uav_ids.front();
            res.deconfliction_plans.push_back(temp_wp_list);
            // [3] Ruta que me manda devuelta a casa
            temp_wp_list.waypoint_list.clear();
            temp_wp.x = conflictive_operations.front().estimated_trajectory.waypoints.front().x;
            temp_wp.y = conflictive_operations.front().estimated_trajectory.waypoints.front().y;
            temp_wp.z = conflictive_operations.front().estimated_trajectory.waypoints.front().z;
            temp_wp.stamp = conflictive_operations.front().estimated_trajectory.waypoints.front().stamp;
            temp_wp_list.waypoint_list.push_back(temp_wp);
            temp_wp.x = conflictive_operations.front().flight_plan.waypoints.front().x;
            temp_wp.y = conflictive_operations.front().flight_plan.waypoints.front().y;
            temp_wp.z = conflictive_operations.front().flight_plan.waypoints.front().z;
            temp_wp_list.waypoint_list.push_back(temp_wp);
            temp_wp_list.maneuver_type = 3;
            temp_wp_list.cost = pathDistance(temp_wp_list);
            temp_wp_list.riskiness = minDistanceToGeofence(temp_wp_list.waypoint_list, res_polygon);
            temp_wp_list.uav_id = req.threat.uav_ids.front();
            res.deconfliction_plans.push_back(temp_wp_list);

            res.message = "Conflict solved";
            res.success = true;
        } else if (req.threat.threat_type == req.threat.GEOFENCE_INTRUSION) {
            nav_msgs::Path res_path;
            std::vector<double> res_times;
            for (int i = 0; i < conflictive_operations.front().estimated_trajectory.waypoints.size(); i++) {
                geometry_msgs::PoseStamped temp_pose;
                temp_pose.pose.position.x = conflictive_operations.front().estimated_trajectory.waypoints.at(i).x;
                temp_pose.pose.position.y = conflictive_operations.front().estimated_trajectory.waypoints.at(i).y;
                temp_pose.pose.position.z = conflictive_operations.front().estimated_trajectory.waypoints.at(i).z;
                res_path.poses.push_back(temp_pose);
                res_times.push_back(conflictive_operations.front().estimated_trajectory.waypoints.at(i).stamp.toSec());
            }
            std::vector<gauss_msgs::Geofence> geofences;
            for (int i = 0; i < req.threat.geofence_ids.size(); i++) {
                for (int j = 0; j < req.geofences.size(); j++) {
                    if (req.threat.uav_ids.at(i) == req.operations.at(j).uav_id) {
                        geofences.push_back(req.geofences.at(j));
                    }
                }
            }
            geometry_msgs::Polygon res_polygon;
            if (geofences.front().cylinder_shape) {
                res_polygon = circleToPolygon(geofences.front().circle.x_center,
                                              geofences.front().circle.y_center,
                                              geofences.front().circle.radius);
            } else {
                for (int i = 0; i < geofences.front().polygon.x.size(); i++) {
                    geometry_msgs::Point32 temp_points;
                    temp_points.x = geofences.front().polygon.x.at(i);
                    temp_points.y = geofences.front().polygon.y.at(i);
                    res_polygon.points.push_back(temp_points);
                }
            }
            // [6] Ruta a mi destino saliendo lo antes posible de la geofence
            // Get min distance to polygon border
            gauss_msgs::Waypoint conflict_point;
            conflict_point.x = conflictive_operations.front().estimated_trajectory.waypoints.front().x;
            conflict_point.y = conflictive_operations.front().estimated_trajectory.waypoints.front().y;
            conflict_point.z = conflictive_operations.front().estimated_trajectory.waypoints.front().z;
            std::map<std::vector<double>, double> point_and_distance;
            for (auto vertex : res_polygon.points) {
                std::vector<double> point;
                point.push_back(vertex.x);
                point.push_back(vertex.y);
                point_and_distance.insert(std::make_pair(point, sqrt(pow(vertex.x - conflict_point.x, 2) +
                                                                     pow(vertex.y - conflict_point.y, 2))));
            }
            for (int i = 0; i < res_polygon.points.size() - 1; i++) {
                point_and_distance.insert(getCoordinatesAndDistance(conflict_point.x, conflict_point.y,
                                                                    res_polygon.points.at(i).x, res_polygon.points.at(i).y,
                                                                    res_polygon.points.at(i + 1).x, res_polygon.points.at(i + 1).y));
            }
            point_and_distance.insert(getCoordinatesAndDistance(conflict_point.x, conflict_point.y,
                                                                res_polygon.points.front().x, res_polygon.points.front().y,
                                                                res_polygon.points.back().x, res_polygon.points.back().y));
            auto min_distance_coordinate = get_min(point_and_distance);
            Eigen::Vector2f p_conflict, p_min_distance, unit_vec, v_out_polygon;
            p_conflict = Eigen::Vector2f(conflict_point.x, conflict_point.y);
            p_min_distance = Eigen::Vector2f(min_distance_coordinate.first.front(), min_distance_coordinate.first.back());
            unit_vec = (p_min_distance - p_conflict) / (p_min_distance - p_conflict).norm();
            double safety_distance = conflictive_operations.front().operational_volume;
            v_out_polygon = unit_vec * safety_distance;
            geometry_msgs::Point init_astar_point, goal_astar_point, min_grid_point, max_grid_point;
            init_astar_point.x = min_distance_coordinate.first.front() + v_out_polygon(0);
            init_astar_point.y = min_distance_coordinate.first.back() + v_out_polygon(1);
            geometry_msgs::Polygon polygon_test_input = res_polygon;
            if (!geofences.front().cylinder_shape) {
                polygon_test_input.points.push_back(polygon_test_input.points.front());
            }
            geometry_msgs::Polygon polygon_test_output;
            decreasePolygon(polygon_test_input, -conflictive_operations.front().operational_volume, polygon_test_output);
            if (!geofences.front().cylinder_shape) {
                polygon_test_output.points.pop_back();
            }
            int init_astar_pos, goal_astar_pos;
            goal_astar_point = findGoalAStarPoint(polygon_test_output, res_path, goal_astar_pos);
            std::vector<double> grid_borders = findGridBorders(polygon_test_output, res_path, init_astar_point, goal_astar_point);
            min_grid_point.x = minX_;
            min_grid_point.y = minY_;
            max_grid_point.x = maxX_;
            max_grid_point.y = maxY_;
            PathFinder path_finder(res_path, init_astar_point, goal_astar_point, polygon_test_output, min_grid_point, max_grid_point);
            nav_msgs::Path a_star_path_res = path_finder.findNewPath();
            std::vector<double> interp_times, a_star_times_res;
            interp_times.push_back(conflictive_operations.front().estimated_trajectory.waypoints.front().stamp.toSec());  // init astar pos time
            interp_times.push_back(res_times.at(goal_astar_pos));
            a_star_times_res = path_finder.interpWaypointList(interp_times, a_star_path_res.poses.size() - 1);
            a_star_times_res.push_back(res_times.at(goal_astar_pos));
            // Solutions of conflict solver are a_star_path_res and a_star_times_res
            gauss_msgs::Waypoint temp_wp;
            gauss_msgs::DeconflictionPlan temp_wp_list;
            temp_wp.x = conflict_point.x;
            temp_wp.y = conflict_point.y;
            temp_wp.z = conflict_point.z;
            temp_wp.stamp = conflictive_operations.front().estimated_trajectory.waypoints.front().stamp;
            temp_wp_list.waypoint_list.push_back(temp_wp);
            for (int i = 0; i < a_star_path_res.poses.size(); i++) {
                temp_wp.x = a_star_path_res.poses.at(i).pose.position.x;
                temp_wp.y = a_star_path_res.poses.at(i).pose.position.y;
                temp_wp.z = a_star_path_res.poses.at(i).pose.position.z;
                temp_wp.stamp = ros::Time(a_star_times_res.at(i));
                temp_wp_list.waypoint_list.push_back(temp_wp);
            }
            temp_wp_list.maneuver_type = 6;
            temp_wp_list.cost = pathDistance(temp_wp_list);
            gauss_msgs::Waypoint intersect_p = intersectingPoint(temp_wp_list.waypoint_list.front(), temp_wp_list.waypoint_list.at(1), res_polygon);
            temp_wp_list.riskiness = pointsDistance(temp_wp_list.waypoint_list.front(), intersect_p);
            temp_wp_list.uav_id = req.threat.uav_ids.front();
            res.deconfliction_plans.push_back(temp_wp_list);
            // [2] Ruta a mi destino por el camino mas corto
            temp_wp_list.waypoint_list.clear();
            temp_wp.x = conflict_point.x;
            temp_wp.y = conflict_point.y;
            temp_wp.z = conflict_point.z;
            temp_wp.stamp = conflictive_operations.front().estimated_trajectory.waypoints.front().stamp;
            temp_wp_list.waypoint_list.push_back(temp_wp);
            temp_wp.x = conflictive_operations.front().flight_plan.waypoints.back().x;
            temp_wp.y = conflictive_operations.front().flight_plan.waypoints.back().y;
            temp_wp.z = conflictive_operations.front().flight_plan.waypoints.back().z;
            temp_wp.stamp = conflictive_operations.front().flight_plan.waypoints.back().stamp;
            temp_wp_list.waypoint_list.push_back(temp_wp);
            temp_wp_list.maneuver_type = 2;
            temp_wp_list.cost = pathDistance(temp_wp_list);
            intersect_p = intersectingPoint(temp_wp_list.waypoint_list.front(), temp_wp_list.waypoint_list.back(), res_polygon);
            temp_wp_list.riskiness = pointsDistance(temp_wp_list.waypoint_list.front(), intersect_p);
            temp_wp_list.uav_id = req.threat.uav_ids.front();
            res.deconfliction_plans.push_back(temp_wp_list);
            // [3] Ruta que me manda de vuelta a casa
            temp_wp_list.waypoint_list.clear();
            temp_wp.x = conflictive_operations.front().estimated_trajectory.waypoints.front().x;
            temp_wp.y = conflictive_operations.front().estimated_trajectory.waypoints.front().y;
            temp_wp.z = conflictive_operations.front().estimated_trajectory.waypoints.front().z;
            temp_wp.stamp = conflictive_operations.front().estimated_trajectory.waypoints.front().stamp;
            temp_wp_list.waypoint_list.push_back(temp_wp);
            temp_wp.x = conflictive_operations.front().flight_plan.waypoints.front().x;
            temp_wp.y = conflictive_operations.front().flight_plan.waypoints.front().y;
            temp_wp.z = conflictive_operations.front().flight_plan.waypoints.front().z;
            temp_wp_list.waypoint_list.push_back(temp_wp);
            temp_wp_list.maneuver_type = 3;
            temp_wp_list.cost = pathDistance(temp_wp_list);
            intersect_p = intersectingPoint(temp_wp_list.waypoint_list.front(), temp_wp_list.waypoint_list.back(), res_polygon);
            temp_wp_list.riskiness = pointsDistance(temp_wp_list.waypoint_list.front(), intersect_p);
            temp_wp_list.uav_id = req.threat.uav_ids.front();
            res.deconfliction_plans.push_back(temp_wp_list);

            res.message = "Conflict solved";
            res.success = true;
        } else if (req.threat.threat_type == req.threat.UAS_OUT_OV) {
            gauss_msgs::Waypoint temp_wp;
            gauss_msgs::DeconflictionPlan temp_wp_list;
            // [9] Ruta para volver lo antes posible al flight geometry y seguir el plan de vuelo.
            std::map<std::vector<double>, double> point_and_distance;
            for (int i = 0; i < conflictive_operations.front().flight_plan.waypoints.size() - 1; i++) {
                point_and_distance.insert(getCoordinatesAndDistance(conflictive_operations.front().estimated_trajectory.waypoints.back().x,
                                                                    conflictive_operations.front().estimated_trajectory.waypoints.back().y,
                                                                    conflictive_operations.front().flight_plan.waypoints.at(i).x,
                                                                    conflictive_operations.front().flight_plan.waypoints.at(i).y,
                                                                    conflictive_operations.front().flight_plan.waypoints.at(i + 1).x,
                                                                    conflictive_operations.front().flight_plan.waypoints.at(i + 1).y));
            }
            auto min_distance_coordinate = get_min(point_and_distance);
            temp_wp.x = min_distance_coordinate.first.front();
            temp_wp.y = min_distance_coordinate.first.back();
            temp_wp.z = conflictive_operations.front().estimated_trajectory.waypoints.back().z;
            temp_wp_list.waypoint_list.push_back(conflictive_operations.front().estimated_trajectory.waypoints.back());
            temp_wp_list.waypoint_list.push_back(temp_wp);
            temp_wp_list.maneuver_type = 9;
            temp_wp_list.cost = pathDistance(temp_wp_list);
            temp_wp_list.riskiness = std::abs(pathDistance(temp_wp_list) - conflictive_operations.front().operational_volume);
            temp_wp_list.uav_id = req.threat.uav_ids.front();
            res.deconfliction_plans.push_back(temp_wp_list);
            double dist_out_ov = temp_wp_list.riskiness;
            double min_dist_to_path = temp_wp_list.cost;
            // [10] Ruta para seguir con el plan de vuelo, da igual que esté más tiempo fuera del Operational Volume.
            temp_wp_list.waypoint_list.clear();
            temp_wp_list.waypoint_list.push_back(conflictive_operations.front().estimated_trajectory.waypoints.back());
            temp_wp_list.waypoint_list.push_back(conflictive_operations.front().flight_plan.waypoints.at(conflictive_operations.front().current_wp + 1));
            temp_wp_list.maneuver_type = 10;
            temp_wp_list.cost = pathDistance(temp_wp_list);
            double alpha = acos(min_dist_to_path / temp_wp_list.cost);
            temp_wp_list.riskiness = std::abs(dist_out_ov / cos(alpha));
            temp_wp_list.uav_id = req.threat.uav_ids.front();
            res.deconfliction_plans.push_back(temp_wp_list);

            res.message = "Conflict solved";
            res.success = true;
        } else if (req.threat.threat_type == req.threat.LACK_OF_BATTERY) {
            for (auto wp_land : conflictive_operations.front().landing_spots.waypoints) {
                gauss_msgs::Waypoint temp_wp;
                gauss_msgs::DeconflictionPlan temp_wp_list;
                temp_wp = conflictive_operations.front().estimated_trajectory.waypoints.front();
                temp_wp_list.waypoint_list.push_back(temp_wp);
                temp_wp_list.maneuver_type = 5;
                double distance = pointsDistance(conflictive_operations.front().estimated_trajectory.waypoints.front(), wp_land);
                temp_wp_list.cost = distance;
                temp_wp_list.waypoint_list.push_back(wp_land);
                temp_wp_list.riskiness = calculateRiskiness(temp_wp_list);
                temp_wp_list.uav_id = req.threat.uav_ids.front();
                res.deconfliction_plans.push_back(temp_wp_list);
            }
            // [5] Ruta que aterrice en un landing spot
            gauss_msgs::Waypoint temp_wp;
            gauss_msgs::DeconflictionPlan temp_wp_list;
            temp_wp_list.maneuver_type = 5;
            temp_wp = conflictive_operations.front().flight_plan.waypoints.front();
            temp_wp_list.waypoint_list.push_back(conflictive_operations.front().estimated_trajectory.waypoints.front());
            double distance = pointsDistance(conflictive_operations.front().estimated_trajectory.waypoints.front(), temp_wp);
            temp_wp_list.cost = distance;
            temp_wp_list.waypoint_list.push_back(temp_wp);
            temp_wp_list.riskiness = calculateRiskiness(temp_wp_list);
            temp_wp_list.uav_id = req.threat.uav_ids.front();
            res.deconfliction_plans.push_back(temp_wp_list);

            res.message = "Conflict solved";
            res.success = true;
        } else if (req.threat.threat_type == req.threat.GNSS_DEGRADATION) {
            for (auto wp_land : conflictive_operations.front().landing_spots.waypoints) {
                gauss_msgs::Waypoint temp_wp;
                gauss_msgs::DeconflictionPlan temp_wp_list;
                temp_wp = conflictive_operations.front().estimated_trajectory.waypoints.front();
                temp_wp_list.waypoint_list.push_back(temp_wp);
                temp_wp_list.maneuver_type = 5;
                double distance = pointsDistance(conflictive_operations.front().estimated_trajectory.waypoints.front(), wp_land);
                temp_wp_list.cost = distance;
                temp_wp_list.waypoint_list.push_back(wp_land);
                temp_wp_list.riskiness = calculateRiskiness(temp_wp_list);
                temp_wp_list.uav_id = req.threat.uav_ids.front();
                res.deconfliction_plans.push_back(temp_wp_list);
            }
            // [5] Ruta que aterrice en un landing spot
            gauss_msgs::Waypoint temp_wp;
            gauss_msgs::DeconflictionPlan temp_wp_list;
            temp_wp_list.maneuver_type = 5;
            temp_wp = conflictive_operations.front().flight_plan.waypoints.front();
            temp_wp_list.waypoint_list.push_back(conflictive_operations.front().estimated_trajectory.waypoints.front());
            double distance = pointsDistance(conflictive_operations.front().estimated_trajectory.waypoints.front(), temp_wp);
            temp_wp_list.cost = distance;
            temp_wp_list.waypoint_list.push_back(temp_wp);
            temp_wp_list.riskiness = calculateRiskiness(temp_wp_list);
            temp_wp_list.uav_id = req.threat.uav_ids.front();
            res.deconfliction_plans.push_back(temp_wp_list);

            res.message = "Conflict solved";
            res.success = true;
        }
    }
    int cont = 1;
    for (auto plan : res.deconfliction_plans){
        nav_msgs::Path wpl;
        wpl.header.frame_id = "map";
        for (int w = 0; w < plan.waypoint_list.size(); w++){
            geometry_msgs::PoseStamped temp_wp;
            temp_wp.pose.position.x = plan.waypoint_list.at(w).x;
            temp_wp.pose.position.y = plan.waypoint_list.at(w).y;
            temp_wp.pose.position.z = plan.waypoint_list.at(w).z;
            wpl.poses.push_back(temp_wp);
        }
        switch (cont)
        {
        case 1:
            pub_sol_1_.publish(wpl);
            break;
        case 2:
            pub_sol_2_.publish(wpl);
            break;
        case 3:
            pub_sol_3_.publish(wpl);
            break;
        case 4:
            pub_sol_4_.publish(wpl);
            break;
        case 5:
            pub_sol_5_.publish(wpl);
            break;
        case 6:
            pub_sol_6_.publish(wpl);
            break;
        case 7:
            pub_sol_7_.publish(wpl);
            break;
        case 8:
            pub_sol_8_.publish(wpl);
            break;
        }
        cont++;
    }

    // for (auto i : res.deconfliction_plans) std::cout << i << "\n ----------- \n";

    return true;
}

// MAIN function
int main(int argc, char *argv[]) {
    ros::init(argc, argv, "conflict_solver");

    // Create a ConflictSolver object
    ConflictSolver *conflict_solver = new ConflictSolver();

    ros::spin();
}
