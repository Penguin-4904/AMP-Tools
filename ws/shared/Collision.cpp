//
// Created by Katrina Braun on 9/12/25.
//

#include "Collision.h"

/// @brief checks if point is inside any of the given obstacles.
bool check_collisions(const Eigen::Vector2d& point, const std::vector<amp::Obstacle2D>& obstacles, const double r) {
    size_t numObstacles = obstacles.size();

    for (int i = 0; i < numObstacles; i++){
        // if point is in an obstacle stop search and return collision as true
        if (collide_object(point, obstacles[i], r)){
            return true;
        }
    }
    return false;
}

/// @brief checks if point is inside obstacle. Assumes convex polygon obstacle.
bool collide_object(const Eigen::Vector2d& point, const amp::Obstacle2D& obstacle, const double r){
    std::vector<Eigen::Vector2d> vertices = obstacle.verticesCCW();
    size_t numVertices = vertices.size();

    for (int i = 0; i < numVertices - 1; i++){
        // check if vectors from point to vertices proceed in CW or CCW direction
        // CW (negative cross product) -> point is not in half plane defined by the vertices
        // Since vertices are defined as CCW -> point outside polygon, return collision as false
        Eigen::Vector2d vec1 = (vertices[i] - point).normalized();
        Eigen::Vector2d vec2 = vertices[i + 1] - point;
        if ((vec1(0) * vec2(1) - vec1(1) * vec2(0)) < -r){ // Add buffer region to prevent validation from indicating a collision due to numerical inaccuracies.
            return false;
        }
    }

    Eigen::Vector2d vec1 = (vertices[numVertices - 1] - point).normalized();
    Eigen::Vector2d vec2 = vertices[0] - point;
    if ((vec1(0) * vec2(1) - vec1(1) * vec2(0)) < -r){
        return false;
    }
    return true;
}

/// @brief checks if manipulator collides with any of the given obstacles for a given state.
bool check_manipulator_collisions(const amp::LinkManipulator2D& manipulator, const amp::ManipulatorState& state, const std::vector<amp::Obstacle2D>& obstacles) {
    size_t numObstacles = obstacles.size();

    // Get joint positions from state to feed to individual object collision checker
    std::vector<Eigen::Vector2d> joints;
    for (int i = 0; i < state.size() + 1; i++){
        joints.push_back(manipulator.getJointLocation(state, i));
    }

    for (int i = 0; i < numObstacles; i++){
        if (collide_chain_object(joints, obstacles[i])){ // Check if joint chain collides with current object
            return true;
        }
    }
    return false;
}

/// @brief checks if the chain of joints/points collides with any of the obstacles
bool check_chain_collisions(const std::vector<Eigen::Vector2d>& points, const std::vector<amp::Obstacle2D>& obstacles){
    size_t numObstacles = obstacles.size();

    for (int i = 0; i < numObstacles; i++){
        if (collide_chain_object(points, obstacles[i])){ // Check if joint chain collides with current object
            return true;
        }
    }
    return false;
}

/// @brief checks if the chain of joints/points collides with obstacle
bool collide_chain_object(const std::vector<Eigen::Vector2d>& joints, const amp::Obstacle2D& obstacle){
    // This algorithm can likely be improved.
    std::vector<Eigen::Vector2d> vertices = obstacle.verticesCCW();
    size_t numVertices = vertices.size();
    size_t numJoints = joints.size();

    for (int i = 0; i < numVertices - 1; i++){
        for (int j = 0; j < numJoints - 1; j++){
            Eigen::Vector2d vec1 = (vertices[i] - joints[numJoints - j - 1]).normalized();
            Eigen::Vector2d vec2 = vertices[i + 1] - joints[numJoints - j - 1];

            Eigen::Vector2d vec3 = (vertices[i] - joints[numJoints - j - 2]).normalized();
            Eigen::Vector2d vec4 = vertices[i + 1] - joints[numJoints - j - 2];

            if ((std::signbit(vec1(0) * vec2(1) - vec1(1) * vec2(0)) != std::signbit(vec3(0) * vec4(1) - vec3(1) * vec4(0))) &&
                (std::signbit(vec1(0) * vec3(1) - vec1(1) * vec3(0)) != std::signbit(vec2(0) * vec4(1) - vec2(1) * vec4(0))) &&
                (vec1(0) * vec2(1) - vec1(1) * vec2(0) != 0) && (vec1(0) * vec3(1) - vec1(1) * vec3(0) != 0) &&
                (vec3(0) * vec4(1) - vec3(1) * vec4(0) != 0) && (vec2(0) * vec4(1) - vec2(1) * vec4(0) != 0)){
                return true;
            }
        }
    }

    for (int j = 0; j < joints.size() - 1; j++){
        Eigen::Vector2d vec1 = (vertices[numVertices - 1] - joints[numJoints - j - 1]).normalized();
        Eigen::Vector2d vec2 = vertices[0] - joints[numJoints - j - 1];

        Eigen::Vector2d vec3 = (vertices[numVertices - 1] - joints[numJoints - j - 2]).normalized();
        Eigen::Vector2d vec4 = vertices[0] - joints[numJoints - j - 2];

        if ((std::signbit(vec1(0) * vec2(1) - vec1(1) * vec2(0)) != std::signbit(vec3(0) * vec4(1) - vec3(1) * vec4(0))) &&
            (std::signbit(vec1(0) * vec3(1) - vec1(1) * vec3(0)) != std::signbit(vec2(0) * vec4(1) - vec2(1) * vec4(0))) &&
            (vec1(0) * vec2(1) - vec1(1) * vec2(0) != 0) && (vec1(0) * vec3(1) - vec1(1) * vec3(0) != 0) &&
            (vec3(0) * vec4(1) - vec3(1) * vec4(0) != 0) && (vec2(0) * vec4(1) - vec2(1) * vec4(0) != 0)){
            return true;
        }
    }

    return false;
    // return collide_object(joints[0], obstacle);
}

/// @breif checks if any part of the cell collides with any obstacle given the center of the cell and the width and height and a vector of obstacles
bool check_cell_collisions(const Eigen::Vector2d center, const double width, const double height, const std::vector<amp::Obstacle2D>& obstacles){

    if (check_collisions(center, obstacles)){
        return true;
    }

    std::vector<Eigen::Vector2d> chain = {center + Eigen::Vector2d(-width/2, -height/2),
                                          center + Eigen::Vector2d(width/2, -height/2),
                                          center + Eigen::Vector2d(width/2, height/2),
                                          center + Eigen::Vector2d(-width/2, height/2),
                                          center + Eigen::Vector2d(-width/2, -height/2)
                                          };

    for (int i = 0; i < obstacles.size(); i++){
        if (collide_chain_object(chain, obstacles[i])){ // Check if joint chain collides with current object
            return true;
        }
    }
    return false;
}

/// @brief Analytically checks if agent paths between the two points collide with each other or the given obstacles.
/// ignore is the number of agents to assume are already collision free and only passed to test other agents against.
bool check_multi_agent_disk_collisions(const Eigen::VectorXd& pointA, const Eigen::VectorXd& pointB,
                                       const std::vector<double>& radii, const std::vector<amp::Obstacle2D>& obstacles,
                                       const size_t ignore){
    size_t numObstacles = obstacles.size();
    size_t numAgents = radii.size();
    if ((numAgents * 2 != pointA.size()) || (numAgents * 2 != pointB.size())){
        LOG("Collision Detection Point Dimensions Mismatched");
        return true;
    }

    for (int i = ignore; i < numAgents; i++){
        for (const amp::Obstacle2D& obstacle : obstacles) {
            if (collide_disk_trajectory_object(pointA({i * 2, i * 2 + 1}), pointB({i * 2, i * 2 + 1}), radii[i], obstacle)) {
                return true;
            }
        }

        for (int j = i - 1; j >= 0; j--){
            if (collide_disk_trajectories(pointA({i * 2, i * 2 + 1}), pointB({i * 2, i * 2 + 1}), radii[i],
                                          pointA({j * 2, j * 2 + 1}), pointB({j * 2, j * 2 + 1}), radii[j])){
                return true;
            }
        }
    }
    return false;
}

/// @brief Analytically checks if the disk path intersects w/ the given obstacle.
bool collide_disk_trajectory_object(const Eigen::Vector2d& start, const Eigen::Vector2d& end, const double& radius, const amp::Obstacle2D& obstacle) {
    std::vector<Eigen::Vector2d> vertices = obstacle.verticesCCW();
    size_t numVertices = vertices.size();
    for (int i = 0; i < numVertices; i++){

        int vertex_1 = i;
        int vertex_2 = i + 1;
        if (i >= numVertices - 1){
            vertex_2 = 0;
        }

        double dist_start = get_closest_dist(start, vertices[vertex_1], vertices[vertex_2]);
        double dist_end = get_closest_dist(end, vertices[vertex_1], vertices[vertex_2]);
        double dist_vert1 = get_closest_dist(vertices[vertex_1], start, end);
        double dist_vert2 = get_closest_dist(vertices[vertex_2], start, end);

        if ((std::signbit(dist_start) != std::signbit(dist_end)) && (std::signbit(dist_vert1) != std::signbit(dist_vert2))){
            return true;
        }
        if ((abs(dist_start) <= radius) || (abs(dist_end) <= radius) || (abs(dist_vert1) <= radius) || (abs(dist_vert2) <= radius)) {
            return true;
        }
    }
    return false;
}

/// @brief Analytically checks if two disk trajectories collide w/ each other
/// assumes the disks move such that they take the same amount of time from start to end.
bool collide_disk_trajectories(const Eigen::Vector2d& start_1, const Eigen::Vector2d& end_1, const double& radius_1,
                               const Eigen::Vector2d& start_2, const Eigen::Vector2d& end_2, const double& radius_2){
    double dist_sqrd = (radius_1 + radius_2) * (radius_1 + radius_2);

    Eigen::Vector2d start_diff = start_1 - start_2;
    Eigen::Vector2d n_diff = end_1 - end_2 - start_diff;

    double a = n_diff.dot(n_diff);
    double b = 2 * n_diff.dot(start_diff);
    double c = start_diff.dot(start_diff);

    if (c <= dist_sqrd){
        return true;
    }

    if (a + b + c <= dist_sqrd){
        return true;
    }

    double t = -b/(2 * a);

    if ((t > 0) && (t < 1) && (c + t * b / 2 <= dist_sqrd)){
        return true;
    }

    return false;
}

/// @brief gets the distance from a point to the given line segment.
/// The sign of the distance shows which side of the half-plane defined by the line segment the point is on.
double get_closest_dist(const Eigen::Vector2d& point, const Eigen::Vector2d& start, const Eigen::Vector2d& end){
    Eigen::Vector2d n = (end - start).normalized();
    Eigen::Vector2d r = point - start;

    if (n.norm() == 0) {
        return r.norm();
    }

    double dist = r(0) * n(1) - r(1) * n(0);
    double t = r.dot(n)/(end - start).norm();

    if (t < 0) {
        return r.norm() * dist/abs(dist);
    }
    if (t > 1) {
        return (point - end).norm() * dist/abs(dist);
    }
    return dist;
}

/// @breif Given a vector of points, times, and a sample time,
/// returns the point at the sample time using linear interpolation between points
Eigen::VectorXd linear_interp(const double& time, const std::vector<Eigen::VectorXd>& points, const std::vector<double>& times) {

    int index = -1;

    for (size_t i = 0; i < times.size(); i++){
        if (times[i] < time){
            index = i;
        }
    }

    if (index < 0) {
        return points[0];
    }

    if (index >= times.size() - 1){
        return points.back();
    }

    return (points[index + 1] - points[index]) * (time - times[index]) / (times[index + 1] - times[index]) + points[index + 1];
}