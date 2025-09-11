#include "Bug1.h"

// Implement your methods in the `.cpp` file, for example:
amp::Path2D Bug1::plan(const amp::Problem2D& problem) {

    // Your algorithm solves the problem and generates a path. Here is a hard-coded to path for now...
    amp::Path2D path;
    path.waypoints.push_back(problem.q_init);

    // Move Towards Goal
    Eigen::Vector2d dir = (problem.q_goal - path.waypoints.back()).normalized();
    Eigen::Vector2d q_next = dir * step_size + path.waypoints.back();
    while (true){
        while (true){
            dir = (problem.q_goal - path.waypoints.back()).normalized();
            q_next = dir * step_size + path.waypoints.back();
            if ((problem.q_goal - path.waypoints.back()).norm() < step_size){
                path.waypoints.push_back(problem.q_goal);
                return path;
            } else if(check_collisions(q_next)) {
                break;
            } else {
                path.waypoints.push_back(q_next);
            }
        }
        Eigen::Vector2d q_hit = path.waypoints.back();
        size_t q_hit_index = path.waypoints.size() - 1;


    }

    path.waypoints.push_back(problem.q_goal);

    return path;
}

/// @brief checks if point is inside any of the given obstacles.
bool Bug1::check_collisions(Eigen::Vector2d point, std::vector<amp::Obstacle2D> obstacles) {
    size_t numObstacles = obstacles.size();

    for (int i = 0; i < numObstacles; i++){
        if (collide_object(point, obstacles[i])){
             return true;
        }
    }
    return false;
}

/// @brief checks if point is inside obstacle. Assumes convex polygon obstacle.
bool Bug1::collide_object(Eigen::Vector2d point, amp::Obstacle2D obstacle){
    std::vector<Eigen::Vector2d>& vertices = obstacle.verticesCCW();
    size_t numVertices = vertices.size();

    for (int i = 0; i < numVertices - 1; i++){
        Eigen::Vector2d vec1 = vertices[i] - point;
        Eigen::Vector2d vec2 = vertices[i + 1] - point;
        if ((vec1(0) * vec2(1) - vec1(1) * vec2(0)) < 0){
            return false;
        }
    }

    Eigen::Vector2d vec1 = vertices[numVertices - 1] - point;
    Eigen::Vector2d vec2 = vertices[0] - point;
    if ((vec1(0) * vec2(1) - vec1(1) * vec2(0)) < 0){
        return false;
    }
    return true;
}


