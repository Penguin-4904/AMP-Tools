#include "Bug1.h"

// Implement your methods in the `.cpp` file, for example:
amp::Path2D Bug1::plan(const amp::Problem2D& problem) {

    amp::Path2D path;
    path.waypoints.push_back(problem.q_init);

    Eigen::Vector2d step = (problem.q_goal - path.waypoints.back()).normalized() * step_size;

    Eigen::Rotation2D<double> rotCW(-M_PI / 36); // rotation matrix for 5 deg CCW
    Eigen::Rotation2D<double> rotCCW(M_PI / 36); // rotation matrix for 5 deg CW

    while (true){
        while (true){ // Move Towards Goal
            if ((problem.q_goal - path.waypoints.back()).norm() < step_size){
                path.waypoints.push_back(problem.q_goal); // reached goal
                return path;
            } else if(check_collisions(step + path.waypoints.back(), problem.obstacles)) {
                break; // detected obstacle
            } else {
                path.waypoints.push_back(step + path.waypoints.back());
            }
        }
        // initialize variables for perimeter following
        size_t q_hit_i = path.waypoints.size() - 1;
        size_t q_leave_i = path.waypoints.size() - 1;
        double dist = (problem.q_goal - path.waypoints[q_hit_i]).norm();
        double best_dist = dist;

        // perimeter following loop
        while (((path.waypoints.size() - q_hit_i) < 4) || (path.waypoints[q_hit_i] - path.waypoints.back()).norm() > step_size){
            // Corner Detection
            if (!check_collisions(rotCW * (rotCW * step) + path.waypoints.back(), problem.obstacles)
                && !check_collisions(step + path.waypoints.back(), problem.obstacles)){
                path.waypoints.push_back(step + path.waypoints.back());
            }
            // Edge Finding
            while (!check_collisions(step + path.waypoints.back(), problem.obstacles)){
                step = rotCW * step;
            }
            while (check_collisions(step + path.waypoints.back(), problem.obstacles)){
                step = rotCCW * step;
            }
            path.waypoints.push_back(step + path.waypoints.back());
            // Check if point is closer to goal than all other points and store it if so.
            dist = (problem.q_goal - path.waypoints.back()).norm();
            if (dist < best_dist) {
                q_leave_i = path.waypoints.size() - 1;
                best_dist = dist;
            }
        }
        // Return to the closest leave point to goal via the fastest path
        if(q_leave_i - q_hit_i < (path.waypoints.size() - 1 - q_leave_i)){
            for(int i = q_hit_i; i <= q_leave_i; i++){
                path.waypoints.push_back(path.waypoints[i]);
            }
        } else {
            for(int i = path.waypoints.size() - 1; i >= q_leave_i; i--){
                path.waypoints.push_back(path.waypoints[i]);
            }
        }
        // Check if there is path towards goal from leave point
        step = (problem.q_goal - path.waypoints.back()).normalized() * step_size;
        if (check_collisions(step_size + path.waypoints.back(), problem.obstacles)){
            LOG("No Path Found!");
            break;
        }
    }
    return path;
}

/// @brief checks if point is inside any of the given obstacles.
bool Bug1::check_collisions(Eigen::Vector2d point, std::vector<amp::Obstacle2D> obstacles) {
    size_t numObstacles = obstacles.size();

    for (int i = 0; i < numObstacles; i++){
        // if point is in an obstacle stop search and return collision as true
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
        // check if vectors from point to vertices proceed in CW or CCW direction
        // CW (negative cross product) -> point is not in half plane defined by the vertices
        // Since vertices are defined as CCW -> point outside polygon, return collision as false
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


