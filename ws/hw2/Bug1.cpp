#include "Bug1.h"

// Implement your methods in the `.cpp` file, for example:
amp::Path2D Bug1::plan(const amp::Problem2D& problem) {

    amp::Path2D path;
    path.waypoints.push_back(problem.q_init);

    Eigen::Vector2d dir = (problem.q_goal - path.waypoints.back()).normalized();
    Eigen::Vector2d q_next = dir * step_size + path.waypoints.back();

    Eigen::Rotation2D<double> rot(M_PI / 36);
    Eigen::Rotation2D<double> rot90CW(-M_PI / 2);

    while (true){
        while (true){ // Move Towards Goal
            dir = (problem.q_goal - path.waypoints.back()).normalized();
            q_next = dir * step_size + path.waypoints.back();
            if ((problem.q_goal - path.waypoints.back()).norm() < step_size){
                path.waypoints.push_back(problem.q_goal); // reached goal
                return path;
            } else if(check_collisions(q_next, problem.obstacles)) {
                break; // detected obstacle
            } else {
                path.waypoints.push_back(q_next);
            }
        }
        // initialize variables for perimeter following
        size_t q_hit_i = path.waypoints.size() - 1;
        size_t q_leave_i = path.waypoints.size() - 1;
        double dist = (problem.q_goal - path.waypoints[q_hit_i]).norm();
        double best_dist = dist;

        while(true){
            // Turn away from obstacle to avoid collision and ensure 90 deg sensor can see obstacle
            while(check_collisions(q_next, problem.obstacles) || !check_collisions((rot90CW * dir) * step_size + path.waypoints.back(), problem.obstacles)){ // Rotate left
                dir = rot * dir;
                q_next = dir * step_size + path.waypoints.back();
            }
            path.waypoints.push_back(q_next);
            // Check if point is closer to goal than all other points and store it if so.
            dist = (problem.q_goal - path.waypoints.back()).norm();
            if (dist < best_dist) {
                q_leave_i = path.waypoints.size() - 1;
                best_dist = dist;
            }
            // Check if returned to leave point
            if (((path.waypoints.size() - q_hit_i) > 4) && (path.waypoints[q_hit_i] - path.waypoints.back()).norm() <= step_size){
                break;
            }
            // Turn & move towards obstacle if robot left perimeter
            while (!check_collisions((rot90CW * dir) * step_size + path.waypoints.back(), problem.obstacles)){
                dir = rot90CW * dir;
                path.waypoints.push_back(dir * step_size + path.waypoints.back());
                // Check if point is closer to goal than all other points and store it if so.
                dist = (problem.q_goal - path.waypoints.back()).norm();
                if (dist < best_dist){
                    q_leave_i = path.waypoints.size() - 1;
                    best_dist = dist;
                }
            }
            // Check if returned to leave point
            if (((path.waypoints.size() - q_hit_i) > 4) && (path.waypoints[q_hit_i] - path.waypoints.back()).norm() <= step_size){
                break;
            }
            // Reset q_next for next loop
            q_next = dir * step_size + path.waypoints.back();
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
        dir = (problem.q_goal - path.waypoints.back()).normalized();
        q_next = dir * step_size + path.waypoints.back();
        if (check_collisions(q_next, problem.obstacles)){
            break;
        }
    }
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


