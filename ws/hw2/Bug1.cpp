#include "Bug1.h"

// Implement your methods in the `.cpp` file, for example:
amp::Path2D Bug1::plan(const amp::Problem2D& problem) {

    amp::Path2D path;
    path.waypoints.push_back(problem.q_init);

    Eigen::Vector2d step = (problem.q_goal - path.waypoints.back()).normalized() * step_size;

    Eigen::Rotation2D<double> rotCW(deg_step * -M_PI / 180); // rotation matrix for 5 deg CCW
    Eigen::Rotation2D<double> rotCCW(deg_step * M_PI / 180); // rotation matrix for 5 deg CW

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
            int i = 1;
            while (!check_collisions(step + path.waypoints.back(), problem.obstacles) && i < 180/deg_step){
                step = rotCW * step;
                i++;
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
        if (check_collisions(step + path.waypoints.back(), problem.obstacles)){
            LOG("No Path Found!");
            break;
        }
    }
    return path;
}


