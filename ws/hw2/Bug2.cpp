#include "Bug2.h"

// Implement your methods in the `.cpp` file, for example:
amp::Path2D Bug2::plan(const amp::Problem2D& problem) {

    amp::Path2D path;
    path.waypoints.push_back(problem.q_init);

    Eigen::Rotation2D<double> rotCW(deg_step * -M_PI / 180); // rotation matrix for 5 deg CCW
    Eigen::Rotation2D<double> rotCCW(deg_step * M_PI / 180); // rotation matrix for 5 deg CW

    Eigen::Vector2d w = (problem.q_goal - problem.q_init).normalized();
    Eigen::Vector2d step = w * step_size;
    Eigen::Vector2d v(-w[1], w[0]);

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
        double dist = (problem.q_goal - path.waypoints.back()).norm();
        int sgn = 1;
        if ((problem.q_init - path.waypoints.back()).norm() > (problem.q_goal - problem.q_init).norm()){
            sgn = -1;
        }

        // perimeter following loop
        while (true){
            // Corner Detection
            if (!check_collisions(rotCW * (rotCW * step) + path.waypoints.back(), problem.obstacles)
                && !check_collisions(step + path.waypoints.back(), problem.obstacles)){
                path.waypoints.push_back(step + path.waypoints.back());
            }
            // Check if crossed start to goal line
            w = path.waypoints.back() - problem.q_goal; // Note: w is being reused to store different vector from original use
            if ((w.dot(v) >= 0 && sgn < 0) || (w.dot(v) < 0 && sgn > 0)){
                sgn = -sgn;
                // Check if closer to goal than hit point and can move towards goal
                if (w.norm() < dist
                    && !check_collisions(-w.normalized() * step_size + path.waypoints.back(), problem.obstacles)) {
                    step = - step / (-(v.dot(path.waypoints[path.waypoints.size() - 2] - problem.q_goal) / w.dot(v)) + 1);
                    path.waypoints.push_back(step + path.waypoints.back()); // To ensure exactly on line
                    step = (problem.q_goal - path.waypoints.back()).normalized() * step_size;
                    break;
                }
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
            // Check if returned to original hit point
            if (((path.waypoints.size() - q_hit_i) > 4) && (path.waypoints[q_hit_i] - path.waypoints.back()).norm() < step_size){
                LOG("No Path Found!");
                return path;
            }
        }
    }
    path.waypoints.push_back(problem.q_goal);
    return path;
}


