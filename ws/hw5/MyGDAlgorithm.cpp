#include "MyGDAlgorithm.h"

// Implement your plan method here, similar to HW2:
amp::Path2D MyGDAlgorithm::plan(const amp::Problem2D& problem) {
    amp::Path2D path;
    path.waypoints.push_back(problem.q_init);
    Eigen::Vector2d q = problem.q_init;
    size_t num_obstacles = problem.obstacles.size();

    MyPotentialFunction potential_func(d_star, zetta, Q_star, eta, problem);

    while ((q - problem.q_goal).norm() >= eps) {

        double closest_dist = (q - problem.q_goal).norm();
        double obstacle_dist;

        for (int i = 0; i < num_obstacles; i++){
            obstacle_dist = (q - potential_func.closestPoint(q, problem.obstacles[i])).norm();
            if (obstacle_dist < closest_dist){
                closest_dist = obstacle_dist;
            }
        }

        Eigen::Vector2d step = - alpha * potential_func.getGradient(q);
        if (step.norm() > closest_dist){
            step.normalize();
            step *= closest_dist;
        }

        q += step;
        path.waypoints.push_back(q);
        // LOG()
    }

    path.waypoints.push_back(problem.q_goal);
    return path;
}

Eigen::Vector2d MyPotentialFunction::closestPoint(const Eigen::Vector2d& point, const amp::Obstacle2D& obstacle) const{
    const std::vector<Eigen::Vector2d>& vertices = obstacle.verticesCCW();
    size_t num_vertices = vertices.size();
    double best_dist = (vertices[0] - point).norm();
    Eigen::Vector2d best_point(0, 0);
    Eigen::Vector2d vec1;
    Eigen::Vector2d vec2;

    for (int i = 0; i < num_vertices - 1; i++){
        vec1 = vertices[i] - point;
        vec2 = vertices[i + 1] - point;
        if (vec1.dot(vec2) < 0){
            vec2 = (vertices[i + 1] - vertices[i]).normalized();
            double dist = vec1[0] * vec2[1] - vec1[1] * vec2[0];
            if (dist < best_dist){
                best_dist = dist;
                best_point[0] = point[0] - dist * vec2[1];
                best_point[1] = point[1] + dist * vec2[0];
            }
        } else {
            if (vec1.norm() < best_dist) {
                best_dist = vec1.norm();
                best_point = vertices[i];
            } else if (vec2.norm() < best_dist){
                best_dist = vec2.norm();
                best_point = vertices[i + 1];
            }
        }
    }

    vec1 = vertices[num_vertices - 1] - point;
    vec2 = vertices[0] - point;
    if (vec1.dot(vec2) < 0){
        vec2 = (vertices[0] - vertices[num_vertices - 1]).normalized();
        double dist = vec1[0] * vec2[1] - vec1[1] * vec2[0];
        if (dist < best_dist){
            best_dist = dist;
            best_point[0] = point[0] - dist * vec2[1];
            best_point[1] = point[1] + dist * vec2[0];
        }
    } else {
        if (vec1.norm() < best_dist) {
            best_dist = vec1.norm();
            best_point = vertices[num_vertices - 1];
        } else if (vec2.norm() < best_dist){
            best_dist = vec2.norm();
            best_point = vertices[0];
        }
    }
    return best_point;
}

double MyPotentialFunction::operator()(const Eigen::Vector2d& q) const {

    size_t num_obstacles = problem.obstacles.size();
    double goal_dist = (problem.q_goal - q).norm();
    double u_att;

    if (goal_dist <= d_star){
        u_att = zetta * goal_dist * goal_dist/ 2;
    } else {
        u_att = d_star * zetta * goal_dist - zetta * d_star * d_star / 2;
    }

    double u_rep = 0;
    Eigen::Vector2d c;
    double obstacle_dist;
    for (int i = 0; i < num_obstacles; i++){
        c = closestPoint(q, problem.obstacles[i]);
        obstacle_dist = (q - c).norm();
        if (obstacle_dist < Q_star){
            u_rep += eta * (1 / obstacle_dist - 1 / Q_star) * (1 / obstacle_dist - 1 / Q_star) / 2;
        }
    }

    return u_att + u_rep;
}

Eigen::Vector2d MyPotentialFunction::getGradient(const Eigen::Vector2d& q) const {
    size_t num_obstacles = problem.obstacles.size();
    Eigen::Vector2d del_u_att;
    Eigen::Vector2d to_goal = q - problem.q_goal;

    if (to_goal.norm() <= d_star){
        del_u_att = zetta * to_goal;
    } else {
        del_u_att = d_star * zetta * to_goal / to_goal.norm();
    }

    Eigen::Vector2d del_u_rep(0, 0);
    Eigen::Vector2d c;
    double obstacle_dist;
    for (int i = 0; i < num_obstacles; i++){
        c = closestPoint(q, problem.obstacles[i]);
        obstacle_dist = (q - c).norm();
        if (obstacle_dist < Q_star){
            del_u_rep -= eta * (1 / obstacle_dist - 1 / Q_star) * (q - c) / (obstacle_dist * obstacle_dist * obstacle_dist);
        } // Double check direction of gradient.
    }

    return del_u_att + del_u_rep;
}
