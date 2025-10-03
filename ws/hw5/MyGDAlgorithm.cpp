#include "MyGDAlgorithm.h"

/// @brief plans a path through the enviorment using gradient descent.
amp::Path2D MyGDAlgorithm::plan(const amp::Problem2D& problem) {
    amp::Path2D path;
    path.waypoints.push_back(problem.q_init);
    Eigen::Vector2d q = problem.q_init;
    size_t num_obstacles = problem.obstacles.size();

    MyPotentialFunction potential_func(d_star, zetta, Q_star, eta, problem);

    size_t i = 0;
    size_t last_i = 0;
    int dir = 1;
    double last_dist = (q - problem.q_goal).norm();

    while (((q - problem.q_goal).norm() >= eps) & (i < 1000)) {

        i++;
        double closest_dist = (q - problem.q_goal).norm();

        Eigen::Vector2d step = - alpha * potential_func.getGradient(q, closest_dist);

        if ((step.norm() > closest_dist)){
            step.normalize();
            step *= closest_dist;
        }

        q += step;


        // if not progressing towards goal sufficiently quickly follow edge? For some reason this works really well.
        
        if (last_dist - (q - problem.q_goal).norm() < step.norm()/10){
            q -= step;
            step.normalize();
            if ((i - last_i) > 5){
                dir *= -1;
            }
            q[0] -= (dir * step[1] * closest_dist);
            q[1] += (dir * step[0] * closest_dist);
            last_i = i;
        }

        last_dist = (q - problem.q_goal).norm();

        path.waypoints.push_back(q);
    }

    path.waypoints.push_back(problem.q_goal);
    return path;
}

/// @brief find the closes point on the obstacel to the given point
Eigen::Vector2d MyPotentialFunction::closestPoint(const Eigen::Vector2d& point, const amp::Obstacle2D& obstacle) const{
    const std::vector<Eigen::Vector2d>& vertices = obstacle.verticesCCW();
    size_t num_vertices = vertices.size();
    double best_dist = (vertices[0] - point).norm();
    Eigen::Vector2d best_point = vertices[0];
    Eigen::Vector2d vec1;
    Eigen::Vector2d vec2;

    for (int i = 0; i < num_vertices - 1; i++){
        vec1 = point - vertices[i];
        vec2 = vertices[i + 1] - vertices[i];
        double length = vec2.norm();
        vec2.normalize();
        double t = vec1.dot(vec2);

        if ((t > 0) & (t < length)) {
            double dist = abs(vec1[0] * vec2[1] - vec1[1] * vec2[0]);
            if (dist < best_dist){
                best_dist = dist;
                best_point = vec2 * t + vertices[i];
            }
        } else if ((t < 0) & (vec1.norm() < best_dist)) {
            best_dist = vec1.norm();
            best_point = vertices[i];
        } else if ((t > length) & ((point - vertices[i + 1]).norm() < best_dist)){
            best_dist = (point - vertices[i + 1]).norm();
            best_point = vertices[i + 1];
        }
    }

    vec1 = point - vertices[num_vertices - 1];
    vec2 = vertices[0] - vertices[num_vertices - 1];
    double length = vec2.norm();
    vec2.normalize();
    double t = vec1.dot(vec2);

    if ((t > 0) & (t < length)) {
        double dist = abs(vec1[0] * vec2[1] - vec1[1] * vec2[0]);
        if (dist < best_dist){
            best_dist = dist;
            best_point = vec2 * t + vertices[num_vertices - 1];
        }
    } else if ((t < 0) & (vec1.norm() < best_dist)) {
        best_dist = vec1.norm();
        best_point = vertices[num_vertices - 1];
    } else if ((t > length) & ((point - vertices[0]).norm() < best_dist)){
        best_dist = (point - vertices[0]).norm();
        best_point = vertices[0];
    }

    return best_point;
}

/// @brief evaluate the potential function at the given point
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

/// @brief get the gradient of the potential function at a given point.
Eigen::Vector2d MyPotentialFunction::getGradient(const Eigen::Vector2d &q) const {
    size_t num_obstacles = problem.obstacles.size();
    Eigen::Vector2d del_u_att;
    Eigen::Vector2d to_goal = q - problem.q_goal;
    // dist = DBL_MAX;

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

/// @breif get the gradient of the potential function at a given point and set the distance reference to the distance to the closest obstacle to that point.
Eigen::Vector2d MyPotentialFunction::getGradient(const Eigen::Vector2d& q, double& dist){
    size_t num_obstacles = problem.obstacles.size();
    Eigen::Vector2d del_u_att;
    Eigen::Vector2d to_goal = q - problem.q_goal;
    // dist = DBL_MAX;

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

        if (obstacle_dist < dist) {
            dist = obstacle_dist;
        }

        if (obstacle_dist < Q_star){
            del_u_rep -= eta * (1 / obstacle_dist - 1 / Q_star) * (q - c) / (obstacle_dist * obstacle_dist * obstacle_dist);
        } // Double check direction of gradient.
    }

    return del_u_att + del_u_rep;
}
