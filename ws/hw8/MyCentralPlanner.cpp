#include "MyMultiAgentPlanners.h"

amp::MultiAgentPath2D MyCentralPlanner::plan(const amp::MultiAgentProblem2D& problem) {

    // std::shared_ptr<amp::Graph<double>> graphPtr = std::make_shared<amp::Graph<double>>();

    std::map<amp::Node, Eigen::VectorXd> nodes;
    std::map<amp::Node, std::vector<amp::Node>> paths;
//    std::map<amp::Node, std::vector<std::vector<double>>> distances;

    size_t numAgents = problem.numAgents();
    std::map<size_t, bool> reached_goal;

    Eigen::VectorXd init(numAgents * 2);
    Eigen::VectorXd goal(numAgents * 2);
    std::vector<double> radii;
    double smallest_radius = problem.agent_properties[0].radius;

//    std::vector<std::vector<Eigen::Vector2d>> init_path;
//    std::vector<std::vector<double>> init_dist;

    amp::MultiAgentPath2D final_path;

    for (size_t i = 0; i < numAgents; i++) {
        init({i * 2, i * 2 + 1}) = problem.agent_properties[i].q_init;
        goal({i * 2, i * 2 + 1}) = problem.agent_properties[i].q_goal;

        radii.push_back(problem.agent_properties[i].radius);

//        init_path.push_back(std::vector<Eigen::Vector2d>{problem.agent_properties[i].q_init});
//        init_dist.push_back(std::vector<double>{0});

        if (smallest_radius > problem.agent_properties[i].radius){
            smallest_radius = problem.agent_properties[i].radius;
        }

        reached_goal[i] = false;
    }

    nodes[0] = init;
    paths[0] = {0};

    size_t subdivisions = std::max(floor(log2(r/smallest_radius)), 0.0) + 4;
    // LOG("Start");
    for (size_t i = 1; i < n; i++){
        Eigen::VectorXd point(2 * numAgents);
        if (amp::RNG::randd() < p_goal){
            point = goal;
        } else {
            for (size_t j = 0; j < numAgents; j++) {
                if (reached_goal[j]) {
                    point({j * 2, j * 2 + 1}) = problem.agent_properties[i].q_goal;
                } else {
                    point({j * 2, j * 2 + 1}) = Eigen::Vector2d(amp::RNG::randd(problem.x_min, problem.x_max),
                                                                amp::RNG::randd(problem.y_min, problem.y_max));
                }
            }
        }

        std::pair<size_t, double> q_nearest = {0, (point - init).norm()};
        for (const auto& [node, neighbor] : nodes){
            if ((point - neighbor).norm() < q_nearest.second){
                q_nearest.second = (point - neighbor).norm();
                q_nearest.first = node;
            }
        }

        point = r * (point - nodes[q_nearest.first]).normalized() + nodes[q_nearest.first];

        std::vector<amp::Node> path = paths[q_nearest.first];
        path.push_back(i);
//        std::vector<std::vector<double>> dist = distances[q_nearest.first];

//        for (size_t j = 0; j < numAgents; j++) {
//            dist[j].push_back(dist[j].back() + (path[j].back() - point({j * 2, j * 2 + 1})).norm());
//            path[j].push_back(point({j * 2, j * 2 + 1}));
//        }

        if (!check_multiagent_collisions_subdivision(point, nodes[q_nearest.first], radii, problem.obstacles, subdivisions)){

            // graphPtr->connect(q_nearest.first, i, r);

            nodes[i] = point;
//            distances[i] = dist;
            paths[i] = path;
//            LOG("# of Nodes: " << i);

//            for (size_t j = 0; j < numAgents; j++) {
//                if ((!reached_goal[j]) && ((point({j * 2, j * 2 + 1}) - problem.agent_properties[j].q_goal).norm() < r)){
//                    LOG(j);
//                    reached_goal[j] = true;
//                    return final_path;
//                }
//            }

            if ((point - goal).norm() < eps){

                for (size_t j = 0; j < numAgents; j++) {
                    amp::Path2D agent_path;
                    for (size_t k = 0; k < path.size(); k++){
                        agent_path.waypoints.push_back(nodes[path[k]]({j * 2, j * 2 + 1}));
                    }
                    agent_path.waypoints.push_back(problem.agent_properties[j].q_goal);
                    final_path.agent_paths.push_back(agent_path);
                    // LOG("Add Path: " << j);
                }
                // LOG("Break");
                // graphPtr->connect(i, n, (point - goal).norm());
                break;
            }
        }
    }

    size = nodes.size();
    // LOG("Size: " << size);
    // LOG("# of Subdivisions: " << subdivisions);
    return final_path;
}

amp::MultiAgentPath2D MyDecentralPlanner::plan(const amp::MultiAgentProblem2D& problem) {

    DecentralGBRRT RRTAlgo;
    std::shared_ptr<LerpTimingfunction> timing_function = std::make_shared<LerpTimingfunction>();

    size = 0;

    RRTAlgo.set_n(n);
    RRTAlgo.set_r(r);
    RRTAlgo.set_eps(eps);
    RRTAlgo.set_p_goal(p_goal);

    RRTAlgo.set_TimingfunctionPtr(timing_function);

    size_t numAgents = problem.numAgents();

    amp::Problem2D agent_problem;
    agent_problem.x_min = problem.x_min;
    agent_problem.x_max = problem.x_max;
    agent_problem.y_min = problem.y_min;
    agent_problem.y_max = problem.y_max;
    agent_problem.obstacles = problem.obstacles;

    amp::MultiAgentPath2D path;

    for (size_t i = 0; i < numAgents; i++){
        // LOG(i);
        RRTAlgo.set_radius(problem.agent_properties[i].radius);
        agent_problem.q_init = problem.agent_properties[i].q_init;
        agent_problem.q_goal = problem.agent_properties[i].q_goal;
        amp::Path2D agent_path = RRTAlgo.plan(agent_problem);
        path.agent_paths.push_back(agent_path);
        // LOG("adding path");
        timing_function->add_path(agent_path.waypoints, problem.agent_properties[i].radius);
        size += RRTAlgo.get_nodes().size();
    }

    return path;
}

amp::Path2D DecentralGBRRT::plan(const amp::Problem2D& problem) {

    last_dist_vector.clear();
    nodes.clear();
    graphPtr->clear();

    std::map<amp::Node, std::vector<amp::Node>> paths;

    nodes[0] = problem.q_init;
    paths[0] = {0};

    size_t subdivision = std::max(floor(log2(r/radius)), 0.0) + 4;

    amp::Path2D final_path;

    final_path.waypoints.push_back(problem.q_init);

    for (size_t i = 1; i < n; i++){
        Eigen::Vector2d point;
        if (amp::RNG::randd() < p_goal){
            point = problem.q_goal;
        } else {
            point = {amp::RNG::randd(problem.x_min, problem.x_max), amp::RNG::randd(problem.y_min, problem.y_max)};
        }

        std::pair<size_t, double> best_dist = {0, (point - problem.q_init).norm()};
        for (const auto& [node, neighbor] : nodes){
            if ((point - neighbor).norm() < best_dist.second){
                best_dist.second = (point - neighbor).norm();
                best_dist.first = node;
            }
        }

        point = r * (point - nodes[best_dist.first]).normalized() + nodes[best_dist.first];

        std::vector<amp::Node> path = paths[best_dist.first];
        path.push_back(i);

        if (!check_decentralmultiagent_collisions_subdivision(point, nodes[best_dist.first], path.size() - 1,
                                                              radius, TimingfunctionPtr, problem.obstacles, subdivision)){
            graphPtr->connect(best_dist.first, i, r);

            nodes[i] = point;
            paths[i] = path;

            if ((point - problem.q_goal).norm() < eps){

                int time_diff = TimingfunctionPtr->get_size() - path.size();
                // LOG("time diff: " << time_diff);
                bool collided = false;
                for (int j = 0; j < time_diff; j++){
                    if ((j == 0) && check_decentralmultiagent_collisions_subdivision(
                                point, problem.q_goal, path.size() + j,
                                radius, TimingfunctionPtr, problem.obstacles, subdivision)){
                        collided = true;
                        break;
                    } else if (check_decentralmultiagent_collisions_subdivision(
                                problem.q_goal, problem.q_goal, path.size() + j,
                                radius, TimingfunctionPtr, problem.obstacles, subdivision)){
                        collided = true;
                        break;
                    }
                    //LOG(i);
                }

                if (!collided){
                    graphPtr->connect(i, n, (point - problem.q_goal).norm());

                    for (size_t k = 0; k < path.size(); k++){
                        final_path.waypoints.push_back(nodes[path[k]]);
                    }

                    // Could cause collision if already determined paths pass by at later time. need to check?
                    break;
                }
            }
        }
    }

    final_path.waypoints.push_back(problem.q_goal);

    return final_path;
}

Eigen::VectorXd LerpTimingfunction::get_locations(const double& time) {
    size_t num_points = paths.size();
    size_t t = floor(time);

    std::vector<double> indices(num_points - 1);
    std::iota(indices.begin(), indices.end(), 0);

    return linear_interp(time, paths, indices);
}

bool check_decentralmultiagent_collisions(const Eigen::Vector2d& q_new, const Eigen::Vector2d& q_near,
                                                      const double& t_new, const double& radius,
                                                      const std::shared_ptr<Timingfunction> timing_function,
                                                      const std::vector<amp::Obstacle2D>& obstacles){

}

bool check_decentralmultiagent_collisions_subdivision(const Eigen::Vector2d& q_new, const Eigen::Vector2d& q_near,
                                                      const double& t_new, const double& radius,
                                                      const std::shared_ptr<Timingfunction> timing_function,
                                                      const std::vector<amp::Obstacle2D>& obstacles, size_t subdivisions) {

    std::vector<double> radii = timing_function->get_radii();

    for (int j = 0; j < subdivisions + 1; j++){
        size_t iters = std::pow(2, double(j - 1.0));
        double step = 0.5/iters;
        if (j == 0) {
            iters = 1;
            step = 1;
        }
        // LOG("Iters: " << iters);
        for (size_t k = 0; k < iters; k++){
            double t = step + 2 * step * k;

            Eigen::Vector2d sample = linear_interp(t, {q_near, q_new}, {0, 1});
            // LOG("Sample: " << sample);
            double time = t + t_new - 1;
            // LOG("Time: " << time);
            if (radii.size() > 0){
                Eigen::VectorXd samples = timing_function->get_locations(time);
                for (size_t l = 0; l < radii.size(); l++){
                    if ((sample - samples({l * 2, l * 2 + 1})).norm() <= (radius + radii[l]) * 1.1) {
                        return true;
                    }
                }
            }

            if (check_disk_collisions(sample, radius * 1.1, obstacles)) {
                return true;
            }
        }
    }

    return false;
}

bool check_multiagent_collisions_subdivision(const Eigen::VectorXd q_new, const Eigen::VectorXd q_near, const std::vector<double>& radii,
                                             const std::vector<amp::Obstacle2D>& obstacles, const size_t subdivisions) {

    size_t numAgents = q_new.size()/2;

//    std::vector<Eigen::VectorXd> point_vector;
//    point_vector.push_back(q_near);
//    point_vector.push_back(q_new);
    // return false; //
    for (int j = 0; j < subdivisions + 1; j++){

        size_t iters = std::pow(2, j - 1);
        double step = 0.5/iters;
        if (j == 0) {
            iters = 1;
            step = 1;
        }

        for (size_t k = 0; k < iters; k++){
            double t = step + 2 * step * k;
            Eigen::VectorXd sample = linear_interp(t, {q_near, q_new}, {0, 1});
            for (size_t i = 0; i < numAgents; i++){
                for (size_t l = i + 1; l < numAgents; l++){
                    if ((sample({i * 2, i * 2 + 1}) - sample({l * 2, l * 2 + 1})).norm() <= radii[i] + radii[l]){
                        return true;
                    }
                }

                if (check_disk_collisions(sample({i * 2, i * 2 + 1}), radii[i], obstacles)) {
                    return true;
                }
            }
        }
    }
    return false;
}

bool check_disk_collisions(const Eigen::Vector2d& center, const double& radius, const std::vector<amp::Obstacle2D>& obstacles) {
    for (const amp::Obstacle2D& obstacle : obstacles) {
        if (collide_disk_object(center, radius, obstacle)) {
            return true;
        }
    }
    return false;
}

bool collide_disk_object(const Eigen::Vector2d& center, const double& radius, const amp::Obstacle2D& obstacle){
    std::vector<Eigen::Vector2d> vertices = obstacle.verticesCCW();
    size_t numVertices = vertices.size();

    double dist;

    for (int i = 0; i < numVertices - 1; i++){

        dist = get_closest_dist(center, vertices[i], vertices[i + 1]);

        if (dist > 0) {
            if (dist <= radius){
                return true;
            }
            return false;
        }
    }

    dist = get_closest_dist(center, vertices[numVertices - 1], vertices[0]);

    if (dist > 0) {
        if (dist <= radius){
            return true;
        }
        return false;
    }

    return true;
}


