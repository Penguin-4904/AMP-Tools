#include "MyMultiAgentPlanners.h"

amp::MultiAgentPath2D MyCentralPlanner::plan(const amp::MultiAgentProblem2D& problem) {

    // std::shared_ptr<amp::Graph<double>> graphPtr = std::make_shared<amp::Graph<double>>();

    std::map<amp::Node, Eigen::VectorXd> nodes;
    std::map<amp::Node, std::vector<std::vector<Eigen::Vector2d>>> paths;
    std::map<amp::Node, std::vector<std::vector<double>>> distances;

    size_t numAgents = problem.numAgents();

    Eigen::VectorXd init(numAgents * 2);
    Eigen::VectorXd goal(numAgents * 2);
    std::vector<double> radii;
    double smallest_radius = problem.agent_properties[0].radius;

    std::vector<std::vector<Eigen::Vector2d>> init_path;
    std::vector<std::vector<double>> init_dist;

    amp::MultiAgentPath2D final_path;

    for (size_t i = 0; i < numAgents; i++) {
        init({i * 2, i * 2 + 1}) = problem.agent_properties[i].q_init;
        goal({i * 2, i * 2 + 1}) = problem.agent_properties[i].q_goal;

        radii.push_back(problem.agent_properties[i].radius);

        init_path.push_back(std::vector<Eigen::Vector2d>{problem.agent_properties[i].q_init});
        init_dist.push_back(std::vector<double>{0});

        if (smallest_radius > problem.agent_properties[i].radius){
            smallest_radius = problem.agent_properties[i].radius;
        }
    }

    nodes[0] = init;
    paths[0] = init_path;
    distances[0] = init_dist;

    size_t subdivisions = std::max(floor(log2(r/smallest_radius)), 0.0) + 4;

    for (size_t i = 1; i < n; i++){
        Eigen::VectorXd point(2 * numAgents);
        if (amp::RNG::randd() < p_goal){
            point = goal;
        } else {
            for (size_t j = 0; j < numAgents; j++) {
                point({j * 2, j * 2 + 1}) = Eigen::Vector2d(amp::RNG::randd(problem.x_min, problem.x_max), amp::RNG::randd(problem.y_min, problem.y_max));
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

        std::vector<std::vector<Eigen::Vector2d>> path = paths[q_nearest.first];
        std::vector<std::vector<double>> dist = distances[q_nearest.first];

        for (size_t j = 0; j < numAgents; j++) {
            dist[j].push_back(dist[j].back() + (path[j].back() - point({j * 2, j * 2 + 1})).norm());
            path[j].push_back(point({j * 2, j * 2 + 1}));
        }

        if (!check_multiagent_collisions_subdivision(path, dist, radii, problem.obstacles, subdivisions, numAgents)){

            //graphPtr->connect(q_nearest.first, i, r);

            nodes[i] = point;
            distances[i] = dist;
            paths[i] = path;

            if ((point - goal).norm() < eps){

                for (size_t j = 0; j < numAgents; j++) {
                    amp::Path2D agent_path;
                    agent_path.waypoints = path[j];
                    agent_path.waypoints.push_back(problem.agent_properties[j].q_goal);
                    final_path.agent_paths.push_back(agent_path);
                }

                //graphPtr->connect(i, n, (point - goal).norm());
                break;
            }
        }
    }

    return final_path;
}

amp::MultiAgentPath2D MyDecentralPlanner::plan(const amp::MultiAgentProblem2D& problem) {

    DecentralGBRRT RRTAlgo;
    std::shared_ptr<LerpTimingfunction> timing_function = std::make_shared<LerpTimingfunction>();

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
        LOG(i);
        RRTAlgo.set_radius(problem.agent_properties[i].radius);
        agent_problem.q_init = problem.agent_properties[i].q_init;
        agent_problem.q_goal = problem.agent_properties[i].q_goal;
        amp::Path2D agent_path = RRTAlgo.plan(agent_problem);
        path.agent_paths.push_back(agent_path);
        timing_function->add_path(agent_path.waypoints, RRTAlgo.get_dist(), problem.agent_properties[i].radius);
    }

    return path;
}

amp::Path2D DecentralGBRRT::plan(const amp::Problem2D& problem) {

    last_dist_vector.clear();
    nodes.clear();
    graphPtr->clear();

    std::map<amp::Node, double> distance;

    nodes[0] = problem.q_init;
    distance[0] = 0;

    size_t subdivision = std::max(floor(log2(r/radius)), 0.0) + 4;

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

        double dist = distance[best_dist.first] + best_dist.second;
        if (!check_decentralmultiagent_collisions_subdivision(point, nodes[best_dist.first], dist,
                                                              distance[best_dist.first], radius, TimingfunctionPtr,
                                                              problem.obstacles, subdivision)){
            graphPtr->connect(best_dist.first, i, r);
            nodes[i] = point;
            distance[i] = dist;

            if ((point - problem.q_goal).norm() < eps){
                graphPtr->connect(i, n, (point - problem.q_goal).norm());
                break;
            }
        }
    }

    nodes[n] = problem.q_goal;

    amp::Path2D path;
    path.waypoints.push_back(problem.q_goal);
    std::vector<amp::Node> parents = graphPtr->parents(n);


    while (parents.size() > 0){
        path.waypoints.push_back(nodes[parents[0]]);
        last_dist_vector.push_back(distance[parents[0]]);
        parents = graphPtr->parents(parents[0]);
    }

    reverse(path.waypoints.begin(), path.waypoints.end());

    return path;
}

std::vector<Eigen::Vector2d> LerpTimingfunction::get_locations(const double& time) {
    std::vector<Eigen::Vector2d> locations;
    size_t numPaths = paths.size();
    for (size_t i = 0; i < numPaths; i++){
        locations.push_back(linear_interp(time, paths[i], times[i]));
    }
    return locations;
}

bool check_decentralmultiagent_collisions_subdivision(const Eigen::Vector2d& q_new, const Eigen::Vector2d& q_near,
                                                      const double& t_new, const double& t_near, const double& radius,
                                                      const std::shared_ptr<Timingfunction> timing_function,
                                                      const std::vector<amp::Obstacle2D>& obstacles, size_t subdivisions) {

    std::vector<double> radii = timing_function->get_radii();

    for (size_t j = 0; j < subdivisions + 1; j++){

        size_t iters = std::pow(2, j - 1);
        double step = 0.5/iters;
        if (j == 0) {
            iters = 1;
        }

        for (size_t k = 0; k < iters; k++){
            double t = step + 2 * step * k;
            Eigen::Vector2d sample = linear_interp(t, {q_near, q_new}, {0, 1});
            double time = t * (t_new - t_near) + t_near;
            std::vector<Eigen::Vector2d> samples = timing_function->get_locations(time);
            for (size_t l = 0; l < samples.size(); l++){
                if ((sample - samples[l]).norm() <= radius + radii[l]) {
                    return true;
                }
            }

            if (check_disk_collisions(sample, radius, obstacles)) {
                return true;
            }
        }
    }
}

bool check_multiagent_collisions_subdivision(const std::vector<std::vector<Eigen::Vector2d>>& paths,
                                             const std::vector<std::vector<double>>& times, const std::vector<double>& radii,
                                             const std::vector<amp::Obstacle2D>& obstacles, size_t subdivisions, size_t numAgents) {

    size_t numPaths = paths.size();

    for (size_t j = 0; j < subdivisions + 1; j++){

        size_t iters = std::pow(2, j - 1);
        double step = 0.5/iters;
        if (j == 0) {
            iters = 1;
        }

        for (size_t k = 0; k < iters; k++){
            double t = step + 2 * step * k;
            for (size_t i = numPaths - numAgents; i < numPaths; i++){
                Eigen::Vector2d sample_i = linear_interp(t, {paths[i][paths[i].size() - 2], paths[i].back()}, {0, 1});
                double time = t * (times[i].back() - times[i][times[i].size() - 2]) + times[i][times[i].size() - 2];
                for (size_t l = 0; l < numPaths; l++){
                    if (l != i){
                        Eigen::Vector2d sample_l = linear_interp(time, paths[l], times[l]);
                        if ((sample_i - sample_l).norm() <= radii[i] + radii[l]) {
                            return true;
                        }
                    }
                }

                if (check_disk_collisions(sample_i, radii[i], obstacles)) {
                    return true;
                }
            }
        }
    }
}

bool check_disk_collisions(const Eigen::Vector2d& center, const double& radius, const std::vector<amp::Obstacle2D>& obstacles) {
    for (const amp::Obstacle2D& obstacle : obstacles) {
        if (collide_disk_object(center, radius, obstacle)) {
            return true;
        }
    }
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


