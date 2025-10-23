#include "MyMultiAgentPlanners.h"

amp::MultiAgentPath2D MyCentralPlanner::plan(const amp::MultiAgentProblem2D& problem) {

    std::shared_ptr<amp::Graph<double>> graphPtr = std::make_shared<amp::Graph<double>>();
    std::map<amp::Node, Eigen::VectorXd> nodes;

    size_t numAgents = problem.numAgents();

    Eigen::VectorXd init(numAgents * 2);
    Eigen::VectorXd goal(numAgents * 2);
    std::vector<double> radii;
    // vector<bool> reached_goal(numAgents);

    for (size_t i = 0; i < numAgents; i++) {
        init({i * 2, i * 2 + 1}) = problem.agent_properties[i].q_init;
        goal({i * 2, i * 2 + 1}) = problem.agent_properties[i].q_goal;
        radii.push_back(problem.agent_properties[i].radius);
    }

    nodes[0] = init;

    for (size_t i = 1; i < n; i++){
        Eigen::VectorXd point;
        if (amp::RNG::randd() < p_goal){
            point = goal;
        } else {
            for (size_t j = 0; j < numAgents; j++) {
                point({j * 2, j * 2 + 1}) = {amp::RNG::randd(problem.x_min, problem.x_max), amp::RNG::randd(problem.y_min, problem.y_max)};
            }
        }

        std::pair<size_t, double> best_dist = {0, (point - init).norm()};
        for (const auto& [node, neighbor] : nodes){
            if ((point - neighbor).norm() < best_dist.second){
                best_dist.second = (point - neighbor).norm();
                best_dist.first = node;
            }
        }

        for (size_t j = 0; j< numAgents; j++){
            point({j * 2, j * 2 + 1}) = r * (point({j * 2, j * 2 + 1}) - nodes[best_dist.first]({j * 2, j * 2 + 1})).normalized();
        }

        point += nodes[best_dist.first];

        if (!check_multi_agent_disk_collisions(point, nodes[best_dist.first], radii, problem.obstacles)){
            graphPtr->connect(best_dist.first, i, std::sqrt(numAgents) * r);
            nodes[i] = point;

            if ((point - problem.q_goal).norm() < eps){
                graphPtr->connect(i, n, (point - problem.q_goal).norm());
                break;
            }
        }
    }

    nodes[n] = goal;

    amp::Path2D path;

    std::vector<amp::Node> parents = graphPtr->parents(n);


    while (parents.size() > 0){
        path.waypoints.push_back(nodes[parents[0]]);
        parents = graphPtr->parents(parents[0]);
    }

    reverse(path.waypoints.begin(), path.waypoints.end());

    return path;

    amp::MultiAgentPath2D path;
    for (const amp::CircularAgentProperties& agent : problem.agent_properties) {
        amp::Path2D agent_path;
        agent_path.waypoints = {agent.q_init, agent.q_goal};
        path.agent_paths.push_back(agent_path);
    }
    return path;
}

amp::MultiAgentPath2D MyDecentralPlanner::plan(const amp::MultiAgentProblem2D& problem) {
    amp::MultiAgentPath2D path;
    for (const amp::CircularAgentProperties& agent : problem.agent_properties) {
        amp::Path2D agent_path;
        agent_path.waypoints = {agent.q_init, agent.q_goal};
        path.agent_paths.push_back(agent_path);
    }
    return path;
}