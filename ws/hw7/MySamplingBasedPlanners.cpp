# include "MySamplingBasedPlanners.h"

// Implement your PRM algorithm here
amp::Path2D MyPRM::plan(const amp::Problem2D& problem) {

    //LOG("Start");
    nodes.clear();
    graphPtr->clear();

//    points.push_back(problem.q_init);
//    points.push_back(problem.q_goal);

    std::pair<amp::Node, double> best_dist_init(0, (problem.q_init - problem.q_goal).norm());
    std::pair<amp::Node, double> best_dist_goal(0, best_dist_init.second);

    for (size_t i = 0; i < n; i++){
        Eigen::Vector2d point(amp::RNG::randd(problem.x_min, problem.x_max), amp::RNG::randd(problem.y_min, problem.y_max));
        if (!check_collisions(point, problem.obstacles)){
            for (const auto& [node, neighbor] : nodes){
                double dist = (point - neighbor).norm();
                if ((dist <= r) && !check_chain_collisions(std::vector<Eigen::Vector2d>{point, neighbor}, problem.obstacles)){
                    graphPtr->connect(i, node, dist);
                    graphPtr->connect(node, i, dist);
                }
            }

            if (((point - problem.q_init).norm() <= best_dist_init.second)) { //&& !check_chain_collisions(std::vector<Eigen::Vector2d>{point, problem.q_init}, problem.obstacles)){
                best_dist_init.first = i;
                best_dist_init.second = (point - problem.q_init).norm();
            }

            if (((point - problem.q_goal).norm() <= best_dist_goal.second)){ // && !check_chain_collisions(std::vector<Eigen::Vector2d>{point, problem.q_goal}, problem.obstacles)){
                best_dist_goal.first = i;
                best_dist_goal.second = (point - problem.q_goal).norm();
            }

            nodes[i] = point;
        }
    }

    graphPtr->connect(n, best_dist_goal.first, best_dist_goal.second);
    graphPtr->connect(best_dist_goal.first, n, best_dist_goal.second);
    nodes[n] = problem.q_goal;

    graphPtr->connect(n + 1, best_dist_init.first, best_dist_init.second);
    graphPtr->connect(best_dist_init.first, n + 1, best_dist_init.second);
    nodes[n + 1] = problem.q_init;

//    LOG("Point Size: " << points.size());
//    LOG("Edges Size: " << edges.size());

    // for (amp::Node i = 0; i < points.size(); ++i) nodes[i] = points[i];
    // for (const auto& [from, to, weight] : edges) graphPtr->connect(from, to, weight);

//    graphPtr->children(0);
//    graphPtr->print();

    LookupSearchHeuristic dist_heuristic;

    for (const auto& [node, point] : nodes) dist_heuristic.heuristic_values[node] = (point - problem.q_goal).norm();

    MyAStarAlgo a_star;

    amp::ShortestPathProblem graph_problem;

    graph_problem.graph = graphPtr;
    graph_problem.init_node = n + 1;
    graph_problem.goal_node = n;

//    LOG("Heuristic " << dist_heuristic.heuristic_values.size());
    MyAStarAlgo::GraphSearchResult result = a_star.search(graph_problem, dist_heuristic);

//    LOG("Result: " << result.success);

    amp::Path2D path;

    for (std::list<amp::Node>::iterator i = result.node_path.begin(); i != result.node_path.end(); i++){
        path.waypoints.push_back(nodes.at(*i));
    }

    if (path_smoothing && result.success){
        size_t iterations = path.waypoints.size();
        for (size_t i = 0; i < iterations; i++){
            int j = amp::RNG::randi(0, path.waypoints.size());
            int k = amp::RNG::randi(0, path.waypoints.size());
            if ((abs(j - k) > 1) && !check_chain_collisions(std::vector<Eigen::Vector2d>{path.waypoints[j], path.waypoints[k]}, problem.obstacles)){
                for (size_t l = 1; l < abs(j - k); l++){
                    path.waypoints.erase(path.waypoints.begin() + std::min(j, k) + 1);
                }
            }
        }
    }

    return path;
}


// Implement your RRT algorithm here
amp::Path2D MyRRT::plan(const amp::Problem2D& problem) {

//    LOG(problem.x_min);
//    LOG(problem.x_max);
//    LOG(problem.y_min);
//    LOG(problem.y_max);
    nodes.clear();
    graphPtr->clear();

    nodes[0] = problem.q_init;

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

        if (!check_chain_collisions(std::vector<Eigen::Vector2d>{point, nodes[best_dist.first]}, problem.obstacles)){
            graphPtr->connect(best_dist.first, i, r);
            nodes[i] = point;

            if ((point - problem.q_goal).norm() < eps){
                graphPtr->connect(i, n, (point - problem.q_goal).norm());
                break;
            }
        }
    }

//    std::pair<size_t, double> best_dist = {0, (problem.q_goal - problem.q_init).norm()};
//    for (const auto& [node, neighbor] : nodes){
//        if ((problem.q_goal - neighbor).norm() < best_dist.second){
//            best_dist.second = (problem.q_goal - neighbor).norm();
//            best_dist.first = node;
//        }
//    }
//
//    graphPtr->connect(best_dist.first, n, best_dist.second);



    nodes[n] = problem.q_goal;

    amp::Path2D path;
    path.waypoints.push_back(problem.q_goal);
    std::vector<amp::Node> parents = graphPtr->parents(n);


    while (parents.size() > 0){
        path.waypoints.push_back(nodes[parents[0]]);
        parents = graphPtr->parents(parents[0]);
    }

    reverse(path.waypoints.begin(), path.waypoints.end());

//    if (path_smoothing){
//        size_t iterations = path.waypoints.size();
//        for (size_t i = 0; i < iterations; i++){
//            int j = amp::RNG::randi(0, path.waypoints.size());
//            int k = amp::RNG::randi(0, path.waypoints.size());
//            if ((abs(j - k) > 1) && !check_chain_collisions(std::vector<Eigen::Vector2d>{path.waypoints[i], path.waypoints[k]}, problem.obstacles)){
//                for (size_t l = 1; l < abs(j - k); l++){
//                    path.waypoints.erase(path.waypoints.begin() + std::min(j, k) + 1);
//                }
//            }
//        }
//    }

//    path.waypoints.push_back(problem.q_init);
//    path.waypoints.push_back(problem.q_goal);

    return path;
}