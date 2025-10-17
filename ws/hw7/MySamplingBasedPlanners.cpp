# include "MySamplingBasedPlanners.h"

// Implement your PRM algorithm here
amp::Path2D MyPRM::plan(const amp::Problem2D& problem) {

    //LOG("Start");
    nodes.clear();
    graphPtr->clear();

    std::vector<Eigen::Vector2d> points;

//    points.push_back(problem.q_init);
//    points.push_back(problem.q_goal);

    std::pair<amp::Node, double> best_dist_init(0, (problem.q_init - problem.q_goal).norm());
    std::pair<amp::Node, double> best_dist_goal(0, best_dist_init.second);

    std::vector<std::tuple<amp::Node, amp::Node, double>> edges;

    for (size_t i = 0; i < n; i++){
        Eigen::Vector2d point(amp::RNG::randd(problem.x_min, problem.x_max), amp::RNG::randd(problem.y_min, problem.y_max));
        if (!check_collisions(point, problem.obstacles)){
            for (size_t j = 0; j < points.size(); j++){
                double dist = (point - points[j]).norm();
                if ((dist <= r) && !check_chain_collisions(std::vector<Eigen::Vector2d>{point, points[j]}, problem.obstacles)){
                    edges.push_back({points.size(), j, dist});
                    edges.push_back({j, points.size(), dist});
                }
            }

            if (((point - problem.q_init).norm() <= best_dist_init.second) &&
                !check_chain_collisions(std::vector<Eigen::Vector2d>{point, problem.q_init}, problem.obstacles)){
                best_dist_init.first = points.size();
                best_dist_init.second = (point - problem.q_init).norm();
            }

            if (((point - problem.q_goal).norm() <= best_dist_goal.second) &&
                !check_chain_collisions(std::vector<Eigen::Vector2d>{point, problem.q_goal}, problem.obstacles)){
                best_dist_goal.first = points.size();
                best_dist_goal.second = (point - problem.q_goal).norm();
            }

            points.push_back(point);
        }
    }

    edges.push_back({points.size(), best_dist_goal.first, best_dist_goal.second});
    edges.push_back({best_dist_goal.first, points.size(), best_dist_goal.second});
    points.push_back(problem.q_goal);

    edges.push_back({points.size(), best_dist_init.first, best_dist_init.second});
    edges.push_back({best_dist_init.first, points.size(), best_dist_init.second});
    points.push_back(problem.q_init);

//    LOG("Point Size: " << points.size());
//    LOG("Edges Size: " << edges.size());

    for (amp::Node i = 0; i < points.size(); ++i) nodes[i] = points[i];
    for (const auto& [from, to, weight] : edges) graphPtr->connect(from, to, weight);

//    graphPtr->children(0);
//    graphPtr->print();

    LookupSearchHeuristic dist_heuristic;

    for (amp::Node i = 0; i < points.size(); ++i) dist_heuristic.heuristic_values[i] = (points[i] - problem.q_goal).norm();

    MyAStarAlgo a_star;

    amp::ShortestPathProblem graph_problem;

    graph_problem.graph = graphPtr;
    graph_problem.init_node = points.size() - 1;
    graph_problem.goal_node = points.size() - 2;

//    LOG("Heuristic " << dist_heuristic.heuristic_values.size());
    MyAStarAlgo::GraphSearchResult result = a_star.search(graph_problem, dist_heuristic);

//    LOG("Result: " << result.success);

    amp::Path2D path;

    for (std::list<amp::Node>::iterator i = result.node_path.begin(); i != result.node_path.end(); i++){
        path.waypoints.push_back(nodes.at(*i));
    }

    return path;
}


// Implement your RRT algorithm here
amp::Path2D MyRRT::plan(const amp::Problem2D& problem) {



    amp::Path2D path;
    path.waypoints.push_back(problem.q_init);
    path.waypoints.push_back(problem.q_goal);
    return path;
}