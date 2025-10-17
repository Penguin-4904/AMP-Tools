#include "MyAStar.h"

/// @breif A-Star implementation returns shortest path given a shortest path problem and a heuristic.
MyAStarAlgo::GraphSearchResult MyAStarAlgo::search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic) {
    // std::cout << "Starting A* Graph Search: Init --> goal | " << problem.init_node << " --> " << problem.goal_node << std::endl;
    std::vector<GraphSearchResult> open_list;
    std::vector<amp::Node> closed_list;
    GraphSearchResult result = {false, {}, 0.0}; // initialize the results object
    result.node_path.push_back(problem.init_node);
    result.path_cost += heuristic(problem.init_node);

    open_list.push_back(result);

    size_t count = 1;
    while (open_list.size() > 0) {

        std::ranges::sort(open_list, [] (const GraphSearchResult& node_a, const GraphSearchResult& node_b){return node_a.path_cost > node_b.path_cost;});

        result = open_list.back();
        open_list.pop_back();

        if (result.node_path.back() == problem.goal_node) {
            result.success = true;
            break;
        }

        closed_list.push_back(result.node_path.back());

        const std::vector<amp::Node>& next_nodes = problem.graph->children(result.node_path.back());
        const std::vector<double>& next_edges = problem.graph->outgoingEdges(result.node_path.back());

        for (int i = 0; i < next_nodes.size(); i++){
            if (find(closed_list.begin(), closed_list.end(), next_nodes[i]) == closed_list.end()){
                open_list.push_back(result);
                open_list.back().path_cost += (next_edges[i] + heuristic(next_nodes[i]) - heuristic(result.node_path.back()));
                open_list.back().node_path.push_back(next_nodes[i]);
                auto duplicate = find_if(open_list.begin(), open_list.end(), [&next_nodes, i] (const GraphSearchResult& node_in) {return node_in.node_path.back() == next_nodes[i];});
                if (duplicate != open_list.end() - 1) {
                    if (duplicate->path_cost > open_list.back().path_cost){
                        open_list.erase(duplicate);
                    } else {
                        open_list.pop_back();
                    }
                }
            }
        }
        count++;

    }

    // LOG("# of iterations: " << count);
    // result.print();
    return result;
}
