// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "hw/HW2.h"
#include "hw/HW5.h"
#include "MySamplingBasedPlanners.h"

using namespace amp;

void benchmark_algo(PointMotionPlanner2D& algo, const Problem2D& problem, const std::vector<std::pair<size_t, double>> benchmarks, const std::string& title = std::string()){
    std::vector<std::string> labels;
    Path2D path;
    std::list<std::vector<double>> time_datas;
    std::list<std::vector<double>> length_datas;

    for (size_t i = 0; i < benchmarks.size(); i++) {
        prm.set_n(benchmarks[i].first);
        prm.set_r(benchmarks[i].second);
        std::vector<double> time_data;
        std::vector<double> length_data;
        for (size_t i = 0; i < 100; i++){
            Timer timer("timer 1");
            path = prm.plan(problem);
            timer.stop();
            if (HW7::check(path, problem, false)) {
                time_data.push_back(timer.now());
                length_data.push_back(path.length());
            }
        }
        time_datas.push_back(time_data);
        length_datas.push_back(length_data);
        labels.push_back("n=" << benchmarks[i].first << "\nr=" << benchmarks[i].second);
        LOG(labels[i] << " # of valid solutions: " << time_data.size());
    }

    Visualizer::makeBoxPlot(time_datas, labels, title << " Computation Time Benchmarks (100 Trials of Each)", "n = # of samples, r = connection radius", "Time (ms)");
    Visualizer::makeBoxPlot(length_datas, labels, title << " Path Length Benchmarks (100 Trials of Each)", "n = # of samples, r = connection radius", "Path Length");

}

int main(int argc, char** argv) {
    HW7::hint(); // Consider implementing an N-dimensional planner 

//    // Example of creating a graph and adding nodes for visualization
    std::shared_ptr<amp::Graph<double>> graphPtr = std::make_shared<amp::Graph<double>>();
    std::map<amp::Node, Eigen::Vector2d> nodes;
//
//    std::vector<Eigen::Vector2d> points = {{3, 3}, {4, 5}, {5, 3}, {6, 5}, {5, 7}, {7, 3}}; // Points to add to the graph
//    for (amp::Node i = 0; i < points.size(); ++i) nodes[i] = points[i]; // Add point-index pair to the map
//    std::vector<std::tuple<amp::Node, amp::Node, double>> edges = {{0, 4, 1}, {0, 5, 1}, {4, 5, 1}, {1, 2, 1}, {1, 3, 1}, {2, 3, 1}}; // Edges to connect
//    for (const auto& [from, to, weight] : edges) graphPtr->connect(from, to, weight); // Connect the edges in the graph
//    graphPtr->print();

    // Ex 1 (a)
    Problem2D problem = HW5::getWorkspace1();

    problem.x_min = -1;
    problem.x_max = 11;

    problem.y_min = -3;
    problem.y_max = 3;

    MyPRM prm;
    prm.set_n(200);
    prm.set_r(1);

    Path2D path = prm.plan(problem);
    LOG("Path Length Ex 1. (a): " << path.length());
    Visualizer::makeFigure(problem, path, *prm.get_graphPtr(), prm.get_nodes());

    std::vector<std::pair<size_t, double>> benchmarks = {{200, 0.5}, {200, 1}, {200, 1.5}, {200, 2}, {500, 0.5}, {500, 1}, {500, 1.5}, {500, 2}};
    benchmark_algo(prm, problem, bechmarks, "Ex 1.(a) (HW5 Ex 2.(a))");

    problem

    prm.set_n(200);
    prm.set_r(1);

    Path2D path = prm.plan(problem);
    LOG("Path Length Ex 1. (a): " << path.length());
    Visualizer::makeFigure(problem, path, *prm.get_graphPtr(), prm.get_nodes());



    // Generate a random problem and test RRT
//    MyRRT rrt;
//    // Path2D path;
//    HW7::generateAndCheck(rrt, path, problem);
//    Visualizer::makeFigure(problem, path, *graphPtr, nodes);
    Visualizer::saveFigures(true, "hw7_figs");
//
//    // Grade method
//    HW7::grade<MyPRM, MyRRT>("firstName.lastName@colorado.edu", argc, argv, std::make_tuple(), std::make_tuple());
//    return 0;
}