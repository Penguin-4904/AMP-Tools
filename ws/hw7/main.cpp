// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "hw/HW2.h"
#include "hw/HW5.h"
#include "MySamplingBasedPlanners.h"

using namespace amp;

void benchmark_algo(MyPMP2D& algo, const Problem2D& problem, const std::vector<std::pair<size_t, double>> benchmarks, const std::string& title = std::string()){
    std::vector<std::string> labels;
    Path2D path;
    std::list<std::vector<double>> time_datas;
    std::list<std::vector<double>> length_datas;

    for (size_t i = 0; i < benchmarks.size(); i++) {
        algo.set_n(benchmarks[i].first);
        algo.set_r(benchmarks[i].second);
        std::vector<double> time_data;
        std::vector<double> length_data;
        for (size_t i = 0; i < 100; i++){
            Timer timer("timer 1");
            path = algo.plan(problem);
            timer.stop();
            if (HW7::check(path, problem, false)) {
                time_data.push_back(timer.now());
                length_data.push_back(path.length());
            }
        }
        time_datas.push_back(time_data);
        length_datas.push_back(length_data);
        labels.push_back("n=" + std::to_string(benchmarks[i].first) + "\nr=" + std::to_string(benchmarks[i].second).substr(0, 4));
        LOG(title << ": " << labels[i] << " # of valid solutions: " << time_data.size());
    }

    Visualizer::makeBoxPlot(time_datas, labels, title + " Computation Time (100 Trials)", "n = # of samples, r = connection radius", "Time (ms)");
    Visualizer::makeBoxPlot(length_datas, labels, title + " Path Length (100 Trials)", "n = # of samples, r = connection radius", "Path Length");

}

void benchmark_algo(MyPMP2D& algo, std::vector<Problem2D> problems, const std::vector<std::string>& problem_labels, const std::string& title = std::string()){

    Path2D path;
    std::list<std::vector<double>> time_datas;
    std::list<std::vector<double>> length_datas;

    for (size_t i = 0; i < problems.size(); i++) {
        std::vector<double> time_data;
        std::vector<double> length_data;
        for (size_t j = 0; j < 100; j++){
            Timer timer("timer 1");
            path = algo.plan(problems[i]);
            timer.stop();
            if (HW7::check(path, problems[i], false)) {
                time_data.push_back(timer.now());
                length_data.push_back(path.length());
            }
        }
        time_datas.push_back(time_data);
        length_datas.push_back(length_data);
        LOG(title << ": " << problem_labels[i] << " # of valid solutions: " << time_data.size());
    }

    Visualizer::makeBoxPlot(time_datas, problem_labels, title + " Computation Time (100 Trials)", "n = # of samples, r = connection radius", "Time (ms)");
    Visualizer::makeBoxPlot(length_datas, problem_labels, title + " Path Length (100 Trials)", "n = # of samples, r = connection radius", "Path Length");

}

int main(int argc, char** argv) {
//    HW7::hint(); // Consider implementing an N-dimensional planner

    MyPRM prm;
    Problem2D problem;
    Path2D path;

    // Ex 1 (a)
    problem = HW5::getWorkspace1();

    problem.x_min = -1;
    problem.x_max = 11;

    problem.y_min = -3;
    problem.y_max = 3;

    std::vector<Problem2D> problems;
    problems.push_back(problem);

    prm.set_n(200);
    prm.set_r(1);

    path = prm.plan(problem);
    LOG("PRM Path Length Ex 1.(a): " << path.length());
    Visualizer::makeFigure(problem, path, *prm.get_graphPtr(), prm.get_nodes());

    std::vector<std::pair<size_t, double>> benchmarks = {{200, 0.5}, {200, 1}, {200, 1.5}, {200, 2}, {500, 0.5}, {500, 1}, {500, 1.5}, {500, 2}};
    benchmark_algo(prm, problem, benchmarks, "Ex 1.(a) (HW5 Ex 2.(a))");
    prm.set_path_smoothing(true);
    benchmark_algo(prm, problem, benchmarks, "Ex 1.(a) (HW5 Ex 2.(a)) \\w smoothing");

    // Ex 1.(b)
    problem = HW2::getWorkspace1();
    problems.push_back(problem);

    prm.set_n(200);
    prm.set_r(2);
    prm.set_path_smoothing(false);

    path = prm.plan(problem);
    LOG("PRM Path Length Ex 1.(b) W1: " << path.length());
    Visualizer::makeFigure(problem, path, *prm.get_graphPtr(), prm.get_nodes());

    prm.set_n(500);

    path = prm.plan(problem);
    LOG("PRM Path Length Ex 1.(b) W1 try 2: " << path.length());
    Visualizer::makeFigure(problem, path, *prm.get_graphPtr(), prm.get_nodes());

    benchmarks = {{200, 1}, {200, 2}, {500, 1}, {500, 2}, {1000, 1}, {1000, 2}};
    benchmark_algo(prm, problem, benchmarks, "Ex 1.(b) W1 (HW2 Ex 2)");
    prm.set_path_smoothing(true);
    benchmark_algo(prm, problem, benchmarks, "Ex 1.(b) W1 (HW2 Ex 2) \\w smoothing");

    problem = HW2::getWorkspace2();
    problems.push_back(problem);

    prm.set_n(200);
    prm.set_r(2);
    prm.set_path_smoothing(false);

    path = prm.plan(problem);
    LOG("PRM Path Length Ex 1.(b) W2: " << path.length());
    Visualizer::makeFigure(problem, path, *prm.get_graphPtr(), prm.get_nodes());

    prm.set_n(1000);

    path = prm.plan(problem);
    LOG("PRM Path Length Ex 1.(b) W2 2nd try: " << path.length());
    Visualizer::makeFigure(problem, path, *prm.get_graphPtr(), prm.get_nodes());

    benchmark_algo(prm, problem, benchmarks, "Ex 1.(b) W2 (HW2 Ex 2)");
    prm.set_path_smoothing(true);
    benchmark_algo(prm, problem, benchmarks, "Ex 1.(b) W2 (HW2 Ex 2) \\w smoothing");

    // Ex 2.(a)
    MyRRT rrt;
    rrt.set_n(5000);
    rrt.set_r(0.5);

    path = rrt.plan(problems[0]);
    LOG("RRT Path Length Ex 1.(a): " << path.length());
    Visualizer::makeFigure(problems[0], path, *rrt.get_graphPtr(), rrt.get_nodes());

    path = rrt.plan(problems[1]);
    LOG("RRT Path Length Ex 1.(b) W1: " << path.length());
    Visualizer::makeFigure(problems[1], path, *rrt.get_graphPtr(), rrt.get_nodes());

    path = rrt.plan(problems[2]);
    LOG("RRT Path Length Ex 1.(b) W2: " << path.length());
    Visualizer::makeFigure(problems[2], path, *rrt.get_graphPtr(), rrt.get_nodes());

    // Ex 2.(b)
    benchmark_algo(rrt, problems, {"Ex 1.(a)", "Ex 1.(b) W1", "Ex 1.(b) W2"}, "Ex 2.(b) (RRT Benchmarks)");

    Visualizer::saveFigures(true, "hw7_figs");

    prm.set_n(1000);
    prm.set_r(2);
    prm.set_path_smoothing(false);

    HW7::grade(prm, rrt, "Katrina.Braun@colorado.edu", argc, argv);
    return 0;
}