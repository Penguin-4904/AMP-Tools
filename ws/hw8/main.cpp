// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "hw/HW8.h"
#include "MyMultiAgentPlanners.h"

using namespace amp;

void timer_example() {
    double startTime;
    amp::Timer timer("timer");
    for (int i=0; i < 5; ++i) {
        startTime = timer.now(TimeUnit::ms);  
        std::cout << "Press any key to continue...\n";
        std::cin.get();
        std::cout << "Time since last run: " << timer.now(TimeUnit::ms) - startTime << std::endl;
    }
    timer.stop();
    std::cout << "Total time since last run: " << Profiler::getTotalProfile("timer") << std::endl;
}

void benchmark_multiagent_algo(MyCentralPlanner& algo, const std::vector<MultiAgentProblem2D>& problems, const size_t numBenchmarks = 100){

    size_t numProblems = problems.size();
    std::vector<std::string> labels;

    std::list<std::vector<double>> time_datas;
    std::list<std::vector<double>> size_datas;

    MultiAgentPath2D path;
    std::vector<std::vector<Eigen::Vector2d>> collision_states;

    for (size_t j = 0; j < numProblems; j++){
        std::vector<double> time_data;
        std::vector<double> size_data;
        for (size_t i = 0; i < numBenchmarks; i++){
            std::cout << "Problem: " << j << " Test: " << i << std::endl;
            Timer timer("timer 1");
            path = algo.plan(problems[j]);
            timer.stop();
            time_data.push_back(timer.now());
            size_data.push_back(algo.get_size());
        }

        collision_states= {{}};
        bool isValid = HW8::check(path, problems[j], collision_states, false);
        Visualizer::makeFigure(problems[j], path, collision_states);

        time_datas.push_back(time_data);
        size_datas.push_back(size_data);
        labels.push_back(std::to_string(problems[j].numAgents()));
    }

    Visualizer::makeBoxPlot(time_datas, labels, "Centralized Multi Agent RRT Planner Computation Time (" + std::to_string(numBenchmarks) + " Trials)", "# of Agents", "Time (ms)");
    Visualizer::makeBoxPlot(size_datas, labels, "Centralized Multi Agent RRT Planner Tree Size ("  + std::to_string(numBenchmarks) +  " Trials)", "# of Agents", "Tree Size");
}

void benchmark_multiagent_algo(MyDecentralPlanner& algo, const std::vector<MultiAgentProblem2D>& problems, const size_t numBenchmarks = 100){

    size_t numProblems = problems.size();
    std::vector<std::string> labels;

    std::list<std::vector<double>> time_datas;
    std::list<std::vector<double>> size_datas;

    MultiAgentPath2D path;
    std::vector<std::vector<Eigen::Vector2d>> collision_states;

    for (size_t j = 0; j < numProblems; j++){
        std::vector<double> time_data;
        std::vector<double> size_data;
        size_t num_success = 0;
        for (size_t i = 0; i < numBenchmarks; i++){
            std::cout << "Problem: " << j << " Test: " << i << std::endl;
            Timer timer("timer 1");
            path = algo.plan(problems[j]);
            timer.stop();
            time_data.push_back(timer.now());
            size_data.push_back(algo.get_size());

            collision_states= {{}};
            bool isValid = HW8::check(path, problems[j], collision_states, false);
            if (isValid) {
                num_success++;
            }

        }
        LOG("Num Success: " << num_success);
        Visualizer::makeFigure(problems[j], path, collision_states);

        time_datas.push_back(time_data);
        size_datas.push_back(size_data);
        labels.push_back(std::to_string(problems[j].numAgents()));
    }

    Visualizer::makeBoxPlot(time_datas, labels, "Decentralized Multi Agent RRT Planner Computation Time (" + std::to_string(numBenchmarks) + " Trials)", "# of Agents", "Time (ms)");
    Visualizer::makeBoxPlot(size_datas, labels, "Decentralized Multi Agent RRT Planner Tree Size ("  + std::to_string(numBenchmarks) +  " Trials)", "# of Agents", "Tree Size");
}


int main(int argc, char** argv) {

    amp::RNG::seed(amp::RNG::randiUnbounded());

    MultiAgentPath2D path;
    MultiAgentProblem2D problem = HW8::getWorkspace1(3);
    std::vector<MultiAgentProblem2D> problems;
    for (size_t i = 2; i < 7; i++) {
        problems.push_back(HW8::getWorkspace1(i));
    }
    std::vector<std::vector<Eigen::Vector2d>> collision_states;

    // Solve using a centralized approach
    MyCentralPlanner central_planner;
    central_planner.set_n(1000000);
    //central_planner.set_r(2.5);
    //central_planner.set_eps(1.25);
    int i = 4;
//    Path2D agent_path;
//    agent_path.waypoints = {problems[i].agent_properties[0].q_init, {2, 8}, {0, 8}, {0, 16}, problems[i].agent_properties[0].q_goal};
//    path.agent_paths.push_back(agent_path);
//    agent_path.waypoints = {problems[i].agent_properties[1].q_init, {2, 12}, {2, 8}, {2, 4}, problems[i].agent_properties[1].q_goal};
//    path.agent_paths.push_back(agent_path);

//    path = central_planner.plan(problems[i]);
//    bool isValid = HW8::check(path, problems[i], collision_states);
//    Visualizer::makeFigure(problems[i], path, collision_states);
//    benchmark_multiagent_algo(central_planner, problems);
//    path = central_planner.plan(problem);
//    bool isValid = HW8::check(path, problem, collision_states);
//    Visualizer::makeFigure(problem, path, collision_states);

    // Solve using a decentralized approach
    MyDecentralPlanner decentral_planner;
    path = decentral_planner.plan(problems[i]);
    bool isValid = HW8::check(path, problems[i], collision_states);
    Visualizer::makeFigure(problems[i], path, collision_states);

    for (const std::vector<Eigen::Vector2d> colls : collision_states){
        LOG("Robot");
        for (const Eigen::Vector2d coll : colls){
            LOG(coll);
        }
    }

    // LOG();
    benchmark_multiagent_algo(decentral_planner, {problems[i]});
    // LOG(std::pow(2, -1));
    collision_states = {{}};


    // Visualize and grade methods
    Visualizer::saveFigures(true, "hw8_figs");
    // HW8::grade<MyCentralPlanner, MyDecentralPlanner>("firstName.lastName@colorado.edu", argc, argv, std::make_tuple(), std::make_tuple());
    return 0;
}