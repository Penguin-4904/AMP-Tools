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

/// @brief Benchmarks my centralized planner
void benchmark_multiagent_algo(MyCentralPlanner& algo, const std::vector<MultiAgentProblem2D>& problems, const size_t numBenchmarks = 100){

    size_t numProblems = problems.size();
    std::vector<std::string> labels;

    std::list<std::vector<double>> time_datas;
    std::list<std::vector<double>> size_datas;

    std::vector<double> average_times;
    std::vector<double> average_sizes;

    MultiAgentPath2D path;
    std::vector<std::vector<Eigen::Vector2d>> collision_states;

    for (size_t j = 0; j < numProblems; j++){
        std::vector<double> time_data;
        std::vector<double> size_data;
        double average_time = 0;
        double average_size = 0;
        size_t num_success = 0;

        MultiAgentPath2D valid_path;

        for (size_t i = 0; i < numBenchmarks; i++){
            std::cout << "Problem: " << j << " Test: " << i << std::endl;
            Timer timer("timer 1");
            path = algo.plan(problems[j]);
            timer.stop();

            time_data.push_back(timer.now());
            size_data.push_back(algo.get_size());

            average_time += timer.now();
            average_size += algo.get_size();

            if (HW8::check(path, problems[j], false)) {


                valid_path = path;
                num_success++;
            }
        }
        LOG("Num Success: " << num_success);
        if (valid_path.agent_paths.size() != 0){
            collision_states= {{}};
            bool isValid = HW8::check(valid_path, problems[j], collision_states, false);
            Visualizer::makeFigure(problems[j], valid_path, collision_states);
        }

        average_size = average_size / numBenchmarks;
        average_time = average_time / numBenchmarks;

        time_datas.push_back(time_data);
        size_datas.push_back(size_data);

        average_times.push_back(average_time);
        average_sizes.push_back(average_size)
        ;
        labels.push_back(std::to_string(problems[j].numAgents()));
    }

    Visualizer::makeBoxPlot(time_datas, labels, "Centralized Multi Agent RRT Computation Time (" + std::to_string(numBenchmarks) + " Trials)", "# of Agents", "Time (ms)");
    Visualizer::makeBoxPlot(size_datas, labels, "Centralized Multi Agent RRT Tree Size ("  + std::to_string(numBenchmarks) +  " Trials)", "# of Agents", "Tree Size");

    Visualizer::makeBarGraph(average_times, labels, "Centralized Multi Agent RRT Average Computation Time (" + std::to_string(numBenchmarks) + " Trials)", "# of Agents", "Time (ms)");
    Visualizer::makeBarGraph(average_sizes, labels, "Centralized Multi Agent RRT Average Tree Size (" + std::to_string(numBenchmarks) + " Trials)", "# of Agents", "Tree Size");

    for (int i = 0; i < numProblems; i++){
        auto time = std::next(time_datas.begin(), i);
        auto size = std::next(size_datas.begin(), i);
        Visualizer::makeBoxPlot({*time}, {labels[i]}, "Centralized Multi Agent RRT Computation Time (" + std::to_string(numBenchmarks) + " Trials)", "# of Agents", "Time (ms)");
        Visualizer::makeBoxPlot({*size}, {labels[i]}, "Centralized Multi Agent RRT Tree Size ("  + std::to_string(numBenchmarks) +  " Trials)", "# of Agents", "Tree Size");

    }
}

/// @brief Benchmarks my decentralized planner (slightly different from above)
void benchmark_multiagent_algo(MyDecentralPlanner& algo, const std::vector<MultiAgentProblem2D>& problems, const size_t numBenchmarks = 100){

    size_t numProblems = problems.size();
    std::vector<std::string> labels;

    std::list<std::vector<double>> time_datas;
    std::list<std::vector<double>> size_datas;

    std::vector<double> average_times;
    std::vector<double> average_sizes;

    MultiAgentPath2D path;
    std::vector<std::vector<Eigen::Vector2d>> collision_states;

    for (size_t j = 0; j < numProblems; j++){
        std::vector<double> time_data;
        std::vector<double> size_data;
        double average_time = 0;
        double average_size = 0;
        size_t num_success = 0;

        MultiAgentPath2D valid_path;

        for (size_t i = 0; i < numBenchmarks; i++){
            std::cout << "Problem: " << j << " Test: " << i << std::endl;
            Timer timer("timer 1");
            path = algo.plan(problems[j]);
            timer.stop();

            if (HW8::check(path, problems[j], false)) {

                time_data.push_back(timer.now());
                size_data.push_back(algo.get_size());

                average_time += timer.now();
                average_size += algo.get_size();

                valid_path = path;
                num_success++;
            }
        }
        LOG("Num Success: " << num_success);

        collision_states= {{}};
        bool isValid = HW8::check(valid_path, problems[j], collision_states, false);
        Visualizer::makeFigure(problems[j], valid_path, collision_states);

        average_size = average_size / num_success;
        average_time = average_time / num_success;

        time_datas.push_back(time_data);
        size_datas.push_back(size_data);

        average_times.push_back(average_time);
        average_sizes.push_back(average_size);

        labels.push_back(std::to_string(problems[j].numAgents()));
    }

    Visualizer::makeBoxPlot(time_datas, labels, "Decentralized Multi Agent RRT Computation Time (" + std::to_string(numBenchmarks) + " Trials)", "# of Agents", "Time (ms)");
    // Visualizer::makeBoxPlot(size_datas, labels, "Decentralized Multi Agent RRT Planner Tree Size ("  + std::to_string(numBenchmarks) +  " Trials)", "# of Agents", "Tree Size");

    Visualizer::makeBarGraph(average_times, labels, "Decentralized Multi Agent RRT Average Computation Time (" + std::to_string(numBenchmarks) + " Trials)", "# of Agents", "Time (ms)");
    // Visualizer::makeBarGraph(average_sizes, labels, "Decentralized Multi Agent RRT Planner Average Tree Size (" + std::to_string(numBenchmarks) + " Trials)", "# of Agents", "Tree Size");

    for (int i = 0; i < numProblems; i++) {
        auto time = std::next(time_datas.begin(), i);
        // auto size = std::next(size_datas.begin(), i);

        Visualizer::makeBoxPlot({*time}, {labels[i]}, "Decentralized Multi Agent RRT Computation Time (" +
                                                      std::to_string(numBenchmarks) + " Trials)", "# of Agents",
                                "Time (ms)");

        if ((i != 0) && (time->size() > 5)) {

            std::sort(time->begin(), time->end());
            // std::sort(size->begin(), size->end());

            while (time->back() > 20) {
                time->pop_back();
                // size->pop_back();
            }
        }
    }

    Visualizer::makeBoxPlot(time_datas, labels, "Decentralized Multi Agent RRT Computation Time (Benchmarks below 20ms)", "# of Agents", "Time (ms)");
    // Visualizer::makeBoxPlot(size_datas, labels, "Decentralized Multi Agent RRT Planner Tree Size ("  + std::to_string(numBenchmarks) +  " Trials)", "# of Agents", "Tree Size");

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

    benchmark_multiagent_algo(central_planner, problems);
    //benchmark_multiagent_algo(central_planner, problems, 1000);

    // Solve using a decentralized approach
    MyDecentralPlanner decentral_planner;

//    int i = 2;
//    path = decentral_planner.plan(problems[i]);
//    bool isValid = HW8::check(path, problems[i], collision_states);
//    Visualizer::makeFigure(problems[i], path, collision_states);

    benchmark_multiagent_algo(decentral_planner, problems);

    // Visualize and grade methods

    central_planner.set_r_scaling(true);
    central_planner.set_n(50000);

    Visualizer::saveFigures(true, "hw8_figs");
    HW8::grade(central_planner, decentral_planner, "Katrina.Braun@colorado.edu", argc, argv);
    return 0;
}