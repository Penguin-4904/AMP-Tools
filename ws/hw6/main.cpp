#include "AMPCore.h"

#include "hw/HW2.h"
#include "hw/HW6.h"

#include "MyAStar.h"
#include "MyCSConstructors.h"
#include "Manipulator2D.h"

using namespace amp;

int main(int argc, char** argv) {
    /* Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    amp::RNG::seed(amp::RNG::randiUnbounded());

    // Pull in all required environments.
    Manipulator2D manipulator;
    Problem2D point_problem_1 = HW2::getWorkspace1();
    Problem2D point_problem_2 = HW2::getWorkspace2();

    Problem2D manip_problem_1 = HW6::getHW4Problem1();
    Problem2D manip_problem_2 = HW6::getHW4Problem2();
    Problem2D manip_problem_3 = HW6::getHW4Problem3();
    
    // Construct point-agent and manipulator cspace instances.
    std::shared_ptr<MyPointAgentCSConstructor> point_agent_ctor_1 = std::make_shared<MyPointAgentCSConstructor>((point_problem_1.x_max - point_problem_1.x_min)/0.25);
    std::shared_ptr<MyPointAgentCSConstructor> point_agent_ctor_2 = std::make_shared<MyPointAgentCSConstructor>((point_problem_2.x_max - point_problem_2.x_min)/0.25);

    std::shared_ptr<MyManipulatorCSConstructor> manipulator_ctor = std::make_shared<MyManipulatorCSConstructor>(50);
    std::shared_ptr<WaveFrontAlgorithm> wf_algo = std::make_shared<MyWaveFrontAlgorithm>();
    
    // Combine your wavefront planner with a cspace object (you do not need to modify these classes).
    PointWaveFrontAlgorithm point_algo_1(wf_algo, point_agent_ctor_1);
    PointWaveFrontAlgorithm point_algo_2(wf_algo, point_agent_ctor_2);
    ManipulatorWaveFrontAlgorithm manip_algo(wf_algo, manipulator_ctor);

    // Exercise 1:
    Path2D path_1 = point_algo_1.plan(point_problem_1);
    LOG("Homework 2 Exercise 2 Path Length 1: " << path_1.length());
    Visualizer::makeFigure(point_problem_1, path_1); // Visualize path in workspace
    Visualizer::makeFigure(*point_algo_1.getCSpace(), path_1); // Visualize path in cspace

    Path2D path_2 = point_algo_2.plan(point_problem_2);
    LOG("Homework 2 Exercise 2 Path Length 2: " << path_2.length());
    Visualizer::makeFigure(point_problem_2, path_2); // Visualize path in workspace
    Visualizer::makeFigure(*point_algo_2.getCSpace(), path_2); // Visualize path in cspace

    // Exercise 2:
    ManipulatorTrajectory2Link trajectory_1 = manip_algo.plan(manipulator, manip_problem_1);
    Visualizer::makeFigure(manip_problem_1, manipulator, trajectory_1);
    Visualizer::makeFigure(*manip_algo.getCSpace(), trajectory_1);

    ManipulatorTrajectory2Link trajectory_2 = manip_algo.plan(manipulator, manip_problem_2);
    Visualizer::makeFigure(manip_problem_2, manipulator, trajectory_2);
    Visualizer::makeFigure(*manip_algo.getCSpace(), trajectory_2);

    ManipulatorTrajectory2Link trajectory_3 = manip_algo.plan(manipulator, manip_problem_3);
    Visualizer::makeFigure(manip_problem_3, manipulator, trajectory_3);
    Visualizer::makeFigure(*manip_algo.getCSpace(), trajectory_3);

    // Exercise 3:
    ShortestPathProblem problem = HW6::getEx3SPP();
    LookupSearchHeuristic heuristic = HW6::getEx3Heuristic();
    MyAStarAlgo algo;
    MyAStarAlgo::GraphSearchResult result1 = algo.search(problem, heuristic);
    MyAStarAlgo::GraphSearchResult result2 = algo.search(problem, SearchHeuristic());

    Visualizer::saveFigures(true, "hw6_figs");

    manipulator_ctor = std::make_shared<MyManipulatorCSConstructor>(100);
    amp::HW6::grade<PointWaveFrontAlgorithm, ManipulatorWaveFrontAlgorithm, MyAStarAlgo>("Katrina.Braun@colorado.edu", argc, argv, std::make_tuple(wf_algo, point_agent_ctor_1), std::make_tuple(wf_algo, manipulator_ctor), std::make_tuple());
    return 0;
}