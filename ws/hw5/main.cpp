// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW5.h"
#include "hw/HW2.h"

// Include any custom headers you created in your workspace
#include "MyGDAlgorithm.h"

using namespace amp;

int main(int argc, char** argv) {
    /* Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    amp::RNG::seed(amp::RNG::randiUnbounded());

    double d_star = 1;
    double zetta = 2;
    double Q_star = 1;
    double eta = 0.1;

    // Test your gradient descent algorithm on a random problem.
    MyGDAlgorithm algo(d_star, zetta, Q_star, eta);
//    Problem2D prob1 = HW5::getWorkspace1();
//    Path2D path1 = algo.plan(prob1);
//    HW5::check(path1, prob1);

    Problem2D prob2 = HW2::getWorkspace1();
    Path2D path2 = algo.plan(prob2);
    HW5::check(path2, prob2);

//    Problem2D prob3 = HW2::getWorkspace2();
//    Path2D path3 = algo.plan(prob3);
//    HW5::check(path3, prob3);

//    Problem2D prob4;
//    Path2D path4;
//    HW5::generateAndCheck(algo, path4, prob4);
//
//    LOG(path.waypoints.back());
//    LOG(path.waypoints[0]);
//    LOG(path.waypoints.size());

    //Visualizer::makeFigure(prob1, path1);
    Visualizer::makeFigure(prob2, path2);
    //Visualizer::makeFigure(prob3, path3);

    // Visualize potential function
    //Visualizer::makeFigure(MyPotentialFunction{d_star, zetta, Q_star, eta, prob1}, prob1, 50);
    Visualizer::makeFigure(MyPotentialFunction{d_star, zetta, Q_star, eta, prob2}, prob2, 100);
    //Visualizer::makeFigure(MyPotentialFunction{d_star, zetta, Q_star, eta, prob3}, prob3, 50);

    //Visualizer::makeFigure(prob4, path4);

    Visualizer::saveFigures(true, "hw5_figs");
    
    // Arguments following argv correspond to the constructor arguments of MyGDAlgorithm:
    HW5::grade<MyGDAlgorithm>("Katrina.Braun@colorado.edu", argc, argv, d_star, zetta, Q_star, eta);
    return 0;
}