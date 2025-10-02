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
    double zetta = 5;
    double Q_star = 5;
    double eta = 0.1;

    // Test your gradient descent algorithm on a random problem.
    MyGDAlgorithm algo(d_star, zetta, Q_star, eta);
    Problem2D prob = HW2::getWorkspace1();
    Path2D path = algo.plan(prob);
    bool success = HW5::check(path, prob);
//
//    LOG(path.waypoints.back());
//    LOG(path.waypoints[0]);
//    LOG(path.waypoints.size());

    Visualizer::makeFigure(prob, path);

    // Visualize your potential function
    Visualizer::makeFigure(MyPotentialFunction{d_star, zetta, Q_star, eta, prob}, prob, 100);
    Visualizer::saveFigures(true, "hw5_figs");
    
    // Arguments following argv correspond to the constructor arguments of MyGDAlgorithm:
    HW5::grade<MyGDAlgorithm>("Katrina.Braun@colorado.edu", argc, argv, d_star, zetta, Q_star, eta);
    return 0;
}