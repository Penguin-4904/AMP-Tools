// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW2.h"

// Include any custom headers you created in your workspace
#include "Bug1.h"
#include "Bug2.h"

using namespace amp;

int main(int argc, char** argv) {
    /*    Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    amp::RNG::seed(amp::RNG::randiUnbounded());

    /*    Randomly generate the problem     */ 

    // Use WO1 from Exercise 2
    Problem2D problem1 = HW2::getWorkspace1();

    // Use WO2 from Exercise 2
    Problem2D problem2 = HW2::getWorkspace2();

    Bug1 algo1;
    {
        // Call algorithm on the problem
        amp::Path2D path = algo1.plan(problem1);

        // Check path to make sure that it does not collide with the environment
        bool success = HW2::check(path, problem1);

        // Log results
        LOG("Found valid solution to workspace 1 (w/ Bug1): " << (success ? "Yes!" : "No :("));
        LOG("path length: " << path.length());

        // Visualize the path and environment
        Visualizer::makeFigure(problem1, path);
    }

    {
        // Call algorithm on the problem
        amp::Path2D path = algo1.plan(problem2);

        // Check path to make sure that it does not collide with the environment
        bool success = HW2::check(path, problem2);

        // Log results
        LOG("Found valid solution to workspace 2 (w/ Bug1): " << (success ? "Yes!" : "No :("));
        LOG("path length: " << path.length());

        // Visualize the path and environment
        Visualizer::makeFigure(problem2, path);
    }

    Bug2 algo2;
    {
        // Call algorithm on the problem
        amp::Path2D path = algo2.plan(problem1);

        // Check path to make sure that it does not collide with the environment
        bool success = HW2::check(path, problem1);

        // Log results
        LOG("Found valid solution to workspace 1 (w/ Bug2): " << (success ? "Yes!" : "No :("));
        LOG("path length: " << path.length());

        // Visualize the path and environment
        Visualizer::makeFigure(problem1, path);
    }

    {
        // Call algorithm on the problem
        amp::Path2D path = algo2.plan(problem2);

        // Check path to make sure that it does not collide with the environment
        bool success = HW2::check(path, problem2);

        // Log results
        LOG("Found valid solution to workspace 2 (w/ Bug2): " << (success ? "Yes!" : "No :("));
        LOG("path length: " << path.length());

        // Visualize the path and environment
        Visualizer::makeFigure(problem2, path);
    }

    Visualizer::saveFigures(true, "hw2_figs");

//    LOG("Grade Bug1: ");
//    HW2::grade(algo1, "Katrina.Braun@colorado.edu", argc, argv);
    LOG("Grade Bug2: ");
    HW2::grade(algo2, "Katrina.Braun@colorado.edu", argc, argv);
    
    /* If you want to reconstruct your bug algorithm object every trial (to reset member variables from scratch or initialize), use this method instead*/
    //HW2::grade<Bug1>("nonhuman.biologic@myspace.edu", argc, argv, constructor_parameter_1, constructor_parameter_2, etc...);
    
    // This will reconstruct using the default constructor every trial
    //HW2::grade<Bug1>("nonhuman.biologic@myspace.edu", argc, argv);

    return 0;
}