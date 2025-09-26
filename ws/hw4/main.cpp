// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW4.h"

// Include the headers for HW4 code
#include "CSpace.h"
#include "Manipulator2D.h"
#include "2DRidgidBodyCSpace.h"


using namespace amp;

int main(int argc, char** argv) {
    /* Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    RNG::seed(amp::RNG::randiUnbounded());

    // Exercise 1 Plotting
    std::vector<Eigen::Vector2d> vertices = {Eigen::Vector2d(0, 0), Eigen::Vector2d(1, 2), Eigen::Vector2d(0, 2)};
    std::size_t n = 12;
    std::vector<double> theta;
    for (int i = 0; i < n; i++){
        theta.push_back(i * 2 * M_PI / n);
    }

    Polygon r(vertices); // Robot
    Obstacle2D o(vertices); // Obstacle
    Obstacle2D obstacle_out; // C-Space Obstacle

    std::vector<Obstacle2D> o_vector = CSpacePolygonRotate(r, o, n); // C-Space Obstacles for rotating (& translating) robot
    Visualizer::makeFigure(o_vector, theta);

    // Exercise 2 Testing & Plotting
    // Part a)
    std::vector<double> links1 = {0.5, 1, 0.5};
    Manipulator2D manipulatorA(link1);

    ManipulatorState test_stateA(3);
    test_stateA << M_PI/6, M_PI/3, 7*M_PI/4;

    Visualizer::makeFigure(manipulatorA, test_stateA);

    std::vector<double> links2 = {1, 0.5, 1};
    Manipulator2D manipulatorB(link2);

    Eigen::Vector2d test_pointB(-0.6, 0.15);

    ManipulatorState test_stateB = manipulator.getConfigurationFromIK(test_pointB);

    Visualizer::makeFigure(manipulatorB, test_stateB);

    // Exercise 3:
    std::vector<double> links = {1, 1};
    Manipulator2D manip(links);

    ManipulatorState state(2);
    state << M_PI, M_PI;

    std::size_t n_cells = 200;
    MyManipulatorCSConstructor cspace_constructor(n_cells);

    // Part a)
    std::unique_ptr<amp::GridCSpace2D> cspacea = cspace_constructor.construct(manip, HW4::getEx3Workspace1());
    //Visualizer::makeFigure(HW4::getEx3Workspace1(), manip, state);
    Visualizer::makeFigure(*cspacea);

    // Part b)
    std::unique_ptr<amp::GridCSpace2D> cspaceb = cspace_constructor.construct(manip, HW4::getEx3Workspace2());
    //Visualizer::makeFigure(HW4::getEx3Workspace2(), manip, state);
    Visualizer::makeFigure(*cspaceb);

    // Part c)
    std::unique_ptr<amp::GridCSpace2D> cspacec = cspace_constructor.construct(manip, HW4::getEx3Workspace3());
    //Visualizer::makeFigure(HW4::getEx3Workspace3(), manip, state);
    Visualizer::makeFigure(*cspacec);

    Visualizer::saveFigures(true, "hw4_figs");

    // Grade method
    amp::HW4::grade<Manipulator2D>(cspace_constructor, "Katrina.Braun@colorado.edu", argc, argv);
    return 0;
}