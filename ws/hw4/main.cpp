// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW4.h"

// Include the headers for HW4 code
#include "CSpaceSkeleton.h"
#include "ManipulatorSkeleton.h"
#include "2DRidgidBodyCSpace.h"

// Include the header of the shared class
#include "HelpfulClass.h"

using namespace amp;

int main(int argc, char** argv) {
    /* Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    amp::RNG::seed(amp::RNG::randiUnbounded());

    std::vector<Eigen::Vector2d> vertices = {Eigen::Vector2d(0, 0), Eigen::Vector2d(1, 2), Eigen::Vector2d(0, 2)};
    std::size_t n = 12;
    std::vector<double> theta;
    for (int i = 0; i < n; i++){
        theta.push_back(i * 2 * M_PI / n);
    }

    Polygon r(vertices);
    Obstacle2D o(vertices);
    Obstacle2D obstacle_out;

    std::vector<Obstacle2D> o_vector = CSpacePolygonRotate(r, o, n);
    Visualizer::makeFigure(o_vector, theta);

    std::vector<double> links2(20, 0.1);
    MyManipulator2D manipulator(links2);

    LOG("Reach: " << manipulator.reach());

    Eigen::Vector2d test_point(0, 1);

    amp::ManipulatorState test_state = manipulator.getConfigurationFromIK(test_point);

    LOG("Location: " << manipulator.getJointLocation(test_state, test_state.size()) - test_point);

    Visualizer::makeFigure(manipulator, test_state);

    // Create the collision space constructor
    std::size_t n_cells = 5;
    MyManipulatorCSConstructor cspace_constructor(n_cells);

    // Create the collision space using a given manipulator and environment
    std::unique_ptr<amp::GridCSpace2D> cspace = cspace_constructor.construct(manipulator, HW4::getEx3Workspace1());

    // You can visualize your cspace
    Visualizer::makeFigure(*cspace);
//
    Visualizer::saveFigures(true, "hw4_figs");

    // Grade method
    amp::HW4::grade<MyManipulator2D>(cspace_constructor, "Katrina.Braun@colorado.edu", argc, argv);
    return 0;
}