// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW4.h"

// Include the headers for HW4 code
#include "CSpace.h"
#include "Manipulator2D.h"
#include "2DRidgidBodyCSpace.h"


using namespace amp;

/// @brief formalizes steps for EX2 part a) into one function call
Eigen::Vector2d nLinkForwardKinematics(const std::vector<double>& link_lengths, const Eigen::VectorXd& state){
    Manipulator2D manipulator(link_lengths);
    Visualizer::makeFigure(manipulator, state);
    Eigen::Vector2d end_effector = manipulator.getJointLocation(state, link_lengths.size());
    LOG("FK End Effector Location: " << end_effector);
    return end_effector;
}

/// @brief formalizes steps for EX2 part b) into one function call
Eigen::VectorXd nLinkInverseKinematics(const std::vector<double>& link_lengths, const Eigen::Vector2d& end_effector){
    Manipulator2D manipulator(link_lengths);
    Eigen::VectorXd state = manipulator.getConfigurationFromIK(end_effector);
    Visualizer::makeFigure(manipulator, state);
    for (int i = 0; i < link_lengths.size(); i++){
        LOG("Theta " << i << " value: " << state[i]);
    }
    return state;
}

/// @brief formalizes steps for EX3 into one function call
void DisplayCSpace(const std::vector<double>& link_lengths, const Environment2D& env, const size_t n = 200){
    Manipulator2D manipulator(link_lengths);
    MyManipulatorCSConstructor cspace_constructor(n);
    std::unique_ptr<amp::GridCSpace2D> cspace = cspace_constructor.construct(manipulator, env);
    Visualizer::makeFigure(env);
    Visualizer::makeFigure(*cspace);
}

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
    ManipulatorState test_stateA(3);
    test_stateA << M_PI/6, M_PI/3, 7*M_PI/4;

    nLinkForwardKinematics(links1, test_stateA);

    // Part b)
    std::vector<double> links2 = {1, 0.5, 1};
    Eigen::Vector2d test_pointB(2, 0);

    nLinkInverseKinematics(links2, test_pointB);

    // Exercise 3:
    std::vector<double> links = {1, 1};
    // Part a)
    DisplayCSpace(links, HW4::getEx3Workspace1());
    // Part b)
    DisplayCSpace(links, HW4::getEx3Workspace2());
    // Part c)
    DisplayCSpace(links, HW4::getEx3Workspace3());

    Visualizer::saveFigures(true, "hw4_figs");

    MyManipulatorCSConstructor cspace_constructor(200);
    // Grade method
    amp::HW4::grade<Manipulator2D>(cspace_constructor, "Katrina.Braun@colorado.edu", argc, argv);
    return 0;
}