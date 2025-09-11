#pragma once

#include "AMPCore.h"
#include "hw/HW2.h"
#include <Eigen/Geometry>

/// @brief Declare your bug algorithm class here. Note this class derives the bug algorithm class declared in HW2.h
class Bug1 : public amp::BugAlgorithm {
    public:

        virtual amp::Path2D plan(const amp::Problem2D& problem) override;

        /// @brief checks if point is inside any of the given obstacles.
        bool check_collisions(Eigen::Vector2d point, std::vector<amp::Obstacle2D> obstacles);

        /// @brief checks if point is inside obstacle. Assumes convex polygon obstacle.
        bool collide_object(Eigen::Vector2d point, amp::Obstacle2D obstacle);

    private:
        // Sets the step size and collision search distance of the algorithm.
        double step_size = 0.01;
};