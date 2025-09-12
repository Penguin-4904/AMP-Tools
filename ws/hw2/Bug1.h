#pragma once

#include "AMPCore.h"
#include "hw/HW2.h"
#include "Collision.h"
#include <Eigen/Geometry>

/// @brief Declare your bug algorithm class here. Note this class derives the bug algorithm class declared in HW2.h
class Bug1 : public amp::BugAlgorithm {
    public:

        virtual amp::Path2D plan(const amp::Problem2D& problem) override;

    private:
        // Sets the step size and collision search distance of the algorithm.
        double step_size = 0.02;
        int deg_step = 5;
};