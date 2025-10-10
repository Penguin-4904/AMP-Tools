#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework headers
#include "hw/HW4.h"
#include "hw/HW6.h"
#include "CSpace.h"
#include "Collision.h"

// Derive the PointAgentCSConstructor class and override the missing method
class MyPointAgentCSConstructor : public amp::PointAgentCSConstructor {
    public:
        // To make things easy, add the number of cells as a ctor param so you can easily play around with it
        MyPointAgentCSConstructor(std::size_t cells_per_dim) : m_cells_per_dim(cells_per_dim) {}

        /// @breif Constructs CSpace for point agent by detecting if any part of the cell collides with an obstacle.
        virtual std::unique_ptr<amp::GridCSpace2D> construct(const amp::Environment2D& env) override;

    private:
        std::size_t m_cells_per_dim;
};

class MyWaveFrontAlgorithm : public amp::WaveFrontAlgorithm {
    public:
        /// @brief runs a wavefront algorithm over the provided CSpace with the same grid size as the CSpace
        virtual amp::Path2D planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace, bool isManipulator) override;

};

