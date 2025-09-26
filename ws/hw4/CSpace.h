#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW4.h"
#include "Collision.h"

class ManipulatorGridCSpace2D : public amp::GridCSpace2D {
    public:
        ManipulatorGridCSpace2D(std::size_t x0_cells, std::size_t x1_cells, double x0_min, double x0_max, double x1_min, double x1_max)
            : amp::GridCSpace2D(x0_cells, x1_cells, x0_min, x0_max, x1_min, x1_max) // Call base class constructor
        {}

        /// @brief determines which cell a continuous point belongs to (continous to discrete)
        virtual std::pair<std::size_t, std::size_t> getCellFromPoint(double x0, double x1) const override;

};

class MyManipulatorCSConstructor : public amp::ManipulatorCSConstructor {
    public:
        // To make things easy, add the number of cells as a ctor param so you can easily play around with it
        MyManipulatorCSConstructor(std::size_t cells_per_dim) : m_cells_per_dim(cells_per_dim) {}

        /// @brief constructs a C-Space for a given 2-link manipulator and environment of obstacles
        virtual std::unique_ptr<amp::GridCSpace2D> construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env) override;

    private:
        std::size_t m_cells_per_dim;
};