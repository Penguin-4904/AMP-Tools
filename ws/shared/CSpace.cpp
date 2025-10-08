#include "CSpace.h"

/// @brief determines which cell a continuous point belongs to (continous to discrete)
std::pair<std::size_t, std::size_t> MyGridCSpace2D::getCellFromPoint(double x0, double x1) const {
    double resx0 = (m_x0_bounds.second - m_x0_bounds.first) / size().first;
    double resx1 = (m_x1_bounds.second - m_x1_bounds.first) / size().second;
    std::size_t cell_x0 = floor((x0 - m_x0_bounds.first)/resx0); // x0 index of cell
    std::size_t cell_x1 = floor((x1 - m_x1_bounds.first)/resx1); // x1 index of cell
    return {cell_x0, cell_x1};
}

/// @brief constructs a C-Space for a given 2-link manipulator and environment of obstacles
std::unique_ptr<amp::GridCSpace2D> MyManipulatorCSConstructor::construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env) {
    // Create an object of my custom cspace type (e.g. ManipulatorGridCSpace2D) and store it in a unique pointer.
    // Pass the constructor parameters to std::make_unique()
    std::unique_ptr<MyGridCSpace2D> cspace_ptr = std::make_unique<MyGridCSpace2D>(m_cells_per_dim, m_cells_per_dim, 0, 2 * M_PI, 0, 2 * M_PI);
    // In order to use the pointer as a regular GridCSpace2D object, we can just create a reference
    MyGridCSpace2D& cspace = *cspace_ptr;

    // Iterate through each dimension to find if cell/configuration is part of C-space obstacle or not
    Eigen::Vector2d state = {M_PI/m_cells_per_dim, M_PI/m_cells_per_dim};
    for (int i = 0; i < m_cells_per_dim; i++){
        for (int j = 0; j < m_cells_per_dim; j++){
            cspace(j, i) = check_manipulator_collisions(manipulator, state, env.obstacles);
            state[0] += 2 * M_PI/m_cells_per_dim;
        }
        state[0] = M_PI/m_cells_per_dim;
        state[1] += 2 * M_PI/m_cells_per_dim;
    }

    // Returning the object of type std::unique_ptr<ManipulatorGridCSpace2D> can automatically cast it to a polymorphic base-class pointer of type std::unique_ptr<amp::GridCSpace2D>.
    // The reason why this works is not super important for our purposes, but if you are curious, look up polymorphism!
    return cspace_ptr;
}
