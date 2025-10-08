#include "MyCSConstructors.h"



// Override this method for computing all of the boolean collision values for each cell in the cspace
std::unique_ptr<amp::GridCSpace2D> MyPointAgentCSConstructor::construct(const amp::Environment2D& env) {
    // Create an object of my custom cspace type (e.g. MyGridCSpace2D) and store it in a unique pointer. 
    // Pass the constructor parameters to std::make_unique()
    std::unique_ptr<MyGridCSpace2D> cspace_ptr = std::make_unique<MyGridCSpace2D>(m_cells_per_dim, m_cells_per_dim, env.x_min, env.x_max, env.y_min, env.y_max);
    // In order to use the pointer as a regular GridCSpace2D object, we can just create a reference
    MyGridCSpace2D& cspace = *cspace_ptr;

    Eigen::Vector2d state = {env.x_min + (env.x_max - env.x_min)/(2 * m_cells_per_dim), env.y_min + (env.y_max - env.y_min)/(2 * m_cells_per_dim)};
    for (int i = 0; i < m_cells_per_dim; i++){
        for (int j = 0; j < m_cells_per_dim; j++){
            cspace(j, i) = check_collisions(state, env.obstacles);
            state[0] += (env.x_max - env.x_min)/m_cells_per_dim;
        }
        state[0] = env.x_min + (env.x_max - env.x_min)/(2 * m_cells_per_dim);
        state[1] += (env.y_max - env.y_min)/m_cells_per_dim;
    }

    // Returning the object of type std::unique_ptr<MyGridCSpace2D> can automatically cast it to a polymorphic base-class pointer of type std::unique_ptr<amp::GridCSpace2D>.
    // The reason why this works is not super important for our purposes, but if you are curious, look up polymorphism!
    return cspace_ptr;
}

amp::Path2D MyWaveFrontAlgorithm::planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace, bool isManipulator) {

    double x_step = (grid_cspace.x0Bounds().second - grid_cspace.x0Bounds().first)/grid_cspace.size().first;
    double y_step = (grid_cspace.x1Bounds().second - grid_cspace.x1Bounds().first)/grid_cspace.size().second;

    // Implement your WaveFront algorithm here
    amp::Path2D path;
    path.waypoints.push_back(q_init);

    Eigen::MatrixXi wave_grid = Eigen::MatrixXi::Zero(grid_cspace.size().first, grid_cspace.size().second);
    std::pair<std::size_t, std::size_t> q_goal_index = grid_cspace.getCellFromPoint(q_goal[0], q_goal[1]);
    std::pair<std::size_t, std::size_t> q_init_index = grid_cspace.getCellFromPoint(q_init[0], q_init[1]);

    std::vector<std::pair<std::size_t, std::size_t>> next_cells;
    std::vector<std::pair<std::size_t, std::size_t>> current_cells;

    current_cells.push_back(q_goal_index);

    int i = 2;
    wave_grid(q_goal_index.first, q_goal_index.second) = 1;

    while (true) {

        if (current_cells.size() == 0){
            break;
        }

        for (int j = 0; j < current_cells.size(); j++){

            if (q_init_index == current_cells[j]){
                next_cells.clear();
                break;
            }

            size_t x = current_cells[j].first;
            size_t y = current_cells[j].second;

            int x_minus = x - 1;
            int x_plus = x + 1;
            int y_minus = y - 1;
            int y_plus = y + 1;

            // handles cspace wrapping.
            if (isManipulator) {
                if (x_minus < 0) {
                    x_minus = grid_cspace.size().first - 1;
                }
                if (y_minus < 0) {
                    y_minus = grid_cspace.size().second - 1;
                }
                if (x_plus >= grid_cspace.size().first) {
                    x_plus = 0;
                }
                if (y_plus >= grid_cspace.size().second) {
                    y_plus = 0;
                }
            }

            if ((x_plus < grid_cspace.size().first) && (wave_grid(x_plus, y) == 0) && !grid_cspace(x_plus, y)){
                wave_grid(x_plus, y) = i;
                next_cells.push_back({x_plus, y});
            }

            if ((y_plus < grid_cspace.size().second) && (wave_grid(x, y_plus) == 0) && !grid_cspace(x, y_plus)){
                wave_grid(x, y_plus) = i;
                next_cells.push_back({x, y_plus});
            }

            if ((x_minus >= 0) && (wave_grid(x_minus, y) == 0) && !grid_cspace(x_minus, y)){
                wave_grid(x_minus, y) = i;
                next_cells.push_back({x_minus, y});
            }

            if ((y_minus >= 0) && (wave_grid(x, y_minus) == 0) && !grid_cspace(x, y_minus)){
                wave_grid(x, y_minus) = i;
                next_cells.push_back({x, y_minus});
            }
        }

        current_cells = next_cells;
        next_cells.clear(); // there is probably a more effcient way to do this with jsut one vector but it is faster for me to implement this.
        LOG(i);
        i++;
    }

    i = wave_grid(q_init_index.first, q_init_index.second);
    path.waypoints.push_back(Eigen::Vector2d(x_step * q_init_index.first + x_step/2 + grid_cspace.x0Bounds().first,
                                             y_step * q_init_index.second + y_step/2 + grid_cspace.x1Bounds().first));

    std::pair<std::size_t, std::size_t> current_cell = q_init_index;

    while (i > 2) {
        size_t x = current_cell.first;
        size_t y = current_cell.second;

        int x_minus = x - 1;
        int x_plus = x + 1;
        int y_minus = y - 1;
        int y_plus = y + 1;

        // handles cspace wrapping.
        if (isManipulator) {
            if (x_minus < 0) {
                x_minus = grid_cspace.size().first - 1;
            }
            if (y_minus < 0) {
                y_minus = grid_cspace.size().second - 1;
            }
            if (x_plus >= grid_cspace.size().first) {
                x_plus = 0;
            }
            if (y_plus >= grid_cspace.size().second) {
                y_plus = 0;
            }
        }

        LOG(i);

        if ((x_plus < grid_cspace.size().first) && (wave_grid(x_plus, y) < i) && (wave_grid(x_plus, y) > 0)){
            i = wave_grid(x_plus, y);
            current_cell = {x_plus, y};
            path.waypoints.push_back(path.waypoints.back() + Eigen::Vector2d(x_step, 0));
        }

        if ((y_plus < grid_cspace.size().second) && (wave_grid(x, y_plus) < i) && (wave_grid(x, y_plus) > 0)){
            i = wave_grid(x, y_plus);
            current_cell = {x, y_plus};
            path.waypoints.push_back(path.waypoints.back() + Eigen::Vector2d(0, y_step));
        }

        if ((x_minus >= 0) && (wave_grid(x_minus, y) < i) && (wave_grid(x_minus, y) > 0)){
            i = wave_grid(x_minus, y);
            current_cell = {x_minus, y};
            path.waypoints.push_back(path.waypoints.back() - Eigen::Vector2d(x_step, 0));
        }

        if ((y_minus >= 0) && (wave_grid(x, y_minus) < i) && (wave_grid(x, y_minus) > 0)){
            i = wave_grid(x, y_minus);
            current_cell = {x, y_minus};
            path.waypoints.push_back(path.waypoints.back() - Eigen::Vector2d(0, y_step));;
        }
    };


    path.waypoints.push_back(q_goal);
    if (isManipulator) {
        Eigen::Vector2d bounds0 = Eigen::Vector2d(0.0, 0.0);
        Eigen::Vector2d bounds1 = Eigen::Vector2d(2*M_PI, 2*M_PI);
        amp::unwrapWaypoints(path.waypoints, bounds0, bounds1);
    }
    return path;
}
