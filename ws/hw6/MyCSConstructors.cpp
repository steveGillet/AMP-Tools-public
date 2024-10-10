#include "MyCSConstructors.h"

////////////////////// THIS IS FROM HW4 //////////////////////

/* You can just move these classes to shared folder and include them instead of copying them to hw6 project*/
std::pair<std::size_t, std::size_t> MyGridCSpace2D::getCellFromPoint(double x0, double x1) const {    
    std::size_t cell_x = static_cast<std::size_t>((x0 - m_x0_min) / ((m_x0_max - m_x0_min) / m_x0_cells));    
    std::size_t cell_y = static_cast<std::size_t>((x1 - m_x1_min) / ((m_x1_max - m_x1_min) / m_x1_cells));    

    // Ensure the indices are within bounds    
    if (cell_x >= m_x0_cells) cell_x = m_x0_cells - 1;    
    if (cell_y >= m_x1_cells) cell_y = m_x1_cells - 1;    
    
    return {cell_x, cell_y};
}

Eigen::Vector2d MyGridCSpace2D::getPointFromCell(std::size_t i, std::size_t j) const {    
    double x0 = m_x0_min + (i + 0.5) * (m_x0_max - m_x0_min) / m_x0_cells;    
    double x1 = m_x1_min + (j + 0.5) * (m_x1_max - m_x1_min) / m_x1_cells;    
    return Eigen::Vector2d(x0, x1);
}

// Override this method for computing all of the boolean collision values for each cell in the cspace
std::unique_ptr<amp::GridCSpace2D> MyManipulatorCSConstructor::construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env) {
    // Create an object of my custom cspace type (e.g. MyGridCSpace2D) and store it in a unique pointer. 
    // Pass the constructor parameters to std::make_unique()
    std::unique_ptr<MyGridCSpace2D> cspace_ptr = std::make_unique<MyGridCSpace2D>(m_cells_per_dim, m_cells_per_dim, env.x_min, env.x_max, env.y_min, env.y_max);
    // In order to use the pointer as a regular GridCSpace2D object, we can just create a reference
    MyGridCSpace2D& cspace = *cspace_ptr;
    std::cout << "Constructing C-space for manipulator" << std::endl;

    // Get the number of cells in each dimension
    std::size_t num_cells_x0 = m_cells_per_dim;
    std::size_t num_cells_x1 = m_cells_per_dim;

    // Get the bounds for theta1 and theta2
    double theta1_min = env.x_min;
    double theta1_max = env.x_max;
    double theta2_min = env.y_min;
    double theta2_max = env.y_max;

    // Iterate over all cells in the C-space grid
    for (std::size_t i = 0; i < num_cells_x0; ++i) {
        for (std::size_t j = 0; j < num_cells_x1; ++j) {
            // Convert cell indices to theta1 and theta2
            double theta1 = theta1_min + (theta1_max - theta1_min) * (static_cast<double>(i) / num_cells_x0);
            double theta2 = theta2_min + (theta2_max - theta2_min) * (static_cast<double>(j) / num_cells_x1);

            // Check if the manipulator configuration is in collision
            bool in_collision = false;

            // Create a ManipulatorState object with the current joint angles
            amp::ManipulatorState state(2);  // Assuming a 2-link manipulator
            state[0] = theta1;
            state[1] = theta2;

            // Get the positions of the joints and end effector
            Eigen::Vector2d base = manipulator.getJointLocation(state, 0);  // Base joint (usually at origin)
            Eigen::Vector2d joint1 = manipulator.getJointLocation(state, 1);  // First joint
            Eigen::Vector2d end_effector = manipulator.getJointLocation(state, 2);  // End effector

            // Check collision with obstacles
            for (const auto& obstacle : env.obstacles) {
                if (isPointInPolygon(joint1, obstacle) ||
                    isPointInPolygon(end_effector, obstacle) ||
                    doLinesIntersect(base, joint1, obstacle) ||
                    doLinesIntersect(joint1, end_effector, obstacle)) {
                    in_collision = true;
                    break;
                }
            }

            // Set the collision state for the current cell
            cspace(i, j) = in_collision;
        }
    }

    // Returning the object of type std::unique_ptr<MyGridCSpace2D> can automatically cast it to a polymorphic base-class pointer of type std::unique_ptr<amp::GridCSpace2D>.
    // The reason why this works is not super important for our purposes, but if you are curious, look up polymorphism!
    return cspace_ptr;
}

//////////////////////////////////////////////////////////////

// Override this method for computing all of the boolean collision values for each cell in the cspace
std::unique_ptr<amp::GridCSpace2D> MyPointAgentCSConstructor::construct(const amp::Environment2D& env) {
    // Create an object of my custom cspace type (e.g. MyGridCSpace2D) and store it in a unique pointer. 
    // Pass the constructor parameters to std::make_unique()
    std::unique_ptr<MyGridCSpace2D> cspace_ptr = std::make_unique<MyGridCSpace2D>(m_cells_per_dim, m_cells_per_dim, env.x_min, env.x_max, env.y_min, env.y_max);
    // In order to use the pointer as a regular GridCSpace2D object, we can just create a reference
    MyGridCSpace2D& cspace = *cspace_ptr;
    std::cout << "Constructing C-space for point agent" << std::endl;

    // Get the number of cells in each dimension
    std::size_t num_cells_x = m_cells_per_dim;
    std::size_t num_cells_y = m_cells_per_dim;

    // Get the bounds of the environment
    double x_min = env.x_min;
    double x_max = env.x_max;
    double y_min = env.y_min;
    double y_max = env.y_max;

    // Calculate cell sizes
    double cell_width = (x_max - x_min) / num_cells_x;
    double cell_height = (y_max - y_min) / num_cells_y;

    // Iterate over all cells in the C-space grid
    for (std::size_t i = 0; i < num_cells_x; ++i) {
        for (std::size_t j = 0; j < num_cells_y; ++j) {
            // Calculate the center point of the current cell
            double x = x_min + (i + 0.5) * cell_width;
            double y = y_min + (j + 0.5) * cell_height;
            Eigen::Vector2d point(x, y);

            // Check if the point is in collision with any obstacle
            bool in_collision = false;
            for (const auto& obstacle : env.obstacles) {
                if (isPointInPolygon(point, obstacle)) {
                    in_collision = true;
                    break;
                }
            }

            // Set the collision state for the current cell
            cspace(i, j) = in_collision;
        }
    }

    // Returning the object of type std::unique_ptr<MyGridCSpace2D> can automatically cast it to a polymorphic base-class pointer of type std::unique_ptr<amp::GridCSpace2D>.
    // The reason why this works is not super important for our purposes, but if you are curious, look up polymorphism!
    return cspace_ptr;
}

amp::Path2D MyWaveFrontAlgorithm::planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace, bool isManipulator) {
    // Downcast to MyGridCSpace2D to access the custom methods
    const MyGridCSpace2D& my_cspace = static_cast<const MyGridCSpace2D&>(grid_cspace);

    std::size_t num_cells_x = my_cspace.getX0Cells();
    std::size_t num_cells_y = my_cspace.getX1Cells();

    // Create a grid for wavefront propagation
    std::vector<std::vector<int>> wavefront_grid(num_cells_x, std::vector<int>(num_cells_y, 0));

    // Convert q_init and q_goal to grid coordinates
    auto init_cell = my_cspace.getCellFromPoint(q_init[0], q_init[1]);
    auto goal_cell = my_cspace.getCellFromPoint(q_goal[0], q_goal[1]);

    // Mark obstacles with -1 and goal with 2
    for (std::size_t i = 0; i < num_cells_x; ++i) {
        for (std::size_t j = 0; j < num_cells_y; ++j) {
            if (grid_cspace(i, j)) {
                wavefront_grid[i][j] = -1; // Obstacle
            }
        }
    }
    wavefront_grid[goal_cell.first][goal_cell.second] = 2;

    // Propagate wavefront
    int current_value = 2;
    bool changed = true;
    while (changed) {
        changed = false;
        for (std::size_t i = 0; i < num_cells_x; ++i) {
            for (std::size_t j = 0; j < num_cells_y; ++j) {
                if (wavefront_grid[i][j] == current_value) {
                    // Check neighbors
                    for (int dx = -1; dx <= 1; ++dx) {
                        for (int dy = -1; dy <= 1; ++dy) {
                            if (dx == 0 && dy == 0) continue;
                            std::size_t ni = i + dx;
                            std::size_t nj = j + dy;
                            if (ni < num_cells_x && nj < num_cells_y && wavefront_grid[ni][nj] == 0) {
                                wavefront_grid[ni][nj] = current_value + 1;
                                changed = true;
                            }
                        }
                    }
                }
            }
        }
        ++current_value;
    }

    // Backtrack from init to goal
    amp::Path2D path;
    std::pair<std::size_t, std::size_t> current = init_cell;
    while (current != goal_cell) {
        // Convert current cell to configuration space coordinates
        Eigen::Vector2d point = my_cspace.getPointFromCell(current.first, current.second);
        path.waypoints.push_back(point);

        // Find neighbor with lowest value
        std::pair<std::size_t, std::size_t> next_cell = current;
        int lowest_value = std::numeric_limits<int>::max();
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                if (dx == 0 && dy == 0) continue;
                std::size_t ni = current.first + dx;
                std::size_t nj = current.second + dy;
                if (ni < num_cells_x && nj < num_cells_y && wavefront_grid[ni][nj] > 0 && wavefront_grid[ni][nj] < lowest_value) {
                    lowest_value = wavefront_grid[ni][nj];
                    next_cell = {ni, nj};
                }
            }
        }
        current = next_cell;
    }

    // Add goal to path
    path.waypoints.push_back(q_goal);

    // Handle manipulator-specific considerations
    if (isManipulator) {
        Eigen::Vector2d bounds0 = Eigen::Vector2d(0.0, 0.0);
        Eigen::Vector2d bounds1 = Eigen::Vector2d(2 * M_PI, 2 * M_PI);
        amp::unwrapWaypoints(path.waypoints, bounds0, bounds1);
    }

    return path;
}


bool MyManipulatorCSConstructor::isPointInPolygon(const Eigen::Vector2d& point, const amp::Polygon& polygon) const {
    int crossings = 0;
    auto vertices = polygon.verticesCCW();
    std::size_t num_vertices = vertices.size();

    for (std::size_t i = 0; i < num_vertices; ++i) {
        Eigen::Vector2d v1 = vertices[i];
        Eigen::Vector2d v2 = vertices[(i + 1) % num_vertices];

        bool cond1 = (v1.y() <= point.y()) && (v2.y() > point.y());
        bool cond2 = (v2.y() <= point.y()) && (v1.y() > point.y());

        if (cond1 || cond2) {
            double slope = (v2.x() - v1.x()) / (v2.y() - v1.y());
            double intersectX = v1.x() + slope * (point.y() - v1.y());
            if (point.x() < intersectX) {
                crossings++;
            }
        }
    }

    return (crossings % 2) != 0;
}

bool MyManipulatorCSConstructor::doLinesIntersect(const Eigen::Vector2d& p1, const Eigen::Vector2d& q1,
                      const Eigen::Vector2d& p2, const Eigen::Vector2d& q2) const {
    auto orientation = [](const Eigen::Vector2d& p, const Eigen::Vector2d& q, const Eigen::Vector2d& r) {
        double val = (q.y() - p.y()) * (r.x() - q.x()) - (q.x() - p.x()) * (r.y() - q.y());
        if (val == 0) return 0;  // collinear
        return (val > 0) ? 1 : 2; // clock or counterclockwise
    };

    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);

    // General case
    if (o1 != o2 && o3 != o4) {
        return true;
    }

    return false; // No intersection
}

bool MyManipulatorCSConstructor::doLinesIntersect(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, 
                                                  const amp::Polygon& polygon) const {
    auto vertices = polygon.verticesCCW();
    for (size_t i = 0; i < vertices.size(); ++i) {
        size_t j = (i + 1) % vertices.size();
        if (doLinesIntersect(p1, p2, vertices[i], vertices[j])) {
            return true;
        }
    }
    return false;
}

bool MyPointAgentCSConstructor::isPointInPolygon(const Eigen::Vector2d& point, const amp::Polygon& polygon) const {
    int crossings = 0;
    auto vertices = polygon.verticesCCW();
    for (size_t i = 0; i < vertices.size(); i++) {
        size_t j = (i + 1) % vertices.size();
        if ((vertices[i].y() <= point.y() && vertices[j].y() > point.y()) ||
            (vertices[i].y() > point.y() && vertices[j].y() <= point.y())) {
            double atX = vertices[i].x() + (point.y() - vertices[i].y()) / (vertices[j].y() - vertices[i].y()) * (vertices[j].x() - vertices[i].x());
            if (atX < point.x()) {
                crossings++;
            }
        }
    }
    return (crossings % 2) == 1;
}