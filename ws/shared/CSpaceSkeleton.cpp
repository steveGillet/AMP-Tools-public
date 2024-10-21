#include "CSpaceSkeleton.h"

std::pair<std::size_t, std::size_t> MyGridCSpace2D::getCellFromPoint(double x0, double x1) const {
    // Get the bounds of the configuration space from the member variables
    double x0_min = m_x0_bounds.first;
    double x0_max = m_x0_bounds.second;
    double x1_min = m_x1_bounds.first;
    double x1_max = m_x1_bounds.second;

    // Get the number of cells per dimension from the member variables
    auto [x0_cells, x1_cells] = size();

    // Calculate the size of each cell
    double x0_cell_size = (x0_max - x0_min) / static_cast<double>(x0_cells);
    double x1_cell_size = (x1_max - x1_min) / static_cast<double>(x1_cells);

    // Calculate the cell index for each dimension
    std::size_t cell_x = static_cast<std::size_t>((x0 - x0_min) / x0_cell_size);
    std::size_t cell_y = static_cast<std::size_t>((x1 - x1_min) / x1_cell_size);

    // Clamp the indices to ensure they are within bounds
    if (cell_x >= x0_cells) cell_x = x0_cells - 1;
    if (cell_y >= x1_cells) cell_y = x1_cells - 1;

    return {cell_x, cell_y};
}


std::unique_ptr<amp::GridCSpace2D> MyManipulatorCSConstructor::construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env) {
    // Create an object of MyGridCSpace2D
    std::unique_ptr<MyGridCSpace2D> cspace_ptr = std::make_unique<MyGridCSpace2D>(m_cells_per_dim, m_cells_per_dim, env.x_min, env.x_max, env.y_min, env.y_max);
    MyGridCSpace2D& cspace = *cspace_ptr;

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

            // Set the manipulator state
            amp::ManipulatorState state(2);
            state(0) = theta1;
            state(1) = theta2;

            // Use the manipulator to determine the link positions
            Eigen::Vector2d base(0, 0);  // Base at the origin
            Eigen::Vector2d joint1 = manipulator.getJointLocation(state, 1); // Position of the first joint
            Eigen::Vector2d end_effector = manipulator.getJointLocation(state, 2); // Position of the end effector

            // Check for collisions with each obstacle in the environment
            bool in_collision = false;
            for (const auto& obstacle : env.obstacles) {
                auto vertices = obstacle.verticesCCW();
                std::size_t num_vertices = vertices.size();

                for (std::size_t k = 0; k < num_vertices; ++k) {
                    Eigen::Vector2d v1 = vertices[k];
                    Eigen::Vector2d v2 = vertices[(k + 1) % num_vertices];

                    // Check if any link intersects with the polygon edge
                    if (doLinesIntersect(base, joint1, v1, v2) || doLinesIntersect(joint1, end_effector, v1, v2)) {
                        in_collision = true;
                        break;
                    }
                }
                if (in_collision) break;
            }

            // Update the C-space grid
            cspace(i, j) = in_collision;
        }
    }

    return cspace_ptr;
}

bool isPointInPolygon(const Eigen::Vector2d& point, const amp::Polygon& polygon) {
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

bool doLinesIntersect(const Eigen::Vector2d& p1, const Eigen::Vector2d& q1,
                      const Eigen::Vector2d& p2, const Eigen::Vector2d& q2) {
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
