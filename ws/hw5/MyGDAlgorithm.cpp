#include "MyGDAlgorithm.h"
#include <cmath>

amp::Path2D MyGDAlgorithm::plan(const amp::Problem2D& problem) {
    amp::Path2D path;

    // Set initial configuration and add to path
    Eigen::Vector2d q = problem.q_init;
    path.waypoints.push_back(q);

    Eigen::Vector2d q_goal = problem.q_goal;
    double epsilon = 0.5; // Termination radius

    // Parameters
    double step_size = eta; // Step size for gradient descent
    double scaling_factor = 1.0; // Control repulsive scaling factor
    int max_iterations = 10000; // Limit the number of iterations
    int iteration = 0;
    double beta = 0.5; // Reduced momentum factor to prevent excessive looping
    Eigen::Vector2d momentum = Eigen::Vector2d::Zero(); // Initialize momentum
    double zetta = 8.0; // Increase attractive scaling factor to provide stronger pull to the goal

    // Threshold for increasing repulsive force near obstacle boundaries
    double min_dist = 0.5; // Minimum distance from obstacles to maintain maximum repulsion
    double velocity_cap = 0.3; // Maximum velocity to cap speed of movement

    while ((q - q_goal).norm() > epsilon && iteration < max_iterations) {
        // Attractive gradient towards the goal
        Eigen::Vector2d grad_U_att = zetta * (q_goal - q);
        
        // Repulsive gradient
        Eigen::Vector2d grad_U_rep = Eigen::Vector2d(0, 0);
        
        // Sum the repulsive contributions from each obstacle
        for (const auto& obstacle : problem.obstacles) {
            // Calculate the center of the obstacle by averaging all vertices
            Eigen::Vector2d obs_center(0, 0);
            for (const auto& vertex : obstacle.verticesCCW()) {
                obs_center += vertex;
            }
            obs_center /= obstacle.verticesCCW().size();

            // Calculate bounding radius (maximum distance from center to a vertex)
            double obstacle_size = 0.0;
            for (const auto& vertex : obstacle.verticesCCW()) {
                double vertex_dist = (vertex - obs_center).norm();
                obstacle_size = std::max(obstacle_size, vertex_dist);
            }

            // Set effective distance for repulsive influence
            double d_eff = obstacle_size * 3.0; // Extend influence distance

            double dist = (q - obs_center).norm();

            // Piecewise repulsive potential to prevent cutting through obstacle edges
            if (dist < d_eff) {
                Eigen::Vector2d direction = q - obs_center;  // Direction from obstacle to robot
                direction.normalize(); // Normalize direction

                double rep_value;
                if (dist < min_dist) {
                    // Stronger repulsion when very close to or inside the obstacle
                    rep_value = 20.0 * (1.0 / dist - 1.0 / min_dist); // Sharp repulsion close to obstacle
                } else {
                    // Gradual repulsion further away from obstacle
                    rep_value = scaling_factor * log(d_eff / dist); // Logarithmic repulsion for smooth growth
                }

                // Decrease repulsion strength dynamically as the robot nears the goal
                double goal_distance = (q - q_goal).norm();
                double reduction_factor = std::min(1.0, goal_distance / 2.0); // Decrease repulsion close to goal
                grad_U_rep += direction * rep_value * d_eff * reduction_factor;
            }
        }

        // Combined gradient
        Eigen::Vector2d grad_U = grad_U_att + grad_U_rep;

        // Cap the maximum speed to avoid overshooting or erratic behavior
        if (grad_U.norm() > velocity_cap) {
            grad_U = (grad_U / grad_U.norm()) * velocity_cap;
        }

        // Adaptive step size with a minimum threshold, reduce it when close to the goal to stabilize
        double gradient_magnitude = grad_U.norm();
        double min_eta = 0.005; // Reduce minimum threshold for larger step size control
        double adaptive_eta = std::max(min_eta, eta / (1.0 + gradient_magnitude));

        // Adjust eta further if the robot is too close to the goal to prevent circling
        if ((q - q_goal).norm() < 1.0) {
            adaptive_eta *= 0.2; // Strongly reduce step size near the goal to avoid oscillation
        }

        // Update momentum
        momentum = beta * momentum + (1 - beta) * grad_U;

        // Update position using momentum
        q += adaptive_eta * momentum;  // Update should move in the direction indicated by the gradient

        // Add the new waypoint to the path if there was significant movement
        if ((q - path.waypoints.back()).norm() > 0.05) {
            path.waypoints.push_back(q);
        }

        iteration++;
    }

    // Add goal to the path if reached
    if ((q - q_goal).norm() <= epsilon) {
        path.waypoints.push_back(q_goal);
    } else {
        std::cerr << "Warning: Maximum iteration limit reached without convergence.\n";
    }

    return path;
}



// Define the potential function
double MyPotentialFunction::operator()(const Eigen::Vector2d& q) const {
    // Attractive Potential
    double U_att = 0.5 * zetta * (q - goal).squaredNorm();

    // Repulsive Potential
    double U_rep = 0.0;

    // Calculate the repulsive potential for each obstacle
    for (const auto& obstacle : obstacles) {
        Eigen::Vector2d obs_center(0, 0);
        for (const auto& vertex : obstacle.verticesCCW()) {
            obs_center += vertex;
        }
        obs_center /= obstacle.verticesCCW().size();

        double dist = (q - obs_center).norm();
        if (dist < Q_star) {
            U_rep += 0.5 * Q_star * pow((1.0 / dist - 1.0 / Q_star), 2);
        }
    }

    return U_att + U_rep;
}
