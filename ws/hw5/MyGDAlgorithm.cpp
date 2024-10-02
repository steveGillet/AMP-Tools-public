#include "MyGDAlgorithm.h"
#include <cmath>

// Implement the plan method for the gradient descent algorithm.
amp::Path2D MyGDAlgorithm::plan(const amp::Problem2D& problem) {
    amp::Path2D path;

    // Set initial configuration and add to path
    Eigen::Vector2d q = problem.q_init;
    path.waypoints.push_back(q);

    Eigen::Vector2d q_goal = problem.q_goal;
    double epsilon = 0.25; // Termination radius

    // Parameters
    double step_size = eta; // Step size for gradient descent

    // Create the potential function object with the necessary parameters
    MyPotentialFunction potential_function(problem.obstacles, q_goal, zetta, Q_star);

    while ((q - q_goal).norm() > epsilon) {
        // Calculate the gradient using the potential function
        Eigen::Vector2d grad_U_att = zetta * (q - q_goal);
        Eigen::Vector2d grad_U_rep = Eigen::Vector2d(0, 0);

        // Calculate repulsive gradient
        for (const auto& obstacle : problem.obstacles) {
            Eigen::Vector2d obs_center(0, 0);
            for (const auto& vertex : obstacle.verticesCCW()) {
                obs_center += vertex;
            }
            obs_center /= obstacle.verticesCCW().size();

            double dist = (q - obs_center).norm();

            if (dist < Q_star) {
                grad_U_rep += (q - obs_center) * (1.0 / dist - 1.0 / Q_star) / (dist * dist);
            }
        }

        grad_U_rep *= Q_star;

        // Gradient descent update
        Eigen::Vector2d grad_U = grad_U_att + grad_U_rep;
        q -= step_size * grad_U;

        // Add the new waypoint to the path
        path.waypoints.push_back(q);
    }

    // Add goal to the path
    path.waypoints.push_back(q_goal);

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
