#include "ManipulatorSkeleton.h"
#include <Eigen/Dense>
#include <cmath>
#include "HelpfulClass.h"

// Constructor: Initialize the manipulator with custom link lengths
MyManipulator2D::MyManipulator2D(const std::vector<double>& link_lengths)
    : LinkManipulator2D(link_lengths) // Now using dynamic link lengths
{}

// Forward kinematics: Calculate the position of a specific joint dynamically
Eigen::Vector2d MyManipulator2D::getJointLocation(const amp::ManipulatorState& state, uint32_t joint_index) const {
    const std::vector<double>& link_lengths = getLinkLengths();
    std::size_t num_links = link_lengths.size();

    if (joint_index > num_links) {
        throw std::out_of_range("Invalid joint index");
    }

    Eigen::Vector2d joint_pos(0.0, 0.0);
    double theta_accum = 0.0;

    for (std::size_t i = 0; i < joint_index; ++i) {
        theta_accum += state[i];
        joint_pos += link_lengths[i] * Eigen::Vector2d(cos(theta_accum), sin(theta_accum));
    }

    return joint_pos;
}


// Inverse kinematics: For a basic 2-DOF or 3-DOF, inverse kinematics becomes tricky for more than 2-3 links.
amp::ManipulatorState MyManipulator2D::getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const {
    
    // Simplified inverse kinematics for 2 or 3 links.
    const std::vector<double>& link_lengths = getLinkLengths();
    std::size_t num_links = link_lengths.size();

    if (num_links != 2 && num_links != 3) {
        throw std::runtime_error("Inverse kinematics only implemented for 2 or 3 links");
    }

    double x = end_effector_location[0];
    double y = end_effector_location[1];

    std::cout << "End Effector Location " << x << " : " << y << std::endl;
    
    double r = sqrt(x * x + y * y); // Distance to end effector

    if (r > std::accumulate(link_lengths.begin(), link_lengths.end(), 0.0)) {
        throw std::runtime_error("Target out of reach.");
    }

    // Inverse kinematics math (for 2 or 3 link manipulator)

    if (num_links == 2) {
        double a1 = link_lengths[0];
        double a2 = link_lengths[1];

        // Standard 2-link IK formulas
        double cos_theta2 = (x*x + y*y - a1*a1 - a2*a2) / (2 * a1 * a2);
        // Clamp cos_theta2 to [-1, 1] to handle floating-point errors
        cos_theta2 = std::min(1.0, std::max(-1.0, cos_theta2));

        // Choose between elbow-up and elbow-down solutions
        double sin_theta2_positive = sqrt(1 - cos_theta2 * cos_theta2);
        double sin_theta2_negative = -sin_theta2_positive;

        // Let's choose the elbow-up solution (positive sin_theta2)
        double sin_theta2 = sin_theta2_positive;
        double theta2 = atan2(sin_theta2, cos_theta2);

        double k1 = a1 + a2 * cos_theta2;
        double k2 = a2 * sin_theta2;
        double theta1 = atan2(y, x) - atan2(k2, k1);

        // Normalize angles to [0, 2π)
        theta1 = fmod(theta1 + 2 * M_PI, 2 * M_PI);
        theta2 = fmod(theta2 + 2 * M_PI, 2 * M_PI);

        amp::ManipulatorState joint_angles(2);
        joint_angles[0] = theta1;
        joint_angles[1] = theta2;

        std::cout << "Joint angles: theta1 = " << joint_angles[0]
                  << ", theta2 = " << joint_angles[1] << std::endl;

        return joint_angles;
    } else {
        // Extended IK for 3-link case
        double theta1 = atan2(y, x);
        double a1 = link_lengths[0];
        double a2 = link_lengths[1];
        double a3 = link_lengths[2];
        
        double relativeX = x - a1 * cos(theta1);
        double relativeY = y - a1 * sin(theta1);
    
        double r2 = sqrt(relativeX * relativeX + relativeY * relativeY);
        double theta3 = M_PI - acos((r2 * r2 - a2 * a2 - a3 * a3) / (-2 * a2 * a3));
        double theta2 = atan2(relativeY, relativeX) - atan2(a3 * sin(theta3), a2 + a3 * cos(theta3));

        amp::ManipulatorState joint_angles(3);
        joint_angles[0] = theta1;
        joint_angles[1] = theta2;
        joint_angles[2] = theta3;
        return joint_angles;
    }
}
