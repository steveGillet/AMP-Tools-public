#include "ManipulatorSkeleton.h"
#include <Eigen/Dense>
#include <cmath>

// Constructor: Initialize the manipulator with custom link lengths
MyManipulator2D::MyManipulator2D()
    : LinkManipulator2D({1.0, 1.0})  // Default to a 3-link manipulator, modify lengths as needed
{}

// Forward kinematics: Calculate the position of a specific joint
Eigen::Vector2d MyManipulator2D::getJointLocation(const amp::ManipulatorState& state, uint32_t joint_index) const {
    // Extract joint angles from the state (theta1, theta2, theta3)
    double theta1 = state[0];
    double theta2 = state[1];
    // double theta3 = state[2];

    const std::vector<double>& link_lengths = getLinkLengths();
    double a1 = link_lengths[0];
    double a2 = link_lengths[1];
    // double a3 = link_lengths[2];

    // Base position (joint 0)
    Eigen::Vector2d joint0(0.0, 0.0);

    // Joint 1 position
    Eigen::Vector2d joint1 = joint0 + a1 * Eigen::Vector2d(cos(theta1), sin(theta1));

    // Joint 2 position
    Eigen::Vector2d joint2 = joint1 + a2 * Eigen::Vector2d(cos(theta1 + theta2), sin(theta1 + theta2));

    // Joint 3 position (end-effector)
    // Eigen::Vector2d joint3 = joint2 + a3 * Eigen::Vector2d(cos(theta1 + theta2 + theta3), sin(theta1 + theta2 + theta3));

    // Return the position of the requested joint
    if (joint_index == 0) {
        return joint0;
    } else if (joint_index == 1) {
        return joint1;
    } else if (joint_index == 2) {
        return joint2;
    } else {
        return Eigen::Vector2d(0.0, 0.0);
    }
}


// Inverse kinematics: Calculate joint angles from end-effector location
amp::ManipulatorState MyManipulator2D::getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const {
    // Get the desired end-effector position (x, y)
    double x = end_effector_location[0];
    double y = end_effector_location[1];

    // Get link lengths
    const std::vector<double>& link_lengths = getLinkLengths();
    double a1 = link_lengths[0];
    double a2 = link_lengths[1];
    // double a3 = link_lengths[2];

    // double theta1 = atan2(y, x);

    // Distance from the base to the end-effector
    double r = sqrt(x * x + y * y);

    // Standard 2-link IK formulas
    double cos_theta2 = (x*x + y*y - a1*a1 - a2*a2) / (2 * a1 * a2);
    // Clamp cos_theta2 to [-1, 1] to handle floating-point errors
    // cos_theta2 = std::min(1.0, std::max(-1.0, cos_theta2));

    // Choose between elbow-up and elbow-down solutions
    double sin_theta2_positive = sqrt(1 - cos_theta2 * cos_theta2);
    double sin_theta2_negative = -sin_theta2_positive;

    // Let's choose the elbow-up solution (positive sin_theta2)
    double sin_theta2 = sin_theta2_negative;
    double theta2 = atan2(sin_theta2, cos_theta2);

    double k1 = a1 + a2 * cos_theta2;
    double k2 = a2 * sin_theta2;
    double theta1 = atan2(y, x) - atan2(k2, k1);

    std::cout << "theta 1: " << theta1 << ". theta 2: " << theta2;

    // Step 4: Build the configuration
    amp::ManipulatorState joint_angles(2);
    joint_angles.setZero(); // Initialize angles to zero
    joint_angles(0) = theta1;
    joint_angles(1) = theta2; // You may need to adjust the signs depending on your coordinate system
    // joint_angles(2) = theta3;

    return joint_angles;
}