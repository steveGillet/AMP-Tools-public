// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW4.h"

// Include the headers for HW4 code
#include "CSpaceSkeleton.h"
#include "ManipulatorSkeleton.h"

// Include the header of the shared class
#include "HelpfulClass.h"
#include <algorithm>
#include <set>

using namespace amp;

// Cross product of two vectors
double crossProduct(const Eigen::Vector2d& O, const Eigen::Vector2d& A, const Eigen::Vector2d& B) {
    return (A.x() - O.x()) * (B.y() - O.y()) - (A.y() - O.y()) * (B.x() - O.x());
}

// Convex Hull using Graham's Scan algorithm
std::vector<Eigen::Vector2d> convexHull(std::vector<Eigen::Vector2d> points) {
    if (points.size() <= 1) {
        return points;
    }

    // Sort points lexicographically (first by x, then by y)
    std::sort(points.begin(), points.end(), [](const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
        return a.x() < b.x() || (a.x() == b.x() && a.y() < b.y());
    });

    // Build lower hull
    std::vector<Eigen::Vector2d> lower;
    for (const auto& p : points) {
        while (lower.size() >= 2 && crossProduct(lower[lower.size() - 2], lower.back(), p) <= 0) {
            lower.pop_back();
        }
        lower.push_back(p);
    }

    // Build upper hull
    std::vector<Eigen::Vector2d> upper;
    for (int i = points.size() - 1; i >= 0; --i) {
        const auto& p = points[i];
        while (upper.size() >= 2 && crossProduct(upper[upper.size() - 2], upper.back(), p) <= 0) {
            upper.pop_back();
        }
        upper.push_back(p);
    }

    // Remove the last point of each half because it's repeated at the beginning of the other half
    lower.pop_back();
    upper.pop_back();

    // Concatenate lower and upper hull
    lower.insert(lower.end(), upper.begin(), upper.end());
    return lower;
}

// Function to print vertices
void printVertices(const std::vector<Eigen::Vector2d>& vertices, double angle) {
    std::cout << "Vertices of the C-space obstacle at angle "<< angle << "(rads) :" << std::endl;
    for (const auto& v : vertices) {
        std::cout << "(" << v.x() << ", " << v.y() << ")" << std::endl;
    }
}

std::vector<Eigen::Vector2d> generateCspaceVertices(const std::vector<Eigen::Vector2d> robotVertices, std::vector<Eigen::Vector2d> obstacleVertices){
    std::vector<Eigen::Vector2d> cSpaceVertices;

    for (const auto& p : obstacleVertices) {
        for (const auto& r : robotVertices) {
            Eigen::Vector2d newVertex = p - r;
            // LOG("C-Space Obstacle Vertices: " << newVertex);
            cSpaceVertices.push_back(newVertex);
        }
    }    

    return cSpaceVertices; 
}

// Function to rotate vertices of the robot by a given angle (in radians)
std::vector<Eigen::Vector2d> rotateVertices(const std::vector<Eigen::Vector2d>& vertices, double angle) {
    std::vector<Eigen::Vector2d> rotatedVertices;
    Eigen::Matrix2d rotationMatrix;
    
    // Define the 2D rotation matrix for the given angle
    rotationMatrix << cos(angle), -sin(angle),
                      sin(angle),  cos(angle);
    
    // Rotate each vertex
    for (const auto& vertex : vertices) {
        rotatedVertices.push_back(rotationMatrix * vertex);
    }
    
    return rotatedVertices;
}

// Helper function to convert degrees to radians
double degreesToRadians(double degrees) {
    return degrees * M_PI / 180.0;
}

int main(int argc, char** argv) {

    Obstacle2D obstacle = HW4::getEx1TriangleObstacle();
    Obstacle2D robot = HW4::getEx1TriangleObstacle();

    std::vector<Eigen::Vector2d> robotVertices = robot.verticesCCW();
    std::vector<Eigen::Vector2d> obstacleVertices = obstacle.verticesCCW();
    std::vector<Eigen::Vector2d> cSpaceVertices;

    cSpaceVertices = generateCspaceVertices(robotVertices, obstacleVertices);

    std::vector<Eigen::Vector2d> sortedVertices = convexHull(cSpaceVertices);

    amp::Polygon cSpacePolygon(sortedVertices);

    std::vector<amp::Polygon> polygons = {cSpacePolygon};
    Visualizer::makeFigure(polygons);

    // Part (b): Now consider rotational motion
    std::vector<amp::Polygon> polygons2;

    std::vector<double> angles;
    int num_angles = 11;
    for (int i = 0; i <= num_angles; i++) {
        double theta = i * (2 * M_PI / num_angles); // 12 equally spaced angles
        angles.push_back(theta);

        // Rotate the obstacle and compute the C-space for each angle
        std::vector<Eigen::Vector2d> rotatedVertices = rotateVertices(robotVertices, theta);
        cSpaceVertices = generateCspaceVertices(rotatedVertices, obstacleVertices);
        sortedVertices = convexHull(cSpaceVertices);

        printVertices(sortedVertices, theta);
        amp::Polygon cSpacePolygon(sortedVertices);
        polygons2.push_back(cSpacePolygon);
    }

    // Visualize the 3D result with rotation
    Visualizer::makeFigure(polygons2, angles);

    /* Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    amp::RNG::seed(amp::RNG::randiUnbounded());

    MyManipulator2D manipulator;

    // // Get link lengths from user
    // std::vector<double> link_lengths(3);
    // std::cout << "Enter the lengths of the 3 links:\n";
    // for (int i = 0; i < 3; ++i) {
    //     std::cout << "Length of link " << (i + 1) << ": ";
    //     std::cin >> link_lengths[i];
    // }

    // // Update the manipulator's link lengths
    // manipulator.getLinkLengths() = link_lengths;

    // // Get joint angles from user (in degrees)
    // std::vector<double> angles_deg(3);
    // std::cout << "Enter the angles for the 3 joints (in degrees):\n";
    // for (int i = 0; i < 3; ++i) {
    //     std::cout << "Angle " << (i + 1) << ": ";
    //     std::cin >> angles_deg[i];
    // }

    // // Convert angles to radians
    // amp::ManipulatorState state(3);
    // for (int i = 0; i < 3; ++i) {
    //     state[i] = degreesToRadians(angles_deg[i]);
    // }
    // // The visualizer uses your implementation of forward kinematics to show the joint positions so you can use that to test your FK algorithm
    // Visualizer::makeFigure(manipulator, state); 

    //     // Print joint locations
    // for (int i = 0; i <= 3; ++i) {
    //     Eigen::Vector2d joint_position = manipulator.getJointLocation(state, i);
    //     std::cout << "Joint " << i << " position: " << joint_position.transpose() << std::endl;
    // }

    // // Get link lengths from user
    // std::vector<double> linkLengthsIK(3);
    // std::cout << "Enter the lengths of the 3 links:\n";
    // for (int i = 0; i < 3; ++i) {
    //     std::cout << "Length of link " << (i + 1) << ": ";
    //     std::cin >> linkLengthsIK[i];
    // }

    // // Update the manipulator's link lengths
    // manipulator.getLinkLengths() = linkLengthsIK;

    // // Get desired end-effector position from user
    // Eigen::Vector2d end_effector_location;
    // std::cout << "Enter the desired end-effector position (x y): ";
    // std::cin >> end_effector_location[0] >> end_effector_location[1];

    // try {
    //     // Calculate joint angles using inverse kinematics
    //     amp::ManipulatorState joint_angles = manipulator.getConfigurationFromIK(end_effector_location);

    //     // Visualize the manipulator configuration
    //     Visualizer::makeFigure(manipulator, joint_angles);

    //     // Print out the calculated joint angles
    //     std::cout << "Calculated joint angles:\n";
    //     std::cout << "Theta1: " << joint_angles[0] << " radians\n";
    //     std::cout << "Theta2: " << joint_angles[1] << " radians\n";
    //     std::cout << "Theta3: " << joint_angles[2] << " radians\n";
    // }
    // catch (const std::exception& e) {
    //     std::cerr << "Error: " << e.what() << std::endl;
    // }

    // Create the collision space constructor
    std::size_t n_cells = 100;
    MyManipulatorCSConstructor cspace_constructor(n_cells);

    // Create the collision space using a given manipulator and environment
    std::unique_ptr<amp::GridCSpace2D> cspace1 = cspace_constructor.construct(manipulator, HW4::getEx3Workspace1());

    Visualizer::makeFigure(HW4::getEx3Workspace1());
    // You can visualize your cspace 
    Visualizer::makeFigure(*cspace1);

        // Create the collision space using a given manipulator and environment
    std::unique_ptr<amp::GridCSpace2D> cspace2 = cspace_constructor.construct(manipulator, HW4::getEx3Workspace2());

    Visualizer::makeFigure(HW4::getEx3Workspace2());
    // You can visualize your cspace 
    Visualizer::makeFigure(*cspace2);

        // Create the collision space using a given manipulator and environment
    std::unique_ptr<amp::GridCSpace2D> cspace3 = cspace_constructor.construct(manipulator, HW4::getEx3Workspace3());
    
    Visualizer::makeFigure(HW4::getEx3Workspace3());
    // You can visualize your cspace 
    Visualizer::makeFigure(*cspace3);

    Visualizer::showFigures();

    // Grade method
    amp::HW4::grade<MyManipulator2D>(cspace_constructor, "stephen.gillet@colorado.edu", argc, argv);
    return 0;
}