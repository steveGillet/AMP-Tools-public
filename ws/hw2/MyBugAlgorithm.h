#pragma once

#include "AMPCore.h"
#include "hw/HW2.h"

/// @brief Declare your bug algorithm class here. Note this class derives the bug algorithm class declared in HW2.h
class MyBugAlgorithm : public amp::BugAlgorithm {
    public:
        virtual amp::Path2D plan(const amp::Problem2D& problem) override;

        std::unique_ptr<amp::Polygon> inObstacle(const std::vector<amp::Polygon>& obstacles, const Eigen::Vector2d& stepPosition);
        Eigen::Vector2d getObstacleCenter(const std::vector<Eigen::Vector2d>& vertices);
        Eigen::Vector2d rotateVector(Eigen::Vector2d vec, double theta);
        Eigen::Vector2d tangentVector(const std::vector<Eigen::Vector2d>& vertices, Eigen::Vector2d currentPosition, bool ccwDirection);
    
    private:
        // Add any member variables here...
};