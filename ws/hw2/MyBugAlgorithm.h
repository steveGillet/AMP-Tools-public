#pragma once

#include "AMPCore.h"
#include "hw/HW2.h"

/// @brief Declare your bug algorithm class here. Note this class derives the bug algorithm class declared in HW2.h
class MyBugAlgorithm : public amp::BugAlgorithm {
    public:
        virtual amp::Path2D plan(const amp::Problem2D& problem) override;
        amp::Path2D bug1(const amp::Problem2D& problem);
        amp::Path2D bug2(const amp::Problem2D& problem);

        std::unique_ptr<amp::Polygon> inObstacle(const std::vector<amp::Polygon>& obstacles, const Eigen::Vector2d& stepPosition);
        Eigen::Vector2d getObstacleCenter(const std::vector<Eigen::Vector2d>& vertices);
        Eigen::Vector2d rotateVector(Eigen::Vector2d vec, double theta);
        Eigen::Vector2d tangentVector(const std::vector<Eigen::Vector2d>& vertices, Eigen::Vector2d currentPosition, bool ccwDirection);
<<<<<<< HEAD
<<<<<<< HEAD
        bool isOnMLine(const Eigen::Vector2d& currentPosition, const Eigen::Vector2d& start, const Eigen::Vector2d& goal, double threshold);
=======
>>>>>>> 20d35d2 (bug1 implemented)
=======
        bool isOnMLine(const Eigen::Vector2d& currentPosition, const Eigen::Vector2d& start, const Eigen::Vector2d& goal, double threshold);
>>>>>>> 206f317 (both bugs working hw1 complete, still get stuck in some situations)
    
    private:
        // Add any member variables here...
};