#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "MyAStar.h"

// Include the correct homework headers
#include "hw/HW7.h"

class MyPRM : public amp::PRM2D {
    public:
        // Override the pure virtual function from PRM2D
        virtual amp::Path2D plan(const amp::Problem2D& problem) override;

        std::tuple<amp::Path2D, MyAStarAlgo::GraphSearchResult, std::map<amp::Node, Eigen::Vector2d>> plan(const amp::Problem2D& problem, std::shared_ptr<amp::Graph<double>>& graphPtr, int n, double r); 
};

class MyRRT : public amp::GoalBiasRRT2D {
    public:
        virtual amp::Path2D plan(const amp::Problem2D& problem) override; 
        std::tuple<amp::MultiAgentPath2D, MyAStarAlgo::GraphSearchResult, std::map<amp::Node, Eigen::VectorXd>> plan(const amp::MultiAgentProblem2D& problem, std::shared_ptr<amp::Graph<double>>& graphPtr, int n, double r, double epsilon, double p); 
        amp::Path2D planDecentralized(const amp::MultiAgentProblem2D& problem, int agentIndex, const amp::MultiAgentPath2D& previousPaths, int n, double r, double epsilon, double p);
};

void smoothPath(amp::MultiAgentPath2D& path, const amp::MultiAgentProblem2D& problem);
bool isCircleInPolygon(const Eigen::Vector2d& center, double radius, const amp::Polygon& polygon);
double pointToLineSegmentDistance(const Eigen::Vector2d& point, const Eigen::Vector2d& start, const Eigen::Vector2d& end);