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
        std::tuple<amp::Path2D, MyAStarAlgo::GraphSearchResult, std::map<amp::Node, Eigen::Vector2d>> plan(const amp::Problem2D& problem, std::shared_ptr<amp::Graph<double>>& graphPtr, int n, double r, double epsilon, double p); 

};

void smoothPath(amp::Path2D& path, const amp::Problem2D& problem);