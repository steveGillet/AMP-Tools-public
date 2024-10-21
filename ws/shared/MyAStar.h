#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework headers
#include "hw/HW6.h"

class MyAStarAlgo : public amp::AStar {
    public:
        virtual GraphSearchResult search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic) override;
};

// Custom heuristic class that overrides the default heuristic behavior
class CustomHeuristic : public amp::SearchHeuristic {
private:
    std::unordered_map<amp::Node, double> heuristic_values;

public:
    // Constructor to initialize heuristic values
    CustomHeuristic(const std::unordered_map<amp::Node, double>& values) : heuristic_values(values) {}

    // Override operator() to return the actual heuristic value
    double operator()(amp::Node node) const override {
        auto it = heuristic_values.find(node);
        if (it != heuristic_values.end()) {
            return it->second;
        }
        return 0.0; // Return 0 if no heuristic is available for the node
    }
};