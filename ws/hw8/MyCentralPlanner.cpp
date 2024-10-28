#include "MyMultiAgentPlanners.h"

amp::MultiAgentPath2D MyCentralPlanner::plan(const amp::MultiAgentProblem2D& problem) {
    MyRRT rrtPlanner;
    std::shared_ptr<amp::Graph<double>> graphPtr = std::make_shared<amp::Graph<double>>();
    int n = 7500;
    double r = 0.5;
    double epsilon = 0.25;
    double p = 0.05;

    auto results = rrtPlanner.plan(problem, graphPtr, n, r, epsilon, p);
    amp::MultiAgentPath2D path = std::get<0>(results);

    return path;
}

amp::MultiAgentPath2D MyDecentralPlanner::plan(const amp::MultiAgentProblem2D& problem) {
    amp::MultiAgentPath2D path;
    MyRRT rrtPlanner;
    int n = 7500;
    double r = 0.5;
    double epsilon = 0.25;
    double p = 0.05;
    int agentIndex = 0;

    for (const amp::CircularAgentProperties& agent : problem.agent_properties) {
        auto agentPath = rrtPlanner.planDecentralized(problem, agentIndex, path, n, r, epsilon, p);
        path.agent_paths.push_back(agentPath);
        agentIndex++;
    }

    return path;
}

