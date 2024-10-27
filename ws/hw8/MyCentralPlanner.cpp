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
    for (const amp::CircularAgentProperties& agent : problem.agent_properties) {
        amp::Path2D agent_path;
        agent_path.waypoints = {agent.q_init, agent.q_goal};
        path.agent_paths.push_back(agent_path);
    }
    return path;
}