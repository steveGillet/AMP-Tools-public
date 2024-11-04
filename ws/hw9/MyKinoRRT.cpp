#include "MyKinoRRT.h"
#include "MyAStar.h"
#include "CSpaceSkeleton.h"
#include <random>
#include <functional>
#include <cfloat>

void MySingleIntegrator::propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) {
    state += dt * control;
};

// amp::KinoPath MyKinoRRT::plan(const amp::KinodynamicProblem2D& problem, amp::DynamicAgent& agent) {
//     amp::KinoPath path;
//     Eigen::VectorXd state = problem.q_init;
//     path.waypoints.push_back(state);
//     for (int i = 0; i < 10; i++) {
//         Eigen::VectorXd control = Eigen::VectorXd::Random(problem.q_init.size());
//         agent.propagate(state, control, 1.0);
//         path.waypoints.push_back(state);
//         path.controls.push_back(control);
//         path.durations.push_back(1.0);
//     } 
//     path.valid = true;
//     return path;
// }

// Define a functor for the zero heuristic
class ZeroHeuristic : public amp::SearchHeuristic {
public:
    double operator()(amp::Node) const override {
        return 0.0;  // Always return 0 for Dijkstra's algorithm (no heuristic)
    }
};

amp::KinoPath MyKinoRRT::plan(const amp::KinodynamicProblem2D& problem, amp::DynamicAgent& agent) {
    int p = 0.05;
    int n = 5000;
    amp::KinoPath path;
    static std::random_device rd;
    static std::mt19937 gen(rd());
    int numStates = problem.q_goal.size();
    std::shared_ptr<amp::Graph<double>> graphPtr = std::make_shared<amp::Graph<double>>();

    
    std::vector<std::tuple<amp::Node, amp::Node, double>> edges;
    std::map<amp::Node, Eigen::VectorXd> nodes;
    std::vector<Eigen::VectorXd> points;
    points.push_back(problem.q_init);
    nodes[0] = problem.q_init;
    int currentNodeIndex = 1;

    // Sampling random points
    int counter = 0;
    bool pathFound = false;
    while (counter < n && !pathFound) {
        Eigen::VectorXd point(numStates);
        std::bernoulli_distribution bernoulli(p);

        if(bernoulli(gen)){
            // Sample a random point within the goal region
            for (int i = 0; i < problem.q_goal.size(); i++) {
                double lower = problem.q_goal[i].first;
                double upper = problem.q_goal[i].second;
                std::uniform_real_distribution<> goalDist(lower, upper);
                point[i] = goalDist(gen);
            }
        }
        else{
            for (int i = 0; i < problem.q_bounds.size(); i++) {
                double lower = problem.q_bounds[i].first;
                double upper = problem.q_bounds[i].second;
                std::uniform_real_distribution<> dis(lower, upper);
                point[i] = dis(gen);
            }
        }

        Eigen::Vector2d qNear = nodes[0];
        double distance = DBL_MAX;
        int qNearIndex = 0;
        for(int i = 0; i < nodes.size(); i++){
            double currentDistance = (point - nodes[i]).norm();
            if(distance > currentDistance){
                qNear = nodes[i];
                qNearIndex = i;
                distance = currentDistance;
            } 
        }

        std::vector<Eigen::VectorXd> potentialControls;
        std::vector<Eigen::VectorXd> potentialQnews;
        int maxAttempts = 50; // will probably have to implement if an infinite loop happens

        while(potentialControls.size() < 5){
            Eigen::VectorXd control(problem.u_bounds.size());
            for (size_t k = 0; k < problem.u_bounds.size(); k++) {
                double lower = problem.u_bounds[k].first;
                double upper = problem.u_bounds[k].second;
                control[k] = amp::RNG::randf(lower, upper);
            }
            double dt = amp::RNG::randf(problem.dt_bounds.first, problem.dt_bounds.second);

            Eigen::VectorXd potentialQnew = qNear;

            agent.propagate(potentialQnew, control, dt); 

            // Check for obstacle intersections
            bool edgeIntersectsObstacle = false;
            for (auto obstacle : problem.obstacles) {
                std::vector<Eigen::Vector2d> vertices = obstacle.verticesCCW();
                for (size_t i = 0; i < vertices.size(); ++i) {
                    Eigen::Vector2d start = vertices[i];
                    Eigen::Vector2d end = vertices[(i + 1) % vertices.size()];
                    if (doLinesIntersect(qNear.head<2>(), potentialQnew.head<2>(), start, end)) {
                        edgeIntersectsObstacle = true;
                        break;
                    }
                }
                if(edgeIntersectsObstacle) break;
            }

            if(!edgeIntersectsObstacle){
                potentialControls.push_back(control);
                potentialQnews.push_back(potentialQnew);
            }
        }

        double closestDistanceToQnew = DBL_MAX;
        Eigen::VectorXd qNew;
        Eigen::VectorXd bestControl;
        for (int i = 0; i < potentialControls.size(); i++) {
            double distance = (potentialQnews[i] - point).norm();
            if (distance < closestDistanceToQnew) {
                closestDistanceToQnew = distance;
                bestControl = potentialControls[i];
                qNew = potentialQnews[i];
            }
        }

        nodes[currentNodeIndex] = qNew;
        points.push_back(qNew);

        edges.push_back(std::make_tuple(qNearIndex, currentNodeIndex, (qNear - qNew).norm()));

        currentNodeIndex++;
        // // Calculate the midpoint of the goal region
        // Eigen::VectorXd goalMidpoint(problem.q_goal.size());
        // for (size_t i = 0; i < problem.q_goal.size(); ++i) {
        //     double lower = problem.q_goal[i].first;
        //     double upper = problem.q_goal[i].second;
        //     goalMidpoint[i] = (lower + upper) / 2.0;  // Calculate midpoint
        // }
            
        if(isWithinGoal(qNew, problem.q_goal)){
            pathFound = true;
        } 

        counter++;
    }
    // Connect the edges in the graph
    for (const auto& [from, to, weight] : edges) graphPtr->connect(from, to, weight);  // Connect the edges in the graph

    // Use A* to find the path from q_init to q_goal
    amp::ShortestPathProblem sp_problem;
    sp_problem.graph = graphPtr;
    sp_problem.init_node = 0; 
    sp_problem.goal_node = nodes.size() - 1;  

    MyAStarAlgo astar;
    ZeroHeuristic heuristic;
    MyAStarAlgo::GraphSearchResult result = astar.search(sp_problem, heuristic);  // No heuristic (Dijkstra)
    // Iterator-based approach to traverse node_path
    // Ensure the initial state is added to waypoints
    path.waypoints.push_back(nodes[*result.node_path.begin()]);

    // Iterate through the path to add segments with controls and durations
    auto it = result.node_path.begin();
    auto next_it = std::next(it);

    for (; next_it != result.node_path.end(); ++it, ++next_it) {
        int fromNode = *it;
        int toNode = *next_it;

        // Get the start and end states for this segment
        const Eigen::VectorXd& startState = nodes[fromNode];
        const Eigen::VectorXd& endState = nodes[toNode];

        // Calculate control and duration to go from startState to endState
        auto [control, dt] = calculateControlAndDuration(startState, endState, problem);

        // Append endState, control, and duration to the path
        path.waypoints.push_back(endState);
        path.controls.push_back(control);
        path.durations.push_back(dt);
    }

    // If a path was found, mark it as valid
    path.valid = result.success;


    // if(result.success){
    //     smoothPath(path, problem);
    // }

    return path;
}

bool isWithinGoal(const Eigen::VectorXd& state, const std::vector<std::pair<double, double>>& goalBounds) {
    for (size_t i = 0; i < goalBounds.size(); ++i) {
        if (state[i] < goalBounds[i].first || state[i] > goalBounds[i].second) {
            return false;
        }
    }
    return true;
}

// Helper function to calculate control and duration between two states
std::pair<Eigen::VectorXd, double> calculateControlAndDuration(
    const Eigen::VectorXd& start, const Eigen::VectorXd& end, const amp::KinodynamicProblem2D& problem) {

    Eigen::VectorXd direction = (end - start).normalized();  // Direction vector from start to end
    double distance = (end - start).norm();

    // Choose duration based on available dt_bounds
    double dt = std::clamp(distance, problem.dt_bounds.first, problem.dt_bounds.second);
    Eigen::VectorXd control = direction * (distance / dt);  // Scale control based on chosen dt

    return {control, dt};
}

// // Implement path smoothing with shortcutting
// void smoothPath(amp::Path2D& path, const amp::Problem2D& problem) {
//     const int maxIterations = 50;  // Number of smoothing attempts
//     for (int i = 0; i < maxIterations; ++i) {
//         // Randomly pick two waypoints in the path
//         if (path.waypoints.size() <= 2) {
//             break;  // No smoothing needed for fewer than 3 waypoints
//         }

//         int index1 = std::rand() % path.waypoints.size();
//         int index2 = std::rand() % path.waypoints.size();

//         // Ensure index1 < index2
//         if (index1 > index2) std::swap(index1, index2);
//         if (index2 - index1 <= 1) continue;  // Skip consecutive points

//         Eigen::Vector2d p1 = path.waypoints[index1];
//         Eigen::Vector2d p2 = path.waypoints[index2];

//         // Check if the straight line between p1 and p2 is collision-free
//         bool collisionFree = true;
//         for (auto& obstacle : problem.obstacles) {
//             std::vector<Eigen::Vector2d> vertices = obstacle.verticesCCW();
//             for (size_t i = 0; i < vertices.size(); ++i) {
//                 Eigen::Vector2d start = vertices[i];
//                 Eigen::Vector2d end = vertices[(i + 1) % vertices.size()];
//                 if (doLinesIntersect(p1, p2, start, end)) {
//                     collisionFree = false;
//                     break;
//                 }
//             }
//         }

//         // If the straight line is valid, remove the intermediate waypoints
//         if (collisionFree) {
//             path.waypoints.erase(path.waypoints.begin() + index1 + 1, path.waypoints.begin() + index2);
//         }
//     }
// }
