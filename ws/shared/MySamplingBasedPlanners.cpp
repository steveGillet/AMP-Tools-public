#include "MySamplingBasedPlanners.h"
#include "MyAStar.h"
#include <random>
#include "CSpaceSkeleton.h"
#include <functional>
#include <cfloat>

// Define a functor for the zero heuristic
class ZeroHeuristic : public amp::SearchHeuristic {
public:
    double operator()(amp::Node) const override {
        return 0.0;  // Always return 0 for Dijkstra's algorithm (no heuristic)
    }
};

// Implement the overridden pure virtual function from PRM2D
amp::Path2D MyPRM::plan(const amp::Problem2D& problem) {
    // You can either implement this method or call the two-argument plan method
    // Create a graphPtr and pass it to the two-argument version
    std::shared_ptr<amp::Graph<double>> graphPtr = std::make_shared<amp::Graph<double>>();
    int n = 500;
    double r = 3;
    auto results = plan(problem, graphPtr, n, r);
    amp::Path2D path = std::get<0>(results);
    return path;
}


// Implement your PRM algorithm here
std::tuple<amp::Path2D, MyAStarAlgo::GraphSearchResult, std::map<amp::Node, Eigen::Vector2d>> MyPRM::plan(const amp::Problem2D& problem, std::shared_ptr<amp::Graph<double>>& graphPtr, int n, double r) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    amp::Path2D path;

    std::map<amp::Node, Eigen::Vector2d> nodes;
    std::vector<Eigen::Vector2d> points;
    points.push_back(problem.q_init);
    points.push_back(problem.q_goal);

    // Sampling random points
    int counter = 0;
    while (counter < n) {
        std::uniform_real_distribution<> xdis(problem.x_min, problem.x_max);
        std::uniform_real_distribution<> ydis(problem.y_min, problem.y_max);
        double x = xdis(gen);
        double y = ydis(gen);
        Eigen::Vector2d point(x, y);

        bool pointInObstacle = false;
        for (auto obstacle : problem.obstacles) {
            if (isPointInPolygon(point, obstacle)) pointInObstacle = true;
        }
        if (!pointInObstacle) points.push_back(point);
        
        counter++;
    }

    // Adding points as nodes to the graph
    for (amp::Node i = 0; i < points.size(); ++i) nodes[i] = points[i]; // Add point-index pair to the map

    // Creating edges between nearby nodes
    std::vector<std::tuple<amp::Node, amp::Node, double>> edges;
    for (auto homeNode : nodes) {
        for (auto neighborNode : nodes) {
            if (homeNode != neighborNode) {
                double distance = (neighborNode.second - homeNode.second).norm();  // Calculate Euclidean distance
                if (distance < r) {
                    std::tuple<amp::Node, amp::Node, double> edge = {homeNode.first, neighborNode.first, distance};

                    // Check for obstacle intersections
                    bool edgeIntersectsObstacle = false;
                    for (auto obstacle : problem.obstacles) {
                        std::vector<Eigen::Vector2d> vertices = obstacle.verticesCCW();
                        for (size_t i = 0; i < vertices.size(); ++i) {
                            Eigen::Vector2d start = vertices[i];
                            Eigen::Vector2d end = vertices[(i + 1) % vertices.size()];
                            if (doLinesIntersect(nodes.at(std::get<0>(edge)), nodes.at(std::get<1>(edge)), start, end)) {
                                edgeIntersectsObstacle = true;
                                break;
                            }
                        }
                    }

                    // Add the edge if it doesn't intersect any obstacles
                    if (!edgeIntersectsObstacle) edges.push_back(edge);
                }
            }
        }
    }

    // Connect the edges in the graph
    for (const auto& [from, to, weight] : edges) graphPtr->connect(from, to, weight);  // Connect the edges in the graph

    // graphPtr->print();

    // Use A* to find the path from q_init to q_goal
    amp::ShortestPathProblem sp_problem;
    sp_problem.graph = graphPtr;
    sp_problem.init_node = 0;  // Assuming q_init is the first node
    sp_problem.goal_node = 1;  // Assuming q_goal is the second node

    MyAStarAlgo astar;
    ZeroHeuristic heuristic;
    MyAStarAlgo::GraphSearchResult result = astar.search(sp_problem, heuristic);  // No heuristic (Dijkstra)

    // Convert the result into a Path2D
    for (auto node : result.node_path) {
        path.waypoints.push_back(nodes[node]);
    }

    // if(result.success){
    //     smoothPath(path, problem);
    // }

    return std::make_tuple(path, result, nodes);
}

// Implement the overridden pure virtual function from PRM2D
amp::Path2D MyRRT::plan(const amp::Problem2D& problem) {
    // You can either implement this method or call the two-argument plan method
    // Create a graphPtr and pass it to the two-argument version
    std::cout << "Can't override virtual 2D method to match XD implementation." << std::endl;
    amp::Path2D path;
    return path;
}

// Implement your RRT algorithm here
std::tuple<amp::MultiAgentPath2D, MyAStarAlgo::GraphSearchResult, std::map<amp::Node, Eigen::VectorXd>> MyRRT::plan(const amp::MultiAgentProblem2D& problem, std::shared_ptr<amp::Graph<double>>& graphPtr, int n, double r, double epsilon, double p) {
    amp::MultiAgentPath2D path;
    static std::random_device rd;
    static std::mt19937 gen(rd());
    int dimensions = problem.numAgents() * 2;
    
    std::vector<std::tuple<amp::Node, amp::Node, double>> edges;
    std::map<amp::Node, Eigen::VectorXd> nodes;
    std::vector<Eigen::VectorXd> points;
    Eigen::VectorXd combinedStart(dimensions);
    Eigen::VectorXd combinedGoal(dimensions);

    double minRadius = DBL_MAX;
    for (auto agent : problem.agent_properties){
        if(agent.radius < minRadius){
            minRadius = agent.radius;
        }
    }

    // Initialize combined start and goal states
    for (int i = 0; i < problem.numAgents(); i++) {
        combinedStart.segment<2>(i * 2) = problem.agent_properties[i].q_init;
        combinedGoal.segment<2>(i * 2) = problem.agent_properties[i].q_goal;
    }

    points.push_back(combinedStart);
    nodes[0] = combinedStart;
    int currentNodeIndex = 1;

    std::vector<bool> agentReachedGoal(problem.numAgents(), false);
    int agentsAtGoal = 0;

    // Sampling random points
    int counter = 0;
    bool pathFound = false;
    while (counter < n && agentsAtGoal < problem.numAgents()) {
        Eigen::VectorXd point(dimensions);
        std::bernoulli_distribution bernoulli(p);

        if(bernoulli(gen)){
            point = combinedGoal;
        }
        else{
            for (int i = 0; i < dimensions; i++){
                std::uniform_real_distribution<> dis((i % 2 == 0) ? problem.x_min : problem.y_min,
                                                  (i % 2 == 0) ? problem.x_max : problem.y_max);
                point[i] = dis(gen);
            }
            
        }

        Eigen::VectorXd qNear = nodes[0];
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

        // Generate a new point qNew, but only for agents that haven't reached their goals
        Eigen::VectorXd qNew = qNear;
        for (int i = 0; i < problem.numAgents(); i++) {
            if (!agentReachedGoal[i]) {
                qNew.segment<2>(i * 2) = qNear.segment<2>(i * 2) + r * (point.segment<2>(i * 2) - qNear.segment<2>(i * 2)).normalized();
            }
        }

        double distanceStep = (qNew - qNear).norm();
        
        int numSteps = static_cast<int>(std::ceil(distanceStep / (minRadius/2)));

        // checking individual agents in obstacles
        bool pointInObstacle = false;
        for (int currentStep = 0; currentStep <= numSteps; currentStep++){
            Eigen::VectorXd currentPoint = qNear + (qNew - qNear) * (static_cast<double>(currentStep) / numSteps);
            for (int i = 0; i < problem.numAgents(); i++) {
                if(!agentReachedGoal[i]){

                Eigen::Vector2d agentPosition = currentPoint.segment<2>(i * 2);
                for (auto obstacle : problem.obstacles) {
                    if (isCircleInPolygon(agentPosition, problem.agent_properties[i].radius, obstacle)){
                        pointInObstacle = true;
                        break;
                    } 
                }
                for(int j = 0; j < problem.numAgents(); j++){
                    if(!agentReachedGoal[j]&& i != j && (currentPoint.segment<2>(i * 2) - currentPoint.segment<2>(j * 2)).norm() < problem.agent_properties[i].radius + problem.agent_properties[j].radius){
                        pointInObstacle = true;
                        break;
                    }
                }
                }
            }
        }

        // Add the edge if it doesn't intersect any obstacles
        if (!pointInObstacle) {
            nodes[currentNodeIndex] = qNew;
            points.push_back(qNew);

            edges.push_back(std::make_tuple(qNearIndex, currentNodeIndex, r));

            currentNodeIndex++;
            
            for (int i = 0; i < problem.numAgents(); i++){
                if (!agentReachedGoal[i] && (qNew.segment<2>(i*2) - problem.agent_properties[i].q_goal).norm() < epsilon){
                    agentReachedGoal[i] = true;
                    agentsAtGoal++;
                    if(agentsAtGoal == problem.numAgents()){
                        nodes[currentNodeIndex] = combinedGoal;
                        points.push_back(combinedGoal);
                        edges.push_back(std::make_tuple(currentNodeIndex - 1, currentNodeIndex, (qNew - combinedGoal).norm()));
                        currentNodeIndex++;
                    } 
                }
            }
        };
        

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


    // Construct the paths for each agent
    for (int i = 0; i < problem.numAgents(); i++) {
        amp::Path2D agentPath;
        // Extract the waypoints for each agent from the nodes
        for (const auto& node : result.node_path) {
            Eigen::VectorXd agentState = nodes[node].segment<2>(i * 2); // Get the 2D state for the agent
            agentPath.waypoints.push_back(agentState);
        }


        path.agent_paths.push_back(agentPath);
    }

    // // smooth path
    // if(result.success){
    //     smoothPath(path, problem);
    // }

    return std::make_tuple(path, result, nodes);
}

// std::tuple<amp::Path2D, bool> MyRRT::planDecentralized(
//     const amp::MultiAgentProblem2D& problem, 
//     int agentIndex, 
//     const amp::MultiAgentPath2D& previousPaths, // Paths of already planned agents
//     int maxRetries
// ) {
//     amp::Path2D agentPath;
//     bool pathFound = false;
//     int attempt = 0;
//     static std::random_device rd;
//     static std::mt19937 gen(rd());

//     Eigen::Vector2d start = problem.agent_properties[agentIndex].q_init;
//     Eigen::Vector2d goal = problem.agent_properties[agentIndex].q_goal;

//     while (!pathFound && attempt < maxRetries) {
//         std::shared_ptr<amp::Graph<double>> graphPtr = std::make_shared<amp::Graph<double>>();
//         std::map<amp::Node, Eigen::Vector2d> nodes;
//         nodes[0] = start;  // Ensure start node is included
//         int currentNodeIndex = 1;
//         std::vector<std::tuple<amp::Node, amp::Node, double>> edges;

//         // Sample random points to create the roadmap
//         for (int i = 0; i < 7500; ++i) {  // Adjust sample size if needed
//             Eigen::Vector2d randomPoint;
//             if (std::bernoulli_distribution(0.05)(gen)) {
//                 randomPoint = goal;
//             } else {
//                 std::uniform_real_distribution<> xdis(problem.x_min, problem.x_max);
//                 std::uniform_real_distribution<> ydis(problem.y_min, problem.y_max);
//                 randomPoint = Eigen::Vector2d(xdis(gen), ydis(gen));
//             }

//             Eigen::Vector2d nearestPoint = nodes[0];
//             double minDistance = DBL_MAX;
//             int nearestIndex = 0;

//             for (const auto& [index, node] : nodes) {
//                 double dist = (node - randomPoint).norm();
//                 if (dist < minDistance) {
//                     nearestPoint = node;
//                     nearestIndex = index;
//                     minDistance = dist;
//                 }
//             }

//             Eigen::Vector2d direction = (randomPoint - nearestPoint).normalized();
//             Eigen::Vector2d newPoint = nearestPoint + 0.5 * direction;  // Step size of 0.5

//             bool collisionFree = true;
//             for (const auto& obstacle : problem.obstacles) {
//                 if (isCircleInPolygon(newPoint, problem.agent_properties[agentIndex].radius, obstacle)) {
//                     collisionFree = false;
//                     break;
//                 }
//             }

//             if (collisionFree) {
//                 nodes[currentNodeIndex] = newPoint;
//                 edges.emplace_back(nearestIndex, currentNodeIndex, minDistance);
//                 currentNodeIndex++;
//             }
//         }

//         // Ensure goal node is included
//         nodes[currentNodeIndex] = goal;

//         // Connect nodes in graph
//         for (const auto& [from, to, weight] : edges) {
//             graphPtr->connect(from, to, weight);
//         }

//         // Run A* search on the generated graph
//         amp::ShortestPathProblem sp_problem;
//         sp_problem.graph = graphPtr;
//         sp_problem.init_node = 0;
//         sp_problem.goal_node = currentNodeIndex;

//         MyAStarAlgo astar;
//         ZeroHeuristic heuristic;
//         MyAStarAlgo::GraphSearchResult result = astar.search(sp_problem, heuristic);

//         // Ensure start and goal are included even if A* fails or if they are not already in the result
//         agentPath.waypoints.clear();
//         agentPath.waypoints.push_back(start);  // Start with start node

//         if (result.success) {
//             for (const auto& node : result.node_path) {
//                 agentPath.waypoints.push_back(nodes[node]);
//             }
//         }

//         if (agentPath.waypoints.back() != goal) {
//             agentPath.waypoints.push_back(goal);  // Ensure goal node is the last waypoint
//         }

//         // Collision check with previous paths
//         pathFound = true;
//         for (size_t i = 0; i < agentPath.waypoints.size(); ++i) {
//             for (const auto& otherAgentPath : previousPaths.agent_paths) {
//                 if (i < otherAgentPath.waypoints.size() && 
//                     (agentPath.waypoints[i] - otherAgentPath.waypoints[i]).norm() < 
//                     problem.agent_properties[agentIndex].radius * 2) {
//                     pathFound = false;
//                     break;
//                 }
//             }
//             if (!pathFound) break;
//         }

//         attempt++;
//     }

//     return {agentPath, pathFound};
// }


amp::Path2D MyRRT::planDecentralized(
    const amp::MultiAgentProblem2D& problem,
    int agentIndex,
    const amp::MultiAgentPath2D& previousPaths,
    int n,
    double r,
    double epsilon,
    double p
) {
    bool failedPath = true;
    amp::Path2D path;
    int maxRetries = 40;
    int attempt = 0;
    double pAdjust = p;

    while (failedPath && attempt < maxRetries) {
        attempt++;
        path = amp::Path2D(); // Reset the path
        failedPath = false;
        static std::random_device rd;
        static std::mt19937 gen(rd());
        std::shared_ptr<amp::Graph<double>> graphPtr = std::make_shared<amp::Graph<double>>();

        Eigen::Vector2d start = problem.agent_properties[agentIndex].q_init;
        Eigen::Vector2d goal = problem.agent_properties[agentIndex].q_goal;

        std::vector<std::tuple<amp::Node, amp::Node, double>> edges;
        std::map<amp::Node, Eigen::Vector2d> nodes;
        std::vector<Eigen::Vector2d> points;
        points.push_back(start);
        nodes[0] = start;
        int currentNodeIndex = 1;

        int counter = 0;
        bool pathFound = false;

        while (counter < n && !pathFound) {
            Eigen::Vector2d point;
            std::bernoulli_distribution bernoulli(pAdjust);
            if (bernoulli(gen)) {
                point = goal;
            } else {
                std::uniform_real_distribution<> xdis(problem.x_min, problem.x_max);
                std::uniform_real_distribution<> ydis(problem.y_min, problem.y_max);
                point = Eigen::Vector2d(xdis(gen), ydis(gen));
            }

            // Find nearest node
            Eigen::Vector2d qNear = nodes[0];
            double distance = DBL_MAX;
            int qNearIndex = 0;
            for (int i = 0; i < nodes.size(); i++) {
                double currentDistance = (point - nodes[i]).norm();
                if (distance > currentDistance) {
                    qNear = nodes[i];
                    qNearIndex = i;
                    distance = currentDistance;
                }
            }

            // Generate qNew with slight perturbation
            Eigen::Vector2d step = r * (point - qNear).normalized();
            std::uniform_real_distribution<> perturb(-0.1 * r, 0.1 * r);
            Eigen::Vector2d qNew = qNear + step + Eigen::Vector2d(perturb(gen), perturb(gen));

            bool pointInObstacle = false;
            for (const auto& obstacle : problem.obstacles) {
                if (isCircleInPolygon(qNew, problem.agent_properties[agentIndex].radius, obstacle)) {
                    pointInObstacle = true;
                    break;
                }
            }

            bool edgeIntersectsObstacle = false;
            for (const auto& obstacle : problem.obstacles) {
                const auto& vertices = obstacle.verticesCCW();
                for (size_t i = 0; i < vertices.size(); ++i) {
                    Eigen::Vector2d start = vertices[i];
                    Eigen::Vector2d end = vertices[(i + 1) % vertices.size()];
                    if (doLinesIntersect(qNear, qNew, start, end)) {
                        edgeIntersectsObstacle = true;
                        break;
                    }
                }
            }

            if (!pointInObstacle && !edgeIntersectsObstacle) {
                nodes[currentNodeIndex] = qNew;
                points.push_back(qNew);
                edges.push_back(std::make_tuple(qNearIndex, currentNodeIndex, r));
                currentNodeIndex++;

                if ((qNew - goal).norm() < epsilon) {
                    pathFound = true;
                    if (qNew != goal) {
                        nodes[currentNodeIndex] = goal;
                        points.push_back(goal);
                        edges.push_back(std::make_tuple(currentNodeIndex - 1, currentNodeIndex, (qNew - goal).norm()));
                        currentNodeIndex++;
                    }
                }
            }
            counter++;
        }

        for (const auto& [from, to, weight] : edges) {
            graphPtr->connect(from, to, weight);
        }

        amp::ShortestPathProblem sp_problem;
        sp_problem.graph = graphPtr;
        sp_problem.init_node = 0;
        sp_problem.goal_node = nodes.size() - 1;
        MyAStarAlgo astar;
        ZeroHeuristic heuristic;
        MyAStarAlgo::GraphSearchResult result = astar.search(sp_problem, heuristic);

        for (auto node : result.node_path) {
            path.waypoints.push_back(nodes[node]);
        }

        if (path.waypoints.empty() || path.waypoints.front() != start) {
            path.waypoints.insert(path.waypoints.begin(), start);
        }
        if (path.waypoints.back() != goal) {
            path.waypoints.push_back(goal);
        }

        for (const auto& point : path.waypoints) {
            for (const auto& previousPath : previousPaths.agent_paths) {
                for (const auto& previousPoint : previousPath.waypoints) {
                    if ((point - previousPoint).norm() < problem.agent_properties[agentIndex].radius * 2) {
                        failedPath = true;
                        break;
                    }
                }
                if (failedPath) break;
            }
            if (failedPath) break;
        }

        if (failedPath) {
            std::cout << "Collision detected on attempt " << attempt << ", retrying...\n";
            pAdjust = std::min(pAdjust + 0.05, 0.95);
        }
    }

    if (attempt >= maxRetries) {
        std::cout << "Warning: Unable to find a collision-free path after " << maxRetries << " attempts.\n";
    }

    return path;
}




// Implement path smoothing with shortcutting
void smoothPath(amp::MultiAgentPath2D& path, const amp::MultiAgentProblem2D& problem) {
    const int maxIterations = 50;  // Number of smoothing attempts
    double minRadius = DBL_MAX;
    for (auto agent : problem.agent_properties){
        if(agent.radius < minRadius){
            minRadius = agent.radius;
        }
    }
    for (int i = 0; i < problem.numAgents(); i++){
        for (int j = 0; j < maxIterations; j++) {
            // Randomly pick two waypoints in the path
            if (path.agent_paths[i].waypoints.size() <= 2) {
                break;  // No smoothing needed for fewer than 3 waypoints
            }

            int index1 = std::rand() % path.agent_paths[i].waypoints.size();
            int index2 = std::rand() % path.agent_paths[i].waypoints.size();

            // Ensure index1 < index2
            if (index1 > index2) std::swap(index1, index2);
            if (index2 - index1 <= 1) continue;  // Skip consecutive points

            Eigen::Vector2d p1 = path.agent_paths[i].waypoints[index1];
            Eigen::Vector2d p2 = path.agent_paths[i].waypoints[index2];

            double distanceStep = (p2 - p1).norm();

            int numSteps = static_cast<int>(std::ceil(distanceStep / (minRadius/2)));

            // checking individual agents in obstacles
            bool collisionFree = true;
            for (int currentStep = 0; currentStep <= numSteps; currentStep++){
                Eigen::Vector2d currentPoint = p1 + (p2 - p1) * (static_cast<double>(currentStep) / numSteps);
                for (auto obstacle : problem.obstacles) {
                    if (isCircleInPolygon(currentPoint, problem.agent_properties[i].radius, obstacle)){
                        collisionFree = false;
                        break;
                    } 
                }
                if (!collisionFree) break;
                for(int k = 0; k < problem.numAgents(); k++){
                    if (index1 >= path.agent_paths[k].waypoints.size() || index2 >= path.agent_paths[k].waypoints.size()) continue;
                    if(i != k){
                        Eigen::Vector2d otherAgentP1 = path.agent_paths[k].waypoints[index1];
                        Eigen::Vector2d otherAgentP2 = path.agent_paths[k].waypoints[index2];
                        Eigen::Vector2d otherIntermediatePoint = otherAgentP1 + currentStep * (otherAgentP2 - otherAgentP1) / numSteps;

                        if((otherIntermediatePoint - currentPoint).norm() < problem.agent_properties[i].radius + problem.agent_properties[k].radius){
                            collisionFree = false;
                            break;
                        }
                    }
                }
                if (!collisionFree) break;
            }

            // If the straight line is valid, remove the intermediate waypoints
            if (collisionFree) {
                path.agent_paths[i].waypoints.erase(path.agent_paths[i].waypoints.begin() + index1 + 1, path.agent_paths[i].waypoints.begin() + index2);
            }
        }
    }
}

bool isCircleInPolygon(const Eigen::Vector2d& center, double radius, const amp::Polygon& polygon) {
    // Check if the center of the circle is inside the polygon
    if (isPointInPolygon(center, polygon)) {
        return true; // Collision if the center is inside
    }

    // Check each edge of the polygon
    std::vector<Eigen::Vector2d> vertices = polygon.verticesCCW();
    for (size_t i = 0; i < vertices.size(); ++i) {
        Eigen::Vector2d start = vertices[i];
        Eigen::Vector2d end = vertices[(i + 1) % vertices.size()];

        // Calculate the distance from the circle's center to the edge
        double distance = pointToLineSegmentDistance(center, start, end);
        if (distance <= radius) {
            return true; // Collision if the distance is less than or equal to the radius
        }
    }

    return false; // No collision detected
}

double pointToLineSegmentDistance(const Eigen::Vector2d& point, const Eigen::Vector2d& start, const Eigen::Vector2d& end) {
    Eigen::Vector2d line = end - start;
    double lineLengthSquared = line.squaredNorm();
    if (lineLengthSquared == 0.0) {
        return (point - start).norm(); // The line segment is a point
    }

    // Project the point onto the line segment
    double t = ((point - start).dot(line)) / lineLengthSquared;
    t = std::max(0.0, std::min(1.0, t)); // Clamp t to the range [0, 1]

    Eigen::Vector2d projection = start + t * line; // Find the closest point on the line segment
    return (point - projection).norm(); // Return the distance from the point to the closest point
}