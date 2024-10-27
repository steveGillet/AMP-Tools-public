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

    // Initialize combined start and goal states
    for (int i = 0; i < problem.numAgents(); i++) {
        combinedStart.segment<2>(i * 2) = problem.agent_properties[i].q_init;
        combinedGoal.segment<2>(i * 2) = problem.agent_properties[i].q_goal;
    }

    points.push_back(combinedStart);
    nodes[0] = combinedStart;
    int currentNodeIndex = 1;

    // Sampling random points
    int counter = 0;
    bool pathFound = false;
    while (counter < n && !pathFound) {
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

        Eigen::VectorXd step = r * (point - qNear).normalized();
        Eigen::VectorXd qNew = qNear + step;

        double distanceStep = (qNew - qNear).norm();
        double minRadius = DBL_MAX;
        for (auto agent : problem.agent_properties){
            if(agent.radius < minRadius){
                minRadius = agent.radius;
            }
        }
        int numSteps = static_cast<int>(std::ceil(distanceStep / (minRadius/2)));

        // checking individual agents in obstacles
        bool pointInObstacle = false;
        for (int currentStep = 0; currentStep <= numSteps; currentStep++){
            Eigen::VectorXd currentPoint = qNear + (qNew - qNear) * (static_cast<double>(currentStep) / numSteps);
            for (int i = 0; i < problem.numAgents(); i++) {
                Eigen::Vector2d agentPosition = currentPoint.segment<2>(i * 2);
                for (auto obstacle : problem.obstacles) {
                    if (isCircleInPolygon(agentPosition, problem.agent_properties[i].radius, obstacle)){
                        pointInObstacle = true;
                        break;
                    } 
                }
                for(int j = 0; j < problem.numAgents(); j++){
                    if(i != j && (currentPoint.segment<2>(i * 2) - currentPoint.segment<2>(j * 2)).norm() < problem.agent_properties[i].radius + problem.agent_properties[j].radius){
                        pointInObstacle = true;
                        break;
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
            
            if((qNew - combinedGoal).norm() < epsilon){
                pathFound = true;
                if(qNew != combinedGoal){
                    nodes[currentNodeIndex] = combinedGoal;
                    points.push_back(combinedGoal);
                    edges.push_back(std::make_tuple(currentNodeIndex - 1, currentNodeIndex, (qNew - combinedGoal).norm()));
                    currentNodeIndex++;
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