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

    if(result.success){
        smoothPath(path, problem);
    }

    return std::make_tuple(path, result, nodes);
}

// Implement the overridden pure virtual function from PRM2D
amp::Path2D MyRRT::plan(const amp::Problem2D& problem) {
    // You can either implement this method or call the two-argument plan method
    // Create a graphPtr and pass it to the two-argument version
    std::shared_ptr<amp::Graph<double>> graphPtr = std::make_shared<amp::Graph<double>>();
    int n = 5000;
    double r = 0.5;
    double epsilon = 0.25;
    double p = 0.05;
    auto results = plan(problem, graphPtr, n, r, epsilon, p);
    amp::Path2D path = std::get<0>(results);
    return path;
}

// Implement your RRT algorithm here
std::tuple<amp::Path2D, MyAStarAlgo::GraphSearchResult, std::map<amp::Node, Eigen::Vector2d>> MyRRT::plan(const amp::Problem2D& problem, std::shared_ptr<amp::Graph<double>>& graphPtr, int n, double r, double epsilon, double p) {
    amp::Path2D path;
    static std::random_device rd;
    static std::mt19937 gen(rd());
    
    std::vector<std::tuple<amp::Node, amp::Node, double>> edges;
    std::map<amp::Node, Eigen::Vector2d> nodes;
    std::vector<Eigen::Vector2d> points;
    points.push_back(problem.q_init);
    nodes[0] = problem.q_init;
    int currentNodeIndex = 1;

    // Sampling random points
    int counter = 0;
    bool pathFound = false;
    while (counter < n && !pathFound) {
        Eigen::Vector2d point;
        std::bernoulli_distribution bernoulli(p);

        if(bernoulli(gen)){
            point = problem.q_goal;
        }
        else{
            std::uniform_real_distribution<> xdis(problem.x_min, problem.x_max);
            std::uniform_real_distribution<> ydis(problem.y_min, problem.y_max);
            double x = xdis(gen);
            double y = ydis(gen);
            point = Eigen::Vector2d(x, y);
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

        Eigen::Vector2d step = r * (point - qNear).normalized();
        Eigen::Vector2d qNew = qNear + step;

        bool pointInObstacle = false;
        for (auto obstacle : problem.obstacles) {
            if (isPointInPolygon(qNew, obstacle)){
                pointInObstacle = true;
                break;
            } 
        }

        // Check for obstacle intersections
        bool edgeIntersectsObstacle = false;
        for (auto obstacle : problem.obstacles) {
            std::vector<Eigen::Vector2d> vertices = obstacle.verticesCCW();
            for (size_t i = 0; i < vertices.size(); ++i) {
                Eigen::Vector2d start = vertices[i];
                Eigen::Vector2d end = vertices[(i + 1) % vertices.size()];
                if (doLinesIntersect(qNear, qNew, start, end)) {
                    edgeIntersectsObstacle = true;
                    break;
                }
            }
        }

        // Add the edge if it doesn't intersect any obstacles
        if (!edgeIntersectsObstacle && !pointInObstacle) {
            nodes[currentNodeIndex] = qNew;
            points.push_back(qNew);

            edges.push_back(std::make_tuple(qNearIndex, currentNodeIndex, r));

            currentNodeIndex++;
            
            if((qNew - problem.q_goal).norm() < epsilon){
                pathFound = true;
                if(qNew != problem.q_goal){
                    nodes[currentNodeIndex] = problem.q_goal;
                    points.push_back(problem.q_goal);
                    edges.push_back(std::make_tuple(currentNodeIndex - 1, currentNodeIndex, (qNew - problem.q_goal).norm()));
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

    // Convert the result into a Path2D
    for (auto node : result.node_path) {
        path.waypoints.push_back(nodes[node]);
    }

    if(result.success){
        smoothPath(path, problem);
    }

    return std::make_tuple(path, result, nodes);
}

// Implement path smoothing with shortcutting
void smoothPath(amp::Path2D& path, const amp::Problem2D& problem) {
    const int maxIterations = 50;  // Number of smoothing attempts
    for (int i = 0; i < maxIterations; ++i) {
        // Randomly pick two waypoints in the path
        if (path.waypoints.size() <= 2) {
            break;  // No smoothing needed for fewer than 3 waypoints
        }

        int index1 = std::rand() % path.waypoints.size();
        int index2 = std::rand() % path.waypoints.size();

        // Ensure index1 < index2
        if (index1 > index2) std::swap(index1, index2);
        if (index2 - index1 <= 1) continue;  // Skip consecutive points

        Eigen::Vector2d p1 = path.waypoints[index1];
        Eigen::Vector2d p2 = path.waypoints[index2];

        // Check if the straight line between p1 and p2 is collision-free
        bool collisionFree = true;
        for (auto& obstacle : problem.obstacles) {
            std::vector<Eigen::Vector2d> vertices = obstacle.verticesCCW();
            for (size_t i = 0; i < vertices.size(); ++i) {
                Eigen::Vector2d start = vertices[i];
                Eigen::Vector2d end = vertices[(i + 1) % vertices.size()];
                if (doLinesIntersect(p1, p2, start, end)) {
                    collisionFree = false;
                    break;
                }
            }
        }

        // If the straight line is valid, remove the intermediate waypoints
        if (collisionFree) {
            path.waypoints.erase(path.waypoints.begin() + index1 + 1, path.waypoints.begin() + index2);
        }
    }
}