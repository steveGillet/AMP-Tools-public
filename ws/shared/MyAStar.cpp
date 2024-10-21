#include "MyAStar.h"
#include <queue>
#include <unordered_map>
#include <limits>

// Define a struct for the priority queue
struct NodeCost {
    double cost;
    amp::Node node;

    // Custom comparator for min-heap
    bool operator<(const NodeCost& other) const {
        return cost > other.cost; // greater cost gets lower priority in min-heap
    }
};

// Implement the search method for the A* algorithm (with a flag for Dijkstra vs A*)
MyAStarAlgo::GraphSearchResult MyAStarAlgo::search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic) {
    bool use_heuristic = false;
    // std::cout << "Starting Graph Search (" << (use_heuristic ? "A*" : "Dijkstra") << "): Init --> goal | " << problem.init_node << " --> " << problem.goal_node << std::endl;

    GraphSearchResult result = {false, {}, 0.0}; // initialize the results object

    // Open list (min-heap priority queue)
    std::priority_queue<NodeCost> open_list;
    open_list.push({0.0, problem.init_node});

    // Closed set and cost to reach nodes
    std::unordered_map<amp::Node, double> g_cost;
    std::unordered_map<amp::Node, amp::Node> came_from;

    g_cost[problem.init_node] = 0.0;

    while (!open_list.empty()) {
        NodeCost current = open_list.top();
        open_list.pop();

        amp::Node current_node = current.node;

        // Check if goal is reached
        if (current_node == problem.goal_node) {
            result.success = true;

            // Construct the path from the came_from map
            while (current_node != problem.init_node) {
                result.node_path.insert(result.node_path.begin(), current_node);
                current_node = came_from[current_node];
            }
            result.node_path.insert(result.node_path.begin(), problem.init_node);
            result.path_cost = g_cost[problem.goal_node];
            // result.print();
            return result;
        }

        // Retrieve children and outgoing edges in parallel
        const auto& neighbors = problem.graph->children(current_node);
        const auto& edge_weights = problem.graph->outgoingEdges(current_node);

        for (size_t i = 0; i < neighbors.size(); ++i) {
            amp::Node neighbor = neighbors[i];
            double weight = edge_weights[i];
            double tentative_g_cost = g_cost[current_node] + weight;

            if (g_cost.find(neighbor) == g_cost.end() || tentative_g_cost < g_cost[neighbor]) {
                // Update cost and parent
                g_cost[neighbor] = tentative_g_cost;
                came_from[neighbor] = current_node;

                // Use heuristic if specified, otherwise treat heuristic as 0 (for Dijkstra's)
                double heuristic_cost = use_heuristic ? heuristic(neighbor) : 0.0;
                double f_cost = tentative_g_cost + heuristic_cost;
                open_list.push({f_cost, neighbor});
            }
        }
    }

    // If the goal is not reachable
    // result.print();
    return result;
}
