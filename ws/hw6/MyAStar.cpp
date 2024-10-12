#include "MyAStar.h"
#include <vector>
#include <algorithm>
#include <tuple>

// Implement the search method for the A* algorithm
MyAStarAlgo::GraphSearchResult MyAStarAlgo::search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic) {
    std::cout << "Starting A* Graph Search: Init --> goal | " << problem.init_node << " --> " << problem.goal_node << std::endl;
    GraphSearchResult result = {false, {}, 0.0}; // initialize the results object
    // result.node_path.push_back(problem.init_node);

    std::vector<std::tuple<amp::Node, amp::Node, int>> openList;
    std::vector<std::tuple<amp::Node, amp::Node, int>> closedList;

    openList.emplace_back(problem.init_node, problem.init_node, 0);
    while(!openList.empty()){
        std::tuple<amp::Node, amp::Node, int> leastCostNode = openList[0];
        // setting this sucker to maxx
        int leastCost = ~0u >> 1;
        // going through open list and finding which has the least cost
        for(auto& node : openList){
            // int parentIndex = -1;
            // const auto& parentsNode = problem.graph->parents(std::get<0>(node));
            // std::cout << "parentsNode size: " << parentsNode.size() << std::endl;
            // for(int i = 0; i < parentsNode.size(); i++){
            //     if(parentsNode[i] == std::get<1>(node)) parentIndex = i;
            // }
            // if(parentIndex != -1) std::get<2>(node) += problem.graph->incomingEdges(std::get<0>(node))[parentIndex];
            int hCost = heuristic(std::get<0>(node));
            int cost = hCost + std::get<2>(node);
            // std::cout << "Node: " << std::get<0>(node) << std::endl;
            // std::cout << "Cost: " << cost << std::endl;
            if(cost < leastCost){
                leastCost = cost;
                leastCostNode = node;
            }
        }

        auto it = std::find(openList.begin(), openList.end(), leastCostNode);
        if (it != openList.end()) {
            openList.erase(it);
        }

        closedList.push_back(leastCostNode);

        if(std::get<0>(leastCostNode) == problem.goal_node) break;

        // std::cout << "Number of children: " << problem.graph->children(std::get<0>(leastCostNode)).size() << ". Number of Edges Outgoing" << problem.graph->outgoingEdges(std::get<0>(leastCostNode)).size();

        const auto& leastCostNodeChildren = problem.graph->children(std::get<0>(leastCostNode));
        const auto& leastCostNodeEdges = problem.graph->outgoingEdges(std::get<0>(leastCostNode));
        // if the children of the current node are not in the open list then put them in there
        if(!leastCostNodeChildren.empty()){
            for(int i = 0; i < leastCostNodeChildren.size(); i++){
                const auto& childNode = leastCostNodeChildren[i];
                int edgeCost = static_cast<int>(leastCostNodeEdges[i]);
                int childCost = std::get<2>(leastCostNode) + edgeCost;
                bool skipChild = false;
                if(!openList.empty()){
                    for(auto& node : openList){
                        if(childNode == std::get<0>(node)){
                            if(childCost < std::get<2>(node)){
                                std::get<1>(node) = std::get<0>(leastCostNode);
                                std::get<2>(node) = childCost;
                            } 
                            skipChild = true;
                            break;
                        }
                    }
                }
                if(!skipChild){
                    // std::cout << "Edge Cost: " << edgeCost << std::endl;
                    // int childCost = static_cast<int>(edgeCost);
                    // int childCost = 0;
                    openList.emplace_back(childNode, std::get<0>(leastCostNode), childCost);
                }
            }
        }
        // std::cout << "Current open list: ";
        // for(const auto& node : openList) {
        //     std::cout << "(" << std::get<0>(node) << ", " << std::get<1>(node) << ", " << std::get<2>(node) << ") ";
        // }
        // std::cout << std::endl;

        // std::cout << "Current closed list: ";
        // for(const auto& node : closedList) {
        //     std::cout << "(" << std::get<0>(node) << ", " << std::get<1>(node) << ", " << std::get<2>(node) << ") ";
        // }
        // std::cout << std::endl << std::endl;

    }

    result.node_path.push_back(problem.goal_node);
    amp::Node childNode = problem.goal_node;

    for(auto node : closedList){
        if(std::get<0>(node)==problem.goal_node) result.path_cost = std::get<2>(node);
    }

    bool initReached = false;
    while(!initReached){
        bool parentFound = false;
        for(auto& node : closedList){
            if(std::get<0>(node) == childNode){
                if(std::get<1>(node) == problem.init_node) initReached = true;
                result.node_path.push_back(std::get<1>(node));
                childNode = std::get<1>(node);
                parentFound = true;
                break;
            }
        }
        if(!parentFound){
            result.success = false;
            std::cout << "No parent found. Path incomplete." << std::endl;
            break;
        }
    }

    std::reverse(result.node_path.begin(), result.node_path.end());
    
    result.success = true;

    result.print();
    return result;
}