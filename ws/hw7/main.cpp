// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "hw/HW2.h"
#include "hw/HW5.h"
#include "MySamplingBasedPlanners.h"
#include <iostream>
#include <vector>
#include <chrono>
#include <cmath>


using namespace amp;

int main(int argc, char** argv) {
    // HW7::hint(); // Consider implementing an N-dimensional planner 

    // Example of creating a graph and adding nodes for visualization

    // Test PRM on Workspace1 of HW2
    // Problem2D problem = HW5::getWorkspace1();
    // problem.x_min = -1;
    // problem.x_max = 11;
    // problem.y_min = -3;
    // problem.y_max = 3;
    // MyPRM prm;
    // amp::Path2D path;
    // std::map<amp::Node, Eigen::Vector2d> nodes;

    // // MULTIPLE RUNS
    // // Define file output stream
    // std::ofstream csv_file("/home/steve0gillet/Desktop/algMotionPlanning/hw7/e1b2w2.csv");

    // // Write the header to the CSV
    // csv_file << "n,r,run,valid,path_length,computation_time\n";
    // std::vector<std::pair<int, double>> params = {
    //     {200, 0.5}, {200, 1.0}, {200, 1.5}, {200, 2.0}, 
    //     {500, 0.5}, {500, 1.0}, {500, 1.5}, {500, 2.0}
    // };
    
    // const int numRuns = 100;

    // for (const auto& [n, r] : params) {
    //     std::vector<double> pathLengths;
    //     std::vector<double> computationTimes;
    //     int validSolutionCount = 0;
        
    //     for (int i = 0; i < numRuns; ++i) {
    //         bool validSolution;
    //         auto start = std::chrono::high_resolution_clock::now();
            
    //         MyAStarAlgo::GraphSearchResult result;
    //         std::shared_ptr<amp::Graph<double>> graphPtr = std::make_shared<amp::Graph<double>>();
    //         std::tie(path, result, nodes) = prm.plan(problem, graphPtr, n, r);
            
    //         auto stop = std::chrono::high_resolution_clock::now();
    //         std::chrono::duration<double> duration = stop - start;

    //         if (result.success) validSolutionCount++;

    //         pathLengths.push_back(result.path_cost);
    //         computationTimes.push_back(duration.count());
    //         // Write each run result to the CSV file
    //         csv_file << n << "," << r << "," << i+1 << "," 
    //                     << result.success << "," << result.path_cost << "," 
    //                     << duration.count() << "\n";
    //     }

    //     // Output statistics for this pair (n, r)
    //     std::cout << "For n = " << n << ", r = " << r << ":\n";
    //     std::cout << "Valid solutions: " << validSolutionCount << "/" << numRuns << "\n";
    //     std::cout << "Average path length: " << std::accumulate(pathLengths.begin(), pathLengths.end(), 0.0) / numRuns << "\n";
    //     std::cout << "Average computation time: " << std::accumulate(computationTimes.begin(), computationTimes.end(), 0.0) / numRuns << " seconds\n";
    // }

    // csv_file.close();

    // // SINGLE RUNS
    // int n = 200;
    // double r = 2.0;
    // MyAStarAlgo::GraphSearchResult result;
    // std::shared_ptr<amp::Graph<double>> graphPtr = std::make_shared<amp::Graph<double>>();
    // std::tie(path, result, nodes) = prm.plan(problem, graphPtr, n, r);
    // std::cout << result.path_cost << " <- Path Length." << std::endl;
    // Visualizer::makeFigure(problem, path, *graphPtr, nodes);

    // // MULTIPLE RUNS RRT
    // // Define file output stream
    // std::ofstream csv_file("/home/steve0gillet/Desktop/algMotionPlanning/hw7/e2bw1.csv");

    // // Write the header to the CSV
    // csv_file << "n,r,run,valid,path_length,computation_time\n";
    
    // const int numRuns = 100;
    // std::vector<double> pathLengths;
    // std::vector<double> computationTimes;
    // int validSolutionCount = 0;
        
    // for (int i = 0; i < numRuns; ++i) {
    //     bool validSolution;
    //     auto start = std::chrono::high_resolution_clock::now();
        
    //     MyAStarAlgo::GraphSearchResult result;
    //     std::shared_ptr<amp::Graph<double>> graphPtr = std::make_shared<amp::Graph<double>>();
    //     std::tie(path, result, nodes) = rrt.plan(problem, graphPtr, n, r, epsilon, p);
        
    //     auto stop = std::chrono::high_resolution_clock::now();
    //     std::chrono::duration<double> duration = stop - start;

    //     if (result.success) validSolutionCount++;

    //     pathLengths.push_back(result.path_cost);
    //     computationTimes.push_back(duration.count());
    //     // Write each run result to the CSV file
    //     csv_file << n << "," << r << "," << i+1 << "," 
    //                 << result.success << "," << result.path_cost << "," 
    //                 << duration.count() << "\n";
    // }

    // // Output statistics for this pair (n, r)
    // std::cout << "For n = " << n << ", r = " << r << ":\n";
    // std::cout << "Valid solutions: " << validSolutionCount << "/" << numRuns << "\n";
    // std::cout << "Average path length: " << std::accumulate(pathLengths.begin(), pathLengths.end(), 0.0) / numRuns << "\n";
    // std::cout << "Average computation time: " << std::accumulate(computationTimes.begin(), computationTimes.end(), 0.0) / numRuns << " seconds\n";

    // SINGLE RUNS RRT
    // // Generate a random problem and test RRT
    // n = 5000;
    // r = 0.5;
    // double epsilon = 0.25;
    // double p = 0.05;
    // MyRRT rrt;
    // Path2D pathRRT;
    // MyAStarAlgo::GraphSearchResult result2;
    // std::shared_ptr<amp::Graph<double>> graphPtr2 = std::make_shared<amp::Graph<double>>();
    // Problem2D problem2;
    // HW7::generateAndCheck(rrt, pathRRT, problem2);
    // std::tie(pathRRT, result2, nodes) = rrt.plan(problem2, graphPtr2, n, r, epsilon, p);
    // Visualizer::makeFigure(problem2, pathRRT, *graphPtr2, nodes);
    // std::cout << result2.path_cost << " <- Path Length." << std::endl;
    // Visualizer::showFigures();

    // Grade method
    HW7::grade<MyPRM, MyRRT>("stephen.gillet@colorado.edu", argc, argv, std::make_tuple(), std::make_tuple());
    return 0;
}