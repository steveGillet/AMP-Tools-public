#include "MyCSConstructors.h"
#include "CSpaceSkeleton.h"
#include "ManipulatorSkeleton.h"

////////////////////// THIS IS FROM HW4 //////////////////////

/* You can just move these classes to shared folder and include them instead of copying them to hw6 project*/

// std::pair<std::size_t, std::size_t> MyGridCSpace2D::getCellFromPoint(double x0, double x1) const {
//     // Implment your discretization procedure here, such that the point (x0, x1) lies within the returned cell
//     std::size_t cell_x = 0; // x index of cell
//     std::size_t cell_y = 0; // x index of cell
//     return {cell_x, cell_y};
// }

// // Override this method for computing all of the boolean collision values for each cell in the cspace
// std::unique_ptr<amp::GridCSpace2D> MyManipulatorCSConstructor::construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env) {
//     // Create an object of my custom cspace type (e.g. MyGridCSpace2D) and store it in a unique pointer. 
//     // Pass the constructor parameters to std::make_unique()
//     std::unique_ptr<MyGridCSpace2D> cspace_ptr = std::make_unique<MyGridCSpace2D>(m_cells_per_dim, m_cells_per_dim, env.x_min, env.x_max, env.y_min, env.y_max);
//     // In order to use the pointer as a regular GridCSpace2D object, we can just create a reference
//     MyGridCSpace2D& cspace = *cspace_ptr;
//     std::cout << "Constructing C-space for manipulator" << std::endl;
//     // Determine if each cell is in collision or not, and store the values the cspace. This `()` operator comes from DenseArray base class
//     cspace(1, 3) = true;
//     cspace(3, 3) = true;
//     cspace(0, 1) = true;
//     cspace(1, 0) = true;
//     cspace(2, 0) = true;
//     cspace(3, 0) = true;
//     cspace(4, 1) = true;

//     // Returning the object of type std::unique_ptr<MyGridCSpace2D> can automatically cast it to a polymorphic base-class pointer of type std::unique_ptr<amp::GridCSpace2D>.
//     // The reason why this works is not super important for our purposes, but if you are curious, look up polymorphism!
//     return cspace_ptr;
// }

//////////////////////////////////////////////////////////////

// Override this method for computing all of the boolean collision values for each cell in the cspace
std::unique_ptr<amp::GridCSpace2D> MyPointAgentCSConstructor::construct(const amp::Environment2D& env) {
    double safetyMargin = -0.1;
    // Create an object of my custom cspace type (e.g. MyGridCSpace2D) and store it in a unique pointer. 
    // Pass the constructor parameters to std::make_unique()
    std::unique_ptr<MyGridCSpace2D> cspace_ptr = std::make_unique<MyGridCSpace2D>(m_cells_per_dim, m_cells_per_dim, env.x_min, env.x_max, env.y_min, env.y_max);
    // In order to use the pointer as a regular GridCSpace2D object, we can just create a reference
    MyGridCSpace2D& cspace = *cspace_ptr;
    std::cout << "Constructing C-space for point agent" << std::endl;
    // Determine if each cell is in collision or not, and store the values the cspace. This `()` operator comes from DenseArray base class
    for (int i = 0; i < cspace.size().first; i++){
        for (int j = 0; j < cspace.size().second; j++){
            for (const auto& obstacle : env.obstacles){
                auto vertices = obstacle.verticesCCW();
                std::size_t num_vertices = vertices.size();

                // Inflate the obstacle
                std::vector<Eigen::Vector2d> inflatedVertices;
                for (std::size_t k = 0; k < num_vertices; ++k) {
                    Eigen::Vector2d v1 = vertices[k];
                    Eigen::Vector2d v2 = vertices[(k + 1) % num_vertices];
                    Eigen::Vector2d edge = v2 - v1;
                    Eigen::Vector2d normal(-edge.y(), edge.x());
                    normal.normalize();
                    inflatedVertices.push_back(v1 + normal * safetyMargin);
                }

                amp::Polygon inflatedObstacle(inflatedVertices);

                if (isPointInPolygon(cspace.getPointFromCell(i, j), inflatedObstacle)){
                    cspace(i, j) = true;
                    break;
                }
                else{
                    cspace(i, j) = false;
                }
            }
        }
    }

    // Returning the object of type std::unique_ptr<MyGridCSpace2D> can automatically cast it to a polymorphic base-class pointer of type std::unique_ptr<amp::GridCSpace2D>.
    // The reason why this works is not super important for our purposes, but if you are curious, look up polymorphism!
    return cspace_ptr;
}

amp::Path2D MyWaveFrontAlgorithm::planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace, bool isManipulator) {
    // Implement your WaveFront algorithm here
    amp::Path2D path;
    path.waypoints.push_back(q_init);
    
    auto [numCellsX, numCellsY] = grid_cspace.size();
    std::vector<std::vector<int>> waveGrid(numCellsX, std::vector<int>(numCellsY, -1));
    auto [goalX, goalY] = grid_cspace.getCellFromPoint(q_goal[0], q_goal[1]);
    auto [initX, initY] = grid_cspace.getCellFromPoint(q_init[0], q_init[1]);


    for (int i = 0; i < numCellsX; i++){
        for (int j = 0; j < numCellsY; j++){
            if(grid_cspace(i,j) == true) waveGrid[i][j] = -1;
            else waveGrid[i][j] = 0;
        }
    }

    waveGrid[goalX][goalY] = 1;

    bool goalReached = false;
    int waveValue = 2;
    int maxIterations = numCellsX * numCellsY;
    int iterations = 0;
    bool changedThisIteration = false;
    while(!goalReached && iterations < maxIterations){
        changedThisIteration = false;
        for (int i = 0; i < numCellsX; i++){
            for (int j = 0; j < numCellsY; j++){
                if(waveGrid[i][j] == 0){
                    for (int k = -1; k <= 1; k++){
                        for (int l = -1; l <= 1; l++){
                            if(k==0 && l==0) continue;
                            
                            // Handle wraparound
                            int ni = (i + k + numCellsX) % numCellsX;
                            int nj = (j + l + numCellsY) % numCellsY;
                            
                            if(waveGrid[ni][nj] == waveValue - 1){
                                waveGrid[i][j] = waveValue;
                                changedThisIteration = true;
                                if(i == initX && j == initY){
                                    goalReached = true;
                                }
                            }
                        }
                    }   
                }
            }
        }
        waveValue++;
        iterations++;
    }
    if (iterations >= maxIterations) {
        // Handle the case where a path couldn't be found
        std::cout << "Path not found within maximum iterations." << std::endl;
        return path;
    }

    waveValue--;
    int previousWaveX = initX;
    int previousWaveY = initY;
    while(waveValue > 1){
        for (int i = 0; i < numCellsX; i++){
            bool breakFlag = false;
            for (int j = 0; j < numCellsY; j++){
                if(waveGrid[i][j] == waveValue) {
                    for (int k = -1; k <= 1; k++){
                        for (int l = -1; l <= 1; l++){
                            if(k==0 && l==0) continue;

                            // Handle wraparound
                            int ni = (i + k + numCellsX) % numCellsX;
                            int nj = (j + l + numCellsY) % numCellsY;

                            if(ni == previousWaveX && nj == previousWaveY){
                                path.waypoints.push_back(grid_cspace.getPointFromCell(i,j));
                                previousWaveX = i;
                                previousWaveY = j;
                                breakFlag = true;
                                break;
                            }
                        }
                        if(breakFlag) break;
                    }                    
                }
                if(breakFlag) break;
            }
            if(breakFlag) break;
        }
        waveValue--;
        
    }

    // for (int i = 0; i < numCellsX; ++i) {
    //     for (int j = 0; j < numCellsY; ++j) {
    //         std::cout << waveGrid[i][j] << " ";
    //     }
    //     std::cout << std::endl;
    // }


    path.waypoints.push_back(q_goal);
    if (isManipulator) {
        Eigen::Vector2d bounds0 = Eigen::Vector2d(0.0, 0.0);
        Eigen::Vector2d bounds1 = Eigen::Vector2d(2*M_PI, 2*M_PI);
        amp::unwrapWaypoints(path.waypoints, bounds0, bounds1);
    }
    return path;
}

