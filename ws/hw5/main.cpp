#include "AMPCore.h"
#include "hw/HW5.h"
#include "MyGDAlgorithm.h"

using namespace amp;

int main(int argc, char** argv) {
    // Seed the random number generator
    amp::RNG::seed(amp::RNG::randiUnbounded());

    // Test your gradient descent algorithm on a random problem.
    MyGDAlgorithm algo(1.0, 1.0, 1.0, 1.0);
    Path2D path;
    Problem2D prob;
    bool success = HW5::generateAndCheck(algo, path, prob);
    Visualizer::makeFigure(prob, path);

    // Visualize your potential function
    Visualizer::makeFigure(MyPotentialFunction{}, prob, 30);
    // Create a MyPotentialFunction object with the required arguments
    MyPotentialFunction potential_function(prob.obstacles, prob.q_goal, 1.0, 1.0);
    amp::Visualizer::makeFigure(potential_function, prob.x_min, prob.x_max, prob.y_min, prob.y_max, 20);
    
    Visualizer::showFigures();
    
    // Arguments following argv correspond to the constructor arguments of MyGDAlgorithm:
    HW5::grade<MyGDAlgorithm>("stephen.gillet@colorado.edu", argc, argv, 1.0, 1.0, 1.0, 1.0);
    return 0;
}
