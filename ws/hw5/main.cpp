#include "AMPCore.h"
#include "hw/HW5.h"
#include "MyGDAlgorithm.h"

using namespace amp;

int main(int argc, char** argv) {
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> 2ab1ca1 (finished hw 5)
    amp::Problem2D prob1 = HW5::getWorkspace1();
    amp::Path2D path1;
    MyGDAlgorithm algo(10, 100, 10, 1);
    path1 = algo.plan(prob1);
    Visualizer::makeFigure(prob1, path1);
    MyPotentialFunction potential_function1(prob1.obstacles, prob1.q_goal, 1.0, 1.0);
    amp::Visualizer::makeFigure(potential_function1, prob1.x_min, prob1.x_max, prob1.y_min, prob1.y_max, 20);



<<<<<<< HEAD
    // Seed the random number generator
    amp::Problem2D prob1 = HW5::getWorkspace1();
    amp::Path2D path1;
    MyGDAlgorithm algo(10, 100, 10, 1);
    path1 = algo.plan(prob1);
    Visualizer::makeFigure(prob1, path1);
    MyPotentialFunction potential_function1(prob1.obstacles, prob1.q_goal, 1.0, 1.0);
    amp::Visualizer::makeFigure(potential_function1, prob1.x_min, prob1.x_max, prob1.y_min, prob1.y_max, 20);



=======
>>>>>>> 356823d (got some part of potential function to work, but hits obstacles)
=======
>>>>>>> 2ab1ca1 (finished hw 5)
    // Seed the random number generator
    amp::RNG::seed(amp::RNG::randiUnbounded());

    // Test your gradient descent algorithm on a random problem.
    amp::Problem2D prob;
    amp::Path2D path2;
    bool success2 = HW5::generateAndCheck(algo, path2, prob);
    Visualizer::makeFigure(prob, path2);
<<<<<<< HEAD
    amp::Problem2D prob;
    amp::Path2D path2;
    bool success2 = HW5::generateAndCheck(algo, path2, prob);
    Visualizer::makeFigure(prob, path2);
=======
>>>>>>> 2ab1ca1 (finished hw 5)

    // Visualize your potential function
<<<<<<< HEAD
=======
    Visualizer::makeFigure(MyPotentialFunction{}, prob, 30);
>>>>>>> 356823d (got some part of potential function to work, but hits obstacles)
    // Create a MyPotentialFunction object with the required arguments
    MyPotentialFunction potential_function(prob.obstacles, prob.q_goal, 1.0, 1.0);
    amp::Visualizer::makeFigure(potential_function, prob.x_min, prob.x_max, prob.y_min, prob.y_max, 20);
    
    Visualizer::showFigures();
    
    // Arguments following argv correspond to the constructor arguments of MyGDAlgorithm:
    HW5::grade<MyGDAlgorithm>("stephen.gillet@colorado.edu", argc, argv, 1.0, 1.0, 1.0, 1.0);
    return 0;
}
