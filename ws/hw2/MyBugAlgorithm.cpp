#include "MyBugAlgorithm.h"

amp::Path2D MyBugAlgorithm::plan(const amp::Problem2D& problem) {
    return bug1(problem);
}

amp::Path2D MyBugAlgorithm::bug1(const amp::Problem2D& problem){

    amp::Path2D path;
    path.waypoints.push_back(problem.q_init);
    Eigen::Vector2d currentPosition = problem.q_init;
    double stepSize = .25;
    double angleIncrement = 0.180; // ~10 degrees

    int counter1 = 0;
<<<<<<< HEAD
<<<<<<< HEAD
    while ((currentPosition - problem.q_goal).norm() > stepSize && counter1 < 1000){
=======
    while ((currentPosition - problem.q_goal).norm() > stepSize && counter1 < 10000){
>>>>>>> 20d35d2 (bug1 implemented)
=======
    while ((currentPosition - problem.q_goal).norm() > stepSize && counter1 < 1000){
>>>>>>> 206f317 (both bugs working hw1 complete, still get stuck in some situations)
        counter1++;
        Eigen::Vector2d directionVector = (problem.q_goal - currentPosition).normalized();
        currentPosition += directionVector * stepSize;

        std::unique_ptr<amp::Polygon> hitObstacle = inObstacle(problem.obstacles, currentPosition);

        if (hitObstacle) {
            bool circumnavigated = false;
            bool leftHitPoint = false;
            std::vector<Eigen::Vector2d> visitedPoints;

            currentPosition -= directionVector * stepSize;
            
            Eigen::Vector2d hitPoint = currentPosition;

            int counter2 = 0;
<<<<<<< HEAD
<<<<<<< HEAD
            while (circumnavigated != true && counter2 < 1000){
=======
            while (circumnavigated != true && counter2 < 10000){
>>>>>>> 20d35d2 (bug1 implemented)
=======
            while (circumnavigated != true && counter2 < 1000){
>>>>>>> 206f317 (both bugs working hw1 complete, still get stuck in some situations)
                counter2 ++;

                directionVector = tangentVector(hitObstacle->verticesCCW(), currentPosition, true);
                currentPosition += directionVector * stepSize;

                int counter3 = 0;
<<<<<<< HEAD
<<<<<<< HEAD
                while (inObstacle(problem.obstacles, currentPosition) && counter3 < 1000){
=======
                while (inObstacle(problem.obstacles, currentPosition) && counter3 < 10000){
>>>>>>> 20d35d2 (bug1 implemented)
=======
                while (inObstacle(problem.obstacles, currentPosition) && counter3 < 1000){
>>>>>>> 206f317 (both bugs working hw1 complete, still get stuck in some situations)
                    counter3++;
                    hitObstacle = inObstacle(problem.obstacles, currentPosition);

                    currentPosition -= directionVector * stepSize;
                    directionVector = rotateVector(directionVector, angleIncrement);
                    currentPosition += directionVector * stepSize;
                }

                path.waypoints.push_back(currentPosition);
                visitedPoints.push_back(currentPosition);

                if((currentPosition - hitPoint).norm() > stepSize && leftHitPoint == false) leftHitPoint = true;
                else if((currentPosition - hitPoint).norm() <= stepSize * 2 && leftHitPoint == true) {
                    circumnavigated = true;

                    Eigen::Vector2d closestPoint = visitedPoints[0];
                    double minDistance = (visitedPoints[0] - problem.q_goal).norm();
                    int closestIndex = 0;

                    for (int i = 0; i < visitedPoints.size(); ++i) {
                        double distance = (visitedPoints[i] - problem.q_goal).norm();
                        if (distance < minDistance) {
                            minDistance = distance;
                            closestPoint = visitedPoints[i];
                            closestIndex = i;
                        }
                    }

                    bool reachedLeavePoint = false;
                    bool directionCCW = false;
                    if (closestIndex < (visitedPoints.size() / 2)) directionCCW = true;

                    int counter4 = 0;
<<<<<<< HEAD
<<<<<<< HEAD
                    while (reachedLeavePoint == false && counter4 < 1000){
=======
                    while (reachedLeavePoint == false && counter4 < 10000){
>>>>>>> 20d35d2 (bug1 implemented)
=======
                    while (reachedLeavePoint == false && counter4 < 1000){
>>>>>>> 206f317 (both bugs working hw1 complete, still get stuck in some situations)
                        counter4++;

                        directionVector = tangentVector(hitObstacle->verticesCCW(), currentPosition, directionCCW);
                        currentPosition += directionVector * stepSize;
                        
                        int counter5 = 4;
<<<<<<< HEAD
<<<<<<< HEAD
                        while (inObstacle(problem.obstacles, currentPosition) && counter5 < 1000){
=======
                        while (inObstacle(problem.obstacles, currentPosition) && counter5 < 10000){
>>>>>>> 20d35d2 (bug1 implemented)
=======
                        while (inObstacle(problem.obstacles, currentPosition) && counter5 < 1000){
>>>>>>> 206f317 (both bugs working hw1 complete, still get stuck in some situations)
                            counter5++;

                            currentPosition -= directionVector * stepSize;
                            
                            if (directionCCW == false) directionVector = rotateVector(directionVector, -angleIncrement);
                            else directionVector = rotateVector(directionVector, angleIncrement);
                            
                            currentPosition += directionVector * stepSize;
                        }

                        if((currentPosition - closestPoint).norm() <= stepSize){
                            reachedLeavePoint = true;
                        }

                        path.waypoints.push_back(currentPosition);
                    }

                }
            }
        }
        path.waypoints.push_back(currentPosition);
    }
    path.waypoints.push_back(problem.q_goal);

    return path;
}

<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> 206f317 (both bugs working hw1 complete, still get stuck in some situations)
amp::Path2D MyBugAlgorithm::bug2(const amp::Problem2D& problem){

    amp::Path2D path;
    path.waypoints.push_back(problem.q_init);
    Eigen::Vector2d currentPosition = problem.q_init;
    double stepSize = .25;
    double angleIncrement = 0.180; // ~10 degrees

    int counter1 = 0;
    while ((currentPosition - problem.q_goal).norm() > stepSize && counter1 < 1000){
        counter1++;
        Eigen::Vector2d directionVector = (problem.q_goal - currentPosition).normalized();
        currentPosition += directionVector * stepSize;

        std::unique_ptr<amp::Polygon> hitObstacle = inObstacle(problem.obstacles, currentPosition);

        if (hitObstacle) {
            bool circumnavigated = false;
            bool leftHitPoint = false;
            bool hitMline = false;
            std::vector<Eigen::Vector2d> visitedPoints;

            currentPosition -= directionVector * stepSize;
            
            Eigen::Vector2d hitPoint = currentPosition;

            int counter2 = 0;
            while (hitMline != true && counter2 < 1000){
                counter2 ++;

                directionVector = tangentVector(hitObstacle->verticesCCW(), currentPosition, true);
                currentPosition += directionVector * stepSize;

                int counter3 = 0;
                while (inObstacle(problem.obstacles, currentPosition) && counter3 < 1000){
                    counter3++;
                    hitObstacle = inObstacle(problem.obstacles, currentPosition);

                    currentPosition -= directionVector * stepSize;
                    directionVector = rotateVector(directionVector, angleIncrement);
                    currentPosition += directionVector * stepSize;
                }

                path.waypoints.push_back(currentPosition);

                if((currentPosition - hitPoint).norm() > stepSize && leftHitPoint == false) leftHitPoint = true;
                else if(leftHitPoint == true) {
                    hitMline = isOnMLine(currentPosition, problem.q_goal, problem.q_init, stepSize);

                    if(hitMline){
                        bool visitedBefore = false;
                        for (const auto& visitedPoint : visitedPoints){
                            if((visitedPoint - currentPosition).norm() < stepSize){
                                visitedBefore = true;
                                break;
                            }
                        }

                        if(!visitedBefore){
                            visitedPoints.push_back(currentPosition);
                            break;
                        }
                        else{
                            hitMline = false;
                        }
                    }
                }
            }
        }
        path.waypoints.push_back(currentPosition);
    }
    path.waypoints.push_back(problem.q_goal);

    return path;
}

<<<<<<< HEAD
=======
>>>>>>> 20d35d2 (bug1 implemented)
=======
>>>>>>> 206f317 (both bugs working hw1 complete, still get stuck in some situations)
std::unique_ptr<amp::Polygon> MyBugAlgorithm::inObstacle(const std::vector<amp::Polygon>& obstacles, const Eigen::Vector2d& stepPosition) {
    for (const auto& obstacle : obstacles){
        const std::vector<Eigen::Vector2d>& vertices = obstacle.verticesCCW();
        int n = vertices.size();
        int count = 0;
        
        for (int i = 0; i < n; i++) {
            Eigen::Vector2d v1 = vertices[i];
            Eigen::Vector2d v2 = vertices[(i + 1) % n];
            
            // Check if the point lies on a horizontal ray passing through the point
            if (((v1.y() <= stepPosition.y() && stepPosition.y() < v2.y()) || (v2.y() <= stepPosition.y() && stepPosition.y() < v1.y())) &&
                (stepPosition.x() < (v2.x() - v1.x()) * (stepPosition.y() - v1.y()) / (v2.y() - v1.y()) + v1.x())) {
                count++;
            }
        }
        
        if (count % 2 == 1) return std::make_unique<amp::Polygon>(obstacle); 
    }
    return nullptr;
}

Eigen::Vector2d MyBugAlgorithm::getObstacleCenter(const std::vector<Eigen::Vector2d>& vertices) {
    Eigen::Vector2d center(0.0, 0.0);
    
    for (const auto& vertex : vertices) {
        center += vertex;  
    }
    
    center /= vertices.size(); 

    return center;
}

Eigen::Vector2d MyBugAlgorithm::tangentVector(const std::vector<Eigen::Vector2d>& vertices, Eigen::Vector2d currentPosition, bool ccwDirection){
    double minDistance = std::numeric_limits<double>::infinity();
    Eigen::Vector2d closestEdgeStart, closestEdgeEnd;
    
    // A factor controlling how many points per unit length (can tweak this for more/less precision)
    double pointsPerUnitLength = 100.0; 
    
    for (int i = 0; i < vertices.size(); ++i) {
        Eigen::Vector2d v1 = vertices[i];
        Eigen::Vector2d v2 = vertices[(i + 1) % vertices.size()];

        // Calculate the length of the edge
        double edgeLength = (v2 - v1).norm();

        // Number of points depends on the edge length
        int numPoints = std::max(10, static_cast<int>(pointsPerUnitLength * edgeLength)); // Ensure at least 10 points

        for (int j = 1; j < numPoints; ++j) {
            double t = static_cast<double>(j) / numPoints;
            Eigen::Vector2d point = v1 + t * (v2 - v1); // Linearly interpolate point along the edge

            double distance = (currentPosition - point).norm();
            if (distance < minDistance) {
                minDistance = distance;
                closestEdgeStart = v1;
                closestEdgeEnd = v2;
            }
        } 
    }

    if (ccwDirection) 
        return (closestEdgeStart - closestEdgeEnd).normalized();
    else 
        return (closestEdgeEnd - closestEdgeStart).normalized();
}

Eigen::Vector2d MyBugAlgorithm::rotateVector(Eigen::Vector2d vec, double theta) {    
    Eigen::Matrix2d rotationMatrix;    
    rotationMatrix << cos(theta), -sin(theta),          
                      sin(theta),  cos(theta);    
    return rotationMatrix * vec;
}
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> 206f317 (both bugs working hw1 complete, still get stuck in some situations)

bool MyBugAlgorithm::isOnMLine(const Eigen::Vector2d& currentPosition, const Eigen::Vector2d& start, const Eigen::Vector2d& goal, double threshold) {
    Eigen::Vector2d lineVector = goal - start;
    Eigen::Vector2d toCurrent = currentPosition - start;
    
    // Project current position onto the M-line vector
    double projectionLength = toCurrent.dot(lineVector.normalized());
    
    // Get the point on the M-line that corresponds to this projection
    Eigen::Vector2d projectionPoint = start + projectionLength * lineVector.normalized();
    
    // Check if the distance between current position and the projection point is within the threshold
    return (currentPosition - projectionPoint).norm() < threshold;
}
<<<<<<< HEAD
=======
>>>>>>> 20d35d2 (bug1 implemented)
=======
>>>>>>> 206f317 (both bugs working hw1 complete, still get stuck in some situations)
