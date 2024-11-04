#pragma once

#include "AMPCore.h"
#include "hw/HW9.h"

class MyKinoRRT : public amp::KinodynamicRRT {
public:
    // Override the pure virtual plan method
    virtual amp::KinoPath plan(const amp::KinodynamicProblem2D& problem, amp::DynamicAgent& agent) override;
};


class MySingleIntegrator : public amp::DynamicAgent {
    public:
        virtual void propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) override;
};

class MyFirstOrderUnicycle : public amp::DynamicAgent {
    public:
        virtual void propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) override {};
};

class MySecondOrderUnicycle : public amp::DynamicAgent {
    public:
        virtual void propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) override {};
};

class MySimpleCar : public amp::DynamicAgent {
    public:
        virtual void propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) override {};
};
bool isWithinGoal(const Eigen::VectorXd& state, const std::vector<std::pair<double, double>>& goalBounds);
// Forward declaration if calculateControlAndDuration is defined later or in another file
std::pair<Eigen::VectorXd, double> calculateControlAndDuration(
    const Eigen::VectorXd& start, const Eigen::VectorXd& end, const amp::KinodynamicProblem2D& problem);
