#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW5.h"

class MyGDAlgorithm : public amp::GDAlgorithm {
	public:
		// Consider defining a class constructor to easily tune parameters, for example: 
		MyGDAlgorithm(double d_star, double zetta, double Q_star, double eta) :
			d_star(d_star),
			zetta(zetta),
			Q_star(Q_star),
			eta(eta) {}

		// Override this method to solve a given problem.
		virtual amp::Path2D plan(const amp::Problem2D& problem) override;
	private:
		double d_star, zetta, Q_star, eta;
		// Add additional member variables here...
};

class MyPotentialFunction : public amp::PotentialFunction2D {
    public:
        MyPotentialFunction(const std::vector<amp::Polygon>& obstacles, const Eigen::Vector2d& goal, double zetta, double Q_star)
            : obstacles(obstacles), goal(goal), zetta(zetta), Q_star(Q_star) {}

        // Returns the potential function value (height) for a given 2D point.
        virtual double operator()(const Eigen::Vector2d& q) const override;

    private:
        std::vector<amp::Polygon> obstacles;
        Eigen::Vector2d goal;
        double zetta;
        double Q_star;
};
