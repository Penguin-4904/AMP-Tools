#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW5.h"

class MyGDAlgorithm : public amp::GDAlgorithm {
	public:
		// Consider defining a class constructor to easily tune parameters, for example: 
		MyGDAlgorithm(double d_star, double zetta, double Q_star, double eta, double alpha = 0.1, double eps = 0.25) :
			d_star(d_star),
			zetta(zetta),
			Q_star(Q_star),
			eta(eta),
            alpha(alpha),
            eps(eps) {}

		// Override this method to solve a given problem.
		virtual amp::Path2D plan(const amp::Problem2D& problem) override;

	private:
		double d_star, zetta, Q_star, eta, alpha, eps;
		// Add additional member variables here...
};

class MyPotentialFunction : public amp::PotentialFunction2D {
    public:
        MyPotentialFunction(double d_star, double zetta, double Q_star, double eta, const amp::Problem2D& problem) :
            d_star(d_star),
            zetta(zetta),
            Q_star(Q_star),
            eta(eta),
            problem(problem) {}

        virtual double operator()(const Eigen::Vector2d& q) const override;

		virtual Eigen::Vector2d getGradient(const Eigen::Vector2d& q) const override;

        Eigen::Vector2d getGradient(const Eigen::Vector2d& q, double& dist);

        Eigen::Vector2d closestPoint(const Eigen::Vector2d& q, const amp::Obstacle2D& obstacle, Eigen::Vector2d& centroid) const;

    private:
        double d_star, zetta, Q_star, eta;
        const amp::Problem2D& problem;
};