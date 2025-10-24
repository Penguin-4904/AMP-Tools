#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework headers
#include "hw/HW8.h"

// This is file is loaded from the shared/ directory
// Overwrite with your MySamplingBasedPlanners.h and MySamplingBasedPlanners.cpp from hw7
#include "MySamplingBasedPlanners.h" 


class MyCentralPlanner : public amp::CentralizedMultiAgentRRT {
    public:
        virtual amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem) override;

        size_t n = 75000;
        double p_goal = 0.05;
        double r = 0.5;
        double eps = 0.25;

};


class MyDecentralPlanner : public amp::DecentralizedMultiAgentRRT {
    public:
        virtual amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem) override;
};

class Decentral_Multi_Agent_RRT_SCC : public SamplingCollisionChecker {
    public:
        virtual bool check_collisions(const Eigen::Vector2d& q_new, const Eigen::Vector2d& q_near, const std::vector<amp::Obstacle2D>& obstacles) override;
};

bool check_multiagent_collisions_subdivision(const std::vector<std::vector<Eigen::Vector2d>>& paths,
                                             const std::vector<std::vector<double>>& times, const std::vector<double>& radii,
                                             const std::vector<amp::Obstacle2D>& obstacles, size_t subdivisions, size_t numAgents);

bool check_disk_collisions(const Eigen::Vector2d& center, const double& radius, const std::vector<amp::Obstacle2D>& obstacles);

bool collide_disk_object(const Eigen::Vector2d& center, const double& radius, const amp::Obstacle2D& obstacle);
