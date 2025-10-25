#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework headers
#include "hw/HW8.h"

// This is file is loaded from the shared/ directory
// Overwrite with your MySamplingBasedPlanners.h and MySamplingBasedPlanners.cpp from hw7
#include "MySamplingBasedPlanners.h" 

class MultiAgentRRTTemplate {
    public:
        void set_n(size_t new_n) {n = new_n;}

        void set_r(double new_r) {r = new_r;}

        void set_eps(double new_eps) {eps= new_eps;}

        void set_p_goal(double new_p_goal) {p_goal = new_p_goal;}

        size_t get_size() {return size;}

    protected:
        size_t n = 7500;
        double r = 0.5;
        double eps = 0.25;
        double p_goal = 0.05;
        size_t size = 0;


};

class Timingfunction {
public:
    virtual std::vector<Eigen::Vector2d> get_locations(const double& time) = 0;

    const std::vector<double>& get_radii() {return radii;}

    virtual ~Timingfunction() {}
protected:
    std::vector<double> radii;
};

class MyCentralPlanner : public amp::CentralizedMultiAgentRRT, public MultiAgentRRTTemplate {
    public:
        virtual amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem) override;
};


class MyDecentralPlanner : public amp::DecentralizedMultiAgentRRT, public MultiAgentRRTTemplate{
    public:
        virtual amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem) override;
};

class DecentralGBRRT : public MyRRT {
    public:
        virtual amp::Path2D plan(const amp::Problem2D& problem) override;

        void set_TimingfunctionPtr(std::shared_ptr<Timingfunction> ptr) {TimingfunctionPtr = ptr;}

        void set_radius(const double r) {radius = r;}

        const std::vector<double> get_dist() {return last_dist_vector;}

    private:
        std::shared_ptr<Timingfunction> TimingfunctionPtr;
        double radius;
        std::vector<double> last_dist_vector;
};

class LerpTimingfunction : public Timingfunction {
    public:
        virtual std::vector<Eigen::Vector2d> get_locations(const double& time) override;

        void add_path(std::vector<Eigen::Vector2d> path, std::vector<double> time, double radius) {
            paths.push_back(path); times.push_back(time); radii.push_back(radius);
        }

    private:
        std::vector<std::vector<Eigen::Vector2d>> paths;
        std::vector<std::vector<double>> times;
};

bool check_decentralmultiagent_collisions_subdivision(const Eigen::Vector2d& q_new, const Eigen::Vector2d& q_near,
                                                 const double& t_new, const double& t_near, const double& radius,
                                                 const std::shared_ptr<Timingfunction> timing_function,
                                                 const std::vector<amp::Obstacle2D>& obstacles, size_t subdivisions);

bool check_multiagent_collisions_subdivision(const std::vector<std::vector<Eigen::Vector2d>>& paths,
                                             const std::vector<std::vector<double>>& times, const std::vector<double>& radii,
                                             const std::vector<amp::Obstacle2D>& obstacles, size_t subdivisions, size_t numAgents);

bool check_disk_collisions(const Eigen::Vector2d& center, const double& radius, const std::vector<amp::Obstacle2D>& obstacles);

bool collide_disk_object(const Eigen::Vector2d& center, const double& radius, const amp::Obstacle2D& obstacle);
