#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework headers
#include "hw/HW8.h"

// This is file is loaded from the shared/ directory
// Overwrite with your MySamplingBasedPlanners.h and MySamplingBasedPlanners.cpp from hw7
#include "MySamplingBasedPlanners.h" 

/// @brief template class for centralized and decentralized planner
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

/// @brief virtual timing function class
class Timingfunction {
public:
    virtual Eigen::VectorXd get_locations(const double& time) = 0;

    const std::vector<double>& get_radii() {return radii;}

    size_t get_num_paths() {return radii.size();}

    virtual size_t get_size() = 0;

    virtual ~Timingfunction() {}
protected:
    std::vector<double> radii;
};

/// @breif Centralized Multi Agent Planner Class
class MyCentralPlanner : public amp::CentralizedMultiAgentRRT, public MultiAgentRRTTemplate {
    public:
        /// @breif Plan multi agent paths with centralized planner
        virtual amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem) override;

        void set_r_scaling(bool setting) {scale_r = setting;}

    private:
        bool scale_r = false;
};

/// @breif Decentralized Multi Agent Planner Class
class MyDecentralPlanner : public amp::DecentralizedMultiAgentRRT, public MultiAgentRRTTemplate{
    public:
        /// @breif Plan multi agent paths with decentralized planner
        virtual amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem) override;

        void set_replan(bool setting) {replan = setting;}

    private:
        bool replan = false;
};

///  @brief virtual class used by decentralized rrt to store already created paths.
class PathBasedTimingfunction : public Timingfunction{
    public:

        /// @breif adds a path to the stored path list.
        void add_path(std::vector<Eigen::Vector2d> path, double radius) {

            size_t iters = std::max(path.size(), paths.size());
            for (size_t i = 0; i < iters; i++){
                Eigen::Vector2d path_point;
                Eigen::VectorXd paths_point;
                // LOG("add Path");
                if (i >= path.size()){
                    path_point = path.back();
                } else {
                    path_point = path[i];

                }

                if (i >= paths.size()){
                    if (paths.size() == 0) {
                        paths_point = Eigen::VectorXd(2);
                    } else {
                        paths_point = paths.back();
                    }
                    paths_point(Eigen::lastN(2)) = path_point;
                    paths.push_back(paths_point);
                } else {
                    paths_point = paths[i];
                    paths_point.conservativeResize(paths_point.size() + 2);
                    paths_point(Eigen::lastN(2)) = path_point;
                    paths[i] = paths_point;
                }
            }
            radii.push_back(radius);
        }

        virtual size_t get_size() override {return paths.size();}

    protected:
        std::vector<Eigen::VectorXd> paths;
};

/// @breif timing function that returns the stored paths at discrete points
class DiscreteTimingfunction : public PathBasedTimingfunction {
    public:
        /// @breif returns the points on all stored paths at the requested index
        virtual Eigen::VectorXd get_locations(const double& time) override;
};

/// @brief RRT used by decentralized RRT to plan paths uses timing function to find past paths.
class DecentralGBRRT : public MyRRT {
    public:
        /// @brief plan method for custom RRT used for decentralized planing
        virtual amp::Path2D plan(const amp::Problem2D& problem) override;

        void set_TimingfunctionPtr(std::shared_ptr<PathBasedTimingfunction> ptr) {TimingfunctionPtr = ptr;}

        void set_radius(const double r) {radius = r;}

        const std::vector<double> get_dist() {return last_dist_vector;}

    private:
        std::shared_ptr<PathBasedTimingfunction> TimingfunctionPtr;
        double radius;
        std::vector<double> last_dist_vector;
};

/* UNUSED Functions

class LerpTimingfunction : public PathBasedTimingfunction {
    public:
        virtual Eigen::VectorXd get_locations(const double& time) override;
};

bool check_decentralmultiagent_collisions_subdivision(const Eigen::Vector2d& q_new, const Eigen::Vector2d& q_near,
                                                      const double& t_new, const double& radius,
                                                      const std::shared_ptr<Timingfunction> timing_function,
                                                      const std::vector<amp::Obstacle2D>& obstacles, size_t subdivisions);

bool check_multiagent_collisions_subdivision(const Eigen::VectorXd q_new, const Eigen::VectorXd q_near, const std::vector<double>& radii,
                                             const std::vector<amp::Obstacle2D>& obstacles, const size_t subdivisions);

bool check_disk_collisions(const Eigen::Vector2d& center, const double& radius, const std::vector<amp::Obstacle2D>& obstacles);

bool collide_disk_object(const Eigen::Vector2d& center, const double& radius, const amp::Obstacle2D& obstacle);
*/