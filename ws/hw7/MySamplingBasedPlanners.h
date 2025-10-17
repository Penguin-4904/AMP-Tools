#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework headers
#include "hw/HW7.h"
#include "Collision.h"
#include "MyAStar.h"

class MyPRM : public amp::PRM2D {
    public:

    virtual amp::Path2D plan(const amp::Problem2D& problem) override;

    size_t get_n() {return n;};

    size_t get_r() {return r;};

    void set_n(size_t new_n) {n = new_n;};

    void set_r(double new_r) {r = new_r;};

    void set_path_smoothing(bool smooth) {path_smoothing = smooth;};

    const std::shared_ptr<amp::Graph<double>> get_graphPtr() {return graphPtr;};

    std::map<amp::Node, Eigen::Vector2d> get_nodes() {return nodes;};

    private:
        size_t n = 200;
        double r = 1;
        bool path_smoothing = false;
        std::shared_ptr<amp::Graph<double>> graphPtr = std::make_shared<amp::Graph<double>>();
        std::map<amp::Node, Eigen::Vector2d> nodes;

};

class MyRRT : public amp::GoalBiasRRT2D {
    public:
        virtual amp::Path2D plan(const amp::Problem2D& problem) override;
};

// Coppied from HW6.h
struct LookupSearchHeuristic : public amp::SearchHeuristic {
    /// @brief Get the heuristic value stored in `heuristic_values`.
    /// @param node Node to get the heuristic value h(node) for.
    /// @return Heuristic value
    virtual double operator()(amp::Node node) const override {return heuristic_values.at(node);}

    /// @brief Store the heursitic values for each node in a map
    std::map<amp::Node, double> heuristic_values;
};