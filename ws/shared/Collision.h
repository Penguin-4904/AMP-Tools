//
// Created by Katrina Braun on 9/12/25.
//

#pragma once

#include "AMPCore.h"


/// @brief checks if point is inside any of the given obstacles.
bool check_collisions(const Eigen::Vector2d& point, const std::vector<amp::Obstacle2D>& obstacles);

/// @brief checks if point is inside obstacle. Assumes convex polygon obstacle.
bool collide_object(const Eigen::Vector2d& point, const amp::Obstacle2D& obstacle);

/// @brief checks if manipulator collides with any of the given obstacles for a given state.
bool check_manipulator_collisions(const amp::LinkManipulator2D& manipulator, const amp::ManipulatorState& state, const std::vector<amp::Obstacle2D>& obstacle);

/// @brief checks if the chain of joints/points collides with obstacle
bool collide_chain_object(const std::vector<Eigen::Vector2d>& joints, const amp::Obstacle2D& obstacle);

/// @breief checks if any part of the cell collides with any of the obstacles.
bool check_cell_collisions(const Eigen::Vector2d center, const double width, const double height, const std::vector<amp::Obstacle2D>& obstacles);