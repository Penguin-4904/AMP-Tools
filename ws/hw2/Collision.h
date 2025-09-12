//
// Created by Katrina Braun on 9/12/25.
//

#pragma once

#include "AMPCore.h"

/// @brief checks if point is inside any of the given obstacles.
bool check_collisions(Eigen::Vector2d point, std::vector<amp::Obstacle2D> obstacles);

/// @brief checks if point is inside obstacle. Assumes convex polygon obstacle.
bool collide_object(Eigen::Vector2d point, amp::Obstacle2D obstacle);