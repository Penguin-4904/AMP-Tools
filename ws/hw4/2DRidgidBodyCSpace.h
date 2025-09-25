//
// Created by Katrina Braun on 9/24/25.
//

#pragma once

#include "AMPCore.h"
#include <Eigen/Geometry>

amp::Obstacle2D CSpacePolygon(amp::Polygon robot, amp::Obstacle2D obstacle);

std::vector<amp::Obstacle2D> CSpacePolygonRotate(amp::Polygon robot, amp::Obstacle2D obstacle, std::size_t n);
