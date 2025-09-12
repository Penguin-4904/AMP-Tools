//
// Created by Katrina Braun on 9/12/25.
//

#include "Collision.h"

/// @brief checks if point is inside any of the given obstacles.
bool check_collisions(Eigen::Vector2d point, std::vector<amp::Obstacle2D> obstacles) {
    size_t numObstacles = obstacles.size();

    for (int i = 0; i < numObstacles; i++){
        // if point is in an obstacle stop search and return collision as true
        if (collide_object(point, obstacles[i])){
            return true;
        }
    }
    return false;
}

/// @brief checks if point is inside obstacle. Assumes convex polygon obstacle.
bool collide_object(Eigen::Vector2d point, amp::Obstacle2D obstacle){
    std::vector<Eigen::Vector2d>& vertices = obstacle.verticesCCW();
    size_t numVertices = vertices.size();

    for (int i = 0; i < numVertices - 1; i++){
        // check if vectors from point to vertices proceed in CW or CCW direction
        // CW (negative cross product) -> point is not in half plane defined by the vertices
        // Since vertices are defined as CCW -> point outside polygon, return collision as false
        Eigen::Vector2d vec1 = (vertices[i] - point).normalized();
        Eigen::Vector2d vec2 = vertices[i + 1] - point;
        if ((vec1(0) * vec2(1) - vec1(1) * vec2(0)) < -0.0001){ // Add buffer region to prevent validation from indicating a collision due to numerical inaccuracies.
            return false;
        }
    }

    Eigen::Vector2d vec1 = (vertices[numVertices - 1] - point).normalized();
    Eigen::Vector2d vec2 = vertices[0] - point;
    if ((vec1(0) * vec2(1) - vec1(1) * vec2(0)) < -0.0001){
        return false;
    }
    return true;
}