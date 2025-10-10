//
// Created by Katrina Braun on 9/12/25.
//

#include "Collision.h"

/// @brief checks if point is inside any of the given obstacles.
bool check_collisions(const Eigen::Vector2d& point, const std::vector<amp::Obstacle2D>& obstacles) {
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
bool collide_object(const Eigen::Vector2d& point, const amp::Obstacle2D& obstacle){
    std::vector<Eigen::Vector2d> vertices = obstacle.verticesCCW();
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

/// @brief checks if manipulator collides with any of the given obstacles for a given state.
bool check_manipulator_collisions(const amp::LinkManipulator2D& manipulator, const amp::ManipulatorState& state, const std::vector<amp::Obstacle2D>& obstacles) {
    size_t numObstacles = obstacles.size();

    // Get joint positions from state to feed to individual object collision checker
    std::vector<Eigen::Vector2d> joints;
    for (int i = 0; i < state.size() + 1; i++){
        joints.push_back(manipulator.getJointLocation(state, i));
    }

    for (int i = 0; i < numObstacles; i++){
        if (collide_chain_object(joints, obstacles[i])){ // Check if joint chain collides with current object
            return true;
        }
    }
    return false;
}

/// @brief checks if the chain of joints/points collides with obstacle
bool collide_chain_object(const std::vector<Eigen::Vector2d>& joints, const amp::Obstacle2D& obstacle){
    std::vector<Eigen::Vector2d> vertices = obstacle.verticesCCW();
    size_t numVertices = vertices.size();
    size_t numJoints = joints.size();

    for (int i = 0; i < numVertices - 1; i++){
        for (int j = 0; j < numJoints - 1; j++){
            Eigen::Vector2d vec1 = (vertices[i] - joints[numJoints - j - 1]).normalized();
            Eigen::Vector2d vec2 = vertices[i + 1] - joints[numJoints - j - 1];

            Eigen::Vector2d vec3 = (vertices[i] - joints[numJoints - j - 2]).normalized();
            Eigen::Vector2d vec4 = vertices[i + 1] - joints[numJoints - j - 2];

            if ((std::signbit(vec1(0) * vec2(1) - vec1(1) * vec2(0)) != std::signbit(vec3(0) * vec4(1) - vec3(1) * vec4(0))) &&
                (std::signbit(vec1(0) * vec3(1) - vec1(1) * vec3(0)) != std::signbit(vec2(0) * vec4(1) - vec2(1) * vec4(0))) &&
                (vec1(0) * vec2(1) - vec1(1) * vec2(0) != 0) && (vec1(0) * vec3(1) - vec1(1) * vec3(0) != 0) &&
                (vec3(0) * vec4(1) - vec3(1) * vec4(0) != 0) && (vec2(0) * vec4(1) - vec2(1) * vec4(0) != 0)){
                return true;
            }
        }
    }

    for (int j = 0; j < joints.size() - 1; j++){
        Eigen::Vector2d vec1 = (vertices[numVertices - 1] - joints[numJoints - j - 1]).normalized();
        Eigen::Vector2d vec2 = vertices[0] - joints[numJoints - j - 1];

        Eigen::Vector2d vec3 = (vertices[numVertices - 1] - joints[numJoints - j - 2]).normalized();
        Eigen::Vector2d vec4 = vertices[0] - joints[numJoints - j - 2];

        if ((std::signbit(vec1(0) * vec2(1) - vec1(1) * vec2(0)) != std::signbit(vec3(0) * vec4(1) - vec3(1) * vec4(0))) &&
            (std::signbit(vec1(0) * vec3(1) - vec1(1) * vec3(0)) != std::signbit(vec2(0) * vec4(1) - vec2(1) * vec4(0))) &&
            (vec1(0) * vec2(1) - vec1(1) * vec2(0) != 0) && (vec1(0) * vec3(1) - vec1(1) * vec3(0) != 0) &&
            (vec3(0) * vec4(1) - vec3(1) * vec4(0) != 0) && (vec2(0) * vec4(1) - vec2(1) * vec4(0) != 0)){
            return true;
        }
    }

    return false;
    // return collide_object(joints[0], obstacle);
}

/// @breif checks if any part of the cell collides with any obstacle given the center of the cell and the width and height and a vector of obstacles
bool check_cell_collisions(const Eigen::Vector2d center, const double width, const double height, const std::vector<amp::Obstacle2D>& obstacles){

    if (check_collisions(center, obstacles)){
        return true;
    }

    std::vector<Eigen::Vector2d> chain = {center + Eigen::Vector2d(-width/2, -height/2),
                                          center + Eigen::Vector2d(width/2, -height/2),
                                          center + Eigen::Vector2d(width/2, height/2),
                                          center + Eigen::Vector2d(-width/2, height/2),
                                          center + Eigen::Vector2d(-width/2, -height/2)
                                          };

    for (int i = 0; i < obstacles.size(); i++){
        if (collide_chain_object(chain, obstacles[i])){ // Check if joint chain collides with current object
            return true;
        }
    }
    return false;
}