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

bool check_manipulator_collisions(const amp::LinkManipulator2D& manipulator, const amp::ManipulatorState& state, const std::vector<amp::Obstacle2D>& obstacles) {
    size_t numObstacles = obstacles.size();

    std::vector<Eigen::Vector2d> joints;
    for (int i = 0; i < state.size() + 1; i++){
        joints.push_back(manipulator.getJointLocation(state, i));
    }

    for (int i = 0; i < numObstacles; i++){
        if (collide_chain_object(joints, obstacles[i])){
            return true;
        }
    }
    return false;
}

bool collide_chain_object(const std::vector<Eigen::Vector2d>& joints, const amp::Obstacle2D& obstacle){
    std::vector<Eigen::Vector2d> vertices = obstacle.verticesCCW();
    size_t numVertices = vertices.size();
    size_t numJoints = joints.size();

    for (int i = 0; i < numVertices - 1; i++){
        for (int j = 0; j < joints.size() - 1; j++){
            Eigen::Vector2d vec1 = (vertices[i] - joints[j]).normalized();
            Eigen::Vector2d vec2 = vertices[i + 1] - joints[j];

            Eigen::Vector2d vec3 = (vertices[i] - joints[j + 1]).normalized();
            Eigen::Vector2d vec4 = vertices[i + 1] - joints[j + 1];

            if ((std::signbit(vec1(0) * vec2(1) - vec1(1) * vec2(0)) != std::signbit(vec3(0) * vec4(1) - vec3(1) * vec4(0))) &
                (std::signbit(vec1(0) * vec3(1) - vec1(1) * vec3(0)) != std::signbit(vec2(0) * vec4(1) - vec2(1) * vec4(0)))){
                return true;
            }
        }
    }

    for (int j = 0; j < joints.size() - 1; j++){
        Eigen::Vector2d vec1 = (vertices[numVertices - 1] - joints[j]).normalized();
        Eigen::Vector2d vec2 = vertices[0] - joints[j];

        Eigen::Vector2d vec3 = (vertices[numVertices - 1] - joints[j + 1]).normalized();
        Eigen::Vector2d vec4 = vertices[0] - joints[j + 1];

        if ((std::signbit(vec1(0) * vec2(1) - vec1(1) * vec2(0)) != std::signbit(vec3(0) * vec4(1) - vec3(1) * vec4(0))) &
            (std::signbit(vec1(0) * vec3(1) - vec1(1) * vec3(0)) != std::signbit(vec2(0) * vec4(1) - vec2(1) * vec4(0)))){
            return true;
        }
    }

    Eigen::Vector2d base = joints[0];
    return collide_object(base, obstacle);
}