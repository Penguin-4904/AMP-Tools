//
// Created by Katrina Braun on 9/24/25.
//

#include "2DRidgidBodyCSpace.h"

amp::Obstacle2D CSpacePolygon(amp::Polygon robot, amp::Obstacle2D obstacle){
    std::vector<Eigen::Vector2d> r_vertices = robot.verticesCCW();
    std::size_t r_start = 0;
    for (int i = 0; i < r_vertices.size(); i++){
        r_vertices[i] = -r_vertices[i];
        if (r_vertices[i][1] < r_vertices[r_start][1]){
            r_start = i;
        }
        if ((r_vertices[i][1] == r_vertices[r_start][1]) & (r_vertices[i][0] < r_vertices[r_start][0])){
            r_start = i;
        }
    }
    std::rotate(r_vertices.begin(), r_vertices.begin() + r_start, r_vertices.end());

    std::vector<Eigen::Vector2d> o_vertices = obstacle.verticesCCW();
    std::size_t o_start = 0;
    for (int i = 0; i < o_vertices.size(); i++){
        if (o_vertices[i][1] < o_vertices[o_start][1]){
            o_start = i;
        }
        if ((o_vertices[i][1] == o_vertices[o_start][1]) & (o_vertices[i][0] < o_vertices[r_start][0])){
            o_start = i;
        }
    }
    std::rotate(o_vertices.begin(), o_vertices.begin() + o_start, o_vertices.end());

    r_vertices.push_back(r_vertices[0]);
    o_vertices.push_back(o_vertices[0]);
    std::size_t i = 0;
    std::size_t j = 0;

    std::vector<Eigen::Vector2d> out_vertices;

    while ((i + 1 < r_vertices.size()) || (j + 1 < o_vertices.size())){
        out_vertices.push_back(r_vertices[i] + o_vertices[j]);
        if (i + 1 >= r_vertices.size()){
            j++;
        } else if (j + 1 >= o_vertices.size()){
            i++;
        } else {
            double r_angle = atan2(r_vertices[i + 1][1] - r_vertices[i][1], r_vertices[i + 1][0] - r_vertices[i][0]);
            if (r_angle < 0) {
                r_angle += 2 * M_PI;
            }
            double o_angle = atan2(o_vertices[j + 1][1] - o_vertices[j][1], o_vertices[j + 1][0] - o_vertices[j][0]);
            if (o_angle < 0) {
                o_angle += 2 * M_PI;
            }
            if (r_angle < o_angle) {
                i++;
            } else if (r_angle > o_angle) {
                j++;
            } else {
                i++;
                j++;
            }
        }
    }
    amp::Obstacle2D out_obstacle(out_vertices);
    return out_obstacle;
}

std::vector<amp::Obstacle2D> CSpacePolygonRotate(amp::Polygon robot, amp::Obstacle2D obstacle, std::size_t n){
    std::vector<amp::Obstacle2D> obstacles;
    Eigen::Rotation2D<double> rot(2 * M_PI / n);
    std::vector<Eigen::Vector2d>& r_vertices = robot.verticesCCW();
    for (int i = 0; i < n; i++){
        obstacles.push_back(CSpacePolygon(robot, obstacle));
        for (int j = 0; j < r_vertices.size(); j++){
            r_vertices[j] = rot*r_vertices[j];
        }
    }
    return obstacles;
}
