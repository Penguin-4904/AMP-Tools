#include "ManipulatorSkeleton.h"


//MyManipulator2D::MyManipulator2D()
//    : LinkManipulator2D({1.0, 1.0}) // Default to a 2-link with all links of 1.0 length
//{}

// Override this method for implementing forward kinematics
Eigen::Vector2d MyManipulator2D::getJointLocation(const amp::ManipulatorState& state, uint32_t joint_index) const {
    Eigen::Vector2d position = m_base_location;
    double angle = 0;
    for (int i = 0; i < joint_index; i++){
        if ((i >= m_link_lengths.size()) || (i >= state.size())) {
            LOG("Joint Index Out of Range");
            break;
        }
        angle += state[i];
        position += (Eigen::Vector2d(cos(angle), sin(angle)) * m_link_lengths[i]);
    }
    return position;
}

// Override this method for implementing inverse kinematics
amp::ManipulatorState MyManipulator2D::getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const {
    // Implement inverse kinematics here

    std::size_t n = nLinks();
    amp::ManipulatorState joint_angles(n);
    joint_angles.setZero();

    Eigen::Vector2d dir = end_effector_location - m_base_location;

    double dist = dir.norm();
    double r = reach();

    if (dist > r){
        LOG("Can not reach desired location");
        return joint_angles;
    }

    double angle;
    double max_angle;
    double last_angle = 0;
    double flip = 1;

    for (int i = 0; i < n - 2; i++){

        dir = end_effector_location - getJointLocation(joint_angles, i);
        dist = dir.norm();

        if (dist >= r){
            angle = 0;
        } else {
            angle = acos(dist/r);
        }

        r -= m_link_lengths[i];

        double max_angle = (dist * dist + m_link_lengths[i] * m_link_lengths[i] - r * r)/(2 * dist * m_link_lengths[i]);

        if (max_angle <= -1) {
            max_angle = M_PI;
        } else if (max_angle >= 1){
            max_angle = 0;
        } else {
            max_angle = acos(max_angle);
        }

        angle = std::min(angle, max_angle);

        joint_angles[i] = atan2(dir[1], dir[0]) + angle * flip - last_angle;
        last_angle = atan2(dir[1], dir[0]) + angle * flip;
        flip *= -1;
    }

    dir = end_effector_location - getJointLocation(joint_angles, n - 2);
    dist = dir.norm();

    r -= m_link_lengths[n - 2];

    angle = acos((dist * dist + m_link_lengths[n - 2] * m_link_lengths[n - 2] - r * r)/(2 * dist * m_link_lengths[n - 2]));

    joint_angles[n - 2] = atan2(dir[1], dir[0]) + angle * flip - last_angle;

    dir = end_effector_location - getJointLocation(joint_angles, n - 1);
    joint_angles[n - 1] = atan2(dir[1], dir[0]) - joint_angles[n - 2] - last_angle;

    return joint_angles;
}