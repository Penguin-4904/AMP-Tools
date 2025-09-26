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

    double max_angle;

    for (int i = 0; i < n - 1; i++){

        r -= m_link_lengths[i];
        max_angle = (dist * dist - r * r + m_link_lengths[i] * m_link_lengths[i])/(2 * dist * m_link_lengths[i]);
        if (max_angle < -1){
            if (i > 0){
                joint_angles[i] = 0;
            } else {
                joint_angles[i] = atan2(dir[1], dir[0]) + M_PI;
            }
            dist += m_link_lengths[i];
        } else if (max_angle > 1) {
            joint_angles[i] = atan2(dir[1], dir[0]);
            dist -= m_link_lengths[i];
        } else {
            max_angle = acos(max_angle);
            if (i > 0){
                joint_angles[i] = M_PI - max_angle;
            } else {
                joint_angles[i] = atan2(dir[1], dir[0]) - max_angle;
            }
            joint_angles[i + 1] = M_PI - acos((-dist * dist + r * r + m_link_lengths[i] * m_link_lengths[i])/(2 * m_link_lengths[i] * r));
            break;
        }
    }

    return joint_angles;
}