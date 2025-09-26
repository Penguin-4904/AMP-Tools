#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW4.h"

// Derive the amp::LinkManipulator2D class
class Manipulator2D : public amp::LinkManipulator2D {
    public:

        // using base class constructors
        using LinkManipulator2D::LinkManipulator2D;

        /// @brief finds the location of the specified joint given a manipulator state (Forward Kinematics)
        virtual Eigen::Vector2d getJointLocation(const amp::ManipulatorState& state, uint32_t joint_index) const override;

        /// @brief finds one configuration for an n-link manipulator given a desired end effector location (Inverse Kinematics)
        virtual amp::ManipulatorState getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const override;
};