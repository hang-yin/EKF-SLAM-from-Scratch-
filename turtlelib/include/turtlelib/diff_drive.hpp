#ifndef DIFFDRIVE_INCLUDE_GUARD_HPP
#define DIFFDRIVE_INCLUDE_GUARD_HPP
/// \file
/// \brief Models kinematics of a differential drive robot

#include "rigid2d.hpp"

namespace turtlelib{
    /// \brief Robot configuration
    struct RobotState{
        /// \brief x position
        double x;
        /// \brief y position
        double y;
        /// \brief heading angle
        double theta;
    };

    /// \brief Wheel angles
    struct WheelAngles{
        /// \brief left wheel angle
        double left;
        /// \brief right wheel angle
        double right;
    };

    /// \brief Wheel velocities
    struct WheelVelocities{
        /// \brief left wheel angle
        double left;
        /// \brief right wheel angle
        double right;
    };

    class DiffDrive{
    private:
        WheelAngles phi;
        WheelVelocities phi_dot;
        RobotState state;
        Twist2D twist;
        // following default parameters defined in nuturtle_description/diff_params.yaml
        double D = 0.08;
        double R = 0.033; 
    
    public: 
        /// \brief Default constructor
        DiffDrive();

        /// \brief Constructor that takes in a robot state, wheel angles, and wheel velocities
        /// \param state robot state
        /// \param phi wheel angles
        /// \param phi_dot wheel velocities
        DiffDrive(RobotState state, WheelAngles phi, WheelVelocities phi_dot);

        /// \brief Set the robot state
        /// \param state robot state
        void setState(RobotState state);

        /// \brief Set the wheel angles
        /// \param phi wheel angles
        void setWheelAngles(WheelAngles phi);

        /// \brief Set the wheel velocities
        /// \param phi_dot wheel velocities
        void setWheelVelocities(WheelVelocities phi_dot);

        /// \brief forward kinematics: wheel angles to robot configuration
        /// \param phi wheel angles
        /// \return robot configuration
        RobotState forwardKinematics(WheelAngles phi);

        /// \brief Gets current twist
        /// \return twist
        Twist2D getTwist(void);

        /// \brief Gets current robot configuration
        /// \return robot configuration
        RobotState getRobotState(void);

        /// \brief inverse kinematics: wheel velocities given twist
        /// \param twist robot twist
        /// \return wheel velocities
        WheelVelocities inverseKinematics(Twist2D twist);

    };
}

#endif
