#include "rigid2d.hpp"
/// \file
/// \brief Models kinematics of a differential drive robot

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
    
    public: 
        /// \brief Default constructor
        DiffDrive();

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

        /// \brief inverse kinematics: wheel velocities given twist
        /// \param twist robot twist
        /// \return wheel velocities
        WheelVelocities inverseKinematics(Twist2D twist);

    };
}
