#include "turtlelib/diff_drive.hpp"
#include "turtlelib/rigid2d.hpp"
#include <stdexcept>

/// \file
/// \brief Models kinematics of a differential drive robot

namespace turtlelib{
    DiffDrive::DiffDrive(){
        phi.left = 0.0;
        phi.right = 0.0;
        phi_dot.left = 0.0;
        phi_dot.right = 0.0;
        state.x = 0.0;
        state.y = 0.0;
        state.theta = 0.0;
        twist = Twist2D();
    }

    DiffDrive::DiffDrive(RobotState state, WheelAngles phi, WheelVelocities phi_dot){
        this->state = state;
        this->phi = phi;
        this->phi_dot = phi_dot;
        this->twist = Twist2D();
    }

    void DiffDrive::setState(RobotState state){
        this->state = state;
    }

    void DiffDrive::setWheelAngles(WheelAngles phi){
        this->phi = phi;
    }

    void DiffDrive::setWheelVelocities(WheelVelocities phi_dot){
        this->phi_dot = phi_dot;
    }

    RobotState DiffDrive::forwardKinematics(WheelAngles phi1){
        // calculate wheel velocities
        WheelVelocities phi_dot1;
        phi_dot1.left = (phi1.left - this->phi.left) / 1.0;
        phi_dot1.right = (phi1.right - this->phi.right) / 1.0;

        // update wheel angles
        this->phi = phi1;

        // calculate twist
        // refer to forward kinematics section of the Kinematics notes
        twist.x = (R / 2.0) * (phi_dot1.left + phi_dot1.right);
        twist.y = 0.0;
        twist.w = (R / (D * 2.0)) * (phi_dot1.right - phi_dot1.left);

        // calculate new state
        RobotState state1;
        Transform2D T_bbprime = integrate_twist(twist);
        Vector2D new_vector;
        new_vector.x = this->state.x;
        new_vector.y = this->state.y;
        Transform2D T_wb = Transform2D(new_vector, state.theta);
        Transform2D T_wbprime = T_wb * T_bbprime;
        state1.x = T_wbprime.translation().x;
        state1.y = T_wbprime.translation().y;
        state1.theta = T_wbprime.rotation();
        this->state = state1;

        return state1;
    }

    Twist2D DiffDrive::getTwist(void){
        return this->twist;
    }

    RobotState DiffDrive::getRobotState(void){
        return this->state;
    }

    WheelVelocities DiffDrive::inverseKinematics(Twist2D twist){
        WheelVelocities phi_dot;
        if (!almost_equal(twist.y, 0.0)){
            throw std::logic_error("This is an invalid twist!");
        }
        else{
            // refer to inverse kinematics section of the Kinematics notes
            phi_dot.left = (twist.x - twist.w * D) / R;
            phi_dot.right = (twist.x + twist.w * D) / R;
        }
        return phi_dot;
    }

}