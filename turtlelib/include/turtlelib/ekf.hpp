#ifndef EKF_INCLUDE_GUARD_HPP
#define EKF_INCLUDE_GUARD_HPP
/// \file
/// \brief Calculates Extended Kalman Filter

#include "rigid2d.hpp"
#include "diff_drive.hpp"
#include <armadillo>

namespace turtlelib{
    class EKF{
    public:
        explicit EKF(int max_landmarks);

        double get_x();

        double get_y();

        double get_theta();

        void predict(Twist2D twist);

        void correct(int obstacle_id, double obstacle_x, double obstacle_y);

        void set_obstacles(std::vector<std::pair<double, double>> obstacles);

        std::vector<std::pair<double, double>> get_obstacles();
    
    private:
        int max_landmarks;
        std::vector<double> state_vector;
    };

}

#endif