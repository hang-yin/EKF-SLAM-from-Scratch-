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
        explicit EKF();

        void set_max_landmarks(int max_landmarks);

        double get_x();

        double get_y();

        double get_theta();

        void predict(Twist2D twist);

        void correct(int obstacle_id, double obstacle_x, double obstacle_y);

        void set_obstacles(std::vector<std::pair<double, double>> obstacles);

        void update_obstacles(std::vector<std::pair<double, double>> obstacles);

        bool has_new_obstacle(double x, double y);

        std::vector<std::pair<double, double>> get_obstacles();
        arma::vec get_obstacles_1();

        void add_landmark(int obstacle_id, double obstacle_x, double obstacle_y);

        std::vector<double> get_euclidean_distances(double obstacle_x, double obstacle_y);
    
    private:
        int max_landmarks;
        arma::vec state_vec;
        arma::vec state_vec_minus;
        arma::vec state_vec_prev;
        arma::mat sigma_mat;
        arma::mat sigma_mat_minus;
        arma::mat sigma_mat_prev;
        RobotState state;
        arma::vec q;
        arma::vec updated_obstacles;
        std::vector<std::pair<double, double>> detected_obstacles;

    };

}

#endif