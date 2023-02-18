#include <armadillo>
#include "turtlelib/ekf.hpp"
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/rigid2d.hpp"
#include <iostream>
#include <cmath>

namespace turtlelib{

    EKF::EKF(int max_landmarks){
        this->max_landmarks = max_landmarks;

        // Initialize state vectors
        this->state_vec = arma::vec(3 + 2 * max_landmarks, arma::fill::zeros);
        this->state_vec_minus = arma::vec(3 + 2 * max_landmarks, arma::fill::zeros);
        this->state_vec_prev = arma::vec(3 + 2 * max_landmarks, arma::fill::zeros);

        // Initialize sigma matrices
        this->sigma_mat_minus = arma::mat(3 + 2 * max_landmarks, 3 + 2 * max_landmarks, arma::fill::zeros);
        arma::mat sigma_submat = arma::eye(2*max_landmarks, 2*max_landmarks) * INT_MAX;
        this->sigma_mat_minus.submat(3,3,3+2*max_landmarks-1,3+2*max_landmarks-1) = sigma_submat;
        this->sigma_mat_prev = this->sigma_mat_minus;
        this->sigma_mat = this->sigma_mat_minus;
    }

    double EKF::get_x(){
        return this->state_vec.at(0);
    }

    double EKF::get_y(){
        return this->state_vec.at(1);
    }

    double EKF::get_theta(){
        return this->state_vec.at(2);
    }

    void EKF::set_obstacles(std::vector<std::pair<double, double>> obstacles){
        // convert vector of pairs of doubles to a vector of doubles
        int number_of_obstacles = obstacles.size();
        int considered_obstacles = std::min(number_of_obstacles, this->max_landmarks);

        std::vector<double> obstacles_vector;
        for (int i = 0; i < considered_obstacles; i++){
            obstacles_vector.push_back(obstacles.at(i).first);
            obstacles_vector.push_back(obstacles.at(i).second);
        }
        // add the obstacles to the state vector
        for (int i = 0; i < obstacles_vector.size(); i++){
            this->state_vec.at(i + 3) = obstacles_vector.at(i);
        }
    }

    std::vector<std::pair<double, double>> EKF::get_obstacles(){
        // convert vector of doubles to a vector of pairs of doubles
        std::vector<std::pair<double, double>> obstacles;
        for (int i = 3; i < this->state_vec.size(); i += 2){
            std::pair<double, double> obstacle;
            obstacle.first = this->state_vec.at(i);
            obstacle.second = this->state_vec.at(i + 1);
            obstacles.push_back(obstacle);
        }
        return obstacles;
    }

    void EKF::predict(Twist2D twist){
        arma::mat Q = arma::mat(3 + 2 * this->max_landmarks, 3 + 2 * this->max_landmarks, arma::fill::zeros);
        Q.submat(0, 0, 2, 2) = arma::mat(3, 3, arma::fill::eye);

        // update state vector
        if (almost_equal(twist.w, 0.0)){
            this->state_vec_minus.at(0) = this->state_vec_prev.at(0);
            this->state_vec_minus.at(1) = this->state_vec_prev.at(1) + twist.x * cos(this->state_vec_prev.at(0));
            this->state_vec_minus.at(2) = this->state_vec_prev.at(2) + twist.x * sin(this->state_vec_prev.at(0));
        }
        else{
            this->state_vec_minus.at(0) = this->state_vec_prev.at(0) + twist.w;
            this->state_vec_minus.at(1) = this->state_vec_prev.at(1) -
                                          twist.x/twist.w*sin(this->state_vec_prev.at(0) +
                                          twist.x/twist.w*sin(this->state_vec_prev.at(0)+twist.w));
            this->state_vec_minus.at(2) = this->state_vec_prev.at(2) -
                                          twist.x/twist.w*cos(this->state_vec_prev.at(0) +
                                          twist.x/twist.w*cos(this->state_vec_prev.at(0)+twist.w));
        }

        // get A matrix
        arma::mat A = arma::mat(3 + 2 * this->max_landmarks, 3 + 2 * this->max_landmarks, arma::fill::eye);
        if (almost_equal(twist.w, 0.0)){
            A.at(1, 2) = -twist.x * sin(this->state_vec_prev.at(0));
            A.at(2, 2) = twist.x * cos(this->state_vec_prev.at(0));
        }
        else{
            A.at(0, 2) = -twist.x/twist.w*cos(this->state_vec_prev.at(0) + twist.x/twist.w*cos(this->state_vec_prev.at(0)+twist.w));
            A.at(1, 2) = -twist.x/twist.w*sin(this->state_vec_prev.at(0) + twist.x/twist.w*sin(this->state_vec_prev.at(0)+twist.w));
        }

        // update sigma matrix
        this->sigma_mat_minus = A * this->sigma_mat_prev * A.t() + Q;
    }
    
}