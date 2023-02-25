#include <armadillo>
#include "turtlelib/ekf.hpp"
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/rigid2d.hpp"
#include <iostream>
#include <fstream>
#include <cmath>

namespace turtlelib{

    EKF::EKF(){}

    void EKF::set_max_landmarks(int max_landmarks){
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

        this->state = {0.0,0.0,0.0};
    }

    double EKF::get_x(){
        return this->state_vec.at(1);
    }

    double EKF::get_y(){
        return this->state_vec.at(2);
    }

    double EKF::get_theta(){
        return this->state_vec.at(0);
    }

    void EKF::set_obstacles(std::vector<std::pair<double, double>> obstacles){
        // convert vector of pairs of doubles to a vector of doubles
        int number_of_obstacles = obstacles.size();
        int considered_obstacles = std::min(number_of_obstacles, this->max_landmarks);

        double distance = 0.0;
        double angle = 0.0;

        std::vector<double> obstacles_vector;
        for (int i = 0; i < considered_obstacles; i++){
            distance = std::sqrt(std::pow(obstacles.at(i).first, 2) + std::pow(obstacles.at(i).second, 2));
            angle = std::atan2(obstacles.at(i).second, obstacles.at(i).first);
            obstacles_vector.push_back(state.x+distance*std::cos(state.theta+angle));
            obstacles_vector.push_back(state.y+distance*std::sin(state.theta+angle));
        }

        // convert obstacles_vector to an arma::vec
        arma::vec obstacles_arma = arma::vec(obstacles_vector);
        arma::vec temp = arma::vec(3, arma::fill::zeros);
        this->state_vec_prev = arma::join_cols(temp, obstacles_arma);
    }

    std::vector<std::pair<double, double>> EKF::get_obstacles(){
        // convert vector of doubles to a vector of pairs of doubles
        std::vector<std::pair<double, double>> obstacles;
        for (int i = 3; i < int(this->state_vec_prev.size()); i += 2){
            std::pair<double, double> obstacle;
            obstacle.first = this->state_vec_prev.at(i);
            obstacle.second = this->state_vec_prev.at(i + 1);
            obstacles.push_back(obstacle);
        }
        return obstacles;
    }

    arma::vec EKF::get_obstacles_1(){
        return this->state_vec_prev;
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
            this->state_vec_minus.at(1) = this->state_vec_prev.at(1) +
                                          (-twist.x/twist.w)*sin(this->state_vec_prev.at(0)) +
                                          (twist.x/twist.w)*sin(this->state_vec_prev.at(0)+twist.w);
            this->state_vec_minus.at(2) = this->state_vec_prev.at(2) +
                                          (-twist.x/twist.w)*cos(this->state_vec_prev.at(0)) +
                                          (twist.x/twist.w)*cos(this->state_vec_prev.at(0)+twist.w);
        }

        // get A matrix
        arma::mat A = arma::mat(3 + 2 * this->max_landmarks, 3 + 2 * this->max_landmarks, arma::fill::eye);
        if (almost_equal(twist.w, 0.0)){
            A.at(1, 0) += (-twist.x) * sin(this->state_vec_prev.at(0));
            A.at(2, 0) += (twist.x) * cos(this->state_vec_prev.at(0));
        }
        else{
            A.at(1, 0) += (-twist.x/twist.w)*cos(this->state_vec_prev.at(0)) + (twist.x/twist.w)*cos(this->state_vec_prev.at(0)+twist.w);
            A.at(2, 0) += (-twist.x/twist.w)*sin(this->state_vec_prev.at(0)) + (twist.x/twist.w)*sin(this->state_vec_prev.at(0)+twist.w);
        }

        // update sigma matrix
        this->sigma_mat_minus = A * this->sigma_mat_prev * A.t() + Q;
    }

    void EKF::correct(int obstacle_id, double obstacle_x, double obstacle_y){
        double x_diff = state_vec_minus.at(3+2*obstacle_id) - state_vec_minus.at(1);
        double y_diff = state_vec_minus.at(3+2*obstacle_id+1) - state_vec_minus.at(2);
        double distance = sqrt(x_diff*x_diff + y_diff*y_diff);
        double angle = atan2(y_diff, x_diff) - state_vec_minus.at(0);
        
        // get H matrix
        arma::mat H = arma::mat(2, 3 + 2 * this->max_landmarks, arma::fill::zeros);
        H.at(0, 1) = -x_diff/distance;
        H.at(0, 2) = -y_diff/distance;
        H.at(0, 3+2*obstacle_id) = x_diff/distance;
        H.at(0, 3+2*obstacle_id+1) = y_diff/distance;
        H.at(1, 0) = -1;
        H.at(1, 1) = y_diff/(distance*distance);
        H.at(1, 2) = -x_diff/(distance*distance);
        H.at(1, 3+2*obstacle_id) = -y_diff/(distance*distance);
        H.at(1, 3+2*obstacle_id+1) = x_diff/(distance*distance);

        // get z and h
        arma::vec z = {sqrt(pow(obstacle_x, 2) + pow(obstacle_y, 2)), normalize_angle(atan2(obstacle_y, obstacle_x))};
        arma::vec h = {distance, angle};
        h.at(1) = normalize_angle(h.at(1));

        // get K matrix
        arma::mat R = arma::mat(2, 2, arma::fill::eye);
        arma::mat K = this->sigma_mat_minus * H.t() * (H * this->sigma_mat_minus * H.t() + R).i();

        arma::vec diff = z - h;
        diff.at(1) = normalize_angle(diff.at(1));

        // update state vector
        this->state_vec = this->state_vec_minus + K * (diff);

        // update sigma matrix
        this->sigma_mat = (arma::mat(3 + 2 * this->max_landmarks, 3 + 2 * this->max_landmarks, arma::fill::eye) - K * H) * this->sigma_mat_minus;

        // update previous variales
        this->state_vec_prev = this->state_vec;
        this->sigma_mat_prev = this->sigma_mat;
    }
    
}