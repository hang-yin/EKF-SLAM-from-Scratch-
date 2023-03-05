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

        this->updated_obstacles = arma::vec(2*this->max_landmarks, arma::fill::zeros);
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

    void EKF::add_landmark(int obstacle_id, double obstacle_x, double obstacle_y){
        this->state_vec_prev.at(3 + 2 * obstacle_id) = obstacle_x;
        this->state_vec_prev.at(3 + 2 * obstacle_id + 1) = obstacle_y;
    }

    std::vector<double> EKF::get_euclidean_distances(double obstacle_x, double obstacle_y){
        std::vector<double> euclidean_distances;
        for (int i = 0; i < this->max_landmarks; i++){
            if (this->state_vec_prev.at(3 + 2 * i) == 0.0 && this->state_vec_prev.at(3 + 2 * i + 1) == 0.0){
                continue;
            }

            double rad = std::sqrt(std::pow(obstacle_x, 2) + std::pow(obstacle_y, 2));
            double angle = std::atan2(obstacle_y, obstacle_x);
            angle = turtlelib::normalize_angle(angle);

            double obstacle_x_tf = this->state_vec_prev.at(1) + rad * std::cos(this->state_vec_prev.at(0) + angle);
            double obstacle_y_tf = this->state_vec_prev.at(2) + rad * std::sin(this->state_vec_prev.at(0) + angle);

            double distance = std::sqrt(std::pow(obstacle_x_tf - this->state_vec_prev.at(3 + 2 * i), 2) + std::pow(obstacle_y_tf - this->state_vec_prev.at(3 + 2 * i + 1), 2));

            //double x_diff = obstacle_x - this->state_vec_prev.at(3 + 2 * i);
            //double y_diff = obstacle_y - this->state_vec_prev.at(3 + 2 * i + 1);
            //double distance = std::sqrt(std::pow(x_diff, 2) + std::pow(y_diff, 2));
            euclidean_distances.push_back(distance);
        }
        return euclidean_distances;
    }

    void EKF::update_obstacles(std::vector<std::pair<double, double>> obstacles){
        // for each obstacle, check if it is close to any current obstacle
        // if so, continue
        // if not, add the new obstacle
        int idx = 0;
        for (int i = 0; i < int(obstacles.size()); i++){
            double distance = std::sqrt(std::pow(obstacles.at(i).first, 2) + std::pow(obstacles.at(i).second, 2));
            double angle = std::atan2(obstacles.at(i).second, obstacles.at(i).first);
            double x = this->state_vec_prev.at(1)+distance*std::cos(state.theta+angle);
            double y = this->state_vec_prev.at(2)+distance*std::sin(state.theta+angle);

            std::vector<std::pair<double, double>> detected_obstacles_temp = this->detected_obstacles;

            if (detected_obstacles_temp.size()==0){
                this->detected_obstacles.push_back(std::make_pair(x, y));
                this->updated_obstacles.at(idx) = x;
                this->updated_obstacles.at(idx + 1) = y;
                idx += 2;
            }else{
                bool new_obstacle = true;
                for (int j = 0; j < int(detected_obstacles_temp.size()); j++){
                    double x_diff = x - detected_obstacles_temp.at(j).first;
                    double y_diff = y - detected_obstacles_temp.at(j).second;
                    double distance = std::sqrt(std::pow(x_diff, 2) + std::pow(y_diff, 2));
                    if (distance < 0.1){
                        new_obstacle = false;
                        break;
                    }
                }
                if (new_obstacle){
                    this->detected_obstacles.push_back(std::make_pair(x, y));
                    this->updated_obstacles.at(idx) = x;
                    this->updated_obstacles.at(idx + 1) = y;
                    idx += 2;
                }
            }
        }
        arma::vec temp = arma::vec(3, arma::fill::zeros);
        
        temp.at(0) = this->state_vec_prev.at(0);
        temp.at(1) = this->state_vec_prev.at(1);
        temp.at(2) = this->state_vec_prev.at(2);
        
        this->state_vec_prev = arma::join_cols(temp, updated_obstacles);
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

    bool EKF::has_new_obstacle(double x, double y){
        std::vector<std::pair<double, double>> prev_obstacles = this->get_obstacles();
        double distance = std::sqrt(std::pow(x, 2) + std::pow(y, 2));
        double angle = std::atan2(y, x);
        double x_diff = this->state_vec_prev.at(1) + distance * std::cos(this->state_vec_prev.at(0) + angle);
        double y_diff = this->state_vec_prev.at(2) + distance * std::sin(this->state_vec_prev.at(0) + angle);
        for (int i = 0; i < int(prev_obstacles.size()); i++){
            double obs_dist = std::sqrt(std::pow(x_diff - prev_obstacles.at(i).first, 2) + std::pow(y_diff - prev_obstacles.at(i).second, 2));
            if (obs_dist < 0.1){
                return false;
            }
        }
        return true;
    }

    arma::vec EKF::get_obstacles_1(){
        return this->state_vec_prev;
    }

    void EKF::predict(Twist2D twist){
        arma::mat Q = arma::mat(3 + 2 * this->max_landmarks, 3 + 2 * this->max_landmarks, arma::fill::zeros);
        Q.submat(0, 0, 2, 2) = arma::mat(3, 3, arma::fill::eye);

        // update state vector
        arma::vec ut = arma::vec(3+2*this->max_landmarks, arma::fill::zeros);
        if (almost_equal(twist.w, 0.0)){
            ut.at(1) = twist.x*cos(this->state_vec_prev.at(0));
            ut.at(2) = twist.x*sin(this->state_vec_prev.at(0));
        }
        else{
            ut.at(0) = twist.w;
            ut.at(1) = ((-twist.x/twist.w)*sin(this->state_vec_prev.at(0))) + ((twist.x/twist.w)*sin(this->state_vec_prev.at(0) + twist.w));
            ut.at(2) = ((-twist.x/twist.w)*cos(this->state_vec_prev.at(0))) + ((twist.x/twist.w)*cos(this->state_vec_prev.at(0) + twist.w));
        }
        this->state_vec_minus = this->state_vec_prev + ut;

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