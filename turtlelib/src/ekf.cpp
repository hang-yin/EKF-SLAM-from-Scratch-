#include <armadillo>
#include "turtlelib/ekf.hpp"
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/rigid2d.hpp"
#include <iostream>
#include <cmath>

namespace turtlelib{

    EKF::EKF(int max_landmarks){
        this->max_landmarks = max_landmarks;
        this->state_vector = {0.0, 0.0, 0.0};
    }

    double EKF::get_x(){
        return this->state_vector.at(0);
    }

    double EKF::get_y(){
        return this->state_vector.at(1);
    }

    double EKF::get_theta(){
        return this->state_vector.at(2);
    }

    void EKF::set_obstacles(std::vector<std::pair<double, double>> obstacles){
        // convert vector of pairs of doubles to a vector of doubles
        std::vector<double> obstacles_vector;
        for (int i = 0; i < obstacles.size(); i++){
            obstacles_vector.push_back(obstacles.at(i).first);
            obstacles_vector.push_back(obstacles.at(i).second);
        }
        // add the obstacles to the state vector
        for (int i = 0; i < obstacles_vector.size(); i++){
            this->state_vector.push_back(obstacles_vector.at(i));
        }
    }

    std::vector<std::pair<double, double>> EKF::get_obstacles(){
        // convert vector of doubles to a vector of pairs of doubles
        std::vector<std::pair<double, double>> obstacles;
        for (int i = 3; i < this->state_vector.size(); i += 2){
            std::pair<double, double> obstacle;
            obstacle.first = this->state_vector.at(i);
            obstacle.second = this->state_vector.at(i + 1);
            obstacles.push_back(obstacle);
        }
        return obstacles;
    }
    
}