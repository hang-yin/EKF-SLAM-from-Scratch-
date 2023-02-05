#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_srvs/srv/emptry.hpp>
#include "nuturtle_control/Control.h"

class Circle : public rclcpp::Node
{

public: 
    Circle()
    : Node("circle")
    {
        angular_velocity_ = 0.0;
        linear_velocity_ = 0.0;
        radius_ = 0.0;

        // Declare cmd_vel publisher
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("~/cmd_vel", 10);

        // Declare control service
        control_srv_ = this->create_service<nuturtle_control::Control>("~/control",
                                                                       std::bind(&Circle::control_callback,
                                                                                 this,
                                                                                 std::placeholders::_1,
                                                                                 std::placeholders::_2));
        
        // Declare reverse service
        reverse_srv_ = this->create_service<std_srvs::srv::Empty>("~/reverse",
                                                                  std::bind(&Circle::reverse_callback,
                                                                            this,
                                                                            std::placeholders::_1,
                                                                            std::placeholders::_2));
        
        // Declare stop service
        stop_srv_ = this->create_service<std_srvs::srv::Empty>("~/stop",
                                                               std::bind(&Circle::stop_callback,
                                                                         this,
                                                                         std::placeholders::_1,
                                                                         std::placeholders::_2));
        
        // Create a timer to publish cmd_vel
        cmd_vel_timer_ = this->create_wall_timer(100ms, std::bind(&Circle::timer_callback, this));
    }

private:
    double angular_velocity_;
    double linear_velocity_;
    double radius_;

    // Implement control service
    void control_callback(const nuturtle_control::Control::Request::SharedPtr request,
                          const nuturtle_control::Control::Response::SharedPtr response)
    {
        angular_velocity_ = request->velocity;
        radius_ = request->radius;
        linear_velocity_ = angular_velocity_ * radius_;
        return true;
    }

    // Implement reverse service
    void reverse_callback(const std_srvs::srv::Empty::Request::SharedPtr request,
                          const std_srvs::srv::Empty::Response::SharedPtr response)
    {
        angular_velocity_ = -angular_velocity_;
        linear_velocity_ = -linear_velocity_;
        return true;
    }

    // Implement stop service
    void stop_callback(const std_srvs::srv::Empty::Request::SharedPtr request,
                       const std_srvs::srv::Empty::Response::SharedPtr response)
    {
        angular_velocity_ = 0.0;
        linear_velocity_ = 0.0;
        return true;
    }

    // Implement cmd_vel timer
    void timer_callback()
    {
        geometry_msgs::msg::Twist msg;
        msg.linear.x = linear_velocity_;
        msg.angular.z = angular_velocity_;
        cmd_vel_pub_->publish(msg);
    }
};