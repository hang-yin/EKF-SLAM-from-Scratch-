#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nuturtlebot_msgs/msg/wheel_commands.hpp>
#include <nuturtlebot_msgs/msg/sensor_data.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <rigid2d/rigid2d.hpp>
#include <nuturtle_description/diff_params.h>
#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include "nuturtle_description/diff_params.h"

using namespace std::chrono_literals;

class TurtleControl : public rclcpp::Node
{
public: 
    TurtleControl()
    : Node("turtle_control")
    {
        // Declare parameters
        this->declare_parameter("wheel_radius", 0.033);
        this->declare_parameter("track_width", 0.160);
        this->declare_parameter("motor_cmd_max", 265);
        this->declare_parameter("motor_cmd_per_rad_sec", 0.024);
        this->declare_parameter("encoder_ticks_per_rad", 0.00153398078);
        this->declare_parameter("collision_radius", 0.11);

        // Check if required parameters are defined
        if (!get_parameter("wheel_radius") ||
            !get_parameter("track_width") ||
            !get_parameter("motor_cmd_max") ||
            !get_parameter("motor_cmd_per_rad_sec") ||
            !get_parameter("encoder_ticks_per_rad") ||
            !get_parameter("collision_radius"))
        {
            RCLCPP_ERROR(this->get_logger(), "Required parameters not defined");
            rclcpp::shutdown();
        }

        // Initialize parameters
        wheel_radius_ = this->get_parameter("wheel_radius").as_double();
        track_width_ = this->get_parameter("track_width").as_double();
        motor_cmd_max_ = this->get_parameter("motor_cmd_max").as_int();
        motor_cmd_per_rad_sec_ = this->get_parameter("motor_cmd_per_rad_sec").as_double();
        encoder_ticks_per_rad_ = this->get_parameter("encoder_ticks_per_rad").as_double();
        collision_radius_ = this->get_parameter("collision_radius").as_double();

        // Declare publisher to joint_states topic with the message type JointState
        joint_states_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("~/joint_states", 10);
        // Declare publisher to wheel_commands topic with the message type WheelCommands
        wheel_commands_pub_ = this->create_publisher<nuturtlebot_msgs::msg::WheelCommands>("~/wheel_commands", 10);

        // Declare subscriber to sensor_data topic with the message type SensorData
        sensor_data_sub_ = this->create_subscription<nuturtlebot_msgs::msg::SensorData>("~/sensor_data",
                                                                                        10,
                                                                                        std::bind(&TurtleControl::sensor_data_callback,
                                                                                                  this,
                                                                                                  std::placeholders::_1));
        // Declare subscriber to cmd_vel topic with the message type Twist
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("~/cmd_vel",
                                                                            10,
                                                                            std::bind(&TurtleControl::cmd_vel_callback,
                                                                                      this,
                                                                                      std::placeholders::_1));
        
    }
private:
    // Declare publisher and subscriber objects
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
    rclcpp::Publisher<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_commands_pub_;
    rclcpp::Subscription<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_data_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    // Declare parameters
    double wheel_radius_;
    double track_width_;
    double motor_cmd_max_;
    double motor_cmd_per_rad_sec_;
    double encoder_ticks_per_rad_;
    double collision_radius_;

    // Declare a diff_drive instance
    turtlelib::DiffDrive diff_drive;

    // Implement the callback functions
    void sensor_data_callback(const nuturtlebot_msgs::msg::SensorData::SharedPtr msg)
    {
        // Create a JointState message
        sensor_msgs::msg::JointState joint_state_msg;
        // Set the header
        joint_state_msg.header.stamp = this->now();
        joint_state_msg.header.frame_id = "base_link";
        // Set the name of the joints
        joint_state_msg.name = {"left_wheel_joint", "right_wheel_joint"};
        // Set the position of the joints
        joint_state_msg.position = {msg->left_encoder, msg->right_encoder};
        // Set the velocity of the joints
        joint_state_msg.velocity = {msg->left_velocity, msg->right_velocity};
        // Publish the message
        joint_states_pub_->publish(joint_state_msg);
    }

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Create a WheelCommands message
        nuturtlebot_msgs::msg::WheelCommands wheel_commands_msg;
        // Initialize twist
        turtlelib::rigid2d::Twist2D twist;
        // Set the linear and angular velocity
        twist.w = msg->angular.z;
        twist.x = msg->linear.x;
        twist.y = msg->linear.y;
        // Calculate inverse kinematics
        turtlelib::WheelVelocities wheel_velocities = diff_drive.inverseKinematics(twist);
        // Set the left and right wheel velocities
        wheel_commands_msg.left_velocity = wheel_velocities.left;
        // Publish the message
        wheel_commands_pub_->publish(wheel_commands_msg);
    }


};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtleControl>());
    rclcpp::shutdown();
    return 0;
}