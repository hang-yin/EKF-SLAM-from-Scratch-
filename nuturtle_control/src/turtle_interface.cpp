#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nuturtlebot_msgs/msg/wheel_commands.hpp>
#include <nuturtlebot_msgs/msg/sensor_data.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"

using namespace std::chrono_literals;

class TurtleControl : public rclcpp::Node
{
public:
  TurtleControl()
  : Node("turtle_control")
  {
    // Declare parameters
    declare_parameter("wheel_radius", 0.033);
    declare_parameter("track_width", 0.160);
    declare_parameter("motor_cmd_max", 265);
    declare_parameter("motor_cmd_per_rad_sec", 0.024);
    declare_parameter("encoder_ticks_per_rad", 0.00153398078);
    declare_parameter("collision_radius", 0.11);

    // Check if required parameters are defined
    if (get_parameter("wheel_radius").get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET ||
      get_parameter("track_width").get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET ||
      get_parameter("motor_cmd_max").get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET ||
      get_parameter("motor_cmd_per_rad_sec").get_type() ==
      rclcpp::ParameterType::PARAMETER_NOT_SET ||
      get_parameter("encoder_ticks_per_rad").get_type() ==
      rclcpp::ParameterType::PARAMETER_NOT_SET ||
      get_parameter("collision_radius").get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET)
    {
      RCLCPP_ERROR(this->get_logger(), "Required parameters not defined");
      rclcpp::shutdown();
    }

    // Initialize parameters
    wheel_radius_ = get_parameter("wheel_radius").as_double();
    track_width_ = get_parameter("track_width").as_double();
    motor_cmd_max_ = get_parameter("motor_cmd_max").as_int();
    motor_cmd_per_rad_sec_ = get_parameter("motor_cmd_per_rad_sec").as_double();
    encoder_ticks_per_rad_ = get_parameter("encoder_ticks_per_rad").as_double();
    collision_radius_ = get_parameter("collision_radius").as_double();

    // Declare publisher to joint_states topic with the message type JointState
    joint_states_pub_ = create_publisher<sensor_msgs::msg::JointState>("red/joint_states", 10);

    // Declare publisher to wheel_commands topic with the message type WheelCommands
    wheel_commands_pub_ = create_publisher<nuturtlebot_msgs::msg::WheelCommands>("/wheel_cmd", 10);

    // Declare subscriber to sensor_data topic with the message type SensorData
    sensor_data_sub_ = create_subscription<nuturtlebot_msgs::msg::SensorData>(
      "/sensor_data",
      10,
      std::bind(
        &TurtleControl::sensor_data_callback,
        this,
        std::placeholders::_1));

    // Declare subscriber to cmd_vel topic with the message type Twist
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel",
      10,
      std::bind(
        &TurtleControl::cmd_vel_callback,
        this,
        std::placeholders::_1));

    // Initialize diff_drive and wheel_velocities
    diff_drive = turtlelib::DiffDrive();
    wheel_velocities = turtlelib::WheelVelocities();
    wheel_velocities.left = 0.0;
    wheel_velocities.right = 0.0;

    // Create timer at frequency of rate
    rate_ = 200.0;
    timer_ = create_wall_timer(1s / rate_, std::bind(&TurtleControl::timer_callback, this));

    joint_state_msg_.name = {"/red/wheel_left_link", "/red/wheel_right_link"};
    joint_state_msg_.position = {0.0, 0.0};
    joint_state_msg_.velocity = {0.0, 0.0};

    wheel_commands_msg_.left_velocity = 0;
    wheel_commands_msg_.right_velocity = 0;

    twist_.w = 0.0;
    twist_.x = 0.0;
    twist_.y = 0.0;
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
  int motor_cmd_max_;
  double motor_cmd_per_rad_sec_;
  double encoder_ticks_per_rad_;
  double collision_radius_;
  double rate_;

// Declare a diff_drive instance
  turtlelib::DiffDrive diff_drive;

// Declare other variables that need to be stored
  turtlelib::WheelVelocities wheel_velocities;

// Create a JointState message
  sensor_msgs::msg::JointState joint_state_msg_;

// Create a WheelCommands message
  nuturtlebot_msgs::msg::WheelCommands wheel_commands_msg_;

// Create a Twist
  turtlelib::Twist2D twist_;

// Initialize timer
  rclcpp::TimerBase::SharedPtr timer_;

// Implement the callback functions
  void sensor_data_callback(const nuturtlebot_msgs::msg::SensorData::SharedPtr msg)
  {
    // Set the position of the joints
    joint_state_msg_.position = {static_cast<double>(msg->left_encoder * encoder_ticks_per_rad_),
      static_cast<double>(msg->right_encoder * encoder_ticks_per_rad_)};
    // Set the velocity of the joints
    joint_state_msg_.velocity = {wheel_velocities.left, wheel_velocities.right};
  }

  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    // Set the linear and angular velocity
    twist_.w = msg->angular.z;
    twist_.x = msg->linear.x;
    twist_.y = msg->linear.y;

    // Calculate inverse kinematics
    wheel_velocities = diff_drive.inverseKinematics(twist_);
    // Set the left and right wheel velocities
    wheel_commands_msg_.left_velocity =
      static_cast<int>(wheel_velocities.left / motor_cmd_per_rad_sec_);
    wheel_commands_msg_.right_velocity =
      static_cast<int>(wheel_velocities.right / motor_cmd_per_rad_sec_);
    // Limit wheel commands to -256 - 256
    if (wheel_commands_msg_.left_velocity > motor_cmd_max_) {
      wheel_commands_msg_.left_velocity = motor_cmd_max_;
    } else if (wheel_commands_msg_.left_velocity < -motor_cmd_max_) {
      wheel_commands_msg_.left_velocity = -motor_cmd_max_;
    }
    if (wheel_commands_msg_.right_velocity > motor_cmd_max_) {
      wheel_commands_msg_.right_velocity = motor_cmd_max_;
    } else if (wheel_commands_msg_.right_velocity < -motor_cmd_max_) {
      wheel_commands_msg_.right_velocity = -motor_cmd_max_;
    }
  }

  void timer_callback()
  {
    // Publish joint state message
    joint_state_msg_.header.stamp = this->now();
    joint_states_pub_->publish(joint_state_msg_);
    // Publish wheel commands message
    wheel_commands_pub_->publish(wheel_commands_msg_);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleControl>());
  rclcpp::shutdown();
  return 0;
}
