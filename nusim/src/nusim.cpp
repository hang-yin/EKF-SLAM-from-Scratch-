#include "visualization_msgs/msg/marker_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nusim/srv/teleport.hpp"
#include <nuturtlebot_msgs/msg/wheel_commands.hpp>
#include <nuturtlebot_msgs/msg/sensor_data.hpp>
#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/path.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <random>

using namespace std::chrono_literals;

class NuSimNode : public rclcpp::Node
{
public:
  NuSimNode()
      : Node("nusim")
  {
    // Declare parameters
    declare_parameter("rate", 200.0);
    declare_parameter("x0", 0.0);
    declare_parameter("y0", 0.0);
    declare_parameter("theta0", 0.0);
    declare_parameter("obstacles_x", std::vector<double>());
    declare_parameter("obstacles_y", std::vector<double>());
    declare_parameter("obstacles_r", 0.0);
    declare_parameter("encoder_ticks_per_rad", 0.00153398078);
    declare_parameter("motor_cmd_per_rad_sec", 0.024);
    declare_parameter("input_noise", 0.0);
    declare_parameter("slip_fraction", 0.0);
    declare_parameter("basic_sensor_variance", 0.01);
    declare_parameter("max_range", 10.0);
    declare_parameter("collision_radius", 0.11);
    declare_parameter("min_lidar_range", 0.16);
    declare_parameter("max_lidar_range", 8.0);
    declare_parameter("angle_increment", 0.0174532924);
    declare_parameter("number_of_samples", 360);
    declare_parameter("resolution", 0.01);
    declare_parameter("noise_level", 0.01);
    declare_parameter("angle_min", 0.0);
    declare_parameter("angle_max", 6.283185307);

    // Get input_noise, slip_fraction, basic sensor variance, and max range
    input_noise_ = get_parameter("input_noise").as_double();
    slip_fraction_ = get_parameter("slip_fraction").as_double();
    basic_sensor_variance_ = get_parameter("basic_sensor_variance").as_double();
    max_range_ = get_parameter("max_range").as_double();
    collision_radius_ = get_parameter("collision_radius").as_double();
    min_lidar_range_ = get_parameter("min_lidar_range").as_double();
    max_lidar_range_ = get_parameter("max_lidar_range").as_double();
    angle_increment_ = get_parameter("angle_increment").as_double();
    number_of_samples_ = get_parameter("number_of_samples").as_int();
    resolution_ = get_parameter("resolution").as_double();
    noise_level_ = get_parameter("noise_level").as_double();
    angle_min_ = get_parameter("angle_min").as_double();
    angle_max_ = get_parameter("angle_max").as_double();

    // Check obstacles input, if length of vectors are not equal, exit node
    const auto obstacles_x = get_parameter("obstacles_x").as_double_array();
    const auto obstacles_y = get_parameter("obstacles_y").as_double_array();
    if (obstacles_x.size() != obstacles_y.size())
    {
      RCLCPP_ERROR(this->get_logger(), "Length of obstacles vectors are not equal");
      rclcpp::shutdown();
    }

    auto obstacles_r = get_parameter("obstacles_r").as_double();
    // Check obstacles input, if radius is negative, exit node
    if (obstacles_r < 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Radius of obstacles cannot be negative");
      rclcpp::shutdown();
    }

    // Create publisher for obstacles
    obstacles_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/obstacles", 10);

    // Initialize marker array
    obstacles_x_ = obstacles_x;
    obstacles_y_ = obstacles_y;
    obstacles_r_ = obstacles_r;
    marker_array_.markers.resize(obstacles_x.size());
    for (size_t i = 0; i < obstacles_x.size(); i++)
    {
      marker_array_.markers[i].header.frame_id = "nusim/world";
      marker_array_.markers[i].header.stamp = get_clock()->now();
      marker_array_.markers[i].ns = "obstacles";
      marker_array_.markers[i].id = i;
      marker_array_.markers[i].type = visualization_msgs::msg::Marker::CYLINDER;
      marker_array_.markers[i].action = visualization_msgs::msg::Marker::ADD;
      marker_array_.markers[i].pose.position.x = obstacles_x.at(i);
      marker_array_.markers[i].pose.position.y = obstacles_y.at(i);
      marker_array_.markers[i].pose.position.z = 0.125;
      marker_array_.markers[i].pose.orientation.x = 0.0;
      marker_array_.markers[i].pose.orientation.y = 0.0;
      marker_array_.markers[i].pose.orientation.z = 0.0;
      marker_array_.markers[i].pose.orientation.w = 1.0;
      marker_array_.markers[i].color.a = 1.0;
      marker_array_.markers[i].color.r = 1.0;
      marker_array_.markers[i].color.g = 0.0;
      marker_array_.markers[i].color.b = 0.0;
      marker_array_.markers[i].scale.x = 2 * obstacles_r;
      marker_array_.markers[i].scale.y = 2 * obstacles_r;
      marker_array_.markers[i].scale.z = 0.25;
    }

    // Initialize timestep to zero
    timestep_ = 0;

    // Create publisher for timestep
    timestep_pub_ = create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);

    // Set rate
    rate_ = get_parameter("rate").as_double();
    if (rate_ <= 0)
    {
      rate_ = 200.0;
    }

    // Create timer at frequency of rate
    timer_ = create_wall_timer(1s / rate_, std::bind(&NuSimNode::timer_callback, this));

    // Create reset service of empty type
    reset_srv_ = create_service<std_srvs::srv::Empty>(
        "~/reset",
        std::bind(
            &NuSimNode::reset_callback,
            this,
            std::placeholders::_1,
            std::placeholders::_2));

    // Create teleport service of Teleport type
    teleport_srv_ = create_service<nusim::srv::Teleport>(
        "~/teleport",
        std::bind(
            &NuSimNode::teleport_callback,
            this,
            std::placeholders::_1,
            std::placeholders::_2));

    // Set initial pose
    x0_ = get_parameter("x0").as_double();
    y0_ = get_parameter("y0").as_double();
    theta0_ = get_parameter("theta0").as_double();
    q0_.setRPY(0, 0, theta0_);

    // Create transform broadcaster
    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Create transform broadcaster from the frame "nusim/world" to the frame "red/base_footprint"
    transformStamped_.header.frame_id = "nusim/world";
    transformStamped_.child_frame_id = "red/base_footprint";
    transformStamped_.transform.translation.x = x0_;
    transformStamped_.transform.translation.y = y0_;
    transformStamped_.transform.translation.z = 0.0;
    transformStamped_.transform.rotation.x = q0_.x();
    transformStamped_.transform.rotation.y = q0_.y();
    transformStamped_.transform.rotation.z = q0_.z();
    transformStamped_.transform.rotation.w = q0_.w();

    // Create subscriber for red/wheel_cmd topic
    wheel_cmd_sub_ = create_subscription<nuturtlebot_msgs::msg::WheelCommands>("/wheel_cmd",
                                                                               10,
                                                                               std::bind(&NuSimNode::wheel_cmd_callback,
                                                                                         this,
                                                                                         std::placeholders::_1));

    // Initialize wheel velocities
    left_wheel_velocity_ = 0.0;
    right_wheel_velocity_ = 0.0;

    // Initialize wheel positions
    left_wheel_position_ = 0.0;
    right_wheel_position_ = 0.0;

    // Create publisher to red/sensor_data topic with nuturtlebot_msgs/SensorData message type
    sensor_data_pub_ = create_publisher<nuturtlebot_msgs::msg::SensorData>("/sensor_data", 10);

    // Initialize parameters
    encoder_ticks_per_rad_ = get_parameter("encoder_ticks_per_rad").as_double();
    motor_cmd_per_rad_sec_ = get_parameter("motor_cmd_per_rad_sec").as_double();

    // Create publisher for wall marker array
    wall_marker_array_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/wall_markers", 10);

    // Declare wall-related parameters
    declare_parameter("~x_length", 10.0);
    declare_parameter("~y_length", 10.0);

    // Get wall-related parameters
    wall_x_ = get_parameter("~x_length").as_double();
    wall_y_ = get_parameter("~y_length").as_double();

    // Initialize wall marker array centered at (0,0), 0.25m tall, and 0.1m thick
    wall_marker_array_.markers.resize(4);
    for (int i = 0; i < 4; i++)
    {
      wall_marker_array_.markers[i].header.frame_id = "nusim/world";
      wall_marker_array_.markers[i].header.stamp = this->now();
      wall_marker_array_.markers[i].ns = "walls";
      wall_marker_array_.markers[i].id = i;
      wall_marker_array_.markers[i].type = visualization_msgs::msg::Marker::CUBE;
      wall_marker_array_.markers[i].action = visualization_msgs::msg::Marker::ADD;
      wall_marker_array_.markers[i].pose.position.z = 0.125;
      wall_marker_array_.markers[i].pose.orientation.x = 0.0;
      wall_marker_array_.markers[i].pose.orientation.y = 0.0;
      wall_marker_array_.markers[i].pose.orientation.z = 0.0;
      wall_marker_array_.markers[i].pose.orientation.w = 1.0;
      wall_marker_array_.markers[i].color.a = 1.0;
      wall_marker_array_.markers[i].color.r = 1.0;
      wall_marker_array_.markers[i].color.g = 0.0;
      wall_marker_array_.markers[i].color.b = 0.0;
    }

    // first wall
    wall_marker_array_.markers[0].pose.position.x = -wall_x_ / 2 + 0.05;
    wall_marker_array_.markers[0].pose.position.y = 0.0;
    wall_marker_array_.markers[0].scale.x = 0.1;
    wall_marker_array_.markers[0].scale.y = wall_y_;
    wall_marker_array_.markers[0].scale.z = 0.25;

    // second wall
    wall_marker_array_.markers[1].pose.position.x = 0.0;
    wall_marker_array_.markers[1].pose.position.y = -wall_y_ / 2 + 0.05;
    wall_marker_array_.markers[1].scale.x = wall_x_;
    wall_marker_array_.markers[1].scale.y = 0.1;
    wall_marker_array_.markers[1].scale.z = 0.25;

    // third wall
    wall_marker_array_.markers[2].pose.position.x = wall_x_ / 2 - 0.05;
    wall_marker_array_.markers[2].pose.position.y = 0.0;
    wall_marker_array_.markers[2].scale.x = 0.1;
    wall_marker_array_.markers[2].scale.y = wall_y_;
    wall_marker_array_.markers[2].scale.z = 0.25;

    // fourth wall
    wall_marker_array_.markers[3].pose.position.x = 0.0;
    wall_marker_array_.markers[3].pose.position.y = wall_y_ / 2 - 0.05;
    wall_marker_array_.markers[3].scale.x = wall_x_;
    wall_marker_array_.markers[3].scale.y = 0.1;
    wall_marker_array_.markers[3].scale.z = 0.25;

    // Create fake sensor publisher
    fake_sensor_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/fake_sensor", 10);

    // Create a fake sensor timer that runs at 5Hz
    fake_sensor_timer_ = create_wall_timer(1s / rate_, std::bind(&NuSimNode::fake_sensor_callback, this));

    // Create laser scan publisher
    laser_scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);

    // Create a laser scan timer that runs at 5Hz
    laser_scan_timer_ = create_wall_timer(1s / rate_, std::bind(&NuSimNode::laser_scan_callback, this));

    // Create a path publisher on /red/path topic
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/red/path", 10);

    // Initialize path message
    path_msg_.header.frame_id = "nusim/world";
    pose_stamped_msg_.header.frame_id = "nusim/world";
  }

private:
  void timer_callback()
  {
    // Publish timestep
    std_msgs::msg::UInt64 msg;
    msg.data = timestep_;
    timestep_pub_->publish(msg);

    // Broadcast transform
    transformStamped_.header.stamp = this->get_clock()->now();
    // robot_state_ = diff_drive_.getRobotState();
    transformStamped_.transform.translation.x = robot_state_.x;
    transformStamped_.transform.translation.y = robot_state_.y;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, robot_state_.theta);
    transformStamped_.transform.rotation.x = q.x();
    transformStamped_.transform.rotation.y = q.y();
    transformStamped_.transform.rotation.z = q.z();
    transformStamped_.transform.rotation.w = q.w();
    broadcaster_->sendTransform(transformStamped_);

    // Publish sensor data
    int left_encoder = static_cast<int>((left_wheel_position_ + left_wheel_velocity_ / rate_) / encoder_ticks_per_rad_) % 4096; // 12-bit encoder
    int right_encoder = static_cast<int>((right_wheel_position_ + left_wheel_velocity_ / rate_) / encoder_ticks_per_rad_) % 4096;
    nuturtlebot_msgs::msg::SensorData sensor_data;
    sensor_data.left_encoder = left_encoder;
    sensor_data.right_encoder = right_encoder;
    sensor_data_pub_->publish(sensor_data);

    // Publish obstacle marker array
    obstacles_pub_->publish(marker_array_);
    // Publish wall marker array
    wall_marker_array_pub_->publish(wall_marker_array_);
    // Publish path message
    path_msg_.header.stamp = this->get_clock()->now();
    pose_stamped_msg_.header.stamp = this->get_clock()->now();
    pose_stamped_msg_.pose.position.x = robot_state_.x;
    pose_stamped_msg_.pose.position.y = robot_state_.y;
    path_msg_.poses.push_back(pose_stamped_msg_);
    path_pub_->publish(path_msg_);
    // Increment timestep
    timestep_++;
  }

  void reset_callback(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                      std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    // Reset timestep to zero
    timestep_ = 0;
    // Reset transform
    transformStamped_.transform.translation.x = x0_;
    transformStamped_.transform.translation.y = y0_;
    transformStamped_.transform.rotation.x = q0_.x();
    transformStamped_.transform.rotation.y = q0_.y();
    transformStamped_.transform.rotation.z = q0_.z();
    transformStamped_.transform.rotation.w = q0_.w();
  }

  void teleport_callback(const std::shared_ptr<nusim::srv::Teleport::Request> request,
                         std::shared_ptr<nusim::srv::Teleport::Response>)
  {
    // Set transform
    transformStamped_.transform.translation.x = request->x;
    transformStamped_.transform.translation.y = request->y;
    tf2::Quaternion q;
    q.setRPY(0, 0, request->theta);
    transformStamped_.transform.rotation.x = q.x();
    transformStamped_.transform.rotation.y = q.y();
    transformStamped_.transform.rotation.z = q.z();
    transformStamped_.transform.rotation.w = q.w();
  }

  void wheel_cmd_callback(const nuturtlebot_msgs::msg::WheelCommands::SharedPtr msg)
  {
    std::normal_distribution<double> vel_dist(0.0, input_noise_);
    left_wheel_velocity_ = static_cast<double>(msg->left_velocity);
    right_wheel_velocity_ = static_cast<double>(msg->right_velocity);
    left_wheel_velocity_ *= motor_cmd_per_rad_sec_;
    right_wheel_velocity_ *= motor_cmd_per_rad_sec_;

    if (msg->left_velocity != 0)
    {
      left_wheel_velocity_ += vel_dist(gen_);
    }
    if (msg->right_velocity != 0)
    {
      right_wheel_velocity_ += vel_dist(gen_);
    }

    // Update wheel positions
    left_wheel_position_ += left_wheel_velocity_ / rate_;
    right_wheel_position_ += right_wheel_velocity_ / rate_;

    // Update robot state
    turtlelib::WheelAngles new_wheel_angles;
    new_wheel_angles.left = left_wheel_position_;
    new_wheel_angles.right = right_wheel_position_;
    robot_state_ = diff_drive_.forwardKinematics(new_wheel_angles);

    // Collision detection
    for (int i = 0; i < int(obstacles_x_.size()); i++){
      double distance = sqrt(pow(robot_state_.x - obstacles_x_[i], 2) + pow(robot_state_.y - obstacles_y_[i], 2));
      if (distance < (obstacles_r_ + collision_radius_)){
        double collision_angle = atan2(robot_state_.y - obstacles_y_[i], robot_state_.x - obstacles_x_[i]);
        turtlelib::RobotState new_robot_state;
        new_robot_state.x = obstacles_x_[i] + (collision_radius_+obstacles_r_) * cos(collision_angle);
        new_robot_state.y = obstacles_y_[i] + (collision_radius_+obstacles_r_) * sin(collision_angle);
        new_robot_state.theta = robot_state_.theta;
        diff_drive_.setState(new_robot_state);
      }
    }

    // Add slip noise
    std::uniform_real_distribution<double> slip_dist(-slip_fraction_, slip_fraction_);
    left_wheel_position_ += slip_dist(gen_);
    right_wheel_position_ += slip_dist(gen_);

    // Update wheel angles
    turtlelib::WheelAngles wheel_angles;
    wheel_angles.left = left_wheel_position_;
    wheel_angles.right = right_wheel_position_;
    turtlelib::WheelVelocities wheel_velocities;
    wheel_velocities.left = left_wheel_velocity_;
    wheel_velocities.right = right_wheel_velocity_;
    diff_drive_.setWheelVelocities(wheel_velocities);
    diff_drive_.setWheelAngles(wheel_angles);
  }

  void fake_sensor_callback()
  {
    turtlelib::Vector2D robot_position;
    robot_position.x = robot_state_.x;
    robot_position.y = robot_state_.y;
    turtlelib::Transform2D Twb(robot_position, robot_state_.theta);
    turtlelib::Transform2D Tbw = Twb.inv();

    fake_sensor_marker_array_.markers.resize(obstacles_x_.size());
    for (auto i = 0; i < (int)(obstacles_x_.size()); i++)
    {
      fake_sensor_marker_array_.markers[i].header.frame_id = "red/base_footprint";
      fake_sensor_marker_array_.markers[i].header.stamp = this->get_clock()->now();
      // fake_sensor_marker_array_.markers[i].ns = "fake_sensor";
      fake_sensor_marker_array_.markers[i].id = i;
      fake_sensor_marker_array_.markers[i].type = visualization_msgs::msg::Marker::CYLINDER;
      fake_sensor_marker_array_.markers[i].pose.position.z = 0.125;
      fake_sensor_marker_array_.markers[i].pose.orientation.x = 0.0;
      fake_sensor_marker_array_.markers[i].pose.orientation.y = 0.0;
      fake_sensor_marker_array_.markers[i].pose.orientation.z = 0.0;
      fake_sensor_marker_array_.markers[i].pose.orientation.w = 1.0;
      fake_sensor_marker_array_.markers[i].scale.x = 2 * obstacles_r_;
      fake_sensor_marker_array_.markers[i].scale.y = 2 * obstacles_r_;
      fake_sensor_marker_array_.markers[i].scale.z = 0.25;
      fake_sensor_marker_array_.markers[i].color.a = 1.0;
      fake_sensor_marker_array_.markers[i].color.r = 1.0;
      fake_sensor_marker_array_.markers[i].color.g = 1.0;
      fake_sensor_marker_array_.markers[i].color.b = 0.0;

      // we need to determine action based on distance to obstacle
      // obstacles are in the body frame!!!
      turtlelib::Vector2D obstacle_position;
      obstacle_position.x = obstacles_x_[i];
      obstacle_position.y = obstacles_y_[i];
      turtlelib::Vector2D obstacle_position_body = Tbw(obstacle_position);
      double distance_to_obstacle = sqrt(pow(obstacle_position_body.x, 2) +
                                         pow(obstacle_position_body.y, 2));
      if (distance_to_obstacle < max_range_)
      {
        fake_sensor_marker_array_.markers[i].action = visualization_msgs::msg::Marker::ADD;
      }
      else
      {
        fake_sensor_marker_array_.markers[i].action = visualization_msgs::msg::Marker::DELETE;
      }

      // we need to add noise to the distance to obstacle
      std::normal_distribution<double> obstacle_dist_dist(0.0, basic_sensor_variance_);
      double obstacle_noise_x = obstacle_dist_dist(gen_);
      double obstacle_noise_y = obstacle_dist_dist(gen_);
      fake_sensor_marker_array_.markers[i].pose.position.x = obstacle_position_body.x + obstacle_noise_x;
      fake_sensor_marker_array_.markers[i].pose.position.y = obstacle_position_body.y + obstacle_noise_y;

      /*
      fake_sensor_marker_array_.markers[i].action = visualization_msgs::msg::Marker::ADD;
      fake_sensor_marker_array_.markers[i].pose.position.x = obstacles_x_[i];
      fake_sensor_marker_array_.markers[i].pose.position.y = obstacles_y_[i];
      */
    }
    fake_sensor_pub_->publish(fake_sensor_marker_array_);
  }

  void laser_scan_callback(){
    laser_scan_msg_.header.stamp = this->get_clock()->now();
    laser_scan_msg_.header.frame_id = "red/base_scan";
    laser_scan_msg_.angle_min = angle_min_;
    laser_scan_msg_.angle_max = angle_max_;
    laser_scan_msg_.angle_increment = angle_increment_;
    laser_scan_msg_.time_increment = 2.98699997074e-05;
    laser_scan_msg_.scan_time = 0.1;
    laser_scan_msg_.range_min = min_lidar_range_;
    laser_scan_msg_.range_max = max_lidar_range_;

    laser_scan_msg_.ranges.resize(number_of_samples_);

    std::normal_distribution<double> laser_noise_dist(0.0, basic_sensor_variance_);
    auto current_sensor_angle = 0.0;

    turtlelib::Vector2D body_vector;
    body_vector.x = robot_state_.x;
    body_vector.y = robot_state_.y;
    turtlelib::Transform2D Twb(body_vector, robot_state_.theta);

    for (auto i = 0; i < number_of_samples_; i++){
      std::vector<double> line_distances;
      // deal with the obstacles
      for (auto i = 0; i < int(obstacles_x_.size()); i++){
        turtlelib::Vector2D obstacle_vector;
        obstacle_vector.x = obstacles_x_[i] + laser_noise_dist(gen_);
        obstacle_vector.y = obstacles_y_[i] + laser_noise_dist(gen_);
        turtlelib::Vector2D min_range_vector;
        min_range_vector.x = min_lidar_range_ * cos(current_sensor_angle);
        min_range_vector.y = min_lidar_range_ * sin(current_sensor_angle);
        turtlelib::Vector2D max_range_vector;
        max_range_vector.x = max_lidar_range_ * cos(current_sensor_angle);
        max_range_vector.y = max_lidar_range_ * sin(current_sensor_angle);

        turtlelib::Transform2D Two(obstacle_vector, 0.0);
        turtlelib::Transform2D Tob = (Two.inv())*Twb;

        min_range_vector = Tob(min_range_vector);
        max_range_vector = Tob(max_range_vector);

        double x1 = max_range_vector.x;
        double y1 = max_range_vector.y;
        double x2 = min_range_vector.x;
        double y2 = min_range_vector.y;

        double dx = x2 - x1;
        double dy = y2 - y1;
        double dr = sqrt(pow(dx, 2) + pow(dy, 2));
        double D = x1*y2 - x2*y1;
        double discriminant = pow(obstacles_r_, 2) * pow(dr, 2) - pow(D, 2);

        // if intersection with this obstacle
        double x_intercept1 = 0.0;
        double x_intercept2 = 0.0;
        double y_intercept1 = 0.0;
        double y_intercept2 = 0.0;
        if (discriminant >= 0.0){
          if (dy > 0.0){
            x_intercept1 = (D*dy + dx*sqrt(discriminant)) / pow(dr, 2);
            x_intercept2 = (D*dy - dx*sqrt(discriminant)) / pow(dr, 2);
          }else{
            x_intercept1 = (D*dy - dx*sqrt(discriminant)) / pow(dr, 2);
            x_intercept2 = (D*dy + dx*sqrt(discriminant)) / pow(dr, 2);
          }
          
          y_intercept1 = (-D*dx + abs(dy)*sqrt(discriminant)) / pow(dr, 2);
          y_intercept2 = (-D*dx - abs(dy)*sqrt(discriminant)) / pow(dr, 2);

          turtlelib::Vector2D intersection1;
          intersection1.x = x_intercept1;
          intersection1.y = y_intercept1;
          turtlelib::Vector2D intersection2;
          intersection2.x = x_intercept2;
          intersection2.y = y_intercept2;

          turtlelib::Vector2D intersection1_body = Tob.inv()(intersection1);
          turtlelib::Vector2D intersection2_body = Tob.inv()(intersection2);

          double distance1 = sqrt(pow(intersection1_body.x, 2) + pow(intersection1_body.y, 2));
          double distance2 = sqrt(pow(intersection2_body.x, 2) + pow(intersection2_body.y, 2));

          line_distances.push_back(std::min(distance1, distance2));
        }
      }
      // deal with the walls
      // use line-line intersection formula from mathworld?
      double x1 = robot_state_.x;
      double y1 = robot_state_.y;
      double x2 = x1 + max_lidar_range_ * cos(current_sensor_angle + robot_state_.theta);
      double y2 = y1 + max_lidar_range_ * sin(current_sensor_angle + robot_state_.theta);
      std::vector<std::pair<double, double>> corners_list = {std::make_pair(-wall_x_/2.0+0.1, -wall_y_/2.0+0.2),
                                                             std::make_pair(-wall_x_/2.0+0.1, wall_y_/2.0-0.1),
                                                             std::make_pair(wall_x_/2.0-0.1, wall_y_/2.0-0.1),
                                                             std::make_pair(wall_x_/2.0-0.1, -wall_y_/2.0+0.2)};
      // loop through lines formed by pairs of corners
      for (auto i = 0; i < 4; i++){
        auto corner1 = corners_list[i];
        auto corner2 = corners_list[(i+1)%4];
        double x3 = corner1.first;
        double y3 = corner1.second;
        double x4 = corner2.first;
        double y4 = corner2.second;
        double denominator = (x1-x2)*(y3-y4) - (y1-y2)*(x3-x4);
        if (denominator != 0.0){
          double x_intercept = ((x1*y2 - y1*x2)*(x3-x4) - (x1-x2)*(x3*y4 - y3*x4)) / denominator;
          double y_intercept = ((x1*y2 - y1*x2)*(y3-y4) - (y1-y2)*(x3*y4 - y3*x4)) / denominator;
          // check if intersection is within the line segment
          if (x_intercept >= std::min(x1, x2) && x_intercept <= std::max(x1, x2) &&
              x_intercept >= std::min(x3, x4) && x_intercept <= std::max(x3, x4)){
            turtlelib::Vector2D intersection;
            intersection.x = x_intercept;
            intersection.y = y_intercept;
            turtlelib::Vector2D intersection_body = Twb.inv()(intersection);
            double distance = sqrt(pow(intersection_body.x, 2) + pow(intersection_body.y, 2));
            line_distances.push_back(distance);
          }
        }
      }
      // find the closest intersection
      if (line_distances.size() > 0){
        laser_scan_msg_.ranges[i] = *std::min_element(line_distances.begin(), line_distances.end());
      }
      else{
        laser_scan_msg_.ranges[i] = max_lidar_range_;
      }
      current_sensor_angle += angle_increment_;
      // check sensor angle against angle_max_
      if (current_sensor_angle > angle_max_){
        current_sensor_angle = 0.0;
      }
    }
    // publish the laser scan message
    laser_scan_pub_->publish(laser_scan_msg_);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacles_pub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv_;
  rclcpp::Service<nusim::srv::Teleport>::SharedPtr teleport_srv_;
  // tf2_ros::TransformBroadcaster broadcaster_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  geometry_msgs::msg::TransformStamped transformStamped_;
  uint64_t timestep_;
  double rate_;
  double x0_;
  double y0_;
  double theta0_;
  tf2::Quaternion q0_;
  visualization_msgs::msg::MarkerArray marker_array_;
  rclcpp::Subscription<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_cmd_sub_;
  double left_wheel_velocity_;
  double right_wheel_velocity_;
  double left_wheel_position_;
  double right_wheel_position_;
  turtlelib::DiffDrive diff_drive_;
  double encoder_ticks_per_rad_;
  double motor_cmd_per_rad_sec_;
  double wall_x_;
  double wall_y_;
  visualization_msgs::msg::MarkerArray wall_marker_array_;
  rclcpp::Publisher<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_data_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr wall_marker_array_pub_;
  turtlelib::RobotState robot_state_;
  double input_noise_;
  double slip_fraction_;
  double basic_sensor_variance_;
  double max_range_;
  double collision_radius_;
  double min_lidar_range_;
  double max_lidar_range_;
  double angle_increment_;
  int number_of_samples_;
  double resolution_;
  double noise_level_;
  double angle_min_;
  double angle_max_;
  std::random_device rd{}; // obtain a random number from hardware
  std::mt19937 gen_{rd()}; // seed the generator
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr fake_sensor_pub_;
  visualization_msgs::msg::MarkerArray fake_sensor_marker_array_;
  std::vector<double> obstacles_x_;
  std::vector<double> obstacles_y_;
  double obstacles_r_;
  rclcpp::TimerBase::SharedPtr fake_sensor_timer_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_pub_;
  rclcpp::TimerBase::SharedPtr laser_scan_timer_;
  sensor_msgs::msg::LaserScan laser_scan_msg_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  nav_msgs::msg::Path path_msg_;
  geometry_msgs::msg::PoseStamped pose_stamped_msg_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  std::vector<double> obstacles_x = {-0.6, 0.7, 0.5};
  std::vector<double> obstacles_y = {-0.8, -0.7, 0.9};
  rclcpp::spin(std::make_shared<NuSimNode>());
  rclcpp::shutdown();
  return 0;
}
