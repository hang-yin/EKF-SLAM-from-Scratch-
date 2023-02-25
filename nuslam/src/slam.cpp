#include <rclcpp/rclcpp.hpp>
#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/ekf.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <armadillo>

using namespace std::chrono_literals;

class SLAMnode : public rclcpp::Node
{
public: 
    SLAMnode()
    : Node("slam")
    {
        // Declare parameters
        declare_parameter("body_id", "blue/base_footprint");
        declare_parameter("world_id", "nusim/world");
        declare_parameter("odom_id", "odom");
        declare_parameter("wheel_left", "left_default");
        declare_parameter("wheel_right", "right_default");
        declare_parameter("obstacle_radius", 0.038);
        declare_parameter("obstacle_height", 0.25);
        declare_parameter("min_lidar_range", 0.12);
        declare_parameter("max_lidar_range", 3.5);
    
        // Check whether parameters are specified
        if (!has_parameter("body_id")) {
            RCLCPP_ERROR(this->get_logger(), "body_id is not specified");
            rclcpp::shutdown();
        }

        if (!has_parameter("wheel_left") || !has_parameter("wheel_right")) {
            RCLCPP_ERROR(this->get_logger(), "name of wheel joints not specified");
            rclcpp::shutdown();
        }

        // Get parameters
        body_id_ = get_parameter("body_id").as_string();
        if (has_parameter("odom_id")) {
            odom_id_ = get_parameter("odom_id").as_string();
        }
        wheel_left_ = get_parameter("wheel_left").as_string();
        wheel_right_ = get_parameter("wheel_right").as_string();
        obstacle_radius_ = get_parameter("obstacle_radius").as_double();
        obstacle_height_ = get_parameter("obstacle_height").as_double();
        lidar_range_min_ = get_parameter("min_lidar_range").as_double();
        lidar_range_max_ = get_parameter("max_lidar_range").as_double();
        world_id_ = get_parameter("world_id").as_string();

        // Create subscriber for joint state
        joint_sub_ = create_subscription<sensor_msgs::msg::JointState>("red/joint_states",
                                                                       10,
                                                                       std::bind(&SLAMnode::joint_callback,
                                                                       this,
                                                                       std::placeholders::_1));
        // Create subscriber for fake sensor
        fake_sensor_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>("/fake_sensor",
                                                                                    10,
                                                                                    std::bind(&SLAMnode::fake_sensor_callback,
                                                                                              this,
                                                                                              std::placeholders::_1));

        // Create publisher for odometry
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        // Create publisher for marker array
        slam_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/slam_marker", 10);

        // Create a transform broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        
        // Create a timer to publish odometry
        rate_ = 200.0;
        timer_ = create_wall_timer(1s / rate_, std::bind(&SLAMnode::timer_callback, this));
        slam_marker_timer_ = create_wall_timer(1s / rate_, std::bind(&SLAMnode::slam_marker_callback, this));

        // Create a path publishers
        odom_path_pub_ = create_publisher<nav_msgs::msg::Path>("/blue/path", 10);
        slam_path_pub_ = create_publisher<nav_msgs::msg::Path>("/green/path", 10);
        // Initialize path message
        odom_path_msg_.header.frame_id = "nusim/world";
        odom_pose_stamped_msg_.header.frame_id = "nusim/world";
        slam_path_msg_.header.frame_id = "map";
        slam_pose_stamped_msg_.header.frame_id = "map";

        // Initialize odometry
        odom_.header.frame_id = "nusim/world";
        odom_.child_frame_id = body_id_;
        odom_.pose.pose.position.x = 0.0;
        odom_.pose.pose.position.y = 0.0;
        odom_.pose.pose.position.z = 0.0;
        odom_.pose.pose.orientation.x = 0.0;
        odom_.pose.pose.orientation.y = 0.0;
        odom_.pose.pose.orientation.z = 0.0;
        odom_.pose.pose.orientation.w = 1.0;
        odom_.twist.twist.linear.x = 0.0;
        odom_.twist.twist.linear.y = 0.0;
        odom_.twist.twist.linear.z = 0.0;
        odom_.twist.twist.angular.x = 0.0;
        odom_.twist.twist.angular.y = 0.0;
        odom_.twist.twist.angular.z = 0.0;

        // Initialize odom->blue/base_footprint transform
        odom_blue_tf_.header.frame_id = "nusim/world";
        odom_blue_tf_.child_frame_id = body_id_;
        odom_blue_tf_.transform.translation.x = 0.0;
        odom_blue_tf_.transform.translation.y = 0.0;
        odom_blue_tf_.transform.translation.z = 0.0;
        odom_blue_tf_.transform.rotation.x = 0.0;
        odom_blue_tf_.transform.rotation.y = 0.0;
        odom_blue_tf_.transform.rotation.z = 0.0;
        odom_blue_tf_.transform.rotation.w = 1.0;

        // Initialize map to odom transform
        map_odom_tf_.header.frame_id = "map";
        map_odom_tf_.child_frame_id = odom_id_;
        map_odom_tf_.transform.translation.x = 0.0;
        map_odom_tf_.transform.translation.y = 0.0;
        map_odom_tf_.transform.translation.z = 0.0;
        map_odom_tf_.transform.rotation.x = 0.0;
        map_odom_tf_.transform.rotation.y = 0.0;
        map_odom_tf_.transform.rotation.z = 0.0;
        map_odom_tf_.transform.rotation.w = 1.0;

        // Initialize odom->green/base_footprint transform
        odom_green_tf_.header.frame_id = odom_id_;
        odom_green_tf_.child_frame_id = "green/base_footprint";
        odom_green_tf_.transform.translation.x = 0.0;
        odom_green_tf_.transform.translation.y = 0.0;
        odom_green_tf_.transform.translation.z = 0.0;
        odom_green_tf_.transform.rotation.x = 0.0;
        odom_green_tf_.transform.rotation.y = 0.0;
        odom_green_tf_.transform.rotation.z = 0.0;
        odom_green_tf_.transform.rotation.w = 1.0;

        // Initialize x, y, theta, wheel positions and velocities
        x_ = 0.0;
        y_ = 0.0;
        theta_ = 0.0;
        left_wheel_pos_ = 0.0;
        right_wheel_pos_ = 0.0;
        left_wheel_vel_ = 0.0;
        right_wheel_vel_ = 0.0;

        ekf_obstacles_set_ = true;

        // Initialize ekf object with 3 obstacles for now
        ekf_.set_max_landmarks(3);
    }

private:
    // Declare variables
    std::string body_id_;
    std::string odom_id_;
    std::string world_id_;
    std::string wheel_left_;
    std::string wheel_right_;
    double x_;
    double y_;
    double theta_;
    double left_wheel_pos_;
    double right_wheel_pos_;
    double left_wheel_vel_;
    double right_wheel_vel_;
    double rate_;

    turtlelib::DiffDrive diff_drive_;
    nav_msgs::msg::Odometry odom_;
    nav_msgs::msg::Path odom_path_msg_;
    nav_msgs::msg::Path slam_path_msg_;
    geometry_msgs::msg::TransformStamped odom_blue_tf_;
    geometry_msgs::msg::TransformStamped odom_green_tf_;
    geometry_msgs::msg::TransformStamped map_odom_tf_;
    geometry_msgs::msg::PoseStamped odom_pose_stamped_msg_;
    geometry_msgs::msg::PoseStamped slam_pose_stamped_msg_;

    // Declare publishers, subscribers, timers, services, and broadcasters
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr slam_marker_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr fake_sensor_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr odom_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr slam_path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr slam_marker_timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Declare EKF class object
    turtlelib::EKF ekf_;

    // Declare slam obstacles array, it is a vector of double pairs
    std::vector<std::pair<double, double>> slam_obstacles_;

    // Declare slam marker array
    visualization_msgs::msg::MarkerArray slam_marker_array_msg_;

    // Declare parameters
    double obstacle_radius_;
    double obstacle_height_;
    double lidar_range_min_;
    double lidar_range_max_;

    // Declare a bool value for setting up ekf obstacles
    bool ekf_obstacles_set_;

    // Declare twist
    turtlelib::Twist2D twist_;

    // Callback function for joint state
    void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // Check if joint state message is valid
        if (msg->name.size() != 2 || msg->position.size() != 2 || msg->velocity.size() != 2) {
            return;
        }
        
        left_wheel_pos_ = msg->position.at(0);
        right_wheel_pos_ = msg->position.at(1);
        left_wheel_vel_ = msg->velocity.at(0);
        right_wheel_vel_ = msg->velocity.at(1);

        // Update diff_drive position and velocity
        turtlelib::WheelAngles wheel_angles;
        wheel_angles.left = left_wheel_pos_;
        wheel_angles.right = right_wheel_pos_;

        turtlelib::WheelVelocities wheel_velocities;
        wheel_velocities.left = left_wheel_vel_;
        wheel_velocities.right = right_wheel_vel_;
        diff_drive_.setWheelAngles(wheel_angles);
        diff_drive_.setWheelVelocities(wheel_velocities);

        // Get new angles
        turtlelib::WheelAngles new_wheel_angles;
        new_wheel_angles.left = left_wheel_pos_ + left_wheel_vel_ / rate_;
        new_wheel_angles.right = right_wheel_pos_ + right_wheel_vel_ / rate_;
        


        // Update x, y, theta through forward kinematics
        turtlelib::RobotState robot_state = diff_drive_.forwardKinematics(new_wheel_angles);
        // turtlelib::RobotState robot_state = diff_drive_.forwardKinematics(wheel_angles);
        twist_ = diff_drive_.getTwist();
        // turtlelib::RobotState robot_state = diff_drive_.forwardKinematics(wheel_angles);
        x_ = robot_state.x;
        y_ = robot_state.y;
        theta_ = robot_state.theta;
        // log info
        // RCLCPP_INFO(this->get_logger(), "x: %f, y: %f, theta: %f", x_, y_, theta_);

        diff_drive_.setWheelAngles(wheel_angles);

    }

    // Callback function for fake sensor
    void fake_sensor_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
    {
        // Check if fake sensor message is valid
        if (msg->markers.size() == 0) {
            return;
        }
        // Extract x and y values from marker array
        std::vector<std::pair<double, double>> fake_sensor_obstacles;
        for (int i = 0; i < int(msg->markers.size()); i++) {
            double x = msg->markers[i].pose.position.x;
            double y = msg->markers[i].pose.position.y;
            fake_sensor_obstacles.push_back(std::make_pair(x, y));
        }
        // Set ekf obstacles

        // log where the obstacles are
        for (int i = 0; i < int(fake_sensor_obstacles.size()); i++) {
            RCLCPP_INFO(this->get_logger(), "Fake x: %f, y: %f", fake_sensor_obstacles[i].first, fake_sensor_obstacles[i].second);
        }
        
        if (ekf_obstacles_set_) {
            ekf_.set_obstacles(fake_sensor_obstacles);
            ekf_obstacles_set_ = false;
        }
        ekf_.predict(twist_);
        for (int i = 0; i < int(fake_sensor_obstacles.size()); i++) {
            //ekf_.set_max_landmarks(3);    
            ekf_.correct(i, fake_sensor_obstacles[i].first, fake_sensor_obstacles[i].second);
        }
        slam_obstacles_ = ekf_.get_obstacles();
        arma::vec state_prev = ekf_.get_obstacles_1();
        // log state_prev
        for (int i = 0; i < int(state_prev.size()); i++) {
            RCLCPP_INFO(this->get_logger(), "state_prev: %f", state_prev[i]);
        }
    }

    void slam_marker_callback()
    {
        // Clear slam marker array
        slam_marker_array_msg_.markers.resize(slam_obstacles_.size());
        // Create slam marker array
        for (int i = 0; i < int(slam_obstacles_.size()); i++) {
            visualization_msgs::msg::Marker slam_marker_msg;
            slam_marker_msg.header.frame_id = "map";
            slam_marker_msg.header.stamp = this->now();
            slam_marker_msg.ns = "slam_obstacles";
            slam_marker_msg.id = i;
            slam_marker_msg.type = visualization_msgs::msg::Marker::CYLINDER;
            double x = slam_obstacles_[i].first - ekf_.get_x();
            double y = slam_obstacles_[i].second - ekf_.get_y();
            double r = std::sqrt(x * x + y * y);
            slam_marker_msg.action = visualization_msgs::msg::Marker::ADD;

            slam_marker_msg.pose.position.x = slam_obstacles_[i].first;
            slam_marker_msg.pose.position.y = slam_obstacles_[i].second;
            // log info
            RCLCPP_INFO(this->get_logger(), "Obstacle i: %d, x: %f, y: %f", i, slam_obstacles_[i].first, slam_obstacles_[i].second);
            slam_marker_msg.pose.position.z = 0.125;
            slam_marker_msg.pose.orientation.x = 0.0;
            slam_marker_msg.pose.orientation.y = 0.0;
            slam_marker_msg.pose.orientation.z = 0.0;
            slam_marker_msg.pose.orientation.w = 1.0;
            slam_marker_msg.scale.x = 2.0*obstacle_radius_;
            slam_marker_msg.scale.y = 2.0*obstacle_radius_;
            slam_marker_msg.scale.z = obstacle_height_;
            slam_marker_msg.color.a = 1.0;
            slam_marker_msg.color.r = 0.0;
            slam_marker_msg.color.g = 1.0;
            slam_marker_msg.color.b = 0.0;
            slam_marker_array_msg_.markers[i] = slam_marker_msg;
        }
        // Publish slam marker array
        slam_marker_pub_->publish(slam_marker_array_msg_);
    }

    void timer_callback()
    {
        // Update and publish odometry message
        odom_.header.stamp = this->now();
        odom_.pose.pose.position.x = x_;
        odom_.pose.pose.position.y = y_;
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, theta_);
        odom_.pose.pose.orientation.x = q.x();
        odom_.pose.pose.orientation.y = q.y();
        odom_.pose.pose.orientation.z = q.z();
        odom_.pose.pose.orientation.w = q.w();
        odom_pub_->publish(odom_);

        // Publish transform from odom to blue/base_link
        q.setRPY(0.0, 0.0, theta_);
        odom_blue_tf_.header.stamp = this->now();
        odom_blue_tf_.transform.translation.x = x_;
        odom_blue_tf_.transform.translation.y = y_;
        odom_blue_tf_.transform.rotation.x = q.x();
        odom_blue_tf_.transform.rotation.y = q.y();
        odom_blue_tf_.transform.rotation.z = q.z();
        odom_blue_tf_.transform.rotation.w = q.w();
        tf_broadcaster_->sendTransform(odom_blue_tf_);

        // Publish transform from map to odom
        turtlelib::Vector2D ekf_pose;
        ekf_pose.x = ekf_.get_x();
        ekf_pose.y = ekf_.get_y();
        double ekf_theta = ekf_.get_theta();
        // log x, y, and theta
        // RCLCPP_INFO(this->get_logger(), "x, y, theta: %f, %f, %f", ekf_pose.x, ekf_pose.y, ekf_theta);
        turtlelib::Transform2D T_map_body(ekf_pose, ekf_theta);
        turtlelib::Vector2D odom_pose;
        odom_pose.x = x_;
        odom_pose.y = y_;
        turtlelib::Transform2D T_odom_body(odom_pose, theta_);
        turtlelib::Transform2D T_map_odom = T_map_body * T_odom_body.inv();
        map_odom_tf_.header.stamp = this->now();
        map_odom_tf_.transform.translation.x = T_map_odom.translation().x;
        map_odom_tf_.transform.translation.y = T_map_odom.translation().y;
        q.setRPY(0.0, 0.0, T_map_odom.rotation());
        map_odom_tf_.transform.rotation.x = q.x();
        map_odom_tf_.transform.rotation.y = q.y();
        map_odom_tf_.transform.rotation.z = q.z();
        map_odom_tf_.transform.rotation.w = q.w();
        tf_broadcaster_->sendTransform(map_odom_tf_);

        // Publish transform from odom to green/base_link
        odom_green_tf_.header.stamp = this->now();
        odom_green_tf_.transform.translation.x = x_;
        odom_green_tf_.transform.translation.y = y_;
        q.setRPY(0.0, 0.0, theta_);
        odom_green_tf_.transform.rotation.x = q.x();
        odom_green_tf_.transform.rotation.y = q.y();
        odom_green_tf_.transform.rotation.z = q.z();
        odom_green_tf_.transform.rotation.w = q.w();
        tf_broadcaster_->sendTransform(odom_green_tf_);

        // TODO: publish slam marker array

        // Publish odom path
        odom_path_msg_.header.stamp = this->get_clock()->now();
        odom_pose_stamped_msg_.header.stamp = this->get_clock()->now();
        odom_pose_stamped_msg_.pose.position.x = x_;
        odom_pose_stamped_msg_.pose.position.y = y_;
        odom_path_msg_.poses.push_back(odom_pose_stamped_msg_);
        odom_path_pub_->publish(odom_path_msg_);

        // Publish slam path
        slam_path_msg_.header.stamp = this->get_clock()->now();
        slam_pose_stamped_msg_.header.stamp = this->get_clock()->now();
        slam_pose_stamped_msg_.pose.position.x = ekf_.get_x();
        slam_pose_stamped_msg_.pose.position.y = ekf_.get_y();
        slam_path_msg_.poses.push_back(slam_pose_stamped_msg_);
        slam_path_pub_->publish(slam_path_msg_);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SLAMnode>());
    rclcpp::shutdown();
    return 0;
}       
