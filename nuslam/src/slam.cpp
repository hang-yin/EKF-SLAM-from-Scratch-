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
#include "nuturtle_control/srv/initial_pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::chrono_literals;

class SLAMnode : public rclcpp::Node
{
public: 
    SLAMnode()
    : Node("slam")
    {
        // Declare parameters
        declare_parameter("body_id", "blue/base_footprint");
        declare_parameter("odom_id", "odom");
        declare_parameter("wheel_left", "left_default");
        declare_parameter("wheel_right", "right_default");
    
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

        // Create subscriber for joint state
        joint_sub_ = create_subscription<sensor_msgs::msg::JointState>("red/joint_states",
                                                                       10,
                                                                       std::bind(&OdometryNode::joint_callback,
                                                                       this,
                                                                       std::placeholders::_1));
        // Create subscriber for laser scan
        laser_scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>("/scan",
                                                                           10,
                                                                           std::bind(&OdometryNode::laser_scan_callback,
                                                                                     this,
                                                                                     std::placeholders::_1));
        // Create publisher for odometry
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        // Create publisher for marker array
        slam_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/slam_marker", 10);

        // Create a transform broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Create a service to reset odometry
        reset_srv_ = create_service<nuturtle_control::srv::InitialPose>("~/initial_pose",
                                                                        std::bind(&OdometryNode::initial_pose_callback,
                                                                                  this,
                                                                                  std::placeholders::_1,
                                                                                  std::placeholders::_2));
        
        // Create a timer to publish odometry
        rate_ = 200.0;
        timer_ = create_wall_timer(1s / rate_, std::bind(&OdometryNode::timer_callback, this));

        // Create a path publishers
        odom_path_pub_ = create_publisher<nav_msgs::msg::Path>("/blue/path", 10);
        slam_path_pub_ = create_publisher<nav_msgs::msg::Path>("/green/path", 10);
        // Initialize path message
        odom_path_msg_.header.frame_id = "nusim/world";
        odom_pose_stamped_msg_.header.frame_id = "nusim/world";
        slam_path_msg_.header.frame_id = "map";
        slam_pose_stamped_msg_.header.frame_id = "map";

        // Initialize odometry
        odom_.header.frame_id = odom_id_;
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
        odom_blue_tf_.header.frame_id = odom_id_;
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

        // Initialize odom->green/base_footprint transform
        odom_green_tf_.header.frame_id = odom_id_;
        odom_green_tf_.child_frame_id = "green/base_footprint";

        // Initialize x, y, theta, wheel positions and velocities
        x_ = 0.0;
        y_ = 0.0;
        theta_ = 0.0;
        left_wheel_pos_ = 0.0;
        right_wheel_pos_ = 0.0;
        left_wheel_vel_ = 0.0;
        right_wheel_vel_ = 0.0;
    }

private:
    // Declare variables
    std::string body_id_;
    std::string odom_id_;
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
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
    rclcpp::Service<nuturtle_control::srv::InitialPose>::SharedPtr reset_srv_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr odom_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr slam_path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Declare EKF class object
    turtlelib::EKF ekf_;
    

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
        x_ = robot_state.x;
        y_ = robot_state.y;
        theta_ = robot_state.theta;
    }

    // Callback function for initial_pose service
    void initial_pose_callback(const nuturtle_control::srv::InitialPose::Request::SharedPtr,
                               const nuturtle_control::srv::InitialPose::Response::SharedPtr)
    {
        x_ = 0.0;
        y_ = 0.0;
        theta_ = 0.0;
        diff_drive_ = turtlelib::DiffDrive();
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
        turtlelib::Vector2D ekf_pose(ekf_.getX(), ekf_.getY());
        double ekf_theta = ekf_.getTheta();
        turtlelib::Transform2D T_map_body(ekf_pose, ekf_theta);
        turtlelib::Vector2D odom_pose(x_, y_);
        turtlelib::Transform2D T_odom_body(odom_pose, theta_);
        turtlelib::Transform2D T_map_odom = T_map_body * T_odom_body.inv();
        map_odom_tf_.header.stamp = this->now();
        map_odom_tf_.transform.translation.x = T_map_odom.x();
        map_odom_tf_.transform.translation.y = T_map_odom.y();
        q.setRPY(0.0, 0.0, T_map_odom.theta());
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
        slam_pose_stamped_msg_.pose.position.x = ekf_.getX();
        slam_pose_stamped_msg_.pose.position.y = ekf_.getY();
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
