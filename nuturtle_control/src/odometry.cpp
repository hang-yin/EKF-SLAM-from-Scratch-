#include <rclcpp/rclcpp.hpp>
#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nuturtle_control/srv/initial_pose.hpp"

using namespace std::chrono_literals;

class OdometryNode : public rclcpp::Node
{
public: 
    OdometryNode()
    : Node("odometry")
    {
        // Declare parameters
        this->declare_parameter("body_id", "blue/base_footprint");
        this->declare_parameter("odom_id", "odom");
        this->declare_parameter("wheel_left", "left_default");
        this->declare_parameter("wheel_right", "right_default");
    
        // Check whether parameters are specified
        if (!this->has_parameter("body_id")) {
            RCLCPP_ERROR(this->get_logger(), "body_id is not specified");
            rclcpp::shutdown();
        }
        if (!this->has_parameter("wheel_left") || !this->has_parameter("wheel_right")) {
            RCLCPP_ERROR(this->get_logger(), "name of wheel joints not specified");
            rclcpp::shutdown();
        }

        // Get parameters
        body_id_ = this->get_parameter("body_id").as_string();
        if (this->has_parameter("odom_id")) {
            odom_id_ = this->get_parameter("odom_id").as_string();
        }
        wheel_left_ = this->get_parameter("wheel_left").as_string();
        wheel_right_ = this->get_parameter("wheel_right").as_string();

        // Create publisher for odometry
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

        // Create subscriber for joint state
        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("blue/joint_states",
                                                                             10,
                                                                             std::bind(&OdometryNode::joint_callback,
                                                                                       this,
                                                                                       std::placeholders::_1));
        
        // Create a transform broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Create a service to reset odometry
        reset_srv_ = this->create_service<nuturtle_control::srv::InitialPose>("~/initial_pose",
                                                                              std::bind(&OdometryNode::initial_pose_callback,
                                                                                        this,
                                                                                        std::placeholders::_1,
                                                                                        std::placeholders::_2));
        
        // Create a timer to publish odometry
        timer_ = this->create_wall_timer(100ms, std::bind(&OdometryNode::timer_callback, this));

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

        // Initialize transform
        odom_tf_.header.frame_id = odom_id_;
        odom_tf_.child_frame_id = body_id_;
        odom_tf_.transform.translation.x = 0.0;
        odom_tf_.transform.translation.y = 0.0;
        odom_tf_.transform.translation.z = 0.0;
        odom_tf_.transform.rotation.x = 0.0;
        odom_tf_.transform.rotation.y = 0.0;
        odom_tf_.transform.rotation.z = 0.0;
        odom_tf_.transform.rotation.w = 1.0;

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
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<nuturtle_control::srv::InitialPose>::SharedPtr reset_srv_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    nav_msgs::msg::Odometry odom_;
    turtlelib::DiffDrive diff_drive_;
    double x_;
    double y_;
    double theta_;
    double left_wheel_pos_;
    double right_wheel_pos_;
    double left_wheel_vel_;
    double right_wheel_vel_;
    geometry_msgs::msg::TransformStamped odom_tf_;

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
        new_wheel_angles.left = left_wheel_pos_ + left_wheel_vel_ * 0.1;
        new_wheel_angles.right = right_wheel_pos_ + right_wheel_vel_ * 0.1;

        // Update x, y, theta through forward kinematics
        turtlelib::RobotState robot_state = diff_drive_.forwardKinematics(new_wheel_angles);
        x_ = robot_state.x;
        y_ = robot_state.y;
        theta_ = robot_state.theta;
    }

    // Callback function for initial_pose service
    void initial_pose_callback(const nuturtle_control::srv::InitialPose::Request::SharedPtr req,
                               const nuturtle_control::srv::InitialPose::Response::SharedPtr res)
    {
        x_ = 0.0;
        y_ = 0.0;
        theta_ = 0.0;
        diff_drive_ = turtlelib::DiffDrive();
        // the request contains the configuration of the robot
        // TODO: Set the configuration of the robot
        // the response is empty
        (void)req;
        (void)res;
    }

    // Callback function for timer
    void timer_callback()
    {
        // Update header
        odom_.header.stamp = this->now();

        // Update pose
        odom_.pose.pose.position.x = x_;
        odom_.pose.pose.position.y = y_;

        // Update orientation
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, theta_);
        odom_.pose.pose.orientation.x = q.x();
        odom_.pose.pose.orientation.y = q.y();
        odom_.pose.pose.orientation.z = q.z();
        odom_.pose.pose.orientation.w = q.w();

        // TODO: Update twist
        
        // Publish odometry
        odom_pub_->publish(odom_);

        // Publish transform
        odom_tf_.header.stamp = this->now();
        odom_tf_.transform.translation.x = x_;
        odom_tf_.transform.translation.y = y_;
        odom_tf_.transform.rotation.x = q.x();
        odom_tf_.transform.rotation.y = q.y();
        odom_tf_.transform.rotation.z = q.z();
        odom_tf_.transform.rotation.w = q.w();
        tf_broadcaster_->sendTransform(odom_tf_);
    } 
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdometryNode>());
    rclcpp::shutdown();
    return 0;
}       
