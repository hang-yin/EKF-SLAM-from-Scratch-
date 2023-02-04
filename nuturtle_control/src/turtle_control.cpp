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
        // Check if required parameters are defined
        if (!this->has_parameter("wheel_radius") || !this->has_parameter("wheel_separation"))
        {
            RCLCPP_ERROR(this->get_logger(), "Required parameters not defined");
            rclcpp::shutdown();
        }
    }
};