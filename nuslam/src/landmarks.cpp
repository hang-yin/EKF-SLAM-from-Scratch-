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

class LandmarksNode : public rclcpp::Node
{
public:
  SLAMnode()
  : Node("landmarks")
  {
    // Declare parameters
    declare_parameter("min_lidar_range", 0.12);
    declare_parameter("max_lidar_range", 3.5);
    declare_parameter("angle_increment", 0.0174532924);
    lidar_range_max_ = get_parameter("max_lidar_range").as_double();
    lidar_range_min_ = get_parameter("min_lidar_range").as_double();
    lidar_angle_increment_ = get_parameter("angle_increment").as_double();

    // Initialize publishers, subscribers, and timers
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan",
                                                                       10,
                                                                       std::bind(&LandmarksNode::scan_callback,
                                                                                 this,
                                                                                 std::placeholders::_1));
    landmark_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/fitted_landmarks", 10);
    /*
    auto rate = 5.0;
    main_timer_ = this->create_wall_timer(1s/rate, std::bind(&LandmarksNode::main_timer_callback, this));
    */
  }

private:
  // Declare variables
  double lidar_range_max_;
  double lidar_range_min_;
  double lidar_angle_increment_;
  // rclcpp::TimerBase::SharedPtr main_timer_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr landmark_pub_;

  // Declare functions
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // Get the scan data
    std::vector<double> ranges = msg->ranges;
    std::vector<double> angles;
    for (int i = 0; i < ranges.size(); i++)
    {
      angles.push_back(msg->angle_min + i * msg->angle_increment);
    }

    // Find the landmarks
    std::vector<std::vector<double>> landmarks = find_landmarks(ranges, angles);

    // Publish the landmarks
    visualization_msgs::msg::MarkerArray marker_array;
    for (int i = 0; i < landmarks.size(); i++)
    {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "red/base_scan";
      marker.header.stamp = msg->header.stamp;
      marker.ns = "landmarks";
      marker.id = i;
      marker.type = visualization_msgs::msg::Marker::CYLINDER;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = landmarks[i][0];
      marker.pose.position.y = landmarks[i][1];
      marker.pose.position.z = 0.125;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.1;
      marker.scale.y = 0.1;
      marker.scale.z = 0.1;
      marker.color.a = 1.0;
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 1.0;
      marker_array.markers.push_back(marker);
    }
    landmark_pub_->publish(marker_array);
  }
  
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LandmarksNode>());
  rclcpp::shutdown();
  return 0;
}
