#include "visualization_msgs/msg/marker_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nusim/srv/teleport.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

using namespace std::chrono_literals;

class NuSimNode : public rclcpp::Node
{
public:
  NuSimNode() : Node("nusim")
  {
	// Declare parameters
	this->declare_parameter("rate", 200.0);
	this->declare_parameter("x0", 0.0);
	this->declare_parameter("y0", 0.0);
	this->declare_parameter("theta0", 0.0);
	this->declare_parameter("obstacles_x", std::vector<double>());
	this->declare_parameter("obstacles_y", std::vector<double>());
	this->declare_parameter("obstacles_r", 0.0);

	// Check obstacles input, if length of vectors are not equal, exit node
	std::vector<double> obstacles_x = this->get_parameter("obstacles_x").as_double_array();
	std::vector<double> obstacles_y = this->get_parameter("obstacles_y").as_double_array();
	if(obstacles_x.size() != obstacles_y.size()) {
	  RCLCPP_ERROR(this->get_logger(), "Length of obstacles vectors are not equal");
	  rclcpp::shutdown();
	}

	float obstacles_r = this->get_parameter("obstacles_r").as_double();
	// Check obstacles input, if radius is negative, exit node
	if (obstacles_r < 0) {
	  RCLCPP_ERROR(this->get_logger(), "Radius of obstacles cannot be negative");
	  rclcpp::shutdown();
	}

	// Create publisher for obstacles
	obstacles_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/obstacles", 10);

	// Initialize marker array
	marker_array_.markers.resize(obstacles_x.size());
	for (int i = 0; i < int(obstacles_x.size()); i++) {
	  marker_array_.markers[i].header.frame_id = "nusim/world";
	  marker_array_.markers[i].header.stamp = this->get_clock()->now();
	  marker_array_.markers[i].ns = "obstacles";
	  marker_array_.markers[i].id = i;
	  marker_array_.markers[i].type = visualization_msgs::msg::Marker::CYLINDER;
	  marker_array_.markers[i].action = visualization_msgs::msg::Marker::ADD;
	  marker_array_.markers[i].pose.position.x = obstacles_x[i];
	  marker_array_.markers[i].pose.position.y = obstacles_y[i];
	  marker_array_.markers[i].pose.position.z = 0;
	  marker_array_.markers[i].pose.orientation.x = 0.0;
	  marker_array_.markers[i].pose.orientation.y = 0.0;
	  marker_array_.markers[i].pose.orientation.z = 0.0;
	  marker_array_.markers[i].pose.orientation.w = 1.0;
	  marker_array_.markers[i].color.a = 1.0;
	  marker_array_.markers[i].color.r = 1.0;
	  marker_array_.markers[i].color.g = 0.0;
	  marker_array_.markers[i].color.b = 0.0;
	  marker_array_.markers[i].scale.x = 2*obstacles_r;
	  marker_array_.markers[i].scale.y = 2*obstacles_r;
	  marker_array_.markers[i].scale.z = 0.25;
	}

	// Initialize timestep to zero
	timestep_ = 0;

	// Create publisher for timestep
	timestep_pub_ = this->create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);

	// Set rate
	rate_ = this->get_parameter("rate").as_double();
	if(rate_ <= 0) {
	  rate_ = 200.0;
	}

	// Create timer at frequency of rate
	timer_ = this->create_wall_timer(1s/rate_, std::bind(&NuSimNode::timer_callback, this));

	// Create reset service of empty type
	reset_srv_ = this->create_service<std_srvs::srv::Empty>("~/reset",
															std::bind(&NuSimNode::reset_callback,
																	  this,
																	  std::placeholders::_1,
																	  std::placeholders::_2));

	// Create teleport service of Teleport type
	teleport_srv_ = this->create_service<nusim::srv::Teleport>("~/teleport",
															   std::bind(&NuSimNode::teleport_callback,
															   			 this,
																		 std::placeholders::_1,
																		 std::placeholders::_2));
	
	// Set initial pose
	x0_ = this->get_parameter("x0").as_double();
	y0_ = this->get_parameter("y0").as_double();
	theta0_ = this->get_parameter("theta0").as_double();
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

  }

private:
	void timer_callback(){
		// Publish timestep
		std_msgs::msg::UInt64 msg;
		msg.data = timestep_;
		timestep_pub_->publish(msg);
		// Broadcast transform
		transformStamped_.header.stamp = this->get_clock()->now();
		broadcaster_->sendTransform(transformStamped_);
		// Publish marker array
		obstacles_pub_->publish(marker_array_);
		// Increment timestep
		timestep_++;
	}
	void reset_callback(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
						std::shared_ptr<std_srvs::srv::Empty::Response> response){
		(void)request;
		(void)response;
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
						   std::shared_ptr<nusim::srv::Teleport::Response> response){
		(void)request;
		(void)response;
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
	float x0_;
	float y0_;
	float theta0_;
	tf2::Quaternion q0_;
	visualization_msgs::msg::MarkerArray marker_array_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  std::vector<float> obstacles_x = {-0.6, 0.7, 0.5};
  std::vector<float> obstacles_y = {-0.8, -0.7, 0.9};
  rclcpp::spin(std::make_shared<NuSimNode>());
  rclcpp::shutdown();
  return 0;
}
