#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nusim/srv/Teleport.srv"
#include <chrono>
#include <functional>
#include <memory>
#include <string>

using namespace std::chrono_literals;

class NuSimNode : public rclcpp::Node
{
public:
  NuSimNode(double rate, float x0, float y0, float theta0) : Node("nusim")
  {
	// Initialize timestep to zero
	timestep_ = 0;

	// Create publisher for timestep
	timestep_pub_ = this->create_publisher<std_msgs::msg::UInt64()>("~/timestep", 10);

	// Set rate
	if(rate > 0) {
	  rate_ = rate;
	} else {
	  rate_ = 200.0;
	}

	// Create timer at frequency of rate
	timer_ = this->create_wall_timer(1s/rate_, std::bind(&NuSimNode::timer_callback, this));

	// Create reset service of empty type
	reset_srv_ = this->create_service<std_srvs::srv::Empty>("~/reset", std::bind(&NuSimNode::reset_callback, this));

	// Create teleport service of Teleport type
	teleport_srv_ = this->create_service<nusim::srv::Teleport>("~/teleport", std::bind(&NuSimNode::teleport_callback, this));
	
	// Set initial pose
	x0_ = x0;
	y0_ = y0;
	theta0_ = theta0;
	q0_.setRPY(0, 0, theta0_);

	// Create transform broadcaster from the frame "nusim/world" to the frame "red/base_footprint"
	// tf2_ros::TransformBroadcaster broadcaster(this);
	// geometry_msgs::msg::TransformStamped transformStamped;
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
		auto msg = std_msgs::msg::UInt64();
		msg.data = timestep_;
		timestep_pub_->publish(msg);
		// Broadcast transform
		transformStamped.header.stamp = this->now();
		broadcaster_.sendTransform(transformStamped_);
		// Increment timestep
		timestep_++;
	}
	void reset_callback(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
						std::shared_ptr<std_srvs::srv::Empty::Response> response){
		// Reset timestep to zero
		timestep_ = 0;
		// Reset transform
		transform_stamped_.transform.translation.x = x0_;
		transform_stamped_.transform.translation.y = y0_;
		transform_stamped_.transform.rotation.x = q0_.x();
		transform_stamped_.transform.rotation.y = q0_.y();
		transform_stamped_.transform.rotation.z = q0_.z();
		transform_stamped_.transform.rotation.w = q0_.w();
	}
	void teleport_callback(const std::shared_ptr<nusim::srv::Teleport::Request> request,
						   std::shared_ptr<nusim::srv::Teleport::Response> response){
		// Set transform
		transform_stamped_.transform.translation.x = request->x;
		transform_stamped_.transform.translation.y = request->y;
		tf2::Quaternion q;
		q.setRPY(0, 0, request->theta);
		transform_stamped_.transform.rotation.x = q.x();
		transform_stamped_.transform.rotation.y = q.y();
		transform_stamped_.transform.rotation.z = q.z();
		transform_stamped_.transform.rotation.w = q.w();
	}
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_pub_;
	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv_;
	tf2_ros::TransformBroadcaster broadcaster_;
	geometry_msgs::msg::TransformStamped transformStamped_;
	uint64_t timestep_;
	double rate_;
	float x0_;
	float y0_;
	float theta0_;
	tf2::Quaternion q0_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  double rate = 0;
  rclcpp::spin(std::make_shared<NuSimNode>(rate, -0.6, 0.8, 1.57));
  rclcpp::shutdown();
  return 0;
}
