#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <string>

using namespace std::chrono_literals;

class NuSimNode : public rclcpp::Node
{
public:
  NuSimNode(double rate) : Node("nusim")
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
	reset_srv_ = this->create_service<std_srvs::srv::Empty>("~/reset",
	  [this](const std::shared_ptr<rmw_request_id_t> request_header,
		const std::shared_ptr<std_srvs::srv::Empty::Request> request,
		std::shared_ptr<std_srvs::srv::Empty::Response> response)
	  {
		(void)request_header;
		(void)request;
		// Reset timestep to zero
		timestep_ = 0;
	  });
  }

private:
	void timer_callback()
	{
		// Publish timestep
		auto msg = std_msgs::msg::UInt64();
		msg.data = timestep_;
		timestep_pub_->publish(msg);
		// Increment timestep
		timestep_++;
	}
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_pub_;
	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv_;
	uint64_t timestep_;
	double rate_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  double rate = 0;
  rclcpp::spin(std::make_shared<NuSimNode>(rate));
  rclcpp::shutdown();
  return 0;
}
