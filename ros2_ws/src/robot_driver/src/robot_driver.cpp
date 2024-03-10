#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "mren_interfaces/msg/pico_sensor_output.hpp"
#include "mren_interfaces/msg/pico_motor_commands.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class RobotDriver : public rclcpp::Node {

	public:
		RobotDriver() : Node("robot_driver") {
			publisher_ = this->create_publisher<std_msgs::msg::String>("testpub", 10);
			pmc_publisher_ = this->create_publisher<mren_interfaces::msg::PicoMotorCommands>("pico_motor_commands", 10);
			pso_subscription_ = this->create_subscription<mren_interfaces::msg::PicoSensorOutput>("pico_sensor_output", 10, std::bind(&RobotDriver::pso_callback, this, _1));
			cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&RobotDriver::cmd_vel_callback, this, _1));
			width = 0.3; // ROBOT WIDTH
		}
	private:
		double width;
		rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
		rclcpp::Subscription<mren_interfaces::msg::PicoSensorOutput>::SharedPtr pso_subscription_;
		rclcpp::Publisher<mren_interfaces::msg::PicoMotorCommands>::SharedPtr pmc_publisher_;
		rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
		void pso_callback(const mren_interfaces::msg::PicoSensorOutput::SharedPtr msg) const {
			auto message = std_msgs::msg::String();
			message.data = "Hello, World! " + std::to_string(msg->left_wheel_velocity);
			publisher_->publish(message);
		}
		void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const {
			auto message = mren_interfaces::msg::PicoMotorCommands();
			double delta = msg->angular.z * width/2;
			message.left_wheel_velocity = msg->linear.x - delta;
			message.right_wheel_velocity = msg->linear.x + delta;
			pmc_publisher_->publish(message);
		}

};


int main(int argc, char* argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<RobotDriver>());
	rclcpp::shutdown();
	return 0;
}
