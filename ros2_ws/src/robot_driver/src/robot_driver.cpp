#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "mren_interfaces/msg/pico_sensor_output.hpp"
#include "mren_interfaces/msg/pico_motor_commands.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

class RobotDriver : public rclcpp::Node {

	public:
		RobotDriver() : Node("robot_driver") {
			odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

			pmc_publisher_ = this->create_publisher<mren_interfaces::msg::PicoMotorCommands>("pico_motor_commands", 10);

			pso_subscription_ = this->create_subscription<mren_interfaces::msg::PicoSensorOutput>("pico_sensor_output", 10, std::bind(&RobotDriver::pso_callback, this, _1));

			cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&RobotDriver::cmd_vel_callback, this, _1));

			tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

			width = 0.3; // ROBOT WIDTH
			dt = 0.1;
		}
	private:
		double width;
		double dt;
		double x;
		double y;
		double th;
		double vx;
		double vy;
		double vth;
		rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
		rclcpp::Subscription<mren_interfaces::msg::PicoSensorOutput>::SharedPtr pso_subscription_;
		rclcpp::Publisher<mren_interfaces::msg::PicoMotorCommands>::SharedPtr pmc_publisher_;
		rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
		std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
		void pso_callback(const mren_interfaces::msg::PicoSensorOutput::SharedPtr msg) const {
			geometry_msgs::msg::TransformStamped t;
			t.header.stamp = now();
			t.header.frame_id = "odom";
			t.child_frame_id = "base_link";
			tf2::Quaternion q;
			q.setRPY(0,0,0); // TODO Set Y to angle
			(void)msg;
			auto odom = nav_msgs::msg::Odometry();
			odom.header.stamp = now();
			odom.header.frame_id = "odom";
			odom.child_frame_id = "base_link";
			//TODO
			tf_broadcaster_->sendTransform(t);
			odom_publisher_->publish(odom);
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
