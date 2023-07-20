/**
 * @brief vehicle arming uORB-ROS2 topic listener and advertiser
 * @file vehicle_arm_toggle_adapter.cpp
 * @addtogroup examples
 * @author Håvard Spilde Ulaland <havard@mohntechnology.no>
 */

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include "std_msgs/msg/bool.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class ArmToggleAdapter : public rclcpp::Node
{
  public:
    px4_msgs::msg::VehicleCommand vehicle_command_msg = px4_msgs::msg::VehicleCommand();

    ArmToggleAdapter() : Node("arm_toggle_adapter")
    {
      vehicle_command_msg.command = VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM; //command = type of command, 400 = VEHICLE_CMD_COMPONENT_ARM_DISARM (1 for arm, 0 for disarm)
      publisher_arming_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("fmu/vehicle_command/in", 10);
      subscription_arm_toggle = this->create_subscription<std_msgs::msg::Bool>(
      "/armed_toggle", 10, std::bind(&ArmToggleAdapter::arming_callback, this, _1));
    }

    void arming_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
      if(msg->data){
        vehicle_command_msg.param1 = 1;
      }else {
        vehicle_command_msg.param1 = 0;
      }
      this->publisher_arming_->publish(vehicle_command_msg);
    }

  private:
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_arm_toggle;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr publisher_arming_;
};

int main(int argc, char *argv[])
{
	std::cout << "vechicle_arm_toggle node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ArmToggleAdapter>());

	rclcpp::shutdown();
	return 0;
}