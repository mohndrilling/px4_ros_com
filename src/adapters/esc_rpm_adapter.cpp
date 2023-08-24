/**
 * @brief esc_rpm uORB topic listener and esc_rpm ROS2 topic advertiser
 * @file esc_rpm_adapter.cpp
 * @addtogroup examples
 * @author Florian Pix <florian.pix.97@outlook.com>
 */

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/esc_rpm.hpp>
#include <px4_msgs/msg/pwm_input.h>
#include <std_msgs/msg/float64_multi_array.hpp>


using std::placeholders::_1;
using namespace std::chrono_literals;

class EscRpmAdapter : public rclcpp::Node
{
public:
	EscRpmAdapter() : Node("esc_rpm_adapter") {
        //Set size of published msg
        esc_rpm_msg_.data.resize(8);
        // create publisher for ESC rpm ROS2 topic
	    publisher_esc_rpm_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/auv/esc_rpm", 10);
        
        // create subscriber for sensor baro uORB-ROS2 topic
        subscription_esc_rpm_ = this->create_subscription<px4_msgs::msg::EscRpm>(
            "fmu/esc_rpm/out", 
            10, 
            std::bind(&EscRpmAdapter::esc_rpm_callback, this, _1)

        );
	}

    void esc_rpm_callback(const px4_msgs::msg::EscRpm::SharedPtr msg)
        {
           
            int index = msg->esc_address-1;

            esc_rpm_msg_.data[index] = msg->esc_rpm;
            this->publisher_esc_rpm_->publish(esc_rpm_msg_);
        }

private:
    std_msgs::msg::Float64MultiArray esc_rpm_msg_;

    rclcpp::Subscription<px4_msgs::msg::EscRpm>::SharedPtr subscription_esc_rpm_;
	rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_esc_rpm_;
};

int main(int argc, char *argv[])
{
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<EscRpmAdapter>());
	rclcpp::shutdown();
	return 0;
}
