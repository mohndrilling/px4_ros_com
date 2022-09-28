/**
 * @brief esc_status uORB topic listener and esc_status ROS2 topic advertiser
 * @file esc_status_adapter.cpp
 * @addtogroup examples
 * @author Florian Pix <florian.pix.97@outlook.com>
 */

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/esc_status.hpp>
#include <px4_msgs/msg/esc_report.hpp>
#include <auv_msgs/msg/esc_status.hpp>
#include <auv_msgs/msg/esc_report.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

class EscStatusAdapter : public rclcpp::Node
{
public:
	EscStatusAdapter() : Node("esc_status_adapter") {
        // create publisher for ESC status ROS2 topic
	    publisher_esc_status_ = this->create_publisher<auv_msgs::msg::EscStatus>("/esc_status", 10);
        
        // create subscriber for sensor baro uORB-ROS2 topic
        subscription_esc_status_ = this->create_subscription<px4_msgs::msg::EscStatus>(
            "fmu/esc_status/out", 
            10, 
            std::bind(&EscStatusAdapter::esc_status_callback, this, _1)
        );
	}

    void esc_status_callback(const px4_msgs::msg::EscStatus::SharedPtr msg)
        {
            auto esc_status_msg = auv_msgs::msg::EscStatus();
            // TODO
            this->publisher_esc_status_->publish(esc_status_msg);
        }

private:
    rclcpp::Subscription<px4_msgs::msg::EscStatus>::SharedPtr subscription_esc_status_;
	rclcpp::Publisher<auv_msgs::msg::EscStatus>::SharedPtr publisher_esc_status_;
};

int main(int argc, char *argv[])
{
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<EscStatusAdapter>());
	rclcpp::shutdown();
	return 0;
}
