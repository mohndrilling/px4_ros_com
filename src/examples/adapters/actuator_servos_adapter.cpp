/**
 * @brief led ROS2 topic listener and actuator servos uORB topic advertiser
 * @file actuator_servos_adapter.cpp
 * @addtogroup examples
 * @author Florian Pix <florian.pix.97@outlook.com>
 */

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/actuator_servos.hpp>
#include "std_msgs/msg/float64.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class ActuatorServosAdapter : public rclcpp::Node
{
public:
    // timer interval in seconds

    px4_msgs::msg::ActuatorServos actuator_servos = px4_msgs::msg::ActuatorServos();

	ActuatorServosAdapter() : Node("actuator_servos_adapter") {
        // init actuator_servos with 0
        for (int i = 0; i < 8; i++){
            actuator_servos.control[i] = 0.0;
        }

        // create publisher for actuator servos uORB topic
        #ifdef ROS_DEFAULT_API
		    publisher_ = this->create_publisher<px4_msgs::msg::ActuatorServos>("/fmu/actuator_servos/in", 10);
        #else
		    publisher_ = this->create_publisher<px4_msgs::msg::ActuatorServos>("/fmu/actuator_servos/in");
        #endif
		
        // create subscribers for thruster ROS2 topics
        led_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
            "/led", 
            10, 
            std::bind(&ActuatorServosAdapter::led_callback, this, _1)
        );
	}

    void led_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        actuator_servos.control[0] = msg->data;
        actuator_servos.timestamp = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()).time_since_epoch().count();

        this->publisher_->publish(actuator_servos);

        RCLCPP_INFO(
            this->get_logger(),
            "\033 [ Publishing to /fmu/actuator_servos/in: time: %llu 0: %.2f 1: %.2f 2: %.2f 3: %.2f 4: %.2f 5: %.2f 6: %.2f 7: %.2f] \033",
            actuator_servos.timestamp,
            actuator_servos.control[0],
            actuator_servos.control[1],
            actuator_servos.control[2],
            actuator_servos.control[3],
            actuator_servos.control[4],
            actuator_servos.control[5],
            actuator_servos.control[6],
            actuator_servos.control[7]
        );
    }

private:
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr led_subscription_;

	rclcpp::Publisher<px4_msgs::msg::ActuatorServos>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
	std::cout << "Starting actuator_servos adapter node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ActuatorServosAdapter>());

	rclcpp::shutdown();
	return 0;
}
