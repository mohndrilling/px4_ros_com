/**
 * @brief actuator_servos uORB topic adverstiser example
 * @file actuator_servos_advertiser.cpp
 * @addtogroup examples
 * @author Florian Pix <florian.pix.97@outlook.com>
 */

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/actuator_servos.hpp>

using namespace std::chrono_literals;

class ActuatorServosAdvertiser : public rclcpp::Node
{
public:
    int led_pin;
    double brightness;
	
    ActuatorServosAdvertiser() : Node("actuator_servos_advertiser",
	    rclcpp::NodeOptions()
        .allow_undeclared_parameters(true)
        .automatically_declare_parameters_from_overrides(true)) 
    {
#ifdef ROS_DEFAULT_API
		publisher_ = this->create_publisher<px4_msgs::msg::ActuatorServos>("/fmu/actuator_servos/in", 10);
#else
		publisher_ = this->create_publisher<px4_msgs::msg::ActuatorServos>("/fmu/actuator_servos/in");
#endif
        led_pin = this->get_parameter("led_pin").as_int();
        brightness = this->get_parameter("brightness").as_double();

        auto timer_callback =
		[this]()->void {
            /*
            Servo control message
            uint64 timestamp			# time since system start (microseconds)
            uint64 timestamp_sample	    # the timestamp the data this control response is based on was sampled

            uint8 NUM_CONTROLS = 8
            float32[8] control # range: [-1, 1], where 1 means maximum positive position,
                               # -1 maximum negative,
                               # and NaN maps to disarmed

            */
            auto actuator_servos = px4_msgs::msg::ActuatorServos();

            actuator_servos.timestamp = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()).time_since_epoch().count();

            for (int i = 0; i < 8; i++){
                actuator_servos.control[i] = 0.0;
            }

            actuator_servos.control[this->led_pin] = this->brightness;

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
            this->publisher_->publish(actuator_servos);
        };
        timer_ = this->create_wall_timer(1000ms, timer_callback);
	}

private:
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<px4_msgs::msg::ActuatorServos>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
	std::cout << "Starting actuator_servos advertiser node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ActuatorServosAdvertiser>());

	rclcpp::shutdown();
	return 0;
}
