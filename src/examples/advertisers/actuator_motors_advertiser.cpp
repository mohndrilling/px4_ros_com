/**
 * @brief actuator_motors uORB topic adverstiser example
 * @file actuator_motors_advertiser.cpp
 * @addtogroup examples
 * @author Florian Pix <florian.pix.97@outlook.com>
 */

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/actuator_motors.hpp>

using namespace std::chrono_literals;

class ActuatorMotorsAdvertiser : public rclcpp::Node
{
public:
    int led_pin;
    double brightness;
	ActuatorMotorsAdvertiser() : Node(
	    "actuator_motors_advertiser",
	    rclcpp::NodeOptions()
            .allow_undeclared_parameters(true)
            .automatically_declare_parameters_from_overrides(true)) {
#ifdef ROS_DEFAULT_API
		publisher_ = this->create_publisher<px4_msgs::msg::ActuatorMotors>("/fmu/actuator_motors/in", 10);
#else
		publisher_ = this->create_publisher<px4_msgs::msg::ActuatorMotors>("/fmu/actuator_motors/in");
#endif
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
            auto actuator_motors = px4_msgs::msg::ActuatorMotors();

            actuator_motors.timestamp = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()).time_since_epoch().count();

            auto controls = this->get_parameter("controls").as_double_array();
            for (int i = 0; i < 8; i++){
                actuator_motors.control[i] = controls[i];
            }

            actuator_motors.reversible_flags = 255;

            this->publisher_->publish(actuator_motors);

            RCLCPP_INFO(
                this->get_logger(),
                "\033 [ Publishing to /fmu/actuator_motors/in: time: %llu 0: %.2f 1: %.2f 2: %.2f 3: %.2f 4: %.2f 5: %.2f 6: %.2f 7: %.2f 8: %.2f ] \033",
                actuator_motors.timestamp,
                actuator_motors.control[0],
                actuator_motors.control[1],
                actuator_motors.control[2],
                actuator_motors.control[3],
                actuator_motors.control[4],
                actuator_motors.control[5],
                actuator_motors.control[6],
                actuator_motors.control[7]
            );
        };
        timer_ = this->create_wall_timer(1000ms, timer_callback);
	}

private:
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<px4_msgs::msg::ActuatorMotors>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
	std::cout << "Starting actuator_motors advertiser node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ActuatorMotorsAdvertiser>());

	rclcpp::shutdown();
	return 0;
}
