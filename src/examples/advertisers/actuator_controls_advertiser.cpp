/**
 * @brief actuator_controls uORB topic adverstiser example
 * @file actuator_controls_advertiser.cpp
 * @addtogroup examples
 * @author Florian Pix <florian.pix.97@outlook.com>
 */

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/actuator_controls.hpp>

using namespace std::chrono_literals;

class ActuatorControlsAdvertiser : public rclcpp::Node
{
public:
    int motor_index = 0;
    int motor_count = 8;
    double thrust = -1.0;
	ActuatorControlsAdvertiser() : Node("actuator_controls_advertiser") {
#ifdef ROS_DEFAULT_API
		publisher_ = this->create_publisher<px4_msgs::msg::ActuatorControls>("/fmu/actuator_controls/in", 10);
#else
		publisher_ = this->create_publisher<px4_msgs::msg::ActuatorControls>("/fmu/actuator_controls/in");
#endif
		auto timer_callback =
		[this]()->void {
		    if(thrust == -1.0){
		        RCLCPP_INFO(this->get_logger(), "Testing motor %i", motor_index);
		    }
		    if (motor_index <= motor_count){
                auto actuator_controls = px4_msgs::msg::ActuatorControls();

                actuator_controls.timestamp = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()).time_since_epoch().count();
                actuator_controls.control[0] = 0.0;
                actuator_controls.control[1] = 0.0;
                actuator_controls.control[2] = 0.0;
                actuator_controls.control[3] = 0.0;
                actuator_controls.control[4] = 0.0;
                actuator_controls.control[5] = 0.0;
                actuator_controls.control[6] = 0.0;
                actuator_controls.control[7] = 0.0;
                actuator_controls.control[motor_index] = thrust;
                thrust += 0.01;
                if (thrust >= 1.0){
                    motor_index++;
                    if (motor_index > 8){
                        actuator_controls.control[8] = 0.0;
                        RCLCPP_INFO(this->get_logger(), "Last message", motor_index);
                        RCLCPP_INFO(
                            this->get_logger(),
                            "\033 [ Publishing to /fmu/actuator_controls/in: time: %llu 0: %.2f 1: %.2f 2: %.2f 3: %.2f 4: %.2f 5: %.2f 6: %.2f 7: %.2f 8: %.2f ] \033",
                            actuator_controls.timestamp,
                            actuator_controls.control[0],
                            actuator_controls.control[1],
                            actuator_controls.control[2],
                            actuator_controls.control[3],
                            actuator_controls.control[4],
                            actuator_controls.control[5],
                            actuator_controls.control[6],
                            actuator_controls.control[7]
                        );
                        this->publisher_->publish(actuator_controls);
                        exit(0);
                    }
                    thrust = -1.0;
                }

                RCLCPP_INFO(
                    this->get_logger(),
                    "\033 [ Publishing to /fmu/actuator_controls/in: time: %llu 0: %.2f 1: %.2f 2: %.2f 3: %.2f 4: %.2f 5: %.2f 6: %.2f 7: %.2f 8: %.2f ] \033",
                    actuator_controls.timestamp,
                    actuator_controls.control[0],
                    actuator_controls.control[1],
                    actuator_controls.control[2],
                    actuator_controls.control[3],
                    actuator_controls.control[4],
                    actuator_controls.control[5],
                    actuator_controls.control[6],
                    actuator_controls.control[7]
                );
                this->publisher_->publish(actuator_controls);
            }
		};
		timer_ = this->create_wall_timer(15ms, timer_callback);
	}

private:
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<px4_msgs::msg::ActuatorControls>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
	std::cout << "Starting actuator_controls advertiser node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ActuatorControlsAdvertiser>());

	rclcpp::shutdown();
	return 0;
}
