/****************************************************************************
 *
 * Copyright 2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief Debug Vect uORB topic adverstiser example
 * @file actuator_controls_advertiser.cpp
 * @addtogroup examples
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
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
