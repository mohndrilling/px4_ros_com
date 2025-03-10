/****************************************************************************
 *
 * Copyright 2017 Proyectos y Sistemas de Mantenimiento SL (eProsima).
 *           2018 PX4 Pro Development Team. All rights reserved.
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
 * @brief Sensor Combined uORB topic listener example
 * @file sensor_combined_listener.cpp
 * @addtogroup examples
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 * @author Vicente Monge
 */

 #include <rclcpp/rclcpp.hpp>
 #include <px4_msgs/msg/sensor_combined.hpp>
 #include <px4_msgs/msg/sensor_mag.hpp>
 #include <std_msgs/msg/float32_multi_array.hpp>

using std::placeholders::_1;

/**
 * @brief Sensor Combined & sensor mag uORB topic data callback
 */
class SensorListener : public rclcpp::Node
{
public:
	SensorListener() : Node("sensor_listener") {
        //Set size of published msg
        gyro_msg_.data.resize(3);
        accel_msg_.data.resize(3);
        mag_msg_.data.resize(3);
        
        // create publishers for ROS2 topic sensors
        publisher_gyro_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/auv/gyro", 10);
        publisher_accel_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/auv/accel", 10);
        publisher_mag_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/auv/mag", 10);

        // create subscriber for uORB-ROS2 topic sensors
		sensor_combined_sub_ = this->create_subscription<px4_msgs::msg::SensorCombined>(
			"fmu/sensor_combined/out",10,std::bind(&SensorListener::sensorCombinedCallback, this, _1));
        mag_sub_ = this->create_subscription<px4_msgs::msg::SensorMag>(
			"fmu/sensor_mag/out",10,std::bind(&SensorListener::sensorMagCallback, this, _1));
	}

    void sensorCombinedCallback(const px4_msgs::msg::SensorCombined::SharedPtr msg)
        {
            gyro_msg_.data[0] = msg->gyro_rad[0];
            gyro_msg_.data[1] = msg->gyro_rad[1];
            gyro_msg_.data[2] = msg->gyro_rad[2];
            accel_msg_.data[0] = msg->accelerometer_m_s2[0];
            accel_msg_.data[1] = msg->accelerometer_m_s2[1];
            accel_msg_.data[2] = msg->accelerometer_m_s2[2];
            this->publisher_gyro_->publish(gyro_msg_);
            this->publisher_accel_->publish(accel_msg_);
        }
    void sensorMagCallback(const px4_msgs::msg::SensorMag::SharedPtr msg)
        {
            mag_msg_.data[0] = msg->x;
            mag_msg_.data[1] = msg->y;
            mag_msg_.data[2] = msg->z;
            this->publisher_mag_->publish(mag_msg_);
        }

    

private:
    std_msgs::msg::Float32MultiArray gyro_msg_, accel_msg_, mag_msg_;

	rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr sensor_combined_sub_;
    rclcpp::Subscription<px4_msgs::msg::SensorMag>::SharedPtr mag_sub_;

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_gyro_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_accel_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_mag_;
};

int main(int argc, char *argv[])
{
	std::cout << "Starting sensor_combined listener node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<SensorListener>());

	rclcpp::shutdown();
	return 0;
}


// #ifdef ROS_DEFAULT_API
//             10,
// #endif
// 			[this](const px4_msgs::msg::SensorCombined::UniquePtr msg) {
// 			std::cout << "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n";
// 			std::cout << "RECEIVED SENSOR COMBINED DATA"   << std::endl;
// 			std::cout << "============================="   << std::endl;
// 			std::cout << "ts: "          << msg->timestamp    << std::endl;
// 			std::cout << "gyro_rad[0]: " << msg->gyro_rad[0]  << std::endl;
// 			std::cout << "gyro_rad[1]: " << msg->gyro_rad[1]  << std::endl;
// 			std::cout << "gyro_rad[2]: " << msg->gyro_rad[2]  << std::endl;
// 			std::cout << "gyro_integral_dt: " << msg->gyro_integral_dt << std::endl;
// 			std::cout << "accelerometer_timestamp_relative: " << msg->accelerometer_timestamp_relative << std::endl;
// 			std::cout << "accelerometer_m_s2[0]: " << msg->accelerometer_m_s2[0] << std::endl;
// 			std::cout << "accelerometer_m_s2[1]: " << msg->accelerometer_m_s2[1] << std::endl;
// 			std::cout << "accelerometer_m_s2[2]: " << msg->accelerometer_m_s2[2] << std::endl;
// 			std::cout << "accelerometer_integral_dt: " << msg->accelerometer_integral_dt << std::endl;