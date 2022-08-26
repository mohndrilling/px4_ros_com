/**
 * @brief baro uORB topic listener and temperature ROS2 topic advertiser
 * @file actuator_motors_adapter.cpp
 * @addtogroup examples
 * @author Florian Pix <florian.pix.97@outlook.com>
 */

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/sensor_baro.hpp>
#include "sensor_msgs/msg/fluid_pressure.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class AUVPressureAdapter : public rclcpp::Node
{
public:

	AUVPressureAdapter() : Node("auv_inner_pressure_adapter") {
        // create publisher for water temperature ROS2 topic
	    publisher_ = this->create_publisher<sensor_msgs::msg::FluidPressure>("/auv_inner_pressure", 10);
		
        // create subscriber for sensor baro uORB-ROS2 topic
        subscription_ = this->create_subscription<px4_msgs::msg::SensorBaro>(
            "fmu/sensor_baro/out", 
            10, 
            std::bind(&AUVPressureAdapter::sensor_baro_callback, this, _1)
        );
	}
void sensor_baro_callback(const px4_msgs::msg::SensorBaro::SharedPtr msg)
    {
        
        auto pressure_msg = sensor_msgs::msg::FluidPressure();
        pressure_msg.fluid_pressure = msg->pressure;
        pressure_msg.header.frame_id = "base_link";
        auto now = this->get_clock()->now();
        pressure_msg.header.stamp.sec = now.seconds();
        pressure_msg.header.stamp.nanosec = (unsigned int) ((long long) now.nanoseconds() - (long long) ((long long) now.seconds() * (long long) 1e9));
        
        this->publisher_->publish(pressure_msg);

        
    }

    private:

    rclcpp::Subscription<px4_msgs::msg::SensorBaro>::SharedPtr subscription_;


	rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<AUVPressureAdapter>());

	rclcpp::shutdown();
	return 0;
}
