/**
 * @brief baro uORB topic listener and temperature ROS2 topic advertiser
 * @file actuator_motors_adapter.cpp
 * @addtogroup examples
 * @author Florian Pix <florian.pix.97@outlook.com>
 */

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_air_data.hpp>
#include "sensor_msgs/msg/temperature.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class WaterTemperatureAdapter : public rclcpp::Node
{
public:

	WaterTemperatureAdapter() : Node("water_temperature_adapter") {
        // create publisher for water temperature ROS2 topic
	    publisher_water_temperature_ = this->create_publisher<sensor_msgs::msg::Temperature>("/water_temperature", 10);
		
        // create subscriber for sensor baro uORB-ROS2 topic
        subscription_ = this->create_subscription<px4_msgs::msg::VehicleAirData>(
            "fmu/vehicle_air_data/out", 
            10, 
            std::bind(&WaterTemperatureAdapter::vehicle_air_data_callback, this, _1)
        );
	}
void vehicle_air_data_callback(const px4_msgs::msg::VehicleAirData::SharedPtr msg)
    {
        
        auto temp_msg = sensor_msgs::msg::Temperature();
        temp_msg.temperature = msg->baro_temp_celcius;
        temp_msg.header.frame_id = "map";
        auto now = this->get_clock()->now();
        temp_msg.header.stamp.sec = now.seconds();
        temp_msg.header.stamp.nanosec = (unsigned int) ((long long) now.nanoseconds() - (long long) ((long long) now.seconds() * (long long) 1e9));
        
        this->publisher_water_temperature_->publish(temp_msg);

        
    }

    private:

    rclcpp::Subscription<px4_msgs::msg::VehicleAirData>::SharedPtr subscription_;


	rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr publisher_water_temperature_;
};

int main(int argc, char *argv[])
{
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<WaterTemperatureAdapter>());

	rclcpp::shutdown();
	return 0;
}
