/**
 * @brief NavSatFix ROS2 topic listener and sensor_gps uORB topic advertiser
 * @file ugps_adapter.cpp
 * @addtogroup examples
 * @author Florian Pix <florian.pix.97@outlook.com>
 */

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/sensor_gps.hpp>
#include "sensor_msgs/msg/nav_sat_fix.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class UGPSAdapter : public rclcpp::Node
{
public:
	UGPSAdapter() : Node("ugps_adapter") {
        // create subscriber for ugps ROS2 topic
        subscription_ugps_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/ugps/gps_data", 
            10, 
            std::bind(&UGPSAdapter::ugps_callback, this, _1)
        );

        // create publisher for sensor_gps uORB topic
	    publisher_sensor_gps_ = this->create_publisher<px4_msgs::msg::SensorGPS>("/fmu/sensor_gps/in", 10);
		
        
	}
void ugps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        auto sensor_gps_msg = px4_msgs::msg::SensorGPS();

        auto now = this->get_clock()->now();
        // TODO should be microseconds
        sensor_gps_msg.timestamp = now.seconds(); 
        // TODO what is timestamp_sample ?
        sensor_gps_msg.timestamp_sample = (unsigned int) ((long long) now.nanoseconds() - (long long) ((long long) now.seconds() * (long long) 1e9));
        
        sensor_gps_msg.device_id = 111;

        sensor_gps_msg.lat = msg->latitude / 1e-7;
        sensor_gps_msg.lon = msg->longitude / 1e-7;
        sensor_gps_msg.alt = msg->altitude / 1e-3;
        sensor_gps_msg.alt_ellipsoid = 0.0; // not used
        sensor_gps_msg.s_variance_m_s = 1.0; // 
        sensor_gps_msg.c_variance_rad = 0.0; // yaw not measured by ugps
        sensor_gps_msg.fix_type = 0;
        sensor_gps_msg.eph = 0.3
        sensor_gps_msg.epv = 10.0;
        sensor_gps_msg.hdop = 0.0 // unknown
        sensor_gps_msg.vdop = 0.0 // unknwon 
        sensor_gps_msg.noise_per_ms
        sensor_gps_msg.automatic_gain_control
        sensor_gps_msg.jamming_state
        sensor_gps_msg.jamming_indicator
        sensor_gps_msg.vel_m_s
        sensor_gps_msg.vel_n_m_s
        sensor_gps_msg.vel_e_m_s
        sensor_gps_msg.vel_d_m_s
        sensor_gps_msg.cog_rad
        sensor_gps_msg.vel_ned_valid
        sensor_gps_msg.timestamp_time_relative
        sensor_gps_msg.time_utc_usec
        sensor_gps_msg.satellites_used
        sensor_gps_msg.heading
        sensor_gps_msg.heading_offset
        sensor_gps_msg.heading_accuracy
        sensor_gps_msg.rtcm_injection_rate
        sensor_gps_msg.selected_rtcm_instance


        this->publisher_sensor_gps_->publish(sensor_gps_msg);
    }

    private:

    rclcpp::Subscription<px4_msgs::msg::VehicleAirData>::SharedPtr subscription_;


	rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr publisher_water_temperature_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_barometer_depth_;
};

int main(int argc, char *argv[])
{
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<WaterTemperatureAdapter>());

	rclcpp::shutdown();
	return 0;
}
