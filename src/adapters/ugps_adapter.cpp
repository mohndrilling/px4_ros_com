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
        bool lat_offset_set = false;
        float lat_offset = 0.0;
        bool long_offset_set = false;
        float long_offset = 0.0;

        UGPSAdapter() : Node("ugps_adapter") {
            // create subscriber for ugps ROS2 topic
            subscription_ugps_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
                "/ugps/gps_data", 
                10, 
                std::bind(&UGPSAdapter::ugps_callback, this, _1)
            );

            // create publisher for sensor_gps uORB topic
            publisher_sensor_gps_ = this->create_publisher<px4_msgs::msg::SensorGps>("/fmu/sensor_gps/in", 10);
        }

        void ugps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
        {
            auto sensor_gps_msg = px4_msgs::msg::SensorGps();

            auto now = this->get_clock()->now();
            // TODO should be microseconds
            sensor_gps_msg.timestamp = (unsigned int) ((long long) now.nanoseconds() - (long long) ((long long) now.seconds() * (long long) 1e9));
            
            sensor_gps_msg.device_id = 111;

            if (!long_offset_set)
            {
                long_offset = msg->longitude;
                long_offset_set = true;
            }
            if (!lat_offset_set)
            {
                lat_offset = msg->latitude;
                lat_offset_set = true;
            }

            sensor_gps_msg.lat = msg->latitude / 1e-7;
            sensor_gps_msg.lon = msg->longitude / 1e-7;
            sensor_gps_msg.alt = msg->altitude / 1e-3;
            sensor_gps_msg.alt_ellipsoid = 0.0; // not used
            sensor_gps_msg.s_variance_m_s = 0.0; 
            sensor_gps_msg.c_variance_rad = 0.0; // yaw not measured by ugps
            sensor_gps_msg.fix_type = 3; // 3D fix required for EKF
            sensor_gps_msg.eph = 0.05;
            sensor_gps_msg.epv = 10.0;
            sensor_gps_msg.hdop = 0.0; // unknown
            sensor_gps_msg.vdop = 0.0; // unknwon 
            sensor_gps_msg.noise_per_ms = 0; // TODO ?
            sensor_gps_msg.automatic_gain_control = 1; // not used
            sensor_gps_msg.jamming_state = 1; // JAMMING_STATE_OK always
            sensor_gps_msg.jamming_indicator = 0; // no jamming
            sensor_gps_msg.vel_n_m_s = 0.0; // pos2vel(msg->latitude, lat_offset);
            sensor_gps_msg.vel_e_m_s = 0.0; // pos2vel(msg->longitude, long_offset);
            sensor_gps_msg.vel_d_m_s = 0.0; // pos2vel(msg->altitude, 0.0);
            sensor_gps_msg.vel_m_s = 0.0; // sensor_gps_msg.vel_n_m_s + sensor_gps_msg.vel_e_m_s;
            sensor_gps_msg.cog_rad = 0.0; // TODO velocity heading
            sensor_gps_msg.vel_ned_valid = true; // assume velocity is always valid
            sensor_gps_msg.timestamp_time_relative = (unsigned int) ((long long) now.nanoseconds() - (long long) ((long long) now.seconds() * (long long) 1e9));
            sensor_gps_msg.time_utc_usec = (unsigned int) ((long long) now.nanoseconds() - (long long) ((long long) now.seconds() * (long long) 1e9));
            sensor_gps_msg.satellites_used = 4;
            sensor_gps_msg.heading = nan("");
            sensor_gps_msg.heading_offset = nan("");
            sensor_gps_msg.heading_accuracy = nan("");

            this->publisher_sensor_gps_->publish(sensor_gps_msg);
        }

        float pos2vel(float vel, float offset)
        {
            float pos = 0.0;

            // lat/long degrees to meters
            pos = 2 * 3.14159265359  * 6371000 * vel / 360;

            // remove offset
            pos = vel - offset;

            // TODO median
            // TODO lowpass
            // TODO 1st derivative

            return pos;
        }

    private:
        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscription_ugps_;
        rclcpp::Publisher<px4_msgs::msg::SensorGps>::SharedPtr publisher_sensor_gps_;
};

int main(int argc, char *argv[])
{
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<UGPSAdapter>());
	rclcpp::shutdown();
	return 0;
}
