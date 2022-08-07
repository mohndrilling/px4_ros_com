/**
 * @brief thruster ROS2 topic listener and actuator controls uORB topic advertiser
 * @file actuator_controls_adapter.cpp
 * @addtogroup examples
 * @author Florian Pix <florian.pix.97@outlook.com>
 */

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/actuator_controls.hpp>
#include "std_msgs/msg/float64.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class ActuatorControlsAdapter : public rclcpp::Node
{
public:
    // timer interval in seconds
    double timer_interval = 0.5;
    
    px4_msgs::msg::ActuatorControls actuator_controls = px4_msgs::msg::ActuatorControls();

	ActuatorControlsAdapter() : Node("actuator_controls_adapter") {
        // init actuator_controls with 0
        actuator_controls.control[0] = 0.0;
        actuator_controls.control[1] = 0.0;
        actuator_controls.control[2] = 0.0;
        actuator_controls.control[3] = 0.0;
        actuator_controls.control[4] = 0.0;
        actuator_controls.control[5] = 0.0;
        actuator_controls.control[6] = 0.0;
        actuator_controls.control[7] = 0.0;

        // create publisher for actuator controls uORB topic

        #ifdef ROS_DEFAULT_API
		    publisher_ = this->create_publisher<px4_msgs::msg::ActuatorControls>("/fmu/actuator_controls/in", 10);
        #else
		    publisher_ = this->create_publisher<px4_msgs::msg::ActuatorControls>("/fmu/actuator_controls/in");
        #endif
		
        // create subscribers for thruster ROS2 topics

        fur_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
            "/fmu/thruster_fur_cmd", 
            10, 
            std::bind(&ActuatorControlsAdapter::fur_callback, this, _1)
        );
        
        ful_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
            "/fmu/thruster_ful_cmd", 
            10, 
            std::bind(&ActuatorControlsAdapter::ful_callback, this, _1)
        );

        fdr_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
            "/fmu/thruster_fdr_cmd", 
            10, 
            std::bind(&ActuatorControlsAdapter::fdr_callback, this, _1)
        );

        fdl_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
            "/fmu/thruster_fdl_cmd", 
            10, 
            std::bind(&ActuatorControlsAdapter::fdl_callback, this, _1)
        );

        bur_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
            "/fmu/thruster_bur_cmd", 
            10, 
            std::bind(&ActuatorControlsAdapter::bur_callback, this, _1)
        );

        bul_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
            "/fmu/thruster_bul_cmd", 
            10, 
            std::bind(&ActuatorControlsAdapter::bul_callback, this, _1)
        );

        bdr_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
            "/fmu/thruster_bdr_cmd", 
            10, 
            std::bind(&ActuatorControlsAdapter::bdr_callback, this, _1)
        );

        bdl_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
            "/fmu/thruster_bdl_cmd", 
            10, 
            std::bind(&ActuatorControlsAdapter::bdl_callback, this, _1)
        );

		timer_ = rclcpp::create_timer(this, this->get_clock(), rclcpp::Duration(timer_interval*1e9), std::bind(&ActuatorControlsAdapter::timer_callback, this));
	}

    void fur_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        actuator_controls.control[0] = msg->data;
    }

    void ful_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        actuator_controls.control[1] = msg->data;
    }

    void fdr_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        actuator_controls.control[2] = msg->data;
    }

    void fdl_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        actuator_controls.control[3] = msg->data;
    }

    void bur_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        actuator_controls.control[4] = msg->data;
    }

    void bul_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        actuator_controls.control[5] = msg->data;
    }

    void bdr_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        actuator_controls.control[6] = msg->data;
    }

    void bdl_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        actuator_controls.control[7] = msg->data;
    }

    void timer_callback()
    {
        actuator_controls.timestamp = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()).time_since_epoch().count();

        this->publisher_->publish(actuator_controls);

        RCLCPP_INFO(
            this->get_logger(),
            "\033 [ Publishing to /fmu/actuator_controls/in: time: %llu 0: %.2f 1: %.2f 2: %.2f 3: %.2f 4: %.2f 5: %.2f 6: %.2f 7: %.2f] \033",
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
    }

private:
	rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr fur_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr ful_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr fdr_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr fdl_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr bur_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr bul_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr bdr_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr bdl_subscription_;

	rclcpp::Publisher<px4_msgs::msg::ActuatorControls>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
	std::cout << "Starting actuator_controls adapter node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ActuatorControlsAdapter>());

	rclcpp::shutdown();
	return 0;
}
