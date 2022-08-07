/**
 * @brief estimator odometry uORB-ROS2 topic listener and odometry ROS2 topic advertiser
 * @file estimator_odometry_adapter.cpp
 * @addtogroup examples
 * @author Florian Pix <florian.pix.97@outlook.com>
 */

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/estimator_odometry.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>

using std::placeholders::_1;
using namespace std::chrono_literals;
using Eigen::MatrixXd;

class EstimatorOdometryAdapter : public rclcpp::Node
{
public:
    MatrixXd pose_covariance = MatrixXd(6,6);
    MatrixXd twist_covariance = MatrixXd(6,6);

	EstimatorOdometryAdapter() : Node("estimator_odometry_adapter") {
        // create publisher for odometry ROS2 topic
	    publisher_odometry_ = this->create_publisher<nav_msgs::msg::Odometry>("/model/auv/odometry", 10);
		
        // create subscriber for estimator odometry uORB-ROS2 topic
        subscription_ = this->create_subscription<px4_msgs::msg::EstimatorOdometry>(
            "fmu/estimator_odometry/out", 
            10, 
            std::bind(&EstimatorOdometryAdapter::odometry_callback, this, _1)
        );
	}

    void odometry_callback(const px4_msgs::msg::EstimatorOdometry::SharedPtr msg)
    {
        switch (msg->local_frame) {
            case 0:
                RCLCPP_INFO(this->get_logger(), "local frame: NED earth-fixed frame");
                break;
            case 1:
                RCLCPP_INFO(this->get_logger(), "local frame: FRD earth-fixed frame, arbitrary heading reference");
                break;
            case 2:
                RCLCPP_INFO(this->get_logger(), "local frame: not aligned with the std frames of reference");
                break;
            case 3:
                RCLCPP_INFO(this->get_logger(), "local frame: using FRD body-fixed frame");
                break;
        }
        switch (msg->velocity_frame) {
            case 0:
                RCLCPP_INFO(this->get_logger(), "velocity frame: NED earth-fixed frame");
                break;
            case 1:
                RCLCPP_INFO(this->get_logger(), "velocity frame: FRD earth-fixed frame, arbitrary heading reference");
                break;
            case 2:
                RCLCPP_INFO(this->get_logger(), "velocity frame: not aligned with the std frames of reference");
                break;
            case 3:
                RCLCPP_INFO(this->get_logger(), "velocity frame: using FRD body-fixed frame");
                break;
        }
        auto odometry = nav_msgs::msg::Odometry();
        odometry.pose.pose.position.x = msg->x;
        odometry.pose.pose.position.y = msg->y;
        odometry.pose.pose.position.z = msg->z;
        odometry.pose.pose.orientation.x = msg->q[0];
        odometry.pose.pose.orientation.y = msg->q[1];
        odometry.pose.pose.orientation.z = msg->q[2];
        odometry.pose.pose.orientation.w = msg->q[3];

        pose_covariance << msg->pose_covariance[0], msg->pose_covariance[1], msg->pose_covariance[2], msg->pose_covariance[3], msg->pose_covariance[4], msg->pose_covariance[5],
                           msg->pose_covariance[1], msg->pose_covariance[6], msg->pose_covariance[7], msg->pose_covariance[8], msg->pose_covariance[9], msg->pose_covariance[10],
                           msg->pose_covariance[2], msg->pose_covariance[7], msg->pose_covariance[11], msg->pose_covariance[12], msg->pose_covariance[13], msg->pose_covariance[14],
                           msg->pose_covariance[3], msg->pose_covariance[8], msg->pose_covariance[12], msg->pose_covariance[15], msg->pose_covariance[16], msg->pose_covariance[17],
                           msg->pose_covariance[4], msg->pose_covariance[9], msg->pose_covariance[13], msg->pose_covariance[16], msg->pose_covariance[18], msg->pose_covariance[19],
                           msg->pose_covariance[5], msg->pose_covariance[10], msg->pose_covariance[14], msg->pose_covariance[17], msg->pose_covariance[19], msg->pose_covariance[20];

        for (int i = 0; i < 36; i++){
            odometry.pose.covariance[i] = pose_covariance(i);
        }

        odometry.twist.twist.linear.x = msg->vx;
        odometry.twist.twist.linear.y = msg->vy;
        odometry.twist.twist.linear.z = msg->vz;
        odometry.twist.twist.angular.x = msg->rollspeed;
        odometry.twist.twist.angular.y = msg->pitchspeed;
        odometry.twist.twist.angular.z = msg->yawspeed;
        
        twist_covariance << msg->velocity_covariance[0], msg->velocity_covariance[1], msg->velocity_covariance[2], msg->velocity_covariance[3], msg->velocity_covariance[4], msg->velocity_covariance[5],
                           msg->velocity_covariance[1], msg->velocity_covariance[6], msg->velocity_covariance[7], msg->velocity_covariance[8], msg->velocity_covariance[9], msg->velocity_covariance[10],
                           msg->velocity_covariance[2], msg->velocity_covariance[7], msg->velocity_covariance[11], msg->velocity_covariance[12], msg->velocity_covariance[13], msg->velocity_covariance[14],
                           msg->velocity_covariance[3], msg->velocity_covariance[8], msg->velocity_covariance[12], msg->velocity_covariance[15], msg->velocity_covariance[16], msg->velocity_covariance[17],
                           msg->velocity_covariance[4], msg->velocity_covariance[9], msg->velocity_covariance[13], msg->velocity_covariance[16], msg->velocity_covariance[18], msg->velocity_covariance[19],
                           msg->velocity_covariance[5], msg->velocity_covariance[10], msg->velocity_covariance[14], msg->velocity_covariance[17], msg->velocity_covariance[19], msg->velocity_covariance[20];

        for (int i = 0; i < 36; i++){
            odometry.twist.covariance[i] = twist_covariance(i);
        }

        odometry.header.stamp.sec = this->get_clock()->now().seconds();

        this->publisher_odometry_->publish(odometry);
    }

private:
    rclcpp::Subscription<px4_msgs::msg::EstimatorOdometry>::SharedPtr subscription_;

	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_odometry_;
};

int main(int argc, char *argv[])
{
	std::cout << "estimator_odometry_adapter node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<EstimatorOdometryAdapter>());

	rclcpp::shutdown();
	return 0;
}
