#include <cmath>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_stamped.h>
#include "servo.h"

class StatePublisher : public rclcpp::Node
{
public:
    StatePublisher(Servo (&servos)[4][3])
        : Node("state_publisher"), servos_(servos)
    {
        auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
        joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", qos_profile);
        broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        RCLCPP_INFO(this->get_logger(), "State Publisher started");

        degree_ = M_PI / 180.0;
        loop_rate_ = 30;

        // robot state
        tilt_ = 0.0;
        tinc_ = degree_;
        swivel_ = 0.0;
        angle_ = 0.0;
        height_ = 0.0;
        hinc_ = 0.005;

        // message declarations
        odom_trans_.header.frame_id = "odom";
        odom_trans_.child_frame_id = "axis";
        joint_state_.name = {"swivel", "tilt", "periscope"};

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000 / loop_rate_),
            std::bind(&StatePublisher::update, this));
    }

private:
    void update()
    {
        // update joint_state
        joint_state_.header.stamp = this->get_clock()->now();
        joint_state_.position = {
            servos_[0][0].read(), servos_[0][1].read(), servos_[0][2].read(),
            servos_[1][0].read(), servos_[1][1].read(), servos_[1][2].read(),
            servos_[2][0].read(), servos_[2][1].read(), servos_[2][2].read(),
            servos_[3][0].read(), servos_[3][1].read(), servos_[3][2].read()
        };

        // update transform
        // (moving in a circle with radius=2)
        odom_trans_.header.stamp = this->get_clock()->now();
        odom_trans_.transform.translation.x = std::cos(angle_) * 2;
        odom_trans_.transform.translation.y = std::sin(angle_) * 2;
        odom_trans_.transform.translation.z = 0.7;
        odom_trans_.transform.rotation = euler_to_quaternion(0, 0, angle_ + M_PI / 2); // roll, pitch, yaw
        broadcaster_->sendTransform(odom_trans_);

        // send the joint state and transform
        joint_pub_->publish(joint_state_);

        // Create new robot state
        tilt_ += tinc_;
        if (tilt_ < -0.5 || tilt_ > 0.0)
        {
            tinc_ *= -1;
        }
        height_ += hinc_;
        if (height_ > 0.2 || height_ < 0.0)
        {
            hinc_ *= -1;
        }
        swivel_ += degree_;
        angle_ += degree_ / 4;
    }

    geometry_msgs::msg::Quaternion euler_to_quaternion(double roll, double pitch, double yaw)
    {
        geometry_msgs::msg::Quaternion q;
        q.x = std::sin(roll / 2) * std::cos(pitch / 2) * std::cos(yaw / 2) - std::cos(roll / 2) * std::sin(pitch / 2) * std::sin(yaw / 2);
        q.y = std::cos(roll / 2) * std::sin(pitch / 2) * std::cos(yaw / 2) + std::sin(roll / 2) * std::cos(pitch / 2) * std::sin(yaw / 2);
        q.z = std::cos(roll / 2) * std::cos(pitch / 2) * std::sin(yaw / 2) - std::sin(roll / 2) * std::sin(pitch / 2) * std::cos(yaw / 2);
        q.w = std::cos(roll / 2) * std::cos(pitch / 2) * std::cos(yaw / 2) + std::sin(roll / 2) * std::sin(pitch / 2) * std::sin(yaw / 2);
        return q;
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
    sensor_msgs::msg::JointState joint_state_;
    geometry_msgs::msg::TransformStamped odom_trans_;
    rclcpp::TimerBase::SharedPtr timer_;

    double degree_;
    int loop_rate_;
    double tilt_, tinc_, swivel_, angle_, height_, hinc_;
    Servo (&servos_)[4][3];
};

// int main(int argc, char *argv[])
// {
//     rclcpp::init(argc, argv);
//     Servo servos[4][3];
//     auto node = std::make_shared<StatePublisher>(servos);
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }
