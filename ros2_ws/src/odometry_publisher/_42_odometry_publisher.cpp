#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include "tf2_ros/transform_broadcaster.h"

using std::placeholders::_1;

class OdometryPublisher : public rclcpp::Node
{
public:
    OdometryPublisher()
    : Node("odometry_publisher")
    {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
        qos.best_effort();

        sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "esp32s3/telemetry",
            qos,
            std::bind(&OdometryPublisher::telemetry_callback, this, _1)
        );

        bat_pub_ = this->create_publisher<std_msgs::msg::Float32>(
            "battery_voltage",
            10
        );

        joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "joint_states",
            10
        );

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "odom",
            10
        );

        tf_broadcaster_ =
            std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        TICKS_PER_REV = 7 * 4 * 298;
        ENCODER_WRAP = pow(2, 31);
        WHEEL_RADIUS = 0.0325;
        WHEEL_CIRCUMFERENCE = 2.0 * M_PI * WHEEL_RADIUS;
        DEG_TO_RAD = M_PI / 180.0;
        GYRO_DEADZONE_RAD = 0.01;

        x_ = 0.0;
        y_ = 0.0;
        yaw_ = 0.0;
        initialized_ = false;

        joint_msg_.name = {
            "left_wheel_joint",
            "right_wheel_joint"
        };

        odom_msg_.header.frame_id = "odom";
        odom_msg_.child_frame_id = "base_footprint";

        tf_msg_.header.frame_id = "odom";
        tf_msg_.child_frame_id = "base_footprint";

        RCLCPP_INFO(this->get_logger(), "Odometry Publisher Started");
    }

private:
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr bat_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std_msgs::msg::Float32 bat_msg_;
    sensor_msgs::msg::JointState joint_msg_;
    nav_msgs::msg::Odometry odom_msg_;
    geometry_msgs::msg::TransformStamped tf_msg_;

    double TICKS_PER_REV;
    double ENCODER_WRAP;
    double WHEEL_RADIUS;
    double WHEEL_CIRCUMFERENCE;
    double DEG_TO_RAD;
    double GYRO_DEADZONE_RAD;

    double x_;
    double y_;
    double yaw_;
    double prev_left_pos_;
    double prev_right_pos_;
    rclcpp::Time prev_time_;
    bool initialized_;

    void telemetry_callback(
        const std_msgs::msg::Int32MultiArray::SharedPtr msg
    )
    {
        if (msg->data.size() != 6)
        {
            return;
        }
        
        const int LEFT_ENCODER_SIGN = -1;
        const int RIGHT_ENCODER_SIGN = -1;
        const int GYRO_SIGN = 1;

        int32_t left_pos = msg->data[0];
        int32_t right_pos = msg->data[1];
        int32_t left_vel = msg->data[2];
        int32_t right_vel = msg->data[3];
        int32_t gyro_deg_s_100 = msg->data[4];
        int32_t bat_voltage_100 = msg->data[5];

        // 실제 로봇 움직임과 RViz odometry 방향을 맞추기 위한 부호 보정
        left_pos = LEFT_ENCODER_SIGN * left_pos;
        right_pos = RIGHT_ENCODER_SIGN * right_pos;
        left_vel = LEFT_ENCODER_SIGN * left_vel;
        right_vel = RIGHT_ENCODER_SIGN * right_vel;
        gyro_deg_s_100 = GYRO_SIGN * gyro_deg_s_100;

        auto now = this->get_clock()->now();

        if (!initialized_)
        {
            prev_left_pos_ = static_cast<double>(left_pos);
            prev_right_pos_ = static_cast<double>(right_pos);
            prev_time_ = now;
            initialized_ = true;
            return;
        }

        double dt = (now - prev_time_).seconds();
        prev_time_ = now;

        if (dt <= 0.0 || dt > 1.0)
        {
            return;
        }

        double left_rpm = left_vel / 100.0;
        double right_rpm = right_vel / 100.0;
        double gyro_deg_s = gyro_deg_s_100 / 100.0;
        double gyro_rad_s = gyro_deg_s * DEG_TO_RAD;

        if (fabs(gyro_rad_s) < GYRO_DEADZONE_RAD)
        {
            gyro_rad_s = 0.0;
        }

        double battery_voltage =
            static_cast<double>(bat_voltage_100) / 100.0;

        double delta_left =
            static_cast<double>(left_pos) - prev_left_pos_;

        double delta_right =
            static_cast<double>(right_pos) - prev_right_pos_;

        if (delta_left > ENCODER_WRAP)
        {
            delta_left -= 2 * ENCODER_WRAP;
        }
        else if (delta_left < -ENCODER_WRAP)
        {
            delta_left += 2 * ENCODER_WRAP;
        }

        if (delta_right > ENCODER_WRAP)
        {
            delta_right -= 2 * ENCODER_WRAP;
        }
        else if (delta_right < -ENCODER_WRAP)
        {
            delta_right += 2 * ENCODER_WRAP;
        }

        prev_left_pos_ = static_cast<double>(left_pos);
        prev_right_pos_ = static_cast<double>(right_pos);

        double left_speed =
            (left_rpm / 60.0) * WHEEL_CIRCUMFERENCE;

        double right_speed =
            (right_rpm / 60.0) * WHEEL_CIRCUMFERENCE;

        double linear_velocity =
            (left_speed + right_speed) / 2.0;

        double left_dist =
            (delta_left / TICKS_PER_REV) * WHEEL_CIRCUMFERENCE;

        double right_dist =
            (delta_right / TICKS_PER_REV) * WHEEL_CIRCUMFERENCE;

        double dist =
            (left_dist + right_dist) / 2.0;

        double delta_yaw =
            gyro_rad_s * dt;

        x_ += dist * cos(yaw_ + delta_yaw * 0.5);
        y_ += dist * sin(yaw_ + delta_yaw * 0.5);
        yaw_ += delta_yaw;
        yaw_ = atan2(sin(yaw_), cos(yaw_));

        double half_yaw = yaw_ * 0.5;
        double qz = sin(half_yaw);
        double qw = cos(half_yaw);

        bat_msg_.data = battery_voltage;
        bat_pub_->publish(bat_msg_);

        double left_rad =
            (static_cast<double>(left_pos) / TICKS_PER_REV)
            * 2.0 * M_PI;

        double right_rad =
            (static_cast<double>(right_pos) / TICKS_PER_REV)
            * 2.0 * M_PI;

        double left_rad_s =
            (left_rpm * 2.0 * M_PI) / 60.0;

        double right_rad_s =
            (right_rpm * 2.0 * M_PI) / 60.0;

        joint_msg_.header.stamp = now;
        joint_msg_.position = {left_rad, right_rad};
        joint_msg_.velocity = {left_rad_s, right_rad_s};
        joint_pub_->publish(joint_msg_);

        odom_msg_.header.stamp = now;
        odom_msg_.pose.pose.position.x = x_;
        odom_msg_.pose.pose.position.y = y_;
        odom_msg_.pose.pose.position.z = 0.0;
        odom_msg_.pose.pose.orientation.x = 0.0;
        odom_msg_.pose.pose.orientation.y = 0.0;
        odom_msg_.pose.pose.orientation.z = qz;
        odom_msg_.pose.pose.orientation.w = qw;
        odom_msg_.twist.twist.linear.x = linear_velocity;
        odom_msg_.twist.twist.angular.z = gyro_rad_s;
        odom_pub_->publish(odom_msg_);

        tf_msg_.header.stamp = now;
        tf_msg_.transform.translation.x = x_;
        tf_msg_.transform.translation.y = y_;
        tf_msg_.transform.translation.z = 0.0;
        tf_msg_.transform.rotation.x = 0.0;
        tf_msg_.transform.rotation.y = 0.0;
        tf_msg_.transform.rotation.z = qz;
        tf_msg_.transform.rotation.w = qw;
        tf_broadcaster_->sendTransform(tf_msg_);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<OdometryPublisher>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
