#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Quaternion.h"

using namespace std::chrono_literals;

class ImuDriver : public rclcpp::Node {
public:
    ImuDriver() : Node("imu_driver") {
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
        timer_ = this->create_wall_timer(100ms, std::bind(&ImuDriver::publish_imu_data, this));
    }

private:
    void publish_imu_data() {
        // Dummy sensor readings (replace these with actual data from IMU)
        double ax = 0.0, ay = 0.0, az = 9.81;  // Accelerometer (m/s²)
        double mx = 1.0, my = 0.0, mz = 0.0;   // Magnetometer (arbitrary units)

        // Compute Roll and Pitch from accelerometer
        double roll = std::atan2(ay, az);  // φ
        double pitch = std::atan2(-ax, std::sqrt(ay * ay + az * az));  // θ

        // Tilt-compensated Magnetometer readings for Yaw
        double mx_comp = mx * std::cos(pitch) + mz * std::sin(pitch);
        double my_comp = mx * std::sin(roll) * std::sin(pitch) + my * std::cos(roll) - mz * std::sin(roll) * std::cos(pitch);
        double yaw = std::atan2(my_comp, mx_comp);  // ψ

        // Convert Roll, Pitch, Yaw to Quaternion
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);  // Roll, Pitch, Yaw -> Quaternion

        // Prepare IMU message
        auto msg = sensor_msgs::msg::Imu();
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "imu_link";

        // Set orientation
        msg.orientation.x = q.x();
        msg.orientation.y = q.y();
        msg.orientation.z = q.z();
        msg.orientation.w = q.w();

        msg.angular_velocity.x = 0.0;
        msg.angular_velocity.y = 0.0;
        msg.angular_velocity.z = 0.0;

        msg.linear_acceleration.x = 0.0;
        msg.linear_acceleration.y = 0.0;
        msg.linear_acceleration.z = 9.81;

        imu_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published IMU data");
    }

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuDriver>());
    rclcpp::shutdown();
    return 0;
}
