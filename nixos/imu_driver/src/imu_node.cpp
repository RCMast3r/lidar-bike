#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Quaternion.h"

#include <driver.hpp>
using namespace std::chrono_literals;

class ImuDriver : public rclcpp::Node {
public:
    ImuDriver() : Node("imu_driver"), _driver("/dev/i2c-1") {
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
        timer_ = this->create_wall_timer(5ms, std::bind(&ImuDriver::publish_imu_data, this));
    }

private:
    void publish_imu_data() {
        auto data = _driver.sample_data();
        
        // Dummy sensor readings (replace these with actual data from IMU)
        double ax = data.accel_data.x, ay = data.accel_data.y, az = data.accel_data.z;  // Accelerometer (m/s²)
        double mx = data.mag_data.x, my = data.mag_data.y, mz = data.mag_data.z;   // Magnetometer (arbitrary units)

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

        msg.angular_velocity.x = data.gyr_data.x;
        msg.angular_velocity.y = data.gyr_data.y;
        msg.angular_velocity.z = data.gyr_data.z;

        msg.linear_acceleration.x =  data.accel_data.x;
        msg.linear_acceleration.y =  data.accel_data.y;
        msg.linear_acceleration.z =  data.accel_data.z;

        imu_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published IMU data");
    }
private:
    I2CDriver _driver;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuDriver>());
    rclcpp::shutdown();
    return 0;
}
