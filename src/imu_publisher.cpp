#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <unistd.h>
#include <cstring>
#include <wiringPi.h>
#include <vector>
#include "imu_cpp/ImuData.h"
#include "imu_cpp/ADIS16460.h"


// GPIO Pins for Data Ready (DR)
#define DR0_PIN 25
#define DR1_PIN 26

#define RST0_PIN 12
#define RST1_PIN 13

/*  imu 0: top imu at end-effector
    imu 1: bottom imu at swinging end of passive joint
*/


class IMUPublisher : public rclcpp::Node {
private:
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
    std::shared_ptr<ADIS16460> imu;
    std::string imu_frame;

    static IMUPublisher* instance;  // Store the instance pointer

public:
    IMUPublisher() : Node("imu_publisher") {

        this->declare_parameter<std::string>("imu_frame", "imu_link");
        imu_frame = this->get_parameter("imu_frame").as_string();

        instance = this; 

        // Initialize GPIO
        if (wiringPiSetupGpio() == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize WiringPi.");
            rclcpp::shutdown();
        }

        // Create the IMU object
        imu = std::make_shared<ADIS16460>(SPI_DEVICE_1, DR1_PIN, RST1_PIN);

        // Reset and configure the IMU
        imu->reset();
        usleep(10000); // Wait after reset
        if (!imu->writeRegister(MSC_CTRL, MSC_CTRL_VAL)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to configure IMU.");
            rclcpp::shutdown();
        } else {
            RCLCPP_INFO(this->get_logger(), "IMU successfully configured.");
        }

        // Create a publisher for the IMU data
        imu_pub = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);

        // Attach interrupt handler for IMU data ready
        wiringPiISR(DR1_PIN, INT_EDGE_RISING, &IMUPublisher::imu_handler_static);

        RCLCPP_INFO(this->get_logger(), "IMU Publisher started.");
    }

private:
    static void imu_handler_static() {
        if (instance) {
            instance->readAndPublishIMUData();
        }
    }


    void readAndPublishIMUData() {
        IMUData data;
        if (!imu->readData(data)) {
            RCLCPP_WARN(this->get_logger(), "Failed to read IMU data.");
            return;
        }

        // Create IMU ROS message
        auto imu_msg = sensor_msgs::msg::Imu();
        imu_msg.header.stamp = this->get_clock()->now();
        imu_msg.header.frame_id = imu_frame;

        // Fill in Gyro data
        imu_msg.angular_velocity.x = data.x_gyro;
        imu_msg.angular_velocity.y = data.y_gyro;
        imu_msg.angular_velocity.z = data.z_gyro;

        // Fill in Accel data
        imu_msg.linear_acceleration.x = data.x_accel;
        imu_msg.linear_acceleration.y = data.y_accel;
        imu_msg.linear_acceleration.z = data.z_accel;

        // Publish IMU data
        imu_pub->publish(imu_msg);
    }
};

IMUPublisher* IMUPublisher::instance = nullptr;

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IMUPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
