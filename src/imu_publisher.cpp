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
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu0_pub;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu1_pub;
    std::shared_ptr<ADIS16460> imu0;
    std::shared_ptr<ADIS16460> imu1;
    std::string imu_frame;

    static IMUPublisher* instance;  // Store the instance pointer
    bool toggle_flag;  //Toogle between IMU0 and IMU1

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
        imu0 = std::make_shared<ADIS16460>(SPI_DEVICE_0, DR0_PIN, RST0_PIN);
        // imu1 = std::make_shared<ADIS16460>(SPI_DEVICE_1, DR1_PIN, RST1_PIN);

        // Reset and configure the IMU
        imu0->reset();
        imu0->setup();

        // usleep(1000000);
        // imu1->reset();
        // imu1->setup();
        // if (!imu->writeRegister(MSC_CTRL, MSC_CTRL_VAL)) {
        //     RCLCPP_ERROR(this->get_logger(), "Failed to configure IMU.");
        //     rclcpp::shutdown();
        // } else {
        //     RCLCPP_INFO(this->get_logger(), "IMU successfully configured.");
        // }

        // Create a publisher for the IMU data
        //imu0_pub = this->create_publisher<sensor_msgs::msg::Imu>("/imu0/data", 10);
        // imu1_pub = this->create_publisher<sensor_msgs::msg::Imu>("/imu1/data", 10);

        // Attach interrupt handler for IMU data ready
        //wiringPiISR(DR0_PIN, INT_EDGE_RISING, &IMUPublisher::imu0_handler_static);
        // wiringPiISR(DR1_PIN, INT_EDGE_RISING, &IMUPublisher::imu1_handler_static);

        RCLCPP_INFO(this->get_logger(), "IMU Publisher started.");
    }

private:
    static void imu0_handler_static() {
        if (instance) {
            instance->readAndPublishIMU0Data();
        }
    }
    static void imu1_handler_static() {
        if (instance) {
            instance->readAndPublishIMU1Data();
        }
    }


    void readAndPublishIMU0Data() {
        IMUData data0;
        if (!imu0->readData(data0)) {
            RCLCPP_WARN(this->get_logger(), "Failed to read IMU data.");
            return;
        }

        // Create IMU ROS message
        auto imu0_msg = sensor_msgs::msg::Imu();
        imu0_msg.header.stamp = this->get_clock()->now();
        imu0_msg.header.frame_id = imu_frame;

        // Fill in Gyro data
        imu0_msg.angular_velocity.x = data0.x_gyro;
        imu0_msg.angular_velocity.y = data0.y_gyro;
        imu0_msg.angular_velocity.z = data0.z_gyro;

        // Fill in Accel data
        imu0_msg.linear_acceleration.x = data0.x_accel;
        imu0_msg.linear_acceleration.y = data0.y_accel;
        imu0_msg.linear_acceleration.z = data0.z_accel;

        // Publish IMU data
        imu0_pub->publish(imu0_msg);
    }

    void readAndPublishIMU1Data() {
        IMUData data1;
        if (!imu1->readData(data1)) {
            RCLCPP_WARN(this->get_logger(), "Failed to read IMU data.");
            return;
        }

        // Create IMU ROS message
        auto imu1_msg = sensor_msgs::msg::Imu();
        imu1_msg.header.stamp = this->get_clock()->now();
        imu1_msg.header.frame_id = imu_frame;

        // Fill in Gyro data
        imu1_msg.angular_velocity.x = data1.x_gyro;
        imu1_msg.angular_velocity.y = data1.y_gyro;
        imu1_msg.angular_velocity.z = data1.z_gyro;

        // Fill in Accel data
        imu1_msg.linear_acceleration.x = data1.x_accel;
        imu1_msg.linear_acceleration.y = data1.y_accel;
        imu1_msg.linear_acceleration.z = data1.z_accel;

        // Publish IMU data
        imu1_pub->publish(imu1_msg);
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


