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

#define CS0_PIN 8
#define CS1_PIN 7

/*  imu 0: top imu at end-effector
    imu 1: bottom imu at swinging end of passive joint
*/


class IMUPublisher : public rclcpp::Node {
private:
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu0_pub;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu1_pub;

    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<ADIS16460> imu0;
    std::shared_ptr<ADIS16460> imu1;

    std::string imu_frame;

    static IMUPublisher* instance;  // Static instance pointer
    bool toggle_flag;  // Toggle between IMU0 and IMU1

public:
    IMUPublisher() : Node("imu_publisher") {
        this->declare_parameter<std::string>("imu_frame", "imu_link");
        imu_frame = this->get_parameter("imu_frame").as_string();


        // Initialize GPIO
        if (wiringPiSetupGpio() == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize WiringPi.");
            rclcpp::shutdown();
        }

        // Create the IMU object
        imu0 = std::make_shared<ADIS16460>(SPI_DEVICE_0, DR0_PIN, RST0_PIN, CS0_PIN);
        imu1 = std::make_shared<ADIS16460>(SPI_DEVICE_1, DR1_PIN, RST1_PIN, CS1_PIN);

        // Reset and configure the IMU
        imu0->reset();
        imu0->setup(false);
        usleep(1000);
        imu1->reset();
        imu1->setup(true);

        // Create a publisher for the IMU data
        // imu0_pub = this->create_publisher<sensor_msgs::msg::Imu>("/adis16460_dev0", rclcpp::SensorDataQoS());
        // imu1_pub = this->create_publisher<sensor_msgs::msg::Imu>("/adis16460_dev1", rclcpp::SensorDataQoS());

        rclcpp::QoS custom_qos = rclcpp::QoS(rclcpp::KeepLast(1))
                                     .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
                                     .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE)
                                     .history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);

        imu0_pub = this->create_publisher<sensor_msgs::msg::Imu>("/adis16460_dev0", custom_qos);
        imu1_pub = this->create_publisher<sensor_msgs::msg::Imu>("/adis16460_dev1", custom_qos);

        // Set up ISR for rising edge on DR1_PIN
        wiringPiISR(DR1_PIN, INT_EDGE_RISING, &IMUPublisher::imu_handler_static); 

        instance = this;  // Set the static instance pointer to this object
        RCLCPP_INFO(this->get_logger(), "IMU Publisher started.");
    }

    // Static handler function for rising edge of DR1_PIN
    static void imu_handler_static() {
        if(instance == nullptr){
            return;
        }
        if (instance->toggle_flag) {
            instance->readAndPublishIMUData(instance->imu1, instance->imu1_pub, "imu1");
        } else {
            instance->readAndPublishIMUData(instance->imu0, instance->imu0_pub, "imu0");
        }

        // instance->readAndPublishIMUData(instance->imu1, instance->imu1_pub, "imu1");

        // Toggle the IMU flag for the next rising edge
        instance->toggle_flag = !(instance->toggle_flag);
    }

private:
    void readAndPublishIMUData(std::shared_ptr<ADIS16460> imu, rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher, const std::string &frame_id) {
        IMUData data;
        if (!imu->readData(data)) {
            // RCLCPP_WARN(this->get_logger(), "Failed to read IMU data from %s.", frame_id.c_str());
            return;
        }

        // RCLCPP_INFO(this->get_logger(), "[%s] Gyro: [%.4f, %.4f, %.4f] Accel: [%.4f, %.4f, %.4f]", 
        //             frame_id.c_str(), 
        //             data.x_gyro, data.y_gyro, data.z_gyro, 
        //             data.x_accel, data.y_accel, data.z_accel);

        auto imu_msg = sensor_msgs::msg::Imu();
        imu_msg.header.stamp = this->get_clock()->now();
        imu_msg.header.frame_id = frame_id;

        imu_msg.angular_velocity.x = data.x_gyro;
        imu_msg.angular_velocity.y = data.y_gyro;
        imu_msg.angular_velocity.z = data.z_gyro;

        imu_msg.linear_acceleration.x = data.x_accel;
        imu_msg.linear_acceleration.y = data.y_accel;
        imu_msg.linear_acceleration.z = data.z_accel;

        publisher->publish(imu_msg);
    }
};

// Initialize the static instance pointer
IMUPublisher* IMUPublisher::instance = nullptr;

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IMUPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
