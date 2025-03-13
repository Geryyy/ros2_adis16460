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
#include <mutex>
#include "imu_cpp/ImuData.h"
#include "imu_cpp/ADIS16460.h"

// GPIO Pins for Data Ready (DR)
#define DR0_PIN 25
#define DR1_PIN 26
#define RST0_PIN 12
#define RST1_PIN 13
#define CS0_PIN 8
#define CS1_PIN 7
#define SYNC_PIN 6

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

    // IMU data storage
    IMUData imu_data_0;
    IMUData imu_data_1;

    // Mutex for data protection
    std::mutex imu_data_mutex;

    static IMUPublisher* instance;  // Static instance pointer
    bool toggle_flag;  // Toggle between IMU0 and IMU1

public:
    IMUPublisher() : Node("imu_publisher"), toggle_flag(true) {
        // Initialize GPIO
        if (wiringPiSetupGpio() == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize WiringPi.");
            rclcpp::shutdown();
        }
        // pinMode(SYNC_PIN, OUTPUT);
        // digitalWrite(SYNC_PIN, HIGH);

        this->declare_parameter<std::string>("imu_frame", "imu_link");
        imu_frame = this->get_parameter("imu_frame").as_string();

        

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
        // imu0_pub = this->create_publisher<sensor_msgs::msg::Imu>("/adis16460_dev0", 10);
        // imu1_pub = this->create_publisher<sensor_msgs::msg::Imu>("/adis16460_dev1", 10);
        
        imu0_pub = this->create_publisher<sensor_msgs::msg::Imu>("/adis16460_dev0", rclcpp::SensorDataQoS());
        imu1_pub = this->create_publisher<sensor_msgs::msg::Imu>("/adis16460_dev1", rclcpp::SensorDataQoS());
        
        // Set up ISR for rising edge on DR1_PIN
        wiringPiISR(DR1_PIN, INT_EDGE_RISING, &IMUPublisher::imu_handler_static);

        instance = this;  // Set the static instance pointer to this object
        RCLCPP_INFO(this->get_logger(), "IMU Publisher started.");

        // Timer for publishing IMU data periodically
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1), 
            std::bind(&IMUPublisher::publishIMUData, this)
        );
    }

    // Static handler function for rising edge of DR1_PIN
    static void imu_handler_static() {
        if (instance == nullptr) {
            return;
        }

        // Lock the mutex to update the IMU data safely
        std::lock_guard<std::mutex> lock(instance->imu_data_mutex);

        if (instance->toggle_flag) {
            instance->readIMUData(instance->imu1, instance->imu_data_1);
        } else {
            instance->readIMUData(instance->imu0, instance->imu_data_0);
        }

        // Toggle the IMU flag for the next rising edge
        instance->toggle_flag = !(instance->toggle_flag);
    }

private:
    void readIMUData(std::shared_ptr<ADIS16460> imu, IMUData &data) {
        if (!imu->readData(data)) {
            RCLCPP_WARN(this->get_logger(), "Failed to read IMU data.");
        }
    }

    void publishIMUData() {
        // Lock the mutex to safely access the IMU data
        std::lock_guard<std::mutex> lock(imu_data_mutex);

        // Publish IMU 0 data
        auto imu_msg0 = sensor_msgs::msg::Imu();
        imu_msg0.header.stamp = this->get_clock()->now();
        imu_msg0.header.frame_id = "imu0";

        imu_msg0.angular_velocity.x = imu_data_0.x_gyro;
        imu_msg0.angular_velocity.y = imu_data_0.y_gyro;
        imu_msg0.angular_velocity.z = imu_data_0.z_gyro;

        imu_msg0.linear_acceleration.x = imu_data_0.x_accel;
        imu_msg0.linear_acceleration.y = imu_data_0.y_accel;
        imu_msg0.linear_acceleration.z = imu_data_0.z_accel;
        imu0_pub->publish(imu_msg0);

        // Publish IMU 1 data
        auto imu_msg1 = sensor_msgs::msg::Imu();
        imu_msg1.header.stamp = this->get_clock()->now();
        imu_msg1.header.frame_id = "imu1";

        imu_msg1.angular_velocity.x = imu_data_1.x_gyro;
        imu_msg1.angular_velocity.y = imu_data_1.y_gyro;
        imu_msg1.angular_velocity.z = imu_data_1.z_gyro;

        imu_msg1.linear_acceleration.x = imu_data_1.x_accel;
        imu_msg1.linear_acceleration.y = imu_data_1.y_accel;
        imu_msg1.linear_acceleration.z = imu_data_1.z_accel;
        imu1_pub->publish(imu_msg1);
    }
};

// Initialize the static instance pointer
IMUPublisher* IMUPublisher::instance = nullptr;

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    // Use a MultiThreadedExecutor to handle both the timer callback and the interrupt handler
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<IMUPublisher>();
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}