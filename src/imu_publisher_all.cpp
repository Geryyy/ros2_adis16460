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

    double bias0_angvel_x;
    double bias0_angvel_y;
    double bias0_angvel_z;

    double bias1_angvel_x;
    double bias1_angvel_y;
    double bias1_angvel_z;

    // IMU data storage
    IMUData imu_data_0;
    IMUData imu_data_1;

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

        // this->declare_parameter<std::string>("imu_frame", "imu_link");
        // imu_frame = this->get_parameter("imu_frame").as_string();

        this->declare_parameter<std::string>("bias0_angvel_x", "bias0_angvel_x");
        bias0_angvel_x = this->get_parameter("bias0_angvel_x").as_double();

        this->declare_parameter<std::string>("bias0_angvel_y", "bias0_angvel_y");
        bias0_angvel_y = this->get_parameter("bias0_angvel_y").as_double();

        this->declare_parameter<std::string>("bias0_angvel_z", "bias0_angvel_z");
        bias0_angvel_y = this->get_parameter("bias0_angvel_z").as_double(); 


        this->declare_parameter<std::string>("bias1_angvel_x", "bias1_angvel_x");
        bias1_angvel_x = this->get_parameter("bias1_angvel_x").as_double();

        this->declare_parameter<std::string>("bias1_angvel_y", "bias1_angvel_y");
        bias1_angvel_y = this->get_parameter("bias1_angvel_y").as_double();

        this->declare_parameter<std::string>("bias1_angvel_z", "bias1_angvel_z");
        bias1_angvel_y = this->get_parameter("bias1_angvel_z").as_double();            


        pinMode(SYNC_PIN, OUTPUT);

        // Create the IMU object
        imu0 = std::make_shared<ADIS16460>(SPI_DEVICE_0, DR0_PIN, RST0_PIN, CS0_PIN);
        imu1 = std::make_shared<ADIS16460>(SPI_DEVICE_1, DR1_PIN, RST1_PIN, CS1_PIN);

        // Reset and configure the IMU
        imu0->reset();
        imu0->setup(false);
        usleep(1000);
        imu1->reset();
        imu1->setup(false);
        
        imu0_pub = this->create_publisher<sensor_msgs::msg::Imu>("/adis16460_dev0", rclcpp::SensorDataQoS());
        imu1_pub = this->create_publisher<sensor_msgs::msg::Imu>("/adis16460_dev1", rclcpp::SensorDataQoS());
        
        RCLCPP_INFO(this->get_logger(), "IMU Publisher started.");

        // Timer for publishing IMU data periodically
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(2), 
            std::bind(&IMUPublisher::publishIMUData, this)
        );
    }


private:
    int readIMUData(std::shared_ptr<ADIS16460> imu, IMUData &data) {
        if (!imu->readData(data)) {
            RCLCPP_WARN(this->get_logger(), "Failed to read IMU data.");
            return -1;
        }
        else{
            return 0;
        }
    }

    void publishIMUData() {
        // trigger data conversion
        digitalWrite(SYNC_PIN, HIGH);
        usleep(25);
        digitalWrite(SYNC_PIN, LOW);

        usleep(700);
        // read imu0
        int imu_status = 0;
        imu_status += readIMUData(imu0, imu_data_0);
        usleep(10);
        imu_status += readIMUData(imu1, imu_data_1);

        if(imu_status != 0){
            imu0->reset();
            imu0->setup(false);
            usleep(1000);
            imu1->reset();
            imu1->setup(false);
            RCLCPP_INFO(this->get_logger(), "Reset IMUs.");
            return;
        }

        // read imu1


        // Publish IMU 0 data
        auto imu_msg0 = sensor_msgs::msg::Imu();
        imu_msg0.header.stamp = this->get_clock()->now();
        imu_msg0.header.frame_id = "imu0";

        imu_msg0.angular_velocity.x = imu_data_0.x_gyro - bias0_angvel_x;
        imu_msg0.angular_velocity.y = imu_data_0.y_gyro - bias0_angvel_y;
        imu_msg0.angular_velocity.z = imu_data_0.z_gyro - bias0_angvel_z;

        imu_msg0.linear_acceleration.x = imu_data_0.x_accel;
        imu_msg0.linear_acceleration.y = imu_data_0.y_accel;
        imu_msg0.linear_acceleration.z = imu_data_0.z_accel;
        imu0_pub->publish(imu_msg0);

        // Publish IMU 1 data
        auto imu_msg1 = sensor_msgs::msg::Imu();
        imu_msg1.header.stamp = this->get_clock()->now();
        imu_msg1.header.frame_id = "imu1";

        imu_msg1.angular_velocity.x = imu_data_1.x_gyro - bias1_angvel_x;
        imu_msg1.angular_velocity.y = imu_data_1.y_gyro - bias1_angvel_y;
        imu_msg1.angular_velocity.z = imu_data_1.z_gyro - bias1_angvel_z;

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