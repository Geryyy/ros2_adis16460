#include <iostream>
#include <fstream>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <unistd.h>
#include <cstring>
#include <wiringPi.h>
#include <vector>
#include "ImuData.h"
#include "ADIS16460.h"



// Global IMU objects
ADIS16460* imu1;


void imu1_handler() {
    IMUData data;
    if (imu1->readData(data)) {
        std::cout << "IMU1 Read Successful!\n";
        // std::cout << "Gyro: [" << data.x_gyro << ", " << data.y_gyro << ", " << data.z_gyro << "]\n";
        // std::cout << "Accel: [" << data.x_accel << ", " << data.y_accel << ", " << data.z_accel << "]\n";
    } else {
        std::cout << "IMU1 Read Failed!\n";
    }
}

int main() {
    // Initialize WiringPi
    if (wiringPiSetupGpio() == -1) {
        std::cerr << "Failed to initialize WiringPi\n";
        return -1;
    }

    // Create IMU objects
    imu1 = new ADIS16460(SPI_DEVICE_1, DR1_PIN, RST1_PIN);

    imu1->reset();

    if (!imu1->writeRegister(MSC_CTRL, MSC_CTRL_VAL)) {
        std::cerr << "Failed to configure IMU1\n";
        return -1;
    }
    else{
        std::cout << "IMU1: Write MSC successfull!" << std::endl;
    }


    // Attach Interrupt Handlers
    wiringPiISR(DR1_PIN, INT_EDGE_RISING, &imu1_handler);

    std::cout << "IMU Reading started. Press Ctrl+C to exit.\n";

    while (true) {
        usleep(10000); // Sleep for 10ms to reduce CPU usage
    }

    // Clean up
    delete imu1;
    return 0;
}
