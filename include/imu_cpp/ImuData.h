
#ifndef IMUDATA_H
#define IMUDATA_H


// Struct to hold IMU Data
struct IMUData {
    static constexpr double scale_gyro = 1e-9;
    static constexpr double scale_accel = 3.7e-8;

    uint16_t diag_stat;
    double x_gyro;
    double y_gyro;
    double z_gyro;
    double x_accel;
    double y_accel;
    double z_accel;
    int16_t temp;
    uint16_t smpl_cntr;
    uint16_t checksum;

    void print() const {
        std::cout << "DiagStat: " << diag_stat << " | "
                  << "Gyro: [" << x_gyro << ", " << y_gyro << ", " << z_gyro << "] | "
                  << "Accel: [" << x_accel << ", " << y_accel << ", " << z_accel << "] | "
                  << "Temp: " << temp << " | "
                  << "Sample Counter: " << smpl_cntr << " | "
                  << "Checksum: " << checksum << "\n";
    }

    static double convert_gyro(uint8_t high, uint8_t low){
        uint16_t value1 = high << 8 | low;
        int32_t value = (value1 << 16);

        double result = static_cast<double>(value) * IMUData::scale_gyro;
        return result;
    }

    static double convert_accel(uint8_t high, uint8_t low){
        uint16_t value1 = high << 8 | low;
        int32_t value = (value1 << 16);

        double result = static_cast<double>((int32_t)value) * IMUData::scale_accel;
        return result;
    }
};

#endif // IMUDATA_H

