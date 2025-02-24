

#ifndef ADIS16460_H
#define ADIS16460_H

#define SPI_DEVICE_0 "/dev/spidev0.0"
#define SPI_DEVICE_1 "/dev/spidev0.1"
#define SPI_SPEED 1000000 // 1 MHz
#define SPI_MODE SPI_MODE_3

#define GLOB_CMD 0x3E
#define BURST_SIZE 22  // 2 bytes per register, 11 registers

#define MSC_CTRL 0x32
#define MSC_CTRL_VAL 0xC5

class ADIS16460 {
    private:
        int spi_fd;
        int dr_pin;
        int rst_pin;
    
    public:
        ADIS16460(const char* device, int dr_pin, int rst_pin) : dr_pin(dr_pin), rst_pin(rst_pin) {
            // Open SPI device
            spi_fd = open(device, O_RDWR);
            if (spi_fd < 0) {
                perror("Failed to open SPI device");
                exit(EXIT_FAILURE);
            }
    
            // Configure SPI
            uint8_t mode = SPI_MODE;
            uint8_t bits = 8;
            uint32_t speed = SPI_SPEED;
    
            ioctl(spi_fd, SPI_IOC_WR_MODE, &mode);
            ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
            ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
    
            // Setup GPIO for Data Ready
            pinMode(dr_pin, INPUT);
            pullUpDnControl(dr_pin, PUD_DOWN);
    
            pinMode(rst_pin, OUTPUT);
            digitalWrite(rst_pin, 1);
        }
    
        ~ADIS16460() {
            if (spi_fd >= 0) {
                close(spi_fd);
            }
        }
    
        void reset(){
            digitalWrite(rst_pin, 0);
            usleep(1000);
            digitalWrite(rst_pin,1);
            usleep(100000);
        }
    
        bool writeRegister(uint8_t reg, uint16_t value) {
            // digitalWrite(cs_pin, LOW);
            // usleep(1);
    
            uint8_t send[4];
            send[0] = reg | 0x80;   // Write command (MSB set)
            send[1] = (value & 0xFF); // Low byte
            
            send[2] = (reg + 1) | 0x80; // Next byte address
            send[3] = (value >> 8); // High byte
    
            struct spi_ioc_transfer tr{};
            tr.tx_buf = (unsigned long)send;
            tr.rx_buf = 0;
            tr.len = 2;
            // tr.len = 2;
            tr.speed_hz = SPI_SPEED;
            tr.bits_per_word = 8;
    
            if (ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr) < 0) {
                perror("SPI write failed");
                // digitalWrite(cs_pin, HIGH);
                return false;
            }
    
            // digitalWrite(cs_pin, HIGH);
            return true;
        }
    
        bool readData(IMUData &imuData) {
            // Ensure DR is HIGH before reading
            // if (digitalRead(dr_pin) == LOW) {
            //     std::cerr << "Data not ready!\n";
            //     return false;
            // }
    
            // Send burst read command
            uint8_t send[BURST_SIZE] = {GLOB_CMD};
            uint8_t recv[BURST_SIZE] = {0};
    
            struct spi_ioc_transfer tr;
            memset(&tr, 0, sizeof(tr));
    
            tr.tx_buf = (unsigned long)send;
            tr.rx_buf = (unsigned long)recv;
            tr.len = BURST_SIZE;
            tr.speed_hz = SPI_SPEED;
            tr.bits_per_word = 8;
    
            if (ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr) < 0) {
                perror("SPI transfer failed");
                return false;
            }
    
            // Store received data
            std::vector<uint8_t> data;
            data.assign(recv + 2, recv + BURST_SIZE); // Ignore first 2 bytes
    
            // Populate IMUData struct
            imuData.diag_stat = (data[0] << 8) | data[1];
            imuData.x_gyro = IMUData::convert_gyro(data[2], data[3]);
            imuData.y_gyro = IMUData::convert_gyro(data[4], data[5]);
            imuData.z_gyro = IMUData::convert_gyro(data[6], data[7]);
            imuData.x_accel = IMUData::convert_accel(data[8], data[9]);
            imuData.y_accel = IMUData::convert_accel(data[10], data[11]);
            imuData.z_accel = IMUData::convert_accel(data[12], data[13]);
            imuData.temp = (data[14] << 8) | data[15];
            imuData.smpl_cntr = (data[16] << 8) | data[17];
            imuData.checksum = (data[18] << 8) | data[19];
    
            // Verify checksum
            uint16_t checksum_received = (data[18] << 8) | data[19];
            uint16_t checksum_calc = 0;
            for (int i = 0; i < 18; i++) checksum_calc += data[i];
    
            if (checksum_received != checksum_calc) {
                std::cerr << "------------->Checksum error!\n";
                return false;
            }
    
            return true;
        }
    };



#endif // ADIS16460_H