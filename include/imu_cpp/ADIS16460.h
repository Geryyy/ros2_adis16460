

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

#define MSC_CTRL 0x32
#define SYNC_OUTPUT 0xCD
#define SYNC_INPUT 0xC5


#define PROD_ID 0x56
#define PROD_ID_VAL 0x404C
    

class ADIS16460 {
    private:
        int spi_fd;
        int dr_pin;
        int rst_pin;
        int cs_pin; 
    
    public:
        ADIS16460(const char* device, int dr_pin, int rst_pin, int cs_pin) : dr_pin(dr_pin), rst_pin(rst_pin), cs_pin(cs_pin) {
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

            pinMode(cs_pin, OUTPUT);
            digitalWrite(cs_pin, HIGH);  // Default to HIGH (inactive)
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
            usleep(500000);
        }

        bool setup(bool is_master) {
            uint16_t sync_config = is_master ? SYNC_OUTPUT : SYNC_INPUT;

            // Attempt to write to the register
            if (!writeRegister(MSC_CTRL, sync_config)) {
                std::cerr << "Failed to write MSC_CTRL register!" << std::endl;
                return false;
            }

            usleep(10000);  // Small delay to allow the IMU to process the write

            // Read back the register
            uint16_t read_value = readRegister(MSC_CTRL);
            if (read_value == 0xFFFF) {
                std::cerr << "Failed to read back MSC_CTRL register!" << std::endl;
                return false;
            }

            // read product id to check communication
            uint16_t product_id = readRegister(PROD_ID);
            std::cout << "product_id: " << product_id << std::endl;
            std::cout << "PROD_ID_VAL: " << PROD_ID_VAL << std::endl;
            if(product_id != PROD_ID_VAL){
                throw std::runtime_error("product id incorrect: check communication!");
            }
            // std::cout << "MSC_CTRL register successfully set to 0x" << std::hex << read_value << std::endl;
            return true;
        }

        uint16_t readRegister(uint8_t reg) {
            constexpr size_t MY_SIZE = 2;
            uint8_t send[MY_SIZE] = {0};
            send[0] = reg;
            uint8_t recv[MY_SIZE] = {0};
    
            struct spi_ioc_transfer tr;
            memset(&tr, 0, sizeof(tr));
    
            tr.tx_buf = (unsigned long)send;
            tr.rx_buf = (unsigned long)recv;
            tr.len = MY_SIZE;
            tr.speed_hz = SPI_SPEED;
            tr.bits_per_word = 8;

            // write address
            if (ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr) < 0) {
                perror("SPI transfer failed");
                digitalWrite(cs_pin, HIGH);
                return false;
            }

            // collect data output
            send[0] = 0;
            if (ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr) < 0) {
                perror("SPI transfer failed");
                digitalWrite(cs_pin, HIGH);
                return false;
            }
            return (static_cast<uint16_t>(recv[1]) + static_cast<uint16_t>(recv[0]<<8));
        }


        bool writeRegister(uint8_t reg, uint16_t value) {
            uint8_t send[4];
            send[0] = reg | 0x80;   // Write command (MSB set)
            send[1] = (value & 0xFF); // Low byte
            send[2] = (reg + 1) | 0x80; // Next byte address
            send[3] = (value >> 8); // High byte

            // std::cout << "send[0]: 0x" << std::hex << static_cast<int>(send[0]) << std::endl;
            // std::cout << "send[1]: 0x" << std::hex << static_cast<int>(send[1]) << std::endl;
            // std::cout << "send[2]: 0x" << std::hex << static_cast<int>(send[2]) << std::endl;
            // std::cout << "send[3]: 0x" << std::hex << static_cast<int>(send[3]) << std::endl;

            // std::cout << "Writing to register 0x" << std::hex << (int)reg << " with value 0x" << value << std::endl;

            struct spi_ioc_transfer tr{};
            tr.tx_buf = (unsigned long)send;
            tr.rx_buf = 0;
            tr.len = 4;
            tr.speed_hz = SPI_SPEED;
            tr.bits_per_word = 8;

            if (ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr) < 0) {
                perror("SPI write failed");
                digitalWrite(cs_pin, HIGH);
                return false;
            }

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

            // digitalWrite(cs_pin, LOW);
            // usleep(5);

            if (ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr) < 0) {
                perror("SPI transfer failed");
                digitalWrite(cs_pin, HIGH);
                return false;
            }
            // usleep(5);  // Allow IMU to finish sending data
            // digitalWrite(cs_pin, HIGH);

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


// bool writeRegister(uint8_t reg, uint16_t value) {
//             uint8_t send[4];
//             uint16_t addresse = ((reg & 0x7F) | 0x80) << 8;   // Write command (MSB set)
//             uint16_t lowWord = (addresse | (value & 0xFF));
//             uint16_t highWord = ((addresse | 0x100) | ((value >> 8) & 0xFF));

//             send[3] = (highWord >> 8);
//             send[2] = (highWord & 0xFF);
//             send[1] = (lowWord >> 8);
//             send[0] = (lowWord & 0xFF);
//             // send[1] = (value & 0xFF); // Low byte
//             // send[2] = (reg + 1) | 0x80; // Next byte address


            
//             // send[3] = (value >> 8); // High byte

//             std::cout << "send[0]: 0x" << std::hex << static_cast<int>(send[0]) << std::endl;
//             std::cout << "send[1]: 0x" << std::hex << static_cast<int>(send[1]) << std::endl;
//             std::cout << "send[2]: 0x" << std::hex << static_cast<int>(send[2]) << std::endl;
//             std::cout << "send[3]: 0x" << std::hex << static_cast<int>(send[3]) << std::endl;

//             std::cout << "Writing to register 0x" << std::hex << (int)reg << " with value 0x" << value << std::endl;

//             struct spi_ioc_transfer tr{};
//             tr.tx_buf = (unsigned long)send;
//             tr.rx_buf = 0;
//             tr.len = 4;
//             tr.speed_hz = SPI_SPEED;
//             tr.bits_per_word = 8;

//             if (ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr) < 0) {
//                 perror("SPI write failed");
//                 digitalWrite(cs_pin, HIGH);
//                 return false;
//             }

//             return true;
//         }