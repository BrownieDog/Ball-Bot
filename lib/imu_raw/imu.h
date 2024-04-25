#ifndef IMU_H
#define IMU_H
extern "C" {
    void initializeIMU(void);
    void readAcc(float *xyz);
    void readGyro(float *xyz);
}

#define I2C_DIRECTION_TRANSMIT 0
#define I2C_DIRECTION_RECIEVE 1

#define IMU_ADDRESS 0x6A
#define IMU_ADDRESS_BACKUP 0x6B

#endif //IMU_H