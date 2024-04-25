
// class MotorSet {
// public:
//     MotorSet();
//     void setVelocity(float x, float y);

// private:
//     float motorA;
//     float motorB;
//     float motorC;

// };

// struct Euler {
//     float roll;
//     float pitch;
//     float yaw;
// };

// struct xyzfloat {
//     float x;
//     float y;
//     float z;
// };

// class PID {
// public:
//     PID();
//     PID(float P, float I, float D);
//     xyzfloat getNeededXY();

// private:
//     float P;
//     float I;
//     float D;
// };



// class IMU {
// public:
//     IMU();
//     Euler getEuler();
//     void updateAttitude(){
//         //read acc
//         //read gyro

//         //calculate euler from gyro vals

//         //fuse accelerometer

//     }
//     void readAcc(float acc[]);
//     void readGyro(float gyro[]);
// private:
//     Euler euler;
// };


// int main(){
//     xyzfloat xyz = {0,0,0};
//     PID pid = PID(0.1, 0.1, 0.1);
//     MotorSet motors;
//     IMU imu;

//     while(1){
//         Euler euler = imu.getEuler();

//         xyz = pid.getNeededXY();
//         motors.setVelocity(xyz.x, xyz.y);
//     }
//     return -1;
// }
#include "stm32f405xx.h"
#include "imu.h"
int main(){
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    GPIOC->MODER |= GPIO_MODER_MODE1_0; //mode 0b01 for output mode 

    GPIOC->ODR |= GPIO_ODR_OD1;
    
    initializeIMU();

    while(1){
        GPIOC->ODR |= GPIO_ODR_OD1;
        for(volatile int i=0; i<250000; i++);
        GPIOC->ODR &= ~GPIO_ODR_OD1;
        for(volatile int i=0; i<750000; i++);
    }
}