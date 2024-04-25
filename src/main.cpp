
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
// #include "stm32f405xx.h"
// #include "imu.h"
// int main(){
//     RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
//     GPIOC->MODER |= GPIO_MODER_MODE1_0; //mode 0b01 for output mode 

//     GPIOC->ODR |= GPIO_ODR_OD1;
    
//     initializeIMU();

//     while(1){
//         GPIOC->ODR |= GPIO_ODR_OD1;
//         for(volatile int i=0; i<250000; i++);
//         GPIOC->ODR &= ~GPIO_ODR_OD1;
//         for(volatile int i=0; i<750000; i++);
//     }
// }

// #define P 0.001
// #define I 0.001
// #define dt 0.01; //100 Hz

// float fun(float in){
//     static float integrator = 0;
//     float p = P*in;
//     integrator += in*dt;
//     float i = I*integrator;

//     return p + i;
// }

#include <Arduino.h>
#include "QuickSilver.hh"
#include <Arduino_LSM6DS3.h>


void setup(){
    IMU.begin();
}

void loop(){
    //setup part 2
    float acc[3] = {0,0,0};
    float gyro[3] = {0,0,0};
    QuickSilver attitude; 
    attitude.initialize();
    unsigned long oldMicros = micros();

    //the real loop
    while(1){ //will this run so fast theat our sensors don't have a chance to properly update?
        float dt = (float)(oldMicros - micros()) / 1000000.0f;
        oldMicros = micros();

        IMU.readAcceleration(acc[0], acc[1], acc[3]);
        IMU.readGyroscope(gyro[0], gyro[1], gyro[2]);

        attitude.update_estimate(acc, gyro, dt);


    }
}