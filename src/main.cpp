
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
// #include "QuickSilver.hh"
#include <Arduino_LSM6DS3.h>
#include "motor.h"

#define P 1
#define I 1
#define MAXV 242981
#define MINV 12

#define X_CAL 0.31724f
#define Y_CAL -2.29015f
#define Z_CAL 0.13525f

static LSM6DS3Class imu(Wire, LSM6DS3_ADDRESS);

float PI_fun(float in, float dt);

void setup(){
    imu.begin();

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    GPIOC->MODER |= GPIO_MODER_MODE1_0; //mode 0b01 for output mode 

    GPIOC->ODR |= GPIO_ODR_OD1;
}

void loop(){
    //setup part 2
    static MotorSet motors;

    static float acc[3] = {0,0,0};
    static float gyro[3] = {0,0,0};
    static float alpha = 0; //when we turn on, we are perfectly straight!
    static float beta = 0;
    // QuickSilver attitude; 
    // attitude.initialize(0.05);
    static unsigned long oldMicros = micros();

    // GPIOC->ODR |= GPIO_ODR_OD1;
    // for(volatile int i=0; i<250000; i++);
    // GPIOC->ODR &= ~GPIO_ODR_OD1;
    // for(volatile int i=0; i<750000; i++);
    GPIOC->ODR ^= GPIO_ODR_OD1;

    float dt = (float)(micros() - oldMicros) / 1000000.0f;

    if(dt >= 0.01){ //limit the main loop to 100 Hz
        oldMicros = micros();

        // IMU.readAcceleration(acc[0], acc[1], acc[3]);
        imu.readGyroscope(gyro[0], gyro[1], gyro[2]);
        gyro[0] -= X_CAL;
        gyro[1] -= Y_CAL;
        gyro[2] -= Z_CAL;


        //Sensor Calbiration
        // static float sum[3] = {0,0,0};
        // static unsigned int count = 0;
        // float avg[3] = {0,0,0};
        // count++;
        // for(int i=0; i<3; i++){
        //     sum[i] += gyro[i];
        //     avg[i] = sum[i] / (float)count;
        // }
        
        

        // attitude.update_estimate(acc, gyro, dt);
        //update orientation from gyro rates
        alpha += gyro[0] * dt;
        beta += gyro[1] * dt;

        //Control Loop
        float Vx = PI_fun(alpha, dt);
        float Vy = PI_fun(beta, dt);

        //Do the maths mathily to get V1, V2, V3
        float V[3] = {0,0,0};

        V[0] = 2*Vx - 2*Vy/sqrt(3);
        V[1] = 2*Vx + 2*Vy/sqrt(3);
        V[2] = Vx;
        
        for(int i=0; i<3; i++){
            V[i] = (V[i]>1) ? 1.0f : V[i];
            V[i] = (V[i]<-1) ? 1.0f : V[i];
            motors.setMotor(i, V[i]);
        }
    }
}

float PI_fun(float in, float dt){
    static float integrator = 0;
    float p = P*in;
    integrator += in*dt;
    float i = I*integrator;
    return p + i;
}