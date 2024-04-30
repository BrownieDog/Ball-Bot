#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

#define MOTOR0_PIN0 PB_11   //RX
#define MOTOR0_PIN1 PB_10   //TX
#define MOTOR1_PIN0 PC_7    //D5
#define MOTOR1_PIN1 PC_6    //D6
#define MOTOR2_PIN0 PB_8    //D9
#define MOTOR2_PIN1 PB_9    //D10

class MotorSet {
public:
    MotorSet();
    void setMotor(uint8_t motor, float power);
private:
    enum MotorDirection {
        FORWARD,
        BACKWARD,
    };
    struct Motor {
        float power;
        MotorDirection direction;
    };
    
    Motor motors[3];
};

#endif //MOTOR_H