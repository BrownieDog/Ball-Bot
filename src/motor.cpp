#include "motor.h"

MotorSet::MotorSet(){
    for(int i=0; i<3; i++){
        motors[i].power = 0;
        motors[i].direction = FORWARD;
    }
}

void MotorSet::setMotor(uint8_t motor, float power) {
    if(motor > 2) return; //catch index out of bounds

    //saturate power between -1 and 1
    power = (power>1)? 1.0f : power;
    power = (power<-1)? -1.0f : power;

    motors[motor].power = power;
    motors[motor].direction = (power<0)? BACKWARD : FORWARD;

    switch(motor){
        case(0):
            pwm_stop(MOTOR0_PIN0);
            pwm_stop(MOTOR0_PIN1);
            if(motors[motor].direction == FORWARD){
                pwm_start(MOTOR0_PIN0, PWM_FREQUENCY, 100*power, PERCENT_COMPARE_FORMAT);
            } else {
                pwm_start(MOTOR0_PIN1, PWM_FREQUENCY, -100*power, PERCENT_COMPARE_FORMAT);
            }
            break;
        case(1):
            pwm_stop(MOTOR1_PIN0);
            pwm_stop(MOTOR1_PIN1);
            if(motors[motor].direction == FORWARD){
                pwm_start(MOTOR1_PIN0, PWM_FREQUENCY, 100*power, PERCENT_COMPARE_FORMAT);
            } else {
                pwm_start(MOTOR1_PIN1, PWM_FREQUENCY, -100*power, PERCENT_COMPARE_FORMAT);
            }
            break;
        case(2):
            pwm_stop(MOTOR2_PIN0);
            pwm_stop(MOTOR2_PIN1);
            if(motors[motor].direction == FORWARD){
                pwm_start(MOTOR2_PIN0, PWM_FREQUENCY, 100*power, PERCENT_COMPARE_FORMAT);
            } else {
                pwm_start(MOTOR2_PIN1, PWM_FREQUENCY, -100*power, PERCENT_COMPARE_FORMAT);
            }
            break;
    }
}