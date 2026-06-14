#ifndef CONFIG_H
#define CONFIG_H

// ROBOT CONFIGURATION
#define WHEEL_DIAMETER 0.1 // in meters
#define ROBOT_DIAMETER 0.3 // in meters
// MOTOR CONFIGURATION
#define MOTOR_MAX_RPS 10.0
#define MAX_RPS_RATIO 0.8
#define MOTOR_OPERATING_VOLTAGE 12.0
#define MOTOR_POWER_MAX_VOLTAGE 12.0
// PID CONFIGURATION
#define PWM_MIN 0
#define PWM_MAX 255
#define K_P 1.0
#define K_I 0.0
#define K_D 0.0

#define DEBUG 1

// MOTOR ENCODER PINS
#define MOTOR1_ENC_A 22
#define MOTOR1_ENC_B 23

#define MOTOR2_ENC_A 18
#define MOTOR2_ENC_B 19

#define MOTOR3_ENC_A 34
#define MOTOR3_ENC_B 35

// MOTOR CONTROL PINS
#define Motor1_IN1 25
#define Motor1_IN2 33
#define Motor1_PWM 4

#define Motor2_IN1 26
#define Motor2_IN2 27
#define Motor2_PWM 14

#define Motor3_IN1 13
#define Motor3_IN2 12
#define Motor3_PWM 15

const int cw[3] = {
    Motor1_IN1,
    Motor2_IN1,
    Motor3_IN1};
const int ccw[3] = {
    Motor1_IN2,
    Motor2_IN2,
    Motor3_IN2};
const int pwm[3] = {
    Motor1_PWM,
    Motor2_PWM,
    Motor3_PWM};

#endif // CONFIG_H