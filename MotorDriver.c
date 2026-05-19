/**************************************************************
* Class:: CSC-615-01 Spring 2026
* Name:: Haibin Cao, Eric Ahsue, Kiran Khatri, John Tsiglieris
* Student ID:: 922711514
* Github-Name:: haibinc, Jasuv, khatri5034, John-Tsiglieris
* Project::
*
* File:: MotorDriver.c
*
* Description:: Implements three driver code funcions for the
* motor:
*	1.	init() to initialize PCA9685 chip with I2C address
*		and PWM frequency
*	2.	run() starts input motor at input direction with input
*		speed
*	3.	stop() stop input motor by setting all related pins
*		pins to null
* 
**************************************************************/

#include "MotorDriver.h"

void motor_init(void)
{
    DEV_I2C_Init(HAT1_ADDR);
    PCA9685_Init(HAT1_ADDR);
    PCA9685_SetPWMFreq(PWM_FREQ);

    DEV_I2C_Init(HAT2_ADDR);
    PCA9685_Init(HAT2_ADDR);
    PCA9685_SetPWMFreq(PWM_FREQ);
}

int motor_run(Motor mot, UWORD speed, Direction dir)
{
    if (speed > 100) speed = 100;

    /* Invert direction for physically flipped motors */
    if (mot == MOTOR_FL || mot == MOTOR_RR)
    {
        dir = (dir == FORWARD) ? BACKWARD : FORWARD;
    }

    switch (mot)
    {
        case MOTOR_FL:
            DEV_I2C_Init(HAT2_ADDR);
            PCA9685_SetPwmDutyCycle(PWMB, speed);
            if (dir == FORWARD) {
                PCA9685_SetLevel(BIN1, 0);
                PCA9685_SetLevel(BIN2, 1);
            } else {
                PCA9685_SetLevel(BIN1, 1);
                PCA9685_SetLevel(BIN2, 0);
            }
            break;

        case MOTOR_FR:
            DEV_I2C_Init(HAT2_ADDR);
            PCA9685_SetPwmDutyCycle(PWMA, speed);
            if (dir == FORWARD) {
                PCA9685_SetLevel(AIN1, 0);
                PCA9685_SetLevel(AIN2, 1);
            } else {
                PCA9685_SetLevel(AIN1, 1);
                PCA9685_SetLevel(AIN2, 0);
            }
            break;

        case MOTOR_RL:
            DEV_I2C_Init(HAT1_ADDR);
            PCA9685_SetPwmDutyCycle(PWMB, speed);
            if (dir == FORWARD) {
                PCA9685_SetLevel(BIN1, 0);
                PCA9685_SetLevel(BIN2, 1);
            } else {
                PCA9685_SetLevel(BIN1, 1);
                PCA9685_SetLevel(BIN2, 0);
            }
            break;

        case MOTOR_RR:
            DEV_I2C_Init(HAT1_ADDR);
            PCA9685_SetPwmDutyCycle(PWMA, speed);
            if (dir == FORWARD) {
                PCA9685_SetLevel(AIN1, 0);
                PCA9685_SetLevel(AIN2, 1);
            } else {
                PCA9685_SetLevel(AIN1, 1);
                PCA9685_SetLevel(AIN2, 0);
            }
            break;
    }

    return 0;
}

int motor_stop(Motor mot)
{
    if (mot == MOTOR_FL || mot == MOTOR_FR)
        DEV_I2C_Init(HAT1_ADDR);
    else
        DEV_I2C_Init(HAT2_ADDR);

    if (mot == MOTOR_FL || mot == MOTOR_RL)
    {
        PCA9685_SetLevel(AIN1, 0);
        PCA9685_SetLevel(AIN2, 0);
        PCA9685_SetPwmDutyCycle(PWMA, 0);
    }
    else
    {
        PCA9685_SetLevel(BIN1, 0);
        PCA9685_SetLevel(BIN2, 0);
        PCA9685_SetPwmDutyCycle(PWMB, 0);
    }

    return 0;
}

void motor_ramp(UBYTE motor,
                UWORD startSpeed, UWORD endSpeed,
                UWORD stepDelay, Direction dir)
{
    UWORD step = 0;
    UWORD speed = startSpeed;
 
    while (1) {
        motor_run(motor, dir, speed);
        usleep(stepDelay);  
        if (speed == endSpeed) break;
        if (endSpeed > startSpeed) 
        {
            speed += 1;
        } 
        else if (endSpeed < startSpeed) 
        {
            speed -= 1;
        }
    }
}

int motor_stop_all(void)
{
    motor_stop(MOTOR_FL);
    motor_stop(MOTOR_FR);
    motor_stop(MOTOR_RL);
    motor_stop(MOTOR_RR);
    return 0;
}
