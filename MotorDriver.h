/**************************************************************
* Class:: CSC-615-01 Spring 2026
* Name:: Haibin Cao, Eric Ahsue, Kiran Khatri, John Tsiglieris
* Student ID:: 922711514
* Github-Name:: haibinc, Jasuv, khatri5034, John-Tsiglieris
* Project::
*
* File:: MotorDriver.h
*
* Description:: includes the sample DEV lib and PCA9685 functions
* to create driver code for the motor
* 
**************************************************************/


#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include "DEV_Config.h"
#include "PCA9685.h"

#define HAT1_ADDR 0x40   // First motor hat
#define HAT2_ADDR 0x51   // Second motor hat

#define PWM_FREQ 100

/* PCA9685 channel mapping */
#define PWMA 0
#define AIN1 1
#define AIN2 2
#define BIN1 3
#define BIN2 4
#define PWMB 5

typedef enum {
    MOTOR_FL,
    MOTOR_FR,
    MOTOR_RL,
    MOTOR_RR
} Motor;

typedef enum {
    FORWARD = 1,
    BACKWARD = 0
} Direction;

/* initialize BOTH hats */
void motor_init(void);

/* run motor */
int motor_run(Motor mot, UWORD speed, Direction dir);

void motor_ramp(UBYTE motor,
                UWORD startSpeed, UWORD endSpeed,
                UWORD stepDelay, Direction dir);

/* stop motor */
int motor_stop(Motor mot);

/* stop all motors */
int motor_stop_all(void);

#endif