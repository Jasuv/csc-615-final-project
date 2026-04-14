/**************************************************************
* Class:: CSC-615-01 Spring 2026
* Name:: Eric Ahsue
* Student ID:: 922711514
* Github-Name:: Jasuv
* Project:: Assignment 3 - Start Your Motor
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

#define PCA9685_ADDR 0x51
#define PWM_FREQ 100

#define PWMA 0
#define AIN1 1
#define AIN2 2
#define BIN1 3
#define BIN2 4
#define PWMB 5

/*
#define PWMA PCA_CHANNEL_0
#define AIN1 PCA_CHANNEL_1
#define AIN2 PCA_CHANNEL_2
#define PWMB PCA_CHANNEL_3
#define BIN1 PCA_CHANNEL_4
#define BIN2 PCA_CHANNEL_5
*/

typedef enum {
	MOTOR_A = 0,
	MOTOR_B = 1
} Motor;

typedef enum {
	FORWARD = 0,
	BACKWARD = 1
} Direction;

// initializes PCA9685 chip with I2C addr and PWM frequency 
void motor_init(void);

// run the specified motor forwards/backwards at "speed"
int motor_run(Motor mot, UWORD speed, Direction dir);

// stop specified motor
int motor_stop(Motor mot);

// stop all motors
int motor_stop_all();

#endif
