/**************************************************************
* Class:: CSC-615-01 Spring 2026
* Name:: Haibin Cao, Eric Ahsue, Kiran Khatri, John Tsiglieris
* Student ID:: 923756077, 922711514, 925750019, 923593954
* GitHub-Name:: haibinc, Jasuv, khatri5034, John-Tsiglieris
* Project:: CSC 615 Final Project - Self-Driving Car
*
* File:: MotorDriver.h
*
* Description:: Declares the motor API used throughout the project:
* MOTOR_A/MOTOR_B and FORWARD/BACKWARD enums, the PCA9685 I2C
* address and PWM frequency, and the motor_init/motor_run/
* motor_stop/motor_stop_all functions built on top of the
* WaveShare DEV_Config and PCA9685 libraries.
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
