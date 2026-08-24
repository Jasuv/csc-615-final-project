/**************************************************************
* Class:: CSC-615-01 Spring 2026
* Name:: Haibin Cao, Eric Ahsue, Kiran Khatri, John Tsiglieris
* Student ID:: 923756077, 922711514, 925750019, 923593954
* GitHub-Name:: haibinc, Jasuv, khatri5034, John-Tsiglieris
* Project:: CSC 615 Final Project - Self-Driving Car
*
* File:: MotorDriver.c
*
* Description:: Motor driver layer for the self-driving car, built
* on top of the PCA9685 PWM controller:
*	1.	motor_init() initializes the PCA9685 chip with its I2C
*		address and PWM frequency
*	2.	motor_run() drives the given motor (A or B) forward or
*		backward at a given speed (1-100%)
*	3.	motor_stop() / motor_stop_all() stop one or both motors
*		by zeroing their direction and PWM channels
*
**************************************************************/

#include "MotorDriver.h"

/*
* initializes PCA9685 chip with custom address (cuz mines different from default)
* and the PWM frequency
*/
void motor_init(void) {
	PCA9685_Init(PCA9685_ADDR);
	PCA9685_SetPWMFreq(PWM_FREQ);
}


/*
* Given a motor, speed that is between 1-100 
* Set the direction channel based of "dir"
* Set the duty cycle percentage to "speed"
*/
int motor_run(Motor mot, UWORD speed, Direction dir) {


	// sanitize and verify speed
	if (speed > 100) speed = 100;
	if (speed < 1) {
		DEBUG("ERROR: speed must be btween 1-100\n");
		return 1;
	}

	if (mot == MOTOR_A) {
		// set direction channels for Motor A
		if (dir == FORWARD) {
			PCA9685_SetLevel(AIN1, 1);
			PCA9685_SetLevel(AIN2, 0);
		} 
		else if (dir == BACKWARD) {
			PCA9685_SetLevel(AIN1, 0);
			PCA9685_SetLevel(AIN2, 1);
		} 
		else {
			DEBUG("ERROR: bad direction, use FORWARD or BACKWARD\n");
			return 1;
		}

		// set duty cycle for pulse width (Motor A)
		PCA9685_SetPwmDutyCycle(PWMA, speed);
	}
	else if (mot == MOTOR_B) {
		// set direction channels for Motor B
		if (dir == FORWARD) {
			PCA9685_SetLevel(BIN1, 1);
			PCA9685_SetLevel(BIN2, 0);
		} 
		else if (dir == BACKWARD) {
			PCA9685_SetLevel(BIN1, 0);
			PCA9685_SetLevel(BIN2, 1);
		} 
		else {
			DEBUG("ERROR: bad direction, use FORWARD or BACKWARD\n");
			return 1;
		}

		// set duty cycle for pulse width (Motor B)
		PCA9685_SetPwmDutyCycle(PWMB, speed);
	}
	else {
		DEBUG("ERROR: invalid motor, use MOTOR_A or MOTOR_B\n");
		return 1;
	}

	return 0;
}


/*
 * switch off the specified motor by setting direction channels and pwm to 0
 */
int motor_stop(Motor mot) {
	if (mot == MOTOR_A) {
		PCA9685_SetLevel(AIN1, 0);
		PCA9685_SetLevel(AIN2, 0);
		PCA9685_SetLevel(PWMA, 0);
	}
	else if (mot == MOTOR_B) {
		PCA9685_SetLevel(BIN1, 0);
		PCA9685_SetLevel(BIN2, 0);
		PCA9685_SetLevel(PWMB, 0);
	}
	else {
		DEBUG("ERROR: invalid motor, use MOTOR_A or MOTOR_B\n");
		return 1;
	}
	return 0;
}

/*
 * switch off all motors
 */
int motor_stop_all() {
	motor_stop(MOTOR_A);
	motor_stop(MOTOR_B);
	return 0;
}
