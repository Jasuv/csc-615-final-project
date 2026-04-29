// line_moto_motor
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <signal.h>
#include <sys/wait.h>
#include <stdint.h>
#include <pigpio.h>
#include <pthread.h>
#include "MotorDriver.h"
#include "DEV_Config.h"
#include "Debug.h"
#include "main.h"

#define LEFT_LINE_SENSOR_PIN 22
#define MIDDLE_LINE_SENSOR_PIN 17
#define RIGHT_LINE_SENSOR_PIN 27

void signal_handler(int sig) {
    printf("\n[Main] Received signal %d, initiating shutdown...\n", sig);
    
    motor_stop_all();
    gpioTerminate();
    exit(0);
}

int main(void)
{

    signal(SIGINT, signal_handler);
    printf("[Main] Initializing pigpio...\n");
    if (gpioInitialise() < 0) {
        printf("[Main] ERROR: pigpio initialization failed\n");
        return 1;
    }
    gpioSetMode(LEFT_LINE_SENSOR_PIN, PI_INPUT);
    gpioSetMode(MIDDLE_LINE_SENSOR_PIN, PI_INPUT);
    gpioSetMode(RIGHT_LINE_SENSOR_PIN, PI_INPUT);
    gpioSetPullUpDown(LEFT_LINE_SENSOR_PIN, PI_PUD_UP);
    gpioSetPullUpDown(MIDDLE_LINE_SENSOR_PIN, PI_PUD_UP);
    gpioSetPullUpDown(RIGHT_LINE_SENSOR_PIN, PI_PUD_UP);

    
    
    // Initialize I2C for motor driver
    if (DEV_ModuleInit() != 0) {
        printf("[Main] ERROR: DEV_ModuleInit failed\n");
        gpioTerminate();
        return 1;
    }

    printf("[Main] Initializing motor...\n");
    motor_init();

    // Configure line sensor input
    // gpioSetMode(LINE_PIN, PI_INPUT);
    // gpioSetPullUpDown(LINE_PIN, PI_PUD_UP);

    printf("Line sensor motor test started...\n");

    while (1)
    {
        //int state = gpioRead(LINE_PIN);
        int state=0;
        int L = gpioRead(LEFT_LINE_SENSOR_PIN);
        int M = gpioRead(MIDDLE_LINE_SENSOR_PIN);
        int R = gpioRead(RIGHT_LINE_SENSOR_PIN);

        if (state == 0)
        {
            
            printf("L: %i, M: %i, R: %i\n", L, M, R);
        }
        else
        {
            printf("Off line → Motor stopped\n");
            //motor_stop(MOTOR_B);
            //motor_stop(MOTOR_A);
            motor_stop(MOTOR_FL);
            motor_stop(MOTOR_FR);
            motor_stop(MOTOR_RL);
            motor_stop(MOTOR_RR);
        }

        gpioDelay(200000); // 200ms delay
    }

    // Cleanup (not reached)
    gpioTerminate();
    DEV_ModuleExit();
    return 0;
}
