#include <stdio.h>
#include <stdlib.h>
#include <pigpio.h>

#include "MotorDriver.h"
#include "DEV_Config.h"

#define LINE_PIN 17

int main(void)
{
    // Initialize I2C for motor driver
    if (DEV_ModuleInit())
        return 1;

    // Initialize pigpio
    if (gpioInitialise() < 0)
        return 1;

    // Initialize motor system
    motor_init();

    // Configure line sensor input
    gpioSetMode(LINE_PIN, PI_INPUT);
    gpioSetPullUpDown(LINE_PIN, PI_PUD_UP);

    printf("Line sensor motor test started...\n");

    while (1)
    {
        int state = gpioRead(LINE_PIN);


    if (state == 0)
    {
        printf("On line → Motor running\n");
        motor_run(MOTOR_B,80,FORWARD);
    }
    else
    {
        printf("Off line → Motor stopped\n");
        motor_stop(MOTOR_B);
    }

        gpioDelay(200000); // 200ms delay
    }

    // Cleanup (not reached)
    gpioTerminate();
    DEV_ModuleExit();
    return 0;
}