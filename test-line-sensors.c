/* Used to test if the line sensors actually can detect lines on the track*/

#include <stdio.h>
#include <stdlib.h>
#include <pigpio.h>
#include <unistd.h>
#include <signal.h>

#define LEFT_LINE_SENSOR_PIN   22
#define MIDDLE_LINE_SENSOR_PIN 27
#define RIGHT_LINE_SENSOR_PIN  17

volatile sig_atomic_t stop_flag = 0;

void sigint_handler(int sig)
{
    stop_flag = 1;
}

int main()
{
    signal(SIGINT, sigint_handler);

    if (gpioInitialise() < 0)
    {
        printf("pigpio initialization failed!\n");
        return 1;
    }

    gpioSetMode(LEFT_LINE_SENSOR_PIN, PI_INPUT);
    gpioSetMode(MIDDLE_LINE_SENSOR_PIN, PI_INPUT);
    gpioSetMode(RIGHT_LINE_SENSOR_PIN, PI_INPUT);

    printf("Line Sensor Monitor Started\n");

    while (!stop_flag)
    {
        int left   = gpioRead(LEFT_LINE_SENSOR_PIN);
        int middle = gpioRead(MIDDLE_LINE_SENSOR_PIN);
        int right  = gpioRead(RIGHT_LINE_SENSOR_PIN);

        printf("LEFT:   %s\n",   left   == 1 ? "LINE DETECTED" : "NO LINE");
        printf("MIDDLE: %s\n",   middle == 1 ? "LINE DETECTED" : "NO LINE");
        printf("RIGHT:  %s\n",   right  == 1 ? "LINE DETECTED" : "NO LINE");

        printf("-----------------------------\n");

        sleep(1);
    }

    gpioTerminate();
    printf("Program terminated\n");

    return 0;
}