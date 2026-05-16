#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <pigpio.h>

#include "MotorDriver.h"
#include "DEV_Config.h"

static volatile sig_atomic_t keep_running = 1;

static void handle_signal(int signal_number)
{
    (void)signal_number;
    keep_running = 0;
}

int main(void)
{
    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);

    if (DEV_ModuleInit() != 0) {
        fprintf(stderr, "Failed to initialize DEV module.\n");
        return 1;
    }

    motor_init();

    motor_run(MOTOR_FL, 100, FORWARD);
    motor_run(MOTOR_FR, 100, FORWARD);
    motor_run(MOTOR_RL, 100, FORWARD);
    motor_run(MOTOR_RR, 100, FORWARD);

    printf("All motors running forward at 100%% power.\n");
    printf("Press Ctrl+C to stop, or wait 120 seconds for an automatic stop.\n");

    for (int second = 0; second < 120 && keep_running; ++second) {
        sleep(1);
    }

    motor_stop_all();
    DEV_ModuleExit();
    gpioTerminate();
    return 0;
}