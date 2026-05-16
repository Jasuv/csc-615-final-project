#include <stdio.h>
#include <string.h>

#include "MotorDriver.h"
#include "main.h"
#include "camera_control.h"

static void drive_all_four(UWORD fl_speed, Direction fl_dir,
                           UWORD fr_speed, Direction fr_dir,
                           UWORD rl_speed, Direction rl_dir,
                           UWORD rr_speed, Direction rr_dir)
{
    motor_run(MOTOR_FL, fl_speed, fl_dir);
    motor_run(MOTOR_FR, fr_speed, fr_dir);
    motor_run(MOTOR_RL, rl_speed, rl_dir);
    motor_run(MOTOR_RR, rr_speed, rr_dir);
}

void apply_camera_bias(const char *bias, const char *log_prefix)
{
    if (log_prefix != NULL) {
        printf("%s %s\n", log_prefix, bias);
    }

    if (strcmp(bias, "CENTER") == 0) {
        drive_all_four(FULL_SPEED, FORWARD,
                       FULL_SPEED, FORWARD,
                       FULL_SPEED, FORWARD,
                       FULL_SPEED, FORWARD);
    } else if (strcmp(bias, "LEFT") == 0) {
        drive_all_four(HARD_SPEED, FORWARD,
                       FULL_SPEED, FORWARD,
                       HARD_SPEED, FORWARD,
                       FULL_SPEED, FORWARD);
    } else if (strcmp(bias, "RIGHT") == 0) {
        drive_all_four(FULL_SPEED, FORWARD,
                       HARD_SPEED, FORWARD,
                       FULL_SPEED, FORWARD,
                       HARD_SPEED, FORWARD);
    } else if (strcmp(bias, "HARD LEFT") == 0) {
        drive_all_four(SOFT_SPEED, FORWARD,
                       FULL_SPEED, FORWARD,
                       SOFT_SPEED, FORWARD,
                       FULL_SPEED, FORWARD);
    } else if (strcmp(bias, "HARD RIGHT") == 0) {
        drive_all_four(FULL_SPEED, FORWARD,
                       SOFT_SPEED, FORWARD,
                       FULL_SPEED, FORWARD,
                       SOFT_SPEED, FORWARD);
    } else {
        if (log_prefix != NULL) {
            printf("%s NO PATH -> STOP\n", log_prefix);
        }
        motor_stop_all();
    }
}
