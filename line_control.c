#include <stdio.h>

#include "MotorDriver.h"
#include "main.h"
#include "line_control.h"

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

void apply_line_pattern(int L, int M, int R, int *last_direction, const char *log_prefix)
{
    if (L == 0 && M == 1 && R == 0) {
        *last_direction = 0;
        drive_all_four(FULL_SPEED, FORWARD,
                       FULL_SPEED, FORWARD,
                       FULL_SPEED, FORWARD,
                       FULL_SPEED, FORWARD);
        if (log_prefix != NULL) {
            printf("%s Straight\n", log_prefix);
        }
    } else if (L == 1 && M == 0 && R == 0) {
        *last_direction = -1;
        drive_all_four(TURN_SPEED, FORWARD,
                       HARD_SPEED, BACKWARD,
                       TURN_SPEED, FORWARD,
                       HARD_SPEED, BACKWARD);
        if (log_prefix != NULL) {
            printf("%s hard right\n", log_prefix);
        }
    } else if (L == 0 && M == 0 && R == 1) {
        *last_direction = 1;
        drive_all_four(HARD_SPEED, BACKWARD,
                       TURN_SPEED, FORWARD,
                       HARD_SPEED, BACKWARD,
                       TURN_SPEED, FORWARD);
        if (log_prefix != NULL) {
            printf("%s hard left\n", log_prefix);
        }
    } else if (L == 1 && M == 1 && R == 0) {
        *last_direction = -1;
        drive_all_four(TURN_SPEED, FORWARD,
                       SOFT_SPEED, BACKWARD,
                       TURN_SPEED, FORWARD,
                       SOFT_SPEED, BACKWARD);
        if (log_prefix != NULL) {
            printf("%s soft right\n", log_prefix);
        }
    } else if (L == 0 && M == 1 && R == 1) {
        *last_direction = 1;
        drive_all_four(SOFT_SPEED, BACKWARD,
                       TURN_SPEED, FORWARD,
                       SOFT_SPEED, BACKWARD,
                       TURN_SPEED, FORWARD);
        if (log_prefix != NULL) {
            printf("%s soft left\n", log_prefix);
        }
    } else {
        if (log_prefix != NULL) {
            printf("%s SEARCHING...\n", log_prefix);
        }

        if (*last_direction <= 0) {
            drive_all_four(SEARCH_SPEED, FORWARD,
                           SEARCH_SPEED, BACKWARD,
                           SEARCH_SPEED, FORWARD,
                           SEARCH_SPEED, BACKWARD);
        } else {
            drive_all_four(SEARCH_SPEED, BACKWARD,
                           SEARCH_SPEED, FORWARD,
                           SEARCH_SPEED, BACKWARD,
                           SEARCH_SPEED, FORWARD);
        }
    }
}
