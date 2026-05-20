/**************************************************************
 * Class:: CSC-615-01 Spring 2026
 * Name:: Haibin Cao, Eric Ahsue, Kiran Khatri, John Tsiglieris
 * Student ID:: 922711514
 * Github-Name:: haibinc, Jasuv, khatri5034, John-Tsiglieris
 * Project::
 *
 * File:: line_controller.c
 *
 * Description::
 * seperate line (and RGB sensor) control logic. uses data from sensors
 * to influence motor directions and speeds.
 * 
 * RGB sensor: waits for two important colors (red and blue) and stops
 * on red, while waiting 5 seconds on blue.
 *
 * line sensors: uses 3 line sensors to determine turn direction and speed
 * while also handling derailing.
 * 
 **************************************************************/

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <pigpio.h>
#include "ColorLib.h"
#include "MotorDriver.h"
#include "main.h"
#include "line_controller.h"

static void drive_all_four(UWORD fl_speed, Direction fl_dir,
                           UWORD fr_speed, Direction fr_dir,
                           UWORD rl_speed, Direction rl_dir,
                           UWORD rr_speed, Direction rr_dir) {
    motor_run(MOTOR_FL, fl_speed, fl_dir);
    motor_run(MOTOR_FR, fr_speed, fr_dir);
    motor_run(MOTOR_RL, rl_speed, rl_dir);
    motor_run(MOTOR_RR, rr_speed, rr_dir);
}

void apply_line_pattern(int L, int M, int R, int *last_direction, const char *log_prefix) {

#ifdef RGB_SENSOR
    static int post_blue_forward = 0;
    static uint32_t post_blue_start = 0;
    uint32_t now = gpioTick();
    static int blue_waiting = 0;
    static uint32_t blue_start_time = 0;
    static int blue_lock = 0;
    ColorResult rgb = ColorLib_GetMatch();

    if (strcmp(rgb.name, "Red") == 0) {
        printf("RED → PERMANENT STOP\n");
        motor_stop_all();
        return;
    }

    if (strcmp(rgb.name, "Blue") == 0 && !blue_lock) {
        printf("BLUE → STOP 5 SECONDS\n");
        motor_stop_all();
        blue_waiting = 1;
        blue_start_time = now;
        blue_lock = 1;
        
    }

    if (blue_waiting) {
        if ((now - blue_start_time) < 5000000) {
            printf("BLUE → STOP 5 SECONDS\n");
            return;
        } else {
            printf("BLUE DONE → MOVE FORWARD BRIEFLY\n");
            blue_waiting = 0;
            post_blue_forward = 1;
            post_blue_start = now;
        }
    }

    if (post_blue_forward) {
        if ((now - post_blue_start) < 1000000) {
            drive_all_four(FULL_SPEED-20, FORWARD,
                           FULL_SPEED, FORWARD,
                           FULL_SPEED-20, FORWARD,
                           FULL_SPEED, FORWARD);
            return;
        } else { post_blue_forward = 0; }
    }

    if (strcmp(rgb.name, "Blue") != 0) blue_lock = 0;

#endif

    if (L == 0 && M == 1 && R == 0) {
        *last_direction = 0;
        drive_all_four(FULL_SPEED, FORWARD,
                       FULL_SPEED, FORWARD,
                       FULL_SPEED, FORWARD,
                       FULL_SPEED - 5, FORWARD);
        if (log_prefix != NULL)
            printf("%s Straight\n", log_prefix);
    
    } else if (L == 1 && M == 0 && R == 0) {
        *last_direction = -1;
        drive_all_four(TURN_SPEED, FORWARD,
                       HARD_SPEED, BACKWARD,
                       TURN_SPEED, FORWARD,
                       HARD_SPEED, BACKWARD);
        if (log_prefix != NULL)
            printf("%s hard right\n", log_prefix);
    
    } else if (L == 0 && M == 0 && R == 1) {
        *last_direction = 1;
        drive_all_four(HARD_SPEED, BACKWARD,
                       TURN_SPEED, FORWARD,
                       HARD_SPEED, BACKWARD,
                       TURN_SPEED, FORWARD);
        if (log_prefix != NULL)
            printf("%s hard left\n", log_prefix);
    
    
    } else if (L == 1 && M == 1 && R == 0) {
        *last_direction = -1;
        drive_all_four(TURN_SPEED, FORWARD,
                       SOFT_SPEED, BACKWARD,
                       TURN_SPEED, FORWARD,
                       SOFT_SPEED, BACKWARD);
        if (log_prefix != NULL)
            printf("%s soft right\n", log_prefix);
    
    } else if (L == 0 && M == 1 && R == 1) {
        *last_direction = 1;
        drive_all_four(SOFT_SPEED, BACKWARD,
                       TURN_SPEED, FORWARD,
                       SOFT_SPEED, BACKWARD,
                       TURN_SPEED, FORWARD);
        if (log_prefix != NULL)
            printf("%s soft left\n", log_prefix);
    
    } else {
        if (log_prefix != NULL)
            printf("%s SEARCHING...\n", log_prefix);

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