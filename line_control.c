#include <stdio.h>
#include <string.h>
#include <pigpio.h>


#include "ColorLib.h"


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
    uint32_t now = gpioTick();

//RGB logic 
#ifdef RGB_SENSOR
    static int blue_waiting = 0;
    static uint32_t blue_start_time = 0;
    static int blue_lock = 0;

    ColorResult rgb = ColorLib_GetMatch();

  //Red completely  STOP 
    if (strcmp(rgb.name, "Red") == 0)
    {
        printf("RED → PERMANENT STOP\n");
        motor_stop_all();
        return;
    }

    // Blue reading logic, stops for 5 seconds and lock the state and reset once the reading is not blue anymore
    if (strcmp(rgb.name, "Blue") == 0 && !blue_lock)
    {
        printf("BLUE → STOP 5 SECONDS\n");
        blue_waiting = 1;
        blue_start_time = now;
        blue_lock = 1;
       
        return;
    }

    // wait while pausing for 5 sec 
    if (blue_waiting)
    {
        if ((now - blue_start_time) < 5000000)
        {
             motor_stop_all();
            return;
        }
        else
        {
            printf("BLUE DONE → CONTINUE\n");
            blue_waiting = 0;
        }
    }

    //Reset lock after leaving blue 
    if (strcmp(rgb.name, "Blue") != 0)
        blue_lock = 0;

#endif

    // normal line logic 

    static int last_valid_line = 0;

    // straight 
    if (L == 0 && M == 1 && R == 0)
    {
        *last_direction = 0;
        last_valid_line = 1;
        drive_all_four(FULL_SPEED, FORWARD,
                       FULL_SPEED, FORWARD,
                       FULL_SPEED, FORWARD,
                       FULL_SPEED, FORWARD);
        return;
    }

    /* HARD RIGHT */
    if (L == 1 && M == 0 && R == 0)
    {
        *last_direction = -1;
        last_valid_line = 1;
        drive_all_four(TURN_SPEED, FORWARD,
                       HARD_SPEED, BACKWARD,
                       TURN_SPEED, FORWARD,
                       HARD_SPEED, BACKWARD);
        return;
    }

    /* HARD LEFT */
    if (L == 0 && M == 0 && R == 1)
    {
        *last_direction = 1;
        last_valid_line = 1;
        drive_all_four(HARD_SPEED, BACKWARD,
                       TURN_SPEED, FORWARD,
                       HARD_SPEED, BACKWARD,
                       TURN_SPEED, FORWARD);
        return;
    }

    /* SOFT RIGHT */
    if (L == 1 && M == 1 && R == 0)
    {
        *last_direction = -1;
        last_valid_line = 1;
        drive_all_four(TURN_SPEED, FORWARD,
                       HARD_SPEED, BACKWARD,
                       TURN_SPEED, FORWARD,
                       HARD_SPEED, BACKWARD);
        return;
    }

    /* SOFT LEFT */
    if (L == 0 && M == 1 && R == 1)
    {
        *last_direction = 1;
        last_valid_line = 1;
        drive_all_four(HARD_SPEED, BACKWARD,
                       TURN_SPEED, FORWARD,
                       HARD_SPEED, BACKWARD,
                       TURN_SPEED, FORWARD);
        return;
    }


    
// 000 lost handling logic, if 000 then keep going straight for few ms to see if that 000 is because of blue reading and in that few ms , rgb would read blue if that 000 is because of blue if not we go in search mode


static int lost000_mode = 0;
static int lost000_phase = 0;   // 0=grace, 1=turn1, 2=turn2
static uint32_t lost000_start = 0;
static int lost000_direction = 0;

#define LOST_GRACE_US  500000     // time to go straight to see if blue 
#define LOST_TURN_US   2005000   // time to search for 120 degree only  , not full 360 degree

if (L == 0 && M == 0 && R == 0)
{
    if (!lost000_mode)
    {
        lost000_mode = 1;
        lost000_start = now;
        lost000_direction = (*last_direction <= 0) ? -1 : 1;

        if (last_valid_line)
            lost000_phase = 0;   // grace forward
        else
            lost000_phase = 1;   // immediate search

        printf("ENTER 000 LOST MODE\n");
    }

    // PHASE 0: GRACE FORWARD 
    if (lost000_phase == 0)
    {
        if ((now - lost000_start) < LOST_GRACE_US)
        {
            if (last_direction == -1)
                drive_all_four(TURN_SPEED, FORWARD,
                       HARD_SPEED, BACKWARD,
                       TURN_SPEED, FORWARD,
                       HARD_SPEED, BACKWARD);
            else if (last_direction==1)
                drive_all_four(HARD_SPEED, BACKWARD,
                       TURN_SPEED, FORWARD,
                       HARD_SPEED, BACKWARD,
                       TURN_SPEED, FORWARD);
            else 
                
                drive_all_four(FULL_SPEED, FORWARD,
                       FULL_SPEED, FORWARD,
                       FULL_SPEED, FORWARD,
                       FULL_SPEED, FORWARD);
            return;
        }
        else
        {
            lost000_phase = 1;
            lost000_start = now;
            
        }
    }

    // PHASE 1: FIRST 120 degree TURN 
    if (lost000_phase == 1)
    {
        if ((now - lost000_start) < LOST_TURN_US)
        {
            if (lost000_direction == -1)
                drive_all_four(SEARCH_SPEED, FORWARD,
                               SEARCH_SPEED, BACKWARD,
                               SEARCH_SPEED, FORWARD,
                               SEARCH_SPEED, BACKWARD);
            else
                drive_all_four(SEARCH_SPEED, BACKWARD,
                               SEARCH_SPEED, FORWARD,
                               SEARCH_SPEED, BACKWARD,
                               SEARCH_SPEED, FORWARD);
            return;
        }
        else
        {
            lost000_phase = 2;
            lost000_start = now;
            lost000_direction *= -1;
        }
    }

    //PHASE 2: OPPOSITE 120 degree  TURN
    if (lost000_phase == 2)
    {
        if ((now - lost000_start) < LOST_TURN_US)
        {
            if (lost000_direction == -1)
                drive_all_four(SEARCH_SPEED, FORWARD,
                               SEARCH_SPEED, BACKWARD,
                               SEARCH_SPEED, FORWARD,
                               SEARCH_SPEED, BACKWARD);
            else
                drive_all_four(SEARCH_SPEED, BACKWARD,
                               SEARCH_SPEED, FORWARD,
                               SEARCH_SPEED, BACKWARD,
                               SEARCH_SPEED, FORWARD);
            return;
        }
        else
        {
            lost000_phase = 1;           // keep searching
            lost000_start = now;
            lost000_direction *= -1;
            return;
        }
    }
}
else
{
    lost000_mode = 0;
}

//111 LOST HANDLING 

static int lost111_mode = 0;
static int lost111_phase = 0;   // 1=turn1, 2=turn2
static uint32_t lost111_start = 0;
static int lost111_direction = 0;

if (L == 1 && M == 1 && R == 1)
{
    if (!lost111_mode)
    {
        lost111_mode = 1;
        lost111_start = now;
        lost111_direction = (*last_direction <= 0) ? -1 : 1;
        lost111_phase = 1;

        printf("ENTER 111 LOST MODE\n");
    }

    // FIRST 120 degree  TURN
    if (lost111_phase == 1)
    {
        if ((now - lost111_start) < LOST_TURN_US)
        {
            if (lost111_direction == -1)
                drive_all_four(SEARCH_SPEED, FORWARD,
                               SEARCH_SPEED, BACKWARD,
                               SEARCH_SPEED, FORWARD,
                               SEARCH_SPEED, BACKWARD);
            else
                drive_all_four(SEARCH_SPEED, BACKWARD,
                               SEARCH_SPEED, FORWARD,
                               SEARCH_SPEED, BACKWARD,
                               SEARCH_SPEED, FORWARD);
            return;
        }
        else
        {
            lost111_phase = 2;
            lost111_start = now;
            lost111_direction *= -1;
        }
    }

    // OPPOSITE 120 degree TURN 
    if (lost111_phase == 2)
    {
        if ((now - lost111_start) < LOST_TURN_US)
        {
            if (lost111_direction == -1)
                drive_all_four(SEARCH_SPEED, FORWARD,
                               SEARCH_SPEED, BACKWARD,
                               SEARCH_SPEED, FORWARD,
                               SEARCH_SPEED, BACKWARD);
            else
                drive_all_four(SEARCH_SPEED, BACKWARD,
                               SEARCH_SPEED, FORWARD,
                               SEARCH_SPEED, BACKWARD,
                               SEARCH_SPEED, FORWARD);
            return;
        }
        else
        {
            lost111_phase = 1;
            lost111_start = now;
            lost111_direction *= -1;
            return;
        }
    }
}
else
{
    lost111_mode = 0;
}}