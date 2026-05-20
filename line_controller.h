/**************************************************************
 * Class:: CSC-615-01 Spring 2026
 * Name:: Haibin Cao, Eric Ahsue, Kiran Khatri, John Tsiglieris
 * Student ID:: 922711514
 * Github-Name:: haibinc, Jasuv, khatri5034, John-Tsiglieris
 * Project::
 *
 * File:: line_controller.h
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

#ifndef LINE_CONTROL_H
#define LINE_CONTROL_H

void apply_line_pattern(int L, int M, int R, int *last_direction, const char *log_prefix);

#endif
