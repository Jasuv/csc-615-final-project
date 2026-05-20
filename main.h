/**************************************************************
* Class:: CSC-615-01 Spring 2026
* Name:: Haibin Cao, Eric Ahsue, Kiran Khatri, John Tsiglieris
* Student ID:: 922711514
* Github-Name:: haibinc, Jasuv, khatri5034, John-Tsiglieris
* Project::
*
* File:: main.h
*
* Description::
* 
* Full implementation of main loop for self driving car:
* * modular sensor threads for all sensors to work individually
* * waits for button press to start program
* * ultrasonic sensor: detects obstacles and initiates avoidance routine
* * line sensors: detects line and adjusts motor speeds accordingly
* * RGB sensor: detects important colors and reacts (red = stop, blue = wait 5 seconds)
* * IR sensors: used for obstacle avoidance routine to detect when obstacle is passed
* 
**************************************************************/

// toggle sensors (for testing purposes)
#define LINE_SENSORS
#define ULTRASONIC_SENSOR
#define IR_SENSORS
#define RGB_SENSOR

 //set various car speeds
#define FULL_SPEED   100
#define TURN_SPEED   100
#define HARD_SPEED   70
#define SOFT_SPEED   50
#define SEARCH_SPEED 70

#define OBSTACLE_AVOID_DIST         250      // 25cm
#define OBSTACLE_DRIFT_RIGHT_US     500000u  // 500ms
#define OBSTACLE_FORWARD_TIMEOUT_US 1000000u // 1s
#define OBSTACLE_TURN_LEFT_US       700000u  // 700ms
#define OBSTACLE_REJOIN_TIMEOUT_US  7000000u // 7s
#define IR_DEBOUNCE_COUNT           10       // 10s
#define LINE_DEBOUNCE_COUNT         1        // 1s

typedef enum {
    OBSTACLE_PHASE_IDLE = 0,
    OBSTACLE_PHASE_DRIFT_RIGHT,
    OBSTACLE_PHASE_FORWARD,
    OBSTACLE_PHASE_TURN_LEFT,
    OBSTACLE_PHASE_REJOIN_LINE
} ObstaclePhase;

#define ECHO_PIN 23
#define TRIG_PIN 24
#define LEFT_LINE_SENSOR_PIN 22
#define MIDDLE_LINE_SENSOR_PIN 27
#define RIGHT_LINE_SENSOR_PIN 17
#define LEFT_IR_PIN 21
#define RIGHT_IR_PIN 20
#define BUTTON_PIN 5

void signal_handler(int sig);
pid_t launch_camera_process(void);
void* vision_reader_thread(void* arg);
void *ultrasonic_sensor_thread(void *arg);
int get_motor_speed_from_vision(const char* status);
