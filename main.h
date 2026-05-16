#include <sys/types.h>

// toggle sensors
// #define AI_CAMERA
#define LINE_SENSORS
#define ULTRASONIC_SENSOR
#define IR_SENSORS
//#define RGB_SENSOR

#define CAMERA_APP "./ai_camera"
#define FIFO_PATH "/tmp/ai_camera_status"

// set various car speeds
/*
#define FULL_SPEED   0
#define HARD_SPEED   0
#define SOFT_SPEED   0
#define SEARCH_SPEED 0
*/

#define FULL_SPEED   100
#define TURN_SPEED   100
#define HARD_SPEED   50
#define SOFT_SPEED   30
#define SEARCH_SPEED 70

#define OBSTACLE_AVOID_DIST 200

#define OBSTACLE_DRIFT_RIGHT_US   200000u
#define OBSTACLE_FORWARD_TIMEOUT_US 1000000u
#define OBSTACLE_TURN_LEFT_US     700000u
#define OBSTACLE_REJOIN_TIMEOUT_US 200000u
#define IR_DEBOUNCE_COUNT         4
#define LINE_DEBOUNCE_COUNT       3

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
#define BUTTON_PIN 5 // NOT USING RIGHT NOW, DO IT LAST

void signal_handler(int sig);
pid_t launch_camera_process(void);
void* vision_reader_thread(void* arg);
void *ultrasonic_sensor_thread(void *arg);
int get_motor_speed_from_vision(const char* status);
