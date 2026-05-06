// #define CONTROL_MODE_CAMERA_ONLY
// #define CONTROL_MODE_LINE_SENSORS_ONLY
#define CONTROL_MODE_HYBRID
// #define ENABLE_OBSTACLE_AVOIDANCE

#define FIFO_PATH "/tmp/ai_camera_status"
#define CAUTION_SPEED 35
#define CLEAR_SPEED 100
#define CAMERA_APP "./ai_camera"
#define ECHO_PIN 23
#define TRIG_PIN 24
#define LEFT_LINE_SENSOR_PIN 22
#define MIDDLE_LINE_SENSOR_PIN 17
#define RIGHT_LINE_SENSOR_PIN 27


void signal_handler(int sig);
pid_t launch_camera_process(void);
void* vision_reader_thread(void* arg);
void *ultrasonic_sensor_thread(void *arg);
int get_motor_speed_from_vision(const char* status);
