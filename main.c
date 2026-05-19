/**************************************************************
* Class:: CSC-615-01 Spring 2026
* Name:: Haibin Cao, Eric Ahsue, Kiran Khatri, John Tsiglieris
* Student ID:: 922711514
* Github-Name:: haibinc, Jasuv, khatri5034, John-Tsiglieris
* Project::
*
* File:: main.c
*
* Description::
* 
**************************************************************/

#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/stat.h>
#include <signal.h>
#include <sys/wait.h>
#include <stdint.h>
#include <time.h>
#include <pigpio.h>
#include <pthread.h>
#include "MotorDriver.h"
#include "DEV_Config.h"
#include "Debug.h"
#include "ColorLib.h"
#include "line_controller.h"
#include "main.h"

volatile int should_exit = 0;
static pthread_t threads[3];
static int thread_count = 0;
static int passed_obstacle = 0;

int line_sensor_state[3] = {1,1,1};
int ir_sensor_state[2] = {0,0};
int obstacle_state = 0;
float distance_cm = -1.0f;
pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;

static ObstaclePhase obstacle_phase = OBSTACLE_PHASE_IDLE;
static uint32_t obstacle_phase_start_tick = 0;
static int obstacle_ir_seen = 0;
static int obstacle_ir_clear_streak = 0;
static int obstacle_line_streak = 0;
static int blue_event_triggered = 0;
static uint32_t blue_timer_start = 0;
static int blue_waiting = 0;
static int red_stop_triggered = 0;

static void print_sensor_dashboard(int L, int M, int R,
                                   int left_ir, int right_ir,
                                   float dist_cm,
                                   int obstacle_active,
                                   ObstaclePhase phase,
                                   const char *cam_bias) {
    time_t now = time(NULL);
    struct tm *tm_now = localtime(&now);
    char ts[32] = "unknown";
    if (tm_now != NULL) strftime(ts, sizeof(ts), "%M:%S", tm_now);

#ifdef LINE_SENSORS
    printf("[Line]   L:%d M:%d R:%d\n", L, M, R);
#else
    printf("[Line]   DISABLED\n");
#endif

#ifdef ULTRASONIC_SENSOR
    if (dist_cm > 0.0f) {
        printf("[Ultra]  Distance: %.1f cm | Obstacle: %s\n", 
                dist_cm, obstacle_active ? "ACTIVE" : "CLEAR");
    } else {
        printf("[Ultra]  Distance: unknown | Obstacle: %s\n",
                obstacle_active ? "ACTIVE" : "CLEAR");
    }
#else
    printf("[Ultra]  DISABLED\n");
#endif

#ifdef IR_SENSORS
    printf("[IR]     Left:%d Right:%d\n", left_ir, right_ir);
    if (obstacle_active) {
        const char *phase_text = "IDLE";
        switch (phase) {
        case OBSTACLE_PHASE_DRIFT_RIGHT:
            phase_text = "DRIFT_RIGHT";
            break;
        case OBSTACLE_PHASE_FORWARD:
            phase_text = "FORWARD";
            break;
        case OBSTACLE_PHASE_TURN_LEFT:
            phase_text = "TURN_LEFT";
            break;
        case OBSTACLE_PHASE_REJOIN_LINE:
            phase_text = "REJOIN_LINE";
            break;
        default:
            break;
        }
        printf("[IR]     Obstacle phase: %s | IR seen:%d clear streak:%d\n",
               phase_text,
               obstacle_ir_seen,
               obstacle_ir_clear_streak);
    }
#else
    printf("[IR]     DISABLED\n");
#endif
    printf("[Time]   %s\n", ts);
    printf("========================================\n");
    fflush(stdout);
}

static void drive_all_four(UWORD fl_speed, Direction fl_dir,
                           UWORD fr_speed, Direction fr_dir,
                           UWORD rl_speed, Direction rl_dir,
                           UWORD rr_speed, Direction rr_dir) {
    motor_run(MOTOR_FL, fl_speed, fl_dir);
    motor_run(MOTOR_FR, fr_speed, fr_dir);
    motor_run(MOTOR_RL, rl_speed, rl_dir);
    motor_run(MOTOR_RR, rr_speed, rr_dir);
}

static void drive_obstacle_drift_right(void) {
    drive_all_four(FULL_SPEED,    FORWARD,
                   FULL_SPEED-20, BACKWARD,
                   FULL_SPEED-20, BACKWARD,
                   FULL_SPEED-20, FORWARD);
}

static void drive_obstacle_forward(void) {
    drive_all_four(FULL_SPEED, FORWARD,
                   FULL_SPEED, FORWARD,
                   FULL_SPEED, FORWARD,
                   FULL_SPEED, FORWARD);
}

static void drive_obstacle_turn_left_45(void) {
    drive_all_four(SOFT_SPEED, BACKWARD,
                   FULL_SPEED, FORWARD,
                   SOFT_SPEED, BACKWARD,
                   FULL_SPEED, FORWARD);
}

static void obstacle_avoidance_reset(void) {
    obstacle_phase = OBSTACLE_PHASE_IDLE;
    obstacle_phase_start_tick = 0;
    passed_obstacle = 0;
    obstacle_ir_seen = 0;
    obstacle_ir_clear_streak = 0;
    obstacle_line_streak = 0;
    obstacle_state = 0;
}

static void obstacle_avoidance_start(uint32_t now_tick) {
    obstacle_phase = OBSTACLE_PHASE_DRIFT_RIGHT;
    passed_obstacle = 0;
    obstacle_ir_seen = 0;
    obstacle_ir_clear_streak = 0;
    obstacle_line_streak = 0;
    obstacle_state = 1;
}

static int obstacle_avoidance_active(void) {
    return obstacle_phase != OBSTACLE_PHASE_IDLE;
}

static int obstacle_distance_cleared(void) {
    float distance_mm = (distance_cm) * 10.0f;
    return (distance_mm > (float)OBSTACLE_AVOID_DIST+100.0f);
}

static int obstacle_distance_triggered(void) {
    float distance_mm = distance_cm * 10.0f;
    return (distance_mm > 0.0f && distance_mm <= (float)OBSTACLE_AVOID_DIST);
}

static int obstacle_avoidance_step(int M, int left_ir, int right_ir, uint32_t now_tick) {
    int ir_blocked = (left_ir != 0) || (right_ir != 0);

    switch (obstacle_phase) {
    
    case OBSTACLE_PHASE_DRIFT_RIGHT:
        drive_obstacle_drift_right();
        if (obstacle_distance_cleared() && passed_obstacle == 0) {
            obstacle_phase_start_tick = now_tick;
            passed_obstacle = 1;
        }
        if (passed_obstacle == 1) {
            if ((now_tick - obstacle_phase_start_tick) >= OBSTACLE_DRIFT_RIGHT_US) {
                obstacle_phase = OBSTACLE_PHASE_FORWARD;
                obstacle_phase_start_tick = now_tick;
                obstacle_ir_seen = 0;
                obstacle_ir_clear_streak = 0;
            }
        }
        break;
    
    case OBSTACLE_PHASE_FORWARD:
        drive_obstacle_forward();

        if (ir_blocked) {
            obstacle_ir_seen = 1;
            obstacle_ir_clear_streak = 0;
        } else if (obstacle_ir_seen) {
            obstacle_ir_clear_streak++;
        }

        if (obstacle_ir_seen && obstacle_ir_clear_streak >= IR_DEBOUNCE_COUNT) {
            obstacle_phase = OBSTACLE_PHASE_TURN_LEFT;
            obstacle_phase_start_tick = now_tick;
        } else if ((now_tick - obstacle_phase_start_tick) >= OBSTACLE_FORWARD_TIMEOUT_US) {
            obstacle_phase = OBSTACLE_PHASE_TURN_LEFT;
            obstacle_phase_start_tick = now_tick;
        }
        break;

    case OBSTACLE_PHASE_TURN_LEFT:
        drive_obstacle_turn_left_45();
        if ((now_tick - obstacle_phase_start_tick) >= OBSTACLE_TURN_LEFT_US) {
            obstacle_phase = OBSTACLE_PHASE_REJOIN_LINE;
            obstacle_phase_start_tick = now_tick;
            obstacle_line_streak = 0;
        }
        break;

    case OBSTACLE_PHASE_REJOIN_LINE:
        drive_obstacle_forward();

        if (M == 0) {
            obstacle_line_streak++;
        } else {
            obstacle_line_streak = 0;
        }

        if (obstacle_line_streak >= LINE_DEBOUNCE_COUNT) {
            obstacle_avoidance_reset();
            return 0;
        }

        if ((now_tick - obstacle_phase_start_tick) >= OBSTACLE_REJOIN_TIMEOUT_US) {
            obstacle_avoidance_reset();
            return 0;
        }
        break;

    case OBSTACLE_PHASE_IDLE:
    default:
        return 0;
    }

    return obstacle_avoidance_active();
}


static int fail_startup(const char *message, int terminate_gpio) {
    printf("%s\n", message);
    if (terminate_gpio) gpioTerminate();
    return 1;
}


void signal_handler(int sig) {
    printf("\n[Main] Received signal %d, initiating shutdown\n", sig);
    should_exit = 1;
    motor_stop_all();
    for(int i = 0; i < thread_count; i++) {
        pthread_cancel(threads[i]);
        pthread_join(threads[i], NULL);
    }
    gpioTerminate();
    exit(0);
}

void *ultrasonic_sensor_thread(void *arg) {
    gpioSetMode(TRIG_PIN, PI_OUTPUT);
    gpioSetMode(ECHO_PIN, PI_INPUT);
    gpioWrite(TRIG_PIN, 0);

    while (!should_exit) {
        gpioWrite(TRIG_PIN, 0);
        gpioDelay(2);
        gpioWrite(TRIG_PIN, 1);
        gpioDelay(10);
        gpioWrite(TRIG_PIN, 0);

        uint32_t start_wait = gpioTick();
        uint32_t timeout = 30000;
        while (gpioRead(ECHO_PIN) == 0) {
            if ((gpioTick() - start_wait) > timeout) break;
        }
        uint32_t t_start = gpioTick();

        start_wait = gpioTick();
        while (gpioRead(ECHO_PIN) == 1) {
            if ((gpioTick() - start_wait) > timeout) break;
        }
        uint32_t t_end = gpioTick();

        if (t_end > t_start) {
            uint32_t diff = t_end - t_start;
            float dist = diff / 58.0f;
            pthread_mutex_lock(&lock);
            distance_cm = dist;
            obstacle_state = 1;
            pthread_mutex_unlock(&lock);
        } else {
            pthread_mutex_lock(&lock);
            distance_cm = -1.0f;
            obstacle_state = 0;
            pthread_mutex_unlock(&lock);
        }

        gpioDelay(60000);
    }

    return NULL;
}

void *line_sensor_thread(void *arg) {
    gpioSetMode(LEFT_LINE_SENSOR_PIN, PI_INPUT);
    gpioSetMode(MIDDLE_LINE_SENSOR_PIN, PI_INPUT);
    gpioSetMode(RIGHT_LINE_SENSOR_PIN, PI_INPUT);
    gpioSetPullUpDown(LEFT_LINE_SENSOR_PIN, PI_PUD_UP);
    gpioSetPullUpDown(MIDDLE_LINE_SENSOR_PIN, PI_PUD_UP);
    gpioSetPullUpDown(RIGHT_LINE_SENSOR_PIN, PI_PUD_UP);

    while (!should_exit) {
        int left_value = gpioRead(LEFT_LINE_SENSOR_PIN);
        int middle_value = gpioRead(MIDDLE_LINE_SENSOR_PIN);
        int right_value = gpioRead(RIGHT_LINE_SENSOR_PIN);

        pthread_mutex_lock(&lock);
        line_sensor_state[0] = left_value;
        line_sensor_state[1] = middle_value;
        line_sensor_state[2] = right_value;
        pthread_mutex_unlock(&lock);

        gpioDelay(20000);
    }

    return NULL;
}

void *ir_sensor_thread(void *arg) {
    gpioSetMode(LEFT_IR_PIN, PI_INPUT);
    gpioSetMode(RIGHT_IR_PIN, PI_INPUT);

    while (!should_exit) {
        int left_value = gpioRead(LEFT_IR_PIN);
        int right_value = gpioRead(RIGHT_IR_PIN);

        pthread_mutex_lock(&lock);
        ir_sensor_state[0] = left_value;
        ir_sensor_state[1] = right_value;
        pthread_mutex_unlock(&lock);

        gpioDelay(20000);
    }

    return NULL;
}

int main(void) {
    printf("[Main] Initializing pigpio\n");
    if (gpioInitialise() < 0)
        return fail_startup("[Main] ERROR: pigpio initialization failed", 0);

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    printf("[Main] Initializing DEV module\n");
    if (DEV_ModuleInit() != 0)
        return fail_startup("[Main] ERROR: DEV_ModuleInit failed", 1);

    printf("[Main] Initializing motor\n");
    motor_init();
    if (motor_stop_all() != 0)
        return fail_startup("[Main] ERROR: Failed to stop motors at startup", 1);

#ifdef RGB_SENSOR
    printf("[Main] Initializing RGB sensor\n");
    if (ColorLib_Init() != 0)
        return fail_startup("[Main] ERROR: TCS34725 Sensor not found", 1);
#endif

#ifdef LINE_SENSORS
    printf("[Main] Starting line sensor thread\n");
    if (pthread_create(&threads[thread_count++], NULL, line_sensor_thread, NULL) != 0)
        return fail_startup("[Main] ERROR: Failed to create line sensor thread", 1);
#endif

#ifdef ULTRASONIC_SENSOR
    printf("[Main] Starting ultrasonic sensor thread\n");
    if (pthread_create(&threads[thread_count++], NULL, ultrasonic_sensor_thread, NULL) != 0)
        return fail_startup("[Main] ERROR: Failed to create ultrasonic thread", 1);
#endif

#ifdef IR_SENSORS
    printf("[Main] Starting IR sensor thread\n");
    if (pthread_create(&threads[thread_count++], NULL, ir_sensor_thread, NULL) != 0)
        return fail_startup("[Main] ERROR: Failed to create IR sensor thread", 1);
#endif

    int last_direction = 0;
    int last_seen_L = 1, last_seen_M = 1, last_seen_R = 1;
    int recovery_mode = 0;
    int recovery_dir = 0;
    int forced_line_mode = 0;
    uint32_t forced_line_start = 0;
    #define FORCED_LINE_DURATION_US (5000u * 1000u)
    int last_hybrid_recovery_mode = -1;
    int last_hybrid_recovery_dir = 0;
    char last_hybrid_bias[16] = "";
    uint32_t last_unknown_history_log_tick = 0;
    #ifdef RGB_SENSOR
    uint32_t last_rgb_print_tick = 0;
    #endif

    while (!should_exit) {
        int L, M, R;
        int left_ir = 0;
        int right_ir = 0;
        float current_distance = -1.0f;
        char local_bias[16];
        pthread_mutex_lock(&lock);
        L = line_sensor_state[0];
        M = line_sensor_state[1];
        R = line_sensor_state[2];
        current_distance = distance_cm;
        pthread_mutex_unlock(&lock);

    #ifdef IR_SENSORS
        pthread_mutex_lock(&lock);
        left_ir = ir_sensor_state[0];
        right_ir = ir_sensor_state[1];
        pthread_mutex_unlock(&lock);
    #endif

        apply_line_pattern(L, M, R, &last_direction, NULL);

        printf("\033[2J\033[H");
        print_sensor_dashboard(L, M, R,
                            left_ir, right_ir,
                            current_distance,
                            obstacle_avoidance_active(),
                            obstacle_phase,
                            local_bias);
     
        if (!obstacle_avoidance_active() && obstacle_distance_triggered()) {
            obstacle_avoidance_start(gpioTick());
        }

        if (obstacle_avoidance_active()) {
            uint32_t now_tick = gpioTick();
            if (obstacle_avoidance_step(M, left_ir, right_ir, now_tick)) {
                gpioDelay(1000);
                continue;
            }
        }

        gpioDelay(1000);
        
    }

    should_exit = 1;
    motor_stop_all();
    for (int i = 0; i < thread_count; i++) {
        pthread_join(threads[i], NULL);
    }
    gpioTerminate();
    printf("[Main] Motor control system shutdown complete.\n");
    return 0;
}
