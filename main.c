/**************************************************************
* Class:: CSC-615-01 Spring 2026
* Name:: Eric Ahsue
* Student ID:: 922711514
* Github-Name:: Jasuv
* Project:: Assignment 3 - Motor Control with AI Vision
*
* File:: main_ai_integrated.c
*
**************************************************************/

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
#include "camera_control.h"
#include "line_control.h"
#include "main.h"

volatile char current_status[32] = "INIT";
volatile int should_exit = 0;
volatile pid_t camera_pid = 0;
volatile int camera_ready = 0;
volatile char camera_bias[16] = "CENTER";

static pthread_t threads[3];
static int thread_count = 0;
static int passed_obstacle = 0;

int line_sensor_state[3] = {1,1,1};
int ir_sensor_state[2] = {0,0};
int obstacle_state = 0;
float distance_cm = -1.0f;
pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;

#ifdef __cplusplus
extern "C" {
#endif
static void drive_all_four(UWORD fl_speed, Direction fl_dir,
                           UWORD fr_speed, Direction fr_dir,
                           UWORD rl_speed, Direction rl_dir,
                           UWORD rr_speed, Direction rr_dir);
#ifdef __cplusplus
}
#endif

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
    if (tm_now != NULL) {
        strftime(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S", tm_now);
    }

#ifdef AI_CAMERA
    printf("[Cam]    Bias:%-10s\n", cam_bias);
#else
    printf("[Cam]    DISABLED\n");
#endif

#ifdef LINE_SENSORS
    printf("[Line]   L:%d M:%d R:%d\n", L, M, R);
#else
    printf("[Line]   DISABLED\n");
#endif

#ifdef ULTRASONIC_SENSOR
    if (dist_cm > 0.0f) {
        printf("[Ultra]  Distance: %.1f cm | Obstacle: %s\n",
               dist_cm,
               obstacle_active ? "ACTIVE" : "CLEAR");
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

static void drive_obstacle_drift_right(void)
{
    drive_all_four(100, FORWARD,    //fr
                   80, BACKWARD,   //fl
                   80, BACKWARD,   //rr
                   80, FORWARD);   //rl
}

static void drive_obstacle_forward(void)
{
    drive_all_four(FULL_SPEED, FORWARD,
                   FULL_SPEED, FORWARD,
                   FULL_SPEED, FORWARD,
                   FULL_SPEED, FORWARD);
}

static void drive_obstacle_turn_left_45(void)
{
    drive_all_four(SOFT_SPEED, BACKWARD,
                   FULL_SPEED, FORWARD,
                   SOFT_SPEED, BACKWARD,
                   FULL_SPEED, FORWARD);
}

static void obstacle_avoidance_reset(void)
{
    obstacle_phase = OBSTACLE_PHASE_IDLE;
    obstacle_phase_start_tick = 0;
    passed_obstacle = 0;
    obstacle_ir_seen = 0;
    obstacle_ir_clear_streak = 0;
    obstacle_line_streak = 0;
    obstacle_state = 0;
}

static void obstacle_avoidance_start(uint32_t now_tick)
{
    obstacle_phase = OBSTACLE_PHASE_DRIFT_RIGHT;
    passed_obstacle = 0;
    obstacle_ir_seen = 0;
    obstacle_ir_clear_streak = 0;
    obstacle_line_streak = 0;
    obstacle_state = 1;
}

static int obstacle_avoidance_active(void)
{
    return obstacle_phase != OBSTACLE_PHASE_IDLE;
}

static int obstacle_distance_cleared(void)
{
    float distance_mm = (distance_cm) * 10.0f;
    return (distance_mm > (float)OBSTACLE_AVOID_DIST+100.0f);
}

static int obstacle_distance_triggered(void)
{
    float distance_mm = distance_cm * 10.0f;
    return (distance_mm > 0.0f && distance_mm <= (float)OBSTACLE_AVOID_DIST);
}

static int obstacle_avoidance_step(int M, int left_ir, int right_ir, uint32_t now_tick)
{
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

static int fail_startup(const char *message, int terminate_gpio)
{
    printf("%s\n", message);

    if (terminate_gpio) {
        gpioTerminate();
    }

    if (camera_pid > 0) {
        kill(camera_pid, SIGTERM);
    }

    return 1;
}


void signal_handler(int sig) {
    printf("\n[Main] Received signal %d, initiating shutdown\n", sig);
    should_exit = 1;
    
    /* Kill the camera process if it's running */
    if (camera_pid > 0) {
        printf("[Main] Terminating camera process (PID: %d)\n", camera_pid);
        kill(camera_pid, SIGTERM);
        sleep(1);
        
        /* Force kill if still running */
        if (kill(camera_pid, 0) == 0) {
            printf("[Main] Force killing camera process\n");
            kill(camera_pid, SIGKILL);
        }
    }
    should_exit = 1;
    motor_stop_all();
    for(int i = 0; i < thread_count; i++)
    {
        pthread_cancel(threads[i]);
        pthread_join(threads[i], NULL);
    }
    gpioTerminate();
    exit(0);
}

/*
 * Launch compiled camera application as background process
 */
pid_t launch_camera_process(void) {
    pid_t pid = fork();

    if (pid < 0) {
        printf("[Main] ERROR: Failed to fork process\n");
        return -1;
    } else if (pid == 0) {
        /* Child process: execute compiled camera application */

        /* Close stdin so child doesn't block on input */
        close(STDIN_FILENO);

        /* Redirect stdout/stderr to avoid interfering with C program output */
        int devnull = open("/dev/null", O_WRONLY);
        dup2(devnull, STDOUT_FILENO);
        dup2(devnull, STDERR_FILENO);
        close(devnull);

        /* Execute camera application from the PATH */
        execlp(CAMERA_APP, CAMERA_APP, (char*)NULL);

        /* If execlp fails, exit child */
        perror("[Camera] ERROR: Failed to execute camera application");
        exit(1);
    } else {
        /* Parent process: return child PID */
        printf("[Main] Camera process launched (PID: %d)\n", pid);
        return pid;
    }
}

void* camera_thread(void* arg)
{
    int fifo_fd;
    char buffer[64];
    ssize_t bytes_read;

    /* Wait until FIFO exists */
    while (access(FIFO_PATH, F_OK) == -1 && !should_exit)
        usleep(100000);

    /* Open FIFO with retry */
    while (!should_exit) {
        fifo_fd = open(FIFO_PATH, O_RDONLY | O_NONBLOCK);
        if (fifo_fd >= 0) break;
        sleep(1);
    }

    if (fifo_fd < 0) {
        perror("[CAMERA] FIFO open failed");
        return NULL;
    }

    printf("[CAMERA] Connected to FIFO\n");

    while (!should_exit)
    {
        memset(buffer, 0, sizeof(buffer));
        bytes_read = read(fifo_fd, buffer, sizeof(buffer) - 1);

        if (bytes_read > 0)
        {
            /* Remove newline */
            if (buffer[bytes_read - 1] == '\n')
                buffer[bytes_read - 1] = '\0';
            else
                buffer[bytes_read] = '\0';

            // printf("[CAMERA] raw: '%s'\n", buffer);

            /* Mark camera ready immediately after first valid read. */
            camera_ready = 1;

            pthread_mutex_lock(&lock);
            strncpy((char*)camera_bias, buffer, sizeof(camera_bias) - 1);
            camera_bias[sizeof(camera_bias) - 1] = '\0';
            pthread_mutex_unlock(&lock);

            // printf("[CAMERA] Bias: '%s'\n", camera_bias);
        }
        else if (bytes_read < 0 && errno != EAGAIN && errno != EWOULDBLOCK)
        {
            /* Real error - try to reconnect */
            printf("[CAMERA] FIFO error, reconnecting\n");
            close(fifo_fd);
            while (!should_exit) {
                fifo_fd = open(FIFO_PATH, O_RDONLY | O_NONBLOCK);
                if (fifo_fd >= 0) break;
                sleep(1);
            }
        }

        usleep(50000);
    }

    close(fifo_fd);
    return NULL;
}

void *ultrasonic_sensor_thread(void *arg)
{
    /* HC-SR04 style measurement using TRIG_PIN and ECHO_PIN */
    gpioSetMode(TRIG_PIN, PI_OUTPUT);
    gpioSetMode(ECHO_PIN, PI_INPUT);
    gpioWrite(TRIG_PIN, 0);

    while (!should_exit) {
        /* Trigger a 10us pulse */
        gpioWrite(TRIG_PIN, 0);
        gpioDelay(2);
        gpioWrite(TRIG_PIN, 1);
        gpioDelay(10);
        gpioWrite(TRIG_PIN, 0);

        /* Wait for echo high with timeout */
        uint32_t start_wait = gpioTick();
        uint32_t timeout = 30000; /* 30ms timeout */
        while (gpioRead(ECHO_PIN) == 0) {
            if ((gpioTick() - start_wait) > timeout) break;
        }
        uint32_t t_start = gpioTick();

        /* Measure how long echo stays high */
        start_wait = gpioTick();
        while (gpioRead(ECHO_PIN) == 1) {
            if ((gpioTick() - start_wait) > timeout) break;
        }
        uint32_t t_end = gpioTick();

        if (t_end > t_start) {
            uint32_t diff = t_end - t_start; /* microseconds */
            float dist = diff / 58.0f; /* convert to cm */
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

        gpioDelay(60000); /* ~60ms between measurements */
    }

    return NULL;
}

void *line_sensor_thread(void *arg)
{
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

void *ir_sensor_thread(void *arg)
{
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
    if (gpioInitialise() < 0) {
        return fail_startup("[Main] ERROR: pigpio initialization failed", 0);
    }

    #ifdef AI_CAMERA
        camera_pid = launch_camera_process();
        if (camera_pid < 0) {
            return fail_startup("[Main] ERROR: Failed to launch camera application", 1);
        }
    #endif

    /* Setup signal handlers for graceful shutdown */
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    printf("[Main] Initializing DEV module\n");
    if (DEV_ModuleInit() != 0) {
        return fail_startup("[Main] ERROR: DEV_ModuleInit failed", 1);
    }

    #ifdef RGB_SENSOR
    printf("[Main] Initializing RGB sensor\n");
    if (ColorLib_Init() != 0) {
        return fail_startup("[Main] ERROR: TCS34725 Sensor not found", 1);
    }
    #endif

    printf("[Main] Initializing motor\n");
    motor_init();
    if (motor_stop_all() != 0) {
        return fail_startup("[Main] ERROR: Failed to stop motors at startup", 1);
    }

    /* Start vision reader thread */
    #ifdef AI_CAMERA
        printf("[Main] Starting vision reader thread\n");
        if (pthread_create(&threads[thread_count++], NULL, camera_thread, NULL) != 0) {
            return fail_startup("[Main] ERROR: Failed to create vision reader thread", 1);
        }
    #endif

    /* Start line sensor thread (3-sensor) */
    #ifdef LINE_SENSORS
    printf("[Main] Starting line sensor thread\n");
    if (pthread_create(&threads[thread_count++], NULL, line_sensor_thread, NULL) != 0) {
        return fail_startup("[Main] ERROR: Failed to create line sensor thread", 1);
    }
    #endif

    /* Start ultrasonic distance thread */
    #ifdef ULTRASONIC_SENSOR
    printf("[Main] Starting ultrasonic sensor thread\n");
    if (pthread_create(&threads[thread_count++], NULL, ultrasonic_sensor_thread, NULL) != 0) {
        return fail_startup("[Main] ERROR: Failed to create ultrasonic thread", 1);
    }
    #else
    /* TODO: Add ultrasonic sensor handling here. */
    #endif

    #ifdef IR_SENSORS
    printf("[Main] Starting IR sensor thread\n");
    if (pthread_create(&threads[thread_count++], NULL, ir_sensor_thread, NULL) != 0) {
        return fail_startup("[Main] ERROR: Failed to create IR sensor thread", 1);
    }
    #endif

    #ifdef AI_CAMERA
        printf("[Main] Waiting for camera to send first frame\n");
        while (!should_exit && !camera_ready)
        {
            printf("[Main] Waiting for camera data\n");
            usleep(500000);  // check every 500ms
        }
        printf("[Main] Camera sending data — starting motors\n");
    #endif


    printf("\n[Main] Motor control starting\n");
    #if defined(AI_CAMERA) && defined(LINE_SENSORS)
        printf("[Main] Mode: Camera + Line Sensors)\n");
    #elif defined(AI_CAMERA)
        printf("[Main] Mode: Camera only\n");
    #elif defined(LINE_SENSORS)
        printf("[Main] Mode: Line sensors only\n");
    #endif
    printf("[Main] Press Ctrl+C to exit\n\n");


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
        /* Correct mapping: index 0=LEFT, 1=MIDDLE, 2=RIGHT */
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

        pthread_mutex_lock(&lock);
        strcpy(local_bias, (char*)camera_bias);
        pthread_mutex_unlock(&lock);

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

    #if defined(AI_CAMERA) && !defined(LINE_SENSORS)
        /* ===== CAMERA ONLY ===== */
        apply_camera_bias(local_bias, "[CAM]");

    #elif defined(LINE_SENSORS) && !defined(AI_CAMERA)
        /* ===== LINE SENSORS ONLY ===== */
        apply_line_pattern(L, M, R, &last_direction, NULL);

    #elif defined(AI_CAMERA) && defined(LINE_SENSORS)
        /* ===== HYBRID: Camera + Line Sensors ===== */
        last_hybrid_recovery_mode = recovery_mode;
        last_hybrid_recovery_dir = recovery_dir;
        strncpy(last_hybrid_bias, local_bias, sizeof(last_hybrid_bias) - 1);
        last_hybrid_bias[sizeof(last_hybrid_bias) - 1] = '\0';

        /* Update recent sensor history when any sensor detects the line */
        if (L == 0 || M == 0 || R == 0) {
            last_seen_L = L;
            last_seen_M = M;
            last_seen_R = R;
        }

        /* Forced-line timeout handling */
        uint32_t now_tick = gpioTick();
        if (forced_line_mode) {
            if ((now_tick - forced_line_start) >= FORCED_LINE_DURATION_US) {
                forced_line_mode = 0;
            }
        }

        /* If already in recovery, continue harsh rotation until middle sensor triggers */
        if (recovery_mode) {
            if (recovery_dir == -1) {
                /* hard right rotation: right side reverse, left side forward */
                drive_all_four(FULL_SPEED, FORWARD,
                            FULL_SPEED, BACKWARD,
                            FULL_SPEED, FORWARD,
                            FULL_SPEED, BACKWARD);
            } else if (recovery_dir == 1) {
                /* hard left rotation: left side reverse, right side forward */
                drive_all_four(FULL_SPEED, BACKWARD,
                            FULL_SPEED, FORWARD,
                            FULL_SPEED, BACKWARD,
                            FULL_SPEED, FORWARD);
            }

            /* Exit recovery only when middle sensor sees the line */
            if (M == 0) {
                recovery_mode = 0;
                recovery_dir = 0;
                last_seen_L = last_seen_M = last_seen_R = 1;
            }

            gpioDelay(1000);
            continue;
        }

        /* If forced_line_mode is active, run the sensor-only decision tree now */
        if (forced_line_mode) {
            apply_line_pattern(L, M, R, &last_direction, "[HYBRID-LINE]");
            gpioDelay(1000);
            continue;
        }

        /* If sensors are all OFF (no detection) consult last seen to decide a hard
        turn; otherwise (mixed readings) prefer the camera's decision. */
        if (L == 1 && M == 1 && R == 1) {
            /* All sensors off — use last seen to choose a recovery hard turn */
            if (last_seen_R == 0) {
                /* middle+right previously saw line → start hard right recovery */
                recovery_mode = 1;
                recovery_dir = -1;
                /* enter forced line-mode for 5s to ensure turn completes */
                forced_line_mode = 1;
                forced_line_start = now_tick;
                motor_run(MOTOR_FR, FULL_SPEED, BACKWARD);
                motor_run(MOTOR_FL, FULL_SPEED, FORWARD);
                motor_run(MOTOR_RR, FULL_SPEED, BACKWARD);
                motor_run(MOTOR_RL, FULL_SPEED, FORWARD);
                /* keep history until recovery completes (wait for middle sensor) */
            } else if (last_seen_L == 0) {
                /* middle+left previously saw line → start hard left recovery */
                recovery_mode = 1;
                recovery_dir = 1;
                /* enter forced line-mode for 5s to ensure turn completes */
                forced_line_mode = 1;
                forced_line_start = now_tick;
                motor_run(MOTOR_FR, FULL_SPEED, FORWARD);
                motor_run(MOTOR_FL, FULL_SPEED, BACKWARD);
                motor_run(MOTOR_RR, FULL_SPEED, FORWARD);
                motor_run(MOTOR_RL, FULL_SPEED, BACKWARD);
                /* keep history until recovery completes (wait for middle sensor) */
            } else {
                last_unknown_history_log_tick = now_tick;
                /* Unknown history — fallback to camera; clear history so we don't loop */
                last_seen_L = last_seen_M = last_seen_R = 1;
                if (strcmp(local_bias, "CENTER") == 0) {
                    motor_run(MOTOR_FR, FULL_SPEED, FORWARD);
                    motor_run(MOTOR_FL, FULL_SPEED, FORWARD);
                    motor_run(MOTOR_RR, FULL_SPEED, FORWARD);
                    motor_run(MOTOR_RL, FULL_SPEED, FORWARD);
                } else if (strcmp(local_bias, "LEFT") == 0) {
                    motor_run(MOTOR_FR, FULL_SPEED, FORWARD);
                    motor_run(MOTOR_FL, HARD_SPEED, FORWARD);
                    motor_run(MOTOR_RR, FULL_SPEED, FORWARD);
                    motor_run(MOTOR_RL, HARD_SPEED, FORWARD);
                } else if (strcmp(local_bias, "RIGHT") == 0) {
                    motor_run(MOTOR_FR, HARD_SPEED, FORWARD);
                    motor_run(MOTOR_FL, FULL_SPEED, FORWARD);
                    motor_run(MOTOR_RR, HARD_SPEED, FORWARD);
                    motor_run(MOTOR_RL, FULL_SPEED, FORWARD);
                } else {
                    motor_stop_all();
                }
            }
        } else {
            /* Sensors are mixed — prefer camera steering decisions */
            /* Detect immediate 90° sensor patterns and force line mode */
            if (L == 1 && M == 1 && R == 0) {
                /* hard right indicated by sensors — start forced line-mode */
                forced_line_mode = 1;
                forced_line_start = now_tick;
                last_direction = -1;
                drive_all_four(TURN_SPEED, FORWARD,
                            TURN_SPEED, BACKWARD,
                            TURN_SPEED, FORWARD,
                            TURN_SPEED, BACKWARD);
                gpioDelay(1000);
                continue;
            } else if (L == 0 && M == 1 && R == 1) {
                /* hard left indicated by sensors — start forced line-mode */
                forced_line_mode = 1;
                forced_line_start = now_tick;
                last_direction = 1;
                drive_all_four(TURN_SPEED, BACKWARD,
                            TURN_SPEED, FORWARD,
                            TURN_SPEED, BACKWARD,
                            TURN_SPEED, FORWARD);
                gpioDelay(1000);
                continue;
            }
            apply_camera_bias(local_bias, NULL);
        }

        #ifdef ULTRASONIC_SENSOR
        /* TODO: Add ultrasonic-aware drive logic here. */
        #endif

        #ifdef IR_SENSORS
        /* TODO: Add IR sensor-aware drive logic here. */
        #endif

    #endif

        gpioDelay(1000);
        
    }

    printf("\n[Main] Vision feedback stopped. Stopping motors\n");

    should_exit = 1;
    motor_stop_all();
    for (int i = 0; i < thread_count; i++) {
        pthread_join(threads[i], NULL);
    }
    
    if (camera_pid > 0) {
        printf("[Main] Terminating camera process\n");
        kill(camera_pid, SIGTERM);
        sleep(1);
        if (kill(camera_pid, 0) == 0) {
            kill(camera_pid, SIGKILL);
        }
        waitpid(camera_pid, NULL, 0);
    }
    
    gpioTerminate();

    printf("[Main] Motor control system shutdown complete.\n");
    return 0;
}
