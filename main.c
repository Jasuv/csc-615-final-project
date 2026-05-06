/**************************************************************
* Class:: CSC-615-01 Spring 2026
* Name:: Eric Ahsue
* Student ID:: 922711514
* Github-Name:: Jasuv
* Project:: Assignment 3 - Motor Control with AI Vision
*
* File:: main_ai_integrated.c
*
* Description:: Motor control system driven by AI camera feedback
* via named pipe (FIFO). Motor speed is controlled by vision:
* - CLEAR   = Full speed (100%)
* - CAUTION = Reduced speed (50%)
* - STOP    = Motor stops
*
 * Usage: Build and run the compiled camera app in one terminal, run this in another:
 *   gcc ai_camera.c -o ai_camera
 *   ./ai_camera
 *   gcc main_ai_integrated.c MotorDriver.c lib/*.c -lpigpio -o motor_ai
 *   ./motor_ai
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
#include <pigpio.h>
#include <pthread.h>
#include "MotorDriver.h"
#include "DEV_Config.h"
#include "Debug.h"
#include "main.h"

#define FIFO_PATH "/tmp/ai_camera_status"
#define CAUTION_SPEED 35
#define CLEAR_SPEED 500
#define CAMERA_APP "./ai_camera"

/* Global variables for thread communication */
volatile char current_status[32] = "INIT";
volatile int should_exit = 0;
volatile pid_t camera_pid = 0;
volatile int camera_ready = 0;

static pthread_t threads[3];
static int thread_count = 3;

/* Shared sensor state + sync */
int line_sensor_state[3] = {1,1,1}; /* 0 = sensor detects line */
int obstacle_state = 0;
float distance_cm = -1.0f; /* measured distance by ultrasonic; -1 = unknown */
pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;

// camera thread
volatile char camera_bias[16] = "CENTER";


void signal_handler(int sig) {
    printf("\n[Main] Received signal %d, initiating shutdown...\n", sig);
    should_exit = 1;
    
    /* Kill the camera process if it's running */
    if (camera_pid > 0) {
        printf("[Main] Terminating camera process (PID: %d)...\n", camera_pid);
        kill(camera_pid, SIGTERM);
        sleep(1);
        
        /* Force kill if still running */
        if (kill(camera_pid, 0) == 0) {
            printf("[Main] Force killing camera process...\n");
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

            printf("[CAMERA] raw: '%s'\n", buffer);

            pthread_mutex_lock(&lock);
            strncpy((char*)camera_bias, buffer, sizeof(camera_bias) - 1);
            camera_bias[sizeof(camera_bias) - 1] = '\0';
            camera_ready = 1;
            pthread_mutex_unlock(&lock);

            printf("[CAMERA] Bias: '%s'\n", camera_bias);
        }
        else if (bytes_read < 0 && errno != EAGAIN && errno != EWOULDBLOCK)
        {
            /* Real error - try to reconnect */
            printf("[CAMERA] FIFO error, reconnecting...\n");
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

        // gpioDelay(20000);
    }

    return NULL;
}

void *ir_sensor_thread(void *arg)
{
    /*
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

        // gpioDelay(20000);
    }
    */

    return NULL;
}

/*
 * Determine motor speed based on vision status
 */
int get_motor_speed_from_vision(const char* status) {
    if (strcmp(status, "STOP") == 0) {
        return 0;
    } else if (strcmp(status, "CAUTION") == 0) {
        return CAUTION_SPEED;
    } else {
        return CLEAR_SPEED;
    }
}



int main(void) {
    printf("========================================\n");
    printf("  AI-Driven Motor Control System\n");
    printf("  Dual Motor (A & B) Configuration\n");
    printf("  Running on Raspberry Pi 4\n");
    printf("========================================\n\n");

    /* Initialize ultrasonic sensor GPIO pins*/
	printf("Setting GPIO pins to output\n");
	gpioSetMode(ECHO_PIN, PI_INPUT);
	gpioSetMode(TRIG_PIN, PI_OUTPUT);
	printf("Finished setting GPIO pins\n");

    camera_pid = launch_camera_process();
    if (camera_pid < 0) {
        printf("[Main] ERROR: Failed to launch camera application\n");
        return 1;
    }

    printf("[Main] Initializing pigpio...\n");
    if (gpioInitialise() < 0) {
        printf("[Main] ERROR: pigpio initialization failed\n");
        if (camera_pid > 0) kill(camera_pid, SIGTERM);
        return 1;
    }

    /* Setup signal handlers for graceful shutdown */
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    printf("[Main] Initializing DEV module...\n");
    if (DEV_ModuleInit() != 0) {
        printf("[Main] ERROR: DEV_ModuleInit failed\n");
        gpioTerminate();
        if (camera_pid > 0) kill(camera_pid, SIGTERM);
        return 1;
    }

    printf("[Main] Initializing motor...\n");
    motor_init();
    if (motor_stop_all() != 0) {
        printf("[Main] ERROR: Failed to stop motors at startup\n");
        gpioTerminate();
        if (camera_pid > 0) kill(camera_pid, SIGTERM);
        return 1;
    }

    /* Start vision reader thread */
    // printf("[Main] Starting vision reader thread...\n");
    if (pthread_create(&threads[0], NULL, camera_thread, NULL) != 0) {
        printf("[Main] ERROR: Failed to create vision reader thread\n");
        gpioTerminate();
        if (camera_pid > 0) kill(camera_pid, SIGTERM);
        return 1;
    }

    /* Start line sensor thread (3-sensor) */
    printf("[Main] Starting line sensor thread...\n");
    if (pthread_create(&threads[1], NULL, line_sensor_thread, NULL) != 0) {
        printf("[Main] ERROR: Failed to create line sensor thread\n");
        gpioTerminate();
        if (camera_pid > 0) kill(camera_pid, SIGTERM);
        return 1;
    }

    /* Start ultrasonic distance thread */
    printf("[Main] Starting ultrasonic sensor thread...\n");
    if (pthread_create(&threads[2], NULL, ultrasonic_sensor_thread, NULL) != 0) {
        printf("[Main] ERROR: Failed to create ultrasonic thread\n");
        gpioTerminate();
        if (camera_pid > 0) kill(camera_pid, SIGTERM);
        return 1;
    }

    printf("[Main] Waiting for camera to send first frame...\n");
    while (!should_exit && !camera_ready)
    {
        printf("[Main] Waiting for camera data...\n");
        usleep(500000);  // check every 500ms
    }
    printf("[Main] Camera sending data — starting motors\n");


    printf("\n[Main] Motor control starting...\n");
#ifdef CONTROL_MODE_CAMERA_ONLY
    printf("[Main] Mode: CAMERA ONLY\n");
#elif defined(CONTROL_MODE_LINE_SENSORS_ONLY)
    printf("[Main] Mode: LINE SENSORS ONLY\n");
#elif defined(CONTROL_MODE_HYBRID)
    printf("[Main] Mode: HYBRID (Camera + Line Sensors)\n");
#endif
    printf("[Main] Press Ctrl+C to exit\n\n");

    /* ================= CONTROL LOOP ================= */

int FULL_SPEED   = 100;
int HARD_SPEED   = 50;
int SOFT_SPEED   = 20;
int SEARCH_SPEED = 70;

int last_direction = 0;
/* Track last seen sensor pattern when any sensor detected the line (0=detect). */
int last_seen_L = 1, last_seen_M = 1, last_seen_R = 1;
/* Recovery state: when set, keep harsh rotation until middle sensor detects line */
int recovery_mode = 0; /* 0 = normal, 1 = recovering */
int recovery_dir = 0;  /* -1 = right, 1 = left */
/* Forced-line mode: lock to line-sensor logic for a short duration when a 90° begins */
int forced_line_mode = 0;
uint32_t forced_line_start = 0;
#define FORCED_LINE_DURATION_US (5000u * 1000u)

while (!should_exit) {
    int L, M, R;
    pthread_mutex_lock(&lock);
    /* Correct mapping: index 0=LEFT, 1=MIDDLE, 2=RIGHT */
    L = line_sensor_state[0];
    M = line_sensor_state[1];
    R = line_sensor_state[2];
    pthread_mutex_unlock(&lock);

    char local_bias[16];
    pthread_mutex_lock(&lock);
    strcpy(local_bias, (char*)camera_bias);
    pthread_mutex_unlock(&lock);

#ifdef ENABLE_OBSTACLE_AVOIDANCE
    const float SAFE_STOP_CM = 20.0f;
    if (measured_distance > 0 && measured_distance < SAFE_STOP_CM) {
        target_left = target_right = 0;
        printf("[US] Object %.1fcm → Emergency stop\n", measured_distance);
    }
    /* Smooth speed transitions for Motor A (left) */
    if (current_speed_a != target_left) {
        if (target_left > current_speed_a) {
            current_speed_a++;
        } else {
            if (current_speed_a - 5 < 0) {
                current_speed_a = 0;
            } else {
                current_speed_a -= 5;
            }
        }   
        motor_run(MOTOR_FR, current_speed_a, FORWARD);
        motor_run(MOTOR_RR, current_speed_a, BACKWARD);
        printf("[Motor A] Speed: %d%% | L, M, R: %d %d %d | [Vision] Status: %s\n", current_speed_a, L, M, R, current_status);
    }
    /* Smooth speed transitions for Motor B (right) */
    if (current_speed_b != target_right) {
        if (target_right > current_speed_b) {
            current_speed_b++;
        } else {
            if (current_speed_b - 5 < 0) {
                current_speed_b = 0;
            } else {
                current_speed_b -= 5;
            }
        }
        motor_run(MOTOR_FL,current_speed_b, FORWARD);
        motor_run(MOTOR_RL,current_speed_b, BACKWARD);
        printf("[Motor B] Speed: %d%% | L, M, R: %d %d %d | [Vision] Status: %s\n", current_speed_a, L, M, R, current_status);
    }
#endif

#ifdef CONTROL_MODE_CAMERA_ONLY
    /* ===== CAMERA ONLY ===== */
    printf("[CAM] %s\n", local_bias);

    if (strcmp(local_bias, "CENTER") == 0)
    {
        motor_run(MOTOR_FR, FULL_SPEED, FORWARD);
        motor_run(MOTOR_FL, FULL_SPEED, FORWARD);
        motor_run(MOTOR_RR, FULL_SPEED, FORWARD);
        motor_run(MOTOR_RL, FULL_SPEED, FORWARD);
    }
    else if (strcmp(local_bias, "LEFT") == 0)
    {
        motor_run(MOTOR_FR, FULL_SPEED, FORWARD);
        motor_run(MOTOR_FL, HARD_SPEED, FORWARD);
        motor_run(MOTOR_RR, FULL_SPEED, FORWARD);
        motor_run(MOTOR_RL, HARD_SPEED, FORWARD);
    }
    else if (strcmp(local_bias, "RIGHT") == 0)
    {
        motor_run(MOTOR_FR, HARD_SPEED, FORWARD);
        motor_run(MOTOR_FL, FULL_SPEED, FORWARD);
        motor_run(MOTOR_RR, HARD_SPEED, FORWARD);
        motor_run(MOTOR_RL, FULL_SPEED, FORWARD);
    }
    else if (strcmp(local_bias, "HARD LEFT") == 0)
    {
        motor_run(MOTOR_FR, FULL_SPEED, FORWARD);
        motor_run(MOTOR_FL, SOFT_SPEED, FORWARD);
        motor_run(MOTOR_RR, FULL_SPEED, FORWARD);
        motor_run(MOTOR_RL, SOFT_SPEED, FORWARD);
    }
    else if (strcmp(local_bias, "HARD RIGHT") == 0)
    {
        motor_run(MOTOR_FR, SOFT_SPEED, FORWARD);
        motor_run(MOTOR_FL, FULL_SPEED, FORWARD);
        motor_run(MOTOR_RR, SOFT_SPEED, FORWARD);
        motor_run(MOTOR_RL, FULL_SPEED, FORWARD);
    }
    else
    {
        printf("[CAM] NO PATH → STOP\n");
        motor_stop_all();
    }

#elif defined(CONTROL_MODE_LINE_SENSORS_ONLY)
    /* ===== LINE SENSORS ONLY ===== */
    printf("L:%d M:%d R:%d\n", L, M, R);

    /* ===== STRAIGHT ===== */
    if (L == 0 && M == 1 && R == 0)
    {
        last_direction = 0;

        motor_run(MOTOR_FL, FULL_SPEED, FORWARD);
        motor_run(MOTOR_FR, FULL_SPEED, FORWARD);
        motor_run(MOTOR_RL, FULL_SPEED, FORWARD);
        motor_run(MOTOR_RR, FULL_SPEED, FORWARD);
        printf("Straight\n");
    }

    /* ===== hard 90 right ===== */
    else if (L == 1 && M == 1 && R == 0)
    {
        last_direction = -1;

        motor_run(MOTOR_FR, HARD_SPEED, BACKWARD);
        motor_run(MOTOR_FL, FULL_SPEED, FORWARD);
        motor_run(MOTOR_RR, HARD_SPEED, BACKWARD);
        motor_run(MOTOR_RL, FULL_SPEED, FORWARD);
        printf("hard right\n");
    }

    /* ===== hard 90 left ===== */
    else if (L == 0 && M == 1 && R == 1)
    {
        last_direction = 1;

        motor_run(MOTOR_FR, FULL_SPEED, FORWARD);
        motor_run(MOTOR_FL, HARD_SPEED, BACKWARD);
        motor_run(MOTOR_RR, FULL_SPEED, FORWARD);
        motor_run(MOTOR_RL, HARD_SPEED, BACKWARD);
        printf("hard left\n");
    }

    /* ===== soft right ===== */
    else if (L == 1 && M == 0 && R == 0)
    {
        last_direction = -1;

        motor_run(MOTOR_FL, FULL_SPEED, FORWARD);
        motor_run(MOTOR_FR, SOFT_SPEED, BACKWARD);
        motor_run(MOTOR_RL, FULL_SPEED, FORWARD);
        motor_run(MOTOR_RR, SOFT_SPEED, BACKWARD);
        printf("soft right\n");
    }

    /* ===== soft left ===== */
    else if (L == 0 && M == 0 && R == 1)
    {
        last_direction = 1;

        motor_run(MOTOR_FR, FULL_SPEED, FORWARD);
        motor_run(MOTOR_FL, SOFT_SPEED, BACKWARD);
        motor_run(MOTOR_RR, FULL_SPEED, FORWARD);
        motor_run(MOTOR_RL, SOFT_SPEED, BACKWARD);
        printf("soft left\n");
    }

    /* ===== LINE LOST ===== */
    else
    {
        printf("SEARCHING...\n");

        if (last_direction <= 0)
        {
            motor_run(MOTOR_FL, SEARCH_SPEED, FORWARD);
            motor_run(MOTOR_FR, SEARCH_SPEED, BACKWARD);
            motor_run(MOTOR_RL, SEARCH_SPEED, FORWARD);
            motor_run(MOTOR_RR, SEARCH_SPEED, BACKWARD);
        }
        else
        {
            motor_run(MOTOR_FL, SEARCH_SPEED, BACKWARD);
            motor_run(MOTOR_FR, SEARCH_SPEED, FORWARD);
            motor_run(MOTOR_RL, SEARCH_SPEED, BACKWARD);
            motor_run(MOTOR_RR, SEARCH_SPEED, FORWARD);
        }
    }

#elif defined(CONTROL_MODE_HYBRID)
    /* ===== HYBRID: Camera + Line Sensors ===== */
    /* Print one clear action line describing exactly what we're doing now */
    if (recovery_mode) {
        const char *dir_str = (recovery_dir == -1) ? "HARD RIGHT" : (recovery_dir == 1 ? "HARD LEFT" : "UNKNOWN");
        printf("[HYBRID-ACTION] RECOVERING: %s | L:%d M:%d R:%d | Cam: %s\n", dir_str, L, M, R, local_bias);
    } else {
        printf("[HYBRID-ACTION] NORMAL   | L:%d M:%d R:%d | Cam: %s\n", L, M, R, local_bias);
    }

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
            printf("[HYBRID] forced line-mode expired after 5s\n");
        }
    }

    /* If already in recovery, continue harsh rotation until middle sensor triggers */
    if (recovery_mode) {
        if (recovery_dir == -1) {
            /* hard right rotation: right side reverse, left side forward */
            motor_run(MOTOR_FR, FULL_SPEED, BACKWARD);
            motor_run(MOTOR_FL, FULL_SPEED, FORWARD);
            motor_run(MOTOR_RR, FULL_SPEED, BACKWARD);
            motor_run(MOTOR_RL, FULL_SPEED, FORWARD);
            printf("[HYBRID-DRIVE] CMD FR:BACKWARD@%d FL:FORWARD@%d RR:BACKWARD@%d RL:FORWARD@%d\n", FULL_SPEED, FULL_SPEED, FULL_SPEED, FULL_SPEED);
        } else if (recovery_dir == 1) {
            /* hard left rotation: left side reverse, right side forward */
            motor_run(MOTOR_FR, FULL_SPEED, FORWARD);
            motor_run(MOTOR_FL, FULL_SPEED, BACKWARD);
            motor_run(MOTOR_RR, FULL_SPEED, FORWARD);
            motor_run(MOTOR_RL, FULL_SPEED, BACKWARD);
            printf("[HYBRID-DRIVE] CMD FR:FORWARD@%d FL:BACKWARD@%d RR:FORWARD@%d RL:BACKWARD@%d\n", FULL_SPEED, FULL_SPEED, FULL_SPEED, FULL_SPEED);
        }

        /* Exit recovery only when middle sensor sees the line */
        if (M == 0) {
            recovery_mode = 0;
            recovery_dir = 0;
            last_seen_L = last_seen_M = last_seen_R = 1;
            printf("[HYBRID-RECOVER] middle sensor triggered — recovery complete\n");
        }

        gpioDelay(1000);
        continue;
    }

    /* If forced_line_mode is active, run the sensor-only decision tree now */
    if (forced_line_mode) {
        printf("[HYBRID] FORCED-LINE active — using sensors for steering (forced %ums)\n", (unsigned int)((FORCED_LINE_DURATION_US - (now_tick - forced_line_start))/1000));

        /* ===== STRAIGHT ===== */
        if (L == 0 && M == 1 && R == 0) {
            last_direction = 0;
            motor_run(MOTOR_FL, FULL_SPEED, FORWARD);
            motor_run(MOTOR_FR, FULL_SPEED, FORWARD);
            motor_run(MOTOR_RL, FULL_SPEED, FORWARD);
            motor_run(MOTOR_RR, FULL_SPEED, FORWARD);
            printf("[HYBRID-LINE] Straight\n");
        }
        else if (L == 1 && M == 1 && R == 0) {
            last_direction = -1;
            motor_run(MOTOR_FR, FULL_SPEED, BACKWARD);
            motor_run(MOTOR_FL, FULL_SPEED, FORWARD);
            motor_run(MOTOR_RR, FULL_SPEED, BACKWARD);
            motor_run(MOTOR_RL, FULL_SPEED, FORWARD);
            printf("[HYBRID-LINE] hard right\n");
        }
        else if (L == 0 && M == 1 && R == 1) {
            last_direction = 1;
            motor_run(MOTOR_FR, FULL_SPEED, FORWARD);
            motor_run(MOTOR_FL, FULL_SPEED, BACKWARD);
            motor_run(MOTOR_RR, FULL_SPEED, FORWARD);
            motor_run(MOTOR_RL, FULL_SPEED, BACKWARD);
            printf("[HYBRID-LINE] hard left\n");
        }
        else if (L == 1 && M == 0 && R == 0) {
            last_direction = -1;
            motor_run(MOTOR_FL, FULL_SPEED, FORWARD);
            motor_run(MOTOR_FR, SOFT_SPEED, BACKWARD);
            motor_run(MOTOR_RL, FULL_SPEED, FORWARD);
            motor_run(MOTOR_RR, SOFT_SPEED, BACKWARD);
            printf("[HYBRID-LINE] soft right\n");
        }
        else if (L == 0 && M == 0 && R == 1) {
            last_direction = 1;
            motor_run(MOTOR_FR, FULL_SPEED, FORWARD);
            motor_run(MOTOR_FL, SOFT_SPEED, BACKWARD);
            motor_run(MOTOR_RR, FULL_SPEED, FORWARD);
            motor_run(MOTOR_RL, SOFT_SPEED, BACKWARD);
            printf("[HYBRID-LINE] soft left\n");
        }
        else {
            printf("[HYBRID-LINE] SEARCHING...\n");
            if (last_direction <= 0) {
                motor_run(MOTOR_FL, SEARCH_SPEED, FORWARD);
                motor_run(MOTOR_FR, SEARCH_SPEED, BACKWARD);
                motor_run(MOTOR_RL, SEARCH_SPEED, FORWARD);
                motor_run(MOTOR_RR, SEARCH_SPEED, BACKWARD);
            } else {
                motor_run(MOTOR_FL, SEARCH_SPEED, BACKWARD);
                motor_run(MOTOR_FR, SEARCH_SPEED, FORWARD);
                motor_run(MOTOR_RL, SEARCH_SPEED, BACKWARD);
                motor_run(MOTOR_RR, SEARCH_SPEED, FORWARD);
            }
        }

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
            printf("[HYBRID-RECOVER] START HARD RIGHT (based on history)\n");
            printf("[HYBRID-DRIVE] CMD FR:BACKWARD@%d FL:FORWARD@%d RR:BACKWARD@%d RL:FORWARD@%d\n", FULL_SPEED, FULL_SPEED, FULL_SPEED, FULL_SPEED);
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
            printf("[HYBRID-RECOVER] START HARD LEFT (based on history)\n");
            printf("[HYBRID-DRIVE] CMD FR:FORWARD@%d FL:BACKWARD@%d RR:FORWARD@%d RL:BACKWARD@%d\n", FULL_SPEED, FULL_SPEED, FULL_SPEED, FULL_SPEED);
            /* keep history until recovery completes (wait for middle sensor) */
        } else {
            printf("[HYBRID-RECOVER] Unknown history — last_seen L:%d R:%d M:%d — falling back to camera\n", last_seen_L, last_seen_R, last_seen_M);
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
            motor_run(MOTOR_FR, FULL_SPEED, BACKWARD);
            motor_run(MOTOR_FL, FULL_SPEED, FORWARD);
            motor_run(MOTOR_RR, FULL_SPEED, BACKWARD);
            motor_run(MOTOR_RL, FULL_SPEED, FORWARD);
            printf("[HYBRID] Detected immediate HARD RIGHT sensor pattern — forcing line-mode\n");
            gpioDelay(1000);
            continue;
        } else if (L == 0 && M == 1 && R == 1) {
            /* hard left indicated by sensors — start forced line-mode */
            forced_line_mode = 1;
            forced_line_start = now_tick;
            last_direction = 1;
            motor_run(MOTOR_FR, FULL_SPEED, FORWARD);
            motor_run(MOTOR_FL, FULL_SPEED, BACKWARD);
            motor_run(MOTOR_RR, FULL_SPEED, FORWARD);
            motor_run(MOTOR_RL, FULL_SPEED, BACKWARD);
            printf("[HYBRID] Detected immediate HARD LEFT sensor pattern — forcing line-mode\n");
            gpioDelay(1000);
            continue;
        }
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
        } else if (strcmp(local_bias, "HARD LEFT") == 0) {
            motor_run(MOTOR_FR, FULL_SPEED, FORWARD);
            motor_run(MOTOR_FL, SOFT_SPEED, FORWARD);
            motor_run(MOTOR_RR, FULL_SPEED, FORWARD);
            motor_run(MOTOR_RL, SOFT_SPEED, FORWARD);
        } else if (strcmp(local_bias, "HARD RIGHT") == 0) {
            motor_run(MOTOR_FR, SOFT_SPEED, FORWARD);
            motor_run(MOTOR_FL, FULL_SPEED, FORWARD);
            motor_run(MOTOR_RR, SOFT_SPEED, FORWARD);
            motor_run(MOTOR_RL, FULL_SPEED, FORWARD);
        } else {
            motor_stop_all();
        }
    }

#endif

    gpioDelay(1000);
    
}

    printf("\n[Main] Vision feedback stopped. Stopping motors...\n");

    /* Cleanup */
    should_exit = 1;
    motor_stop_all();
    pthread_join(threads[0], NULL);
    pthread_join(threads[1], NULL);
    pthread_join(threads[2], NULL);
    
    /* Terminate camera process */
    if (camera_pid > 0) {
        printf("[Main] Terminating camera process...\n");
        kill(camera_pid, SIGTERM);
        sleep(1);
        
        /* Force kill if still running */
        if (kill(camera_pid, 0) == 0) {
            kill(camera_pid, SIGKILL);
        }
        
        /* Wait for child process to finish */
        waitpid(camera_pid, NULL, 0);
    }
    
    gpioTerminate();

    printf("[Main] Motor control system shutdown complete.\n");
    return 0;
}
