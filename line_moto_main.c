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

static pthread_t threads[3];
static int thread_count = 3;

/* Shared sensor state + sync */
int line_sensor_state[3] = {1,1,1}; /* 0 = sensor detects line */
int obstacle_state = 0;
float distance_cm = -1.0f; /* measured distance by ultrasonic; -1 = unknown */
pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;

int ir_sensor_state[2] = {0,0}; /* 0 = no obstacle, 1 = obstacle detected */
int ir_sensor_count = 2;

// camera thread
volatile char camera_bias[16] = "CENTER";


/*
 * Signal handler for graceful cleanup
 */

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
    for (int i = 0; i < ir_sensor_count; i++)
    {
        pthread_cancel(ir_sensor_thread[i]);
        pthread_join(ir_sensor_thread[i], NULL);
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

/*
 * Thread function: reads from named pipe and updates status
 */
void* vision_reader_thread(void* arg) {
    int fifo_fd;
    char buffer[64];
    ssize_t bytes_read;

    /* Try to open the FIFO - with non-blocking mode */
    while (!should_exit) {
        fifo_fd = open(FIFO_PATH, O_RDONLY | O_NONBLOCK);
        
        if (fifo_fd >= 0) {
            break;
        }
        sleep(1);
    }

    /* Read loop */
    while (!should_exit) {
        memset(buffer, 0, sizeof(buffer));
        bytes_read = read(fifo_fd, buffer, sizeof(buffer) - 1);

        if (bytes_read > 0) {
            /* Remove newline */
            if (buffer[bytes_read - 1] == '\n') {
                buffer[bytes_read - 1] = '\0';
            } else {
                buffer[bytes_read] = '\0';
            }

            /* Update global status */
            strncpy((char*)current_status, buffer, sizeof(current_status) - 1);
        } else if (bytes_read < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            /* Real error (not just no data available) */
            close(fifo_fd);
            
            /* Try to reconnect */
            while (!should_exit) {
                fifo_fd = open(FIFO_PATH, O_RDONLY | O_NONBLOCK);
                if (fifo_fd >= 0) break;
                sleep(1);
            }
        }

        usleep(50000);
    }

    if (fifo_fd >= 0) {
        close(fifo_fd);
    }
    return NULL;
}


///camera 
void* camera_thread(void* arg)
{
    int fifo_fd;
    char buffer[32];

    /* Wait until FIFO exists */
    while (access(FIFO_PATH, F_OK) == -1 && !should_exit)
        usleep(100000);

    fifo_fd = open(FIFO_PATH, O_RDONLY | O_NONBLOCK);
    if (fifo_fd < 0) {
        perror("FIFO open failed");
        return NULL;
    }

    while (!should_exit)
    {
        int bytes = read(fifo_fd, buffer, sizeof(buffer)-1);

        if (bytes > 0)
        {
            buffer[bytes] = '\0';

            
            char *newline = strchr(buffer, '\n');
            if (newline)
                *newline = '\0';

            pthread_mutex_lock(&lock);
            strncpy((char*)camera_bias, buffer, sizeof(camera_bias)-1);
            camera_bias[sizeof(camera_bias)-1] = '\0';  
            pthread_mutex_unlock(&lock);

            printf("[CAMERA] Bias: '%s'\n", camera_bias);
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
        int left  = gpioRead(LEFT_IR_PIN);
        int right = gpioRead(RIGHT_IR_PIN);
        pthread_mutex_lock(&lock);
        ir_sensor_state[0]  = left;
        ir_sensor_state[1]  = right;
        pthread_mutex_unlock(&lock);
        usleep(20000);                      // not sure if we should use usleep or gpioDelay
    }

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
    // pthread_t reader_thread;
    // pthread_t line_thread;
    // pthread_t ultrasonic_thread_id;
    int target_speed = 50, current_speed_a = 0, current_speed_b = 0;

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

    /* Line sensors initialized in the line_sensor_thread (3 sensors) */


    /* Launch compiled camera application as background process */
    // printf("[Main] Launching camera application...\n");
     /* Start camera reader thread FIRST */
    pthread_t cam_thread;
    if (pthread_create(&threads[0], NULL, camera_thread, NULL) != 0) {
        perror("Camera thread failed");
        return 1;
    }

    /* THEN launch ai_camera */
    //camera_pid = launch_camera_process();
    if (camera_pid < 0) {
        printf("[Main] ERROR: Failed to launch camera application\n");
        return 1;
    }

    /* Initialize hardware */
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
    // if (pthread_create(&reader_thread, NULL, vision_reader_thread, NULL) != 0) {
    //     printf("[Main] ERROR: Failed to create vision reader thread\n");
    //     gpioTerminate();
    //     if (camera_pid > 0) kill(camera_pid, SIGTERM);
    //     return 1;
    // }

    /* Start line sensor thread (3-sensor) */
    printf("[Main] Starting line sensor thread...\n");
    if (pthread_create(&threads[1], NULL, line_sensor_thread, NULL) != 0) {
        printf("[Mpthread_createain] ERROR: Failed to create line sensor thread\n");
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

    /* Give camera application time to initialize and create the FIFO */
    //  printf("[Main] Waiting for camera application to initialize...\n");
    //  while(!should_exit && strcmp((char*)current_status, "INIT") == 0)
    //  {
    //      usleep(100000);
    //  }
    //  printf("[Main] Camera Ready\n");

//thread for camera

    printf("\n[Main] Motor control starting...\n");
    // printf("[Main] Motor speed controlled by AI vision feedback\n");
    printf("[Main] Press Ctrl+C to exit\n\n");

/* Start ir sensor thread */
printf("[Main] Starting IR sensor threads...\n");
if (pthread_create(&threads[3], NULL, ir_sensor_thread, NULL) != 0) {
    printf("[Main] ERROR: Failed to create IR sensor thread\n");
    gpioTerminate();
    if (camera_pid > 0) kill(camera_pid, SIGTERM);
    return 1;
}
    /* ================= LINE FOLLOWING LOOP ================= */

int BASE_SPEED   = 90;   // straight speed
int SOFT_SPEED   =40;   // gentle correction
int HARD_SPEED   = 50;    // sharp arc (one side stopped)
int SEARCH_SPEED = 70;   // recovery arc

int last_direction = 0;  // -1 = left, 1 = right
int turn = 5;

// witout camera

while (!should_exit)
{
    // int L, M, R;
    int ir_left, ir_right;

    // pthread_mutex_lock(&lock);
    // R = line_sensor_state[0];
    // M = line_sensor_state[1];
    // L = line_sensor_state[2];
    // pthread_mutex_unlock(&lock);

    pthread_mutex_lock(&lock);
    ir_left = ir_sensor_state[0];
    ir_right = ir_sensor_state[1];
    pthread_mutex_unlock(&lock);

    // printf("L:%d M:%d R:%d\n", L, M, R);
    printf("Left IR state: %d  Right IR state: %d\n", ir_left, ir_right);

}                                                                                         // delete this 
    

    /* ===== STRAIGHT ===== */
//     if (L == 0 && M == 1 && R == 0)
//     {
//         last_direction = 0;

//         motor_run(MOTOR_FL, BASE_SPEED , FORWARD);
//         motor_run(MOTOR_FR, BASE_SPEED , FORWARD);
//         motor_run(MOTOR_RL, BASE_SPEED , FORWARD);
//         motor_run(MOTOR_RR, BASE_SPEED , FORWARD);
//         printf("Straight");
//     }

//     /* ===== hard 90 right ===== */
//     else if (L == 1 && M == 1 && R == 0)
//     {
//         last_direction = -1;

//         motor_run(MOTOR_FR,HARD_SPEED , BACKWARD);
//         motor_run(MOTOR_FL, BASE_SPEED, FORWARD);
//         motor_run(MOTOR_RR,HARD_SPEED, BACKWARD);
//         motor_run(MOTOR_RL, BASE_SPEED, FORWARD);
//         printf("hard right");
       
//     }

//     /* ===== hard 90  left ===== */
//     else if (L == 0 && M == 1 && R == 1)
//     {
//         last_direction = 1;

//         motor_run(MOTOR_FR, BASE_SPEED, FORWARD);
//         motor_run(MOTOR_FL, HARD_SPEED, BACKWARD);
//         motor_run(MOTOR_RR, BASE_SPEED, FORWARD);
//         motor_run(MOTOR_RL, HARD_SPEED, BACKWARD);
//         printf("hard left");
//     }

//     /* ===== soft right  ===== */
//     else if (L == 1 && M == 0 && R == 0)
//     {
//          last_direction = -1;

//        motor_run(MOTOR_FL, BASE_SPEED, FORWARD);
//        motor_run(MOTOR_FR,SOFT_SPEED, BACKWARD);
//        motor_run(MOTOR_RL,BASE_SPEED, FORWARD);
//        motor_run(MOTOR_RR, SOFT_SPEED, BACKWARD);
//        printf("soft right");
//     }

//     /* ===== soft left   ===== */
//     else if (L == 0 && M == 0 && R == 1)
//     {
//         last_direction = 1;

//        motor_run(MOTOR_FR, BASE_SPEED, FORWARD);
//        motor_run(MOTOR_FL,SOFT_SPEED, BACKWARD);
//        motor_run(MOTOR_RR,BASE_SPEED, FORWARD);
//        motor_run(MOTOR_RL, SOFT_SPEED, BACKWARD);
//        printf("soft left");
       
       
//     }

//     /* ===== LINE LOST ===== */
//     else
//     {
//         printf("SEARCHING...\n");

//         if (last_direction <= 0)
//         {
//             motor_run(MOTOR_FL, SEARCH_SPEED, FORWARD);
//             motor_run(MOTOR_FR, SEARCH_SPEED, BACKWARD);
//             motor_run(MOTOR_RL, SEARCH_SPEED, FORWARD);
//             motor_run(MOTOR_RR, SEARCH_SPEED, BACKWARD);
//         }
//         else
//         {
//             motor_run(MOTOR_FL, SEARCH_SPEED, BACKWARD);
//             motor_run(MOTOR_FR,SEARCH_SPEED, FORWARD);
//             motor_run(MOTOR_RL, SEARCH_SPEED, BACKWARD);
//             motor_run(MOTOR_RR, SEARCH_SPEED, FORWARD);
//         }
//     }

//     gpioDelay(140000);   // 
// }

 //wioth camera
// while (!should_exit)
// {
//     char local_bias[16];

//     pthread_mutex_lock(&lock);
//     strcpy(local_bias, (char*)camera_bias);
//     pthread_mutex_unlock(&lock);

//     printf("Camera: %s\n", local_bias);

//     /* ===== CAMERA IS 100% MASTER ===== */

//     if (strcmp(local_bias, "CENTER") == 0)
//     {
//         motor_run(MOTOR_FL, 100, FORWARD);
//         motor_run(MOTOR_FR, 100, FORWARD);
//         motor_run(MOTOR_RL, 100, FORWARD);
//         motor_run(MOTOR_RR, 100, FORWARD);
//     }

//     else if (strcmp(local_bias, "LEFT") == 0)
//     {
//         motor_run(MOTOR_FL, 60, FORWARD);
//         motor_run(MOTOR_FR, 100, FORWARD);
//         motor_run(MOTOR_RL, 60, FORWARD);
//         motor_run(MOTOR_RR, 100, FORWARD);
//     }

//     else if (strcmp(local_bias, "RIGHT") == 0)
//     {
//         motor_run(MOTOR_FL, 100, FORWARD);
//         motor_run(MOTOR_FR, 60, FORWARD);
//         motor_run(MOTOR_RL, 100, FORWARD);
//         motor_run(MOTOR_RR, 60, FORWARD);
//     }

//     else if (strcmp(local_bias, "HARD LEFT") == 0)
//     {
//         motor_run(MOTOR_FL, 20, FORWARD);
//         motor_run(MOTOR_FR, 120, FORWARD);
//         motor_run(MOTOR_RL, 20, FORWARD);
//         motor_run(MOTOR_RR, 120, FORWARD);
//     }

//     else if (strcmp(local_bias, "HARD RIGHT") == 0)
//     {
//         motor_run(MOTOR_FL, 120, FORWARD);
//         motor_run(MOTOR_FR, 20, FORWARD);
//         motor_run(MOTOR_RL, 120, FORWARD);
//         motor_run(MOTOR_RR, 20, FORWARD);
//     }

//     else
//     {
//         printf("NO PATH → STOP\n");
//         motor_stop_all();
//     }

//     gpioDelay(10000);  // 100ms loop
// }
/* =============================================================== */

        /* Ultrasonic safety override: if object closer than SAFE_STOP_CM, stop */
        // const float SAFE_STOP_CM = 20.0f;
        // if (measured_distance > 0 && measured_distance < SAFE_STOP_CM) {
        //     target_left = target_right = 0;
        //     printf("[US] Object %.1fcm → Emergency stop\n", measured_distance);
        // }
      
        // /* Smooth speed transitions for Motor A (left) */
        // if (current_speed_a != target_left) {
        //     if (target_left > current_speed_a) {
        //         current_speed_a++;
        //     } else {
        //         if (current_speed_a - 5 < 0) {
        //             current_speed_a = 0;
        //         } else {
        //             current_speed_a -= 5;
        //         }
        //     }

        //     motor_run(MOTOR_FR, current_speed_a, FORWARD);
        //     motor_run(MOTOR_RR, current_speed_a, BACKWARD);
        //     printf("[Motor A] Speed: %d%% | L, M, R: %d %d %d | [Vision] Status: %s\n", current_speed_a, L, M, R, current_status);
        // }

        // /* Smooth speed transitions for Motor B (right) */
        // if (current_speed_b != target_right) {
        //     if (target_right > current_speed_b) {
        //         current_speed_b++;
        //     } else {
        //         if (current_speed_b - 5 < 0) {
        //             current_speed_b = 0;
        //         } else {
        //             current_speed_b -= 5;
        //         }
        //     }

        //     motor_run(MOTOR_FL,current_speed_b, FORWARD);
        //     motor_run(MOTOR_RL,current_speed_b, BACKWARD);
        //     printf("[Motor B] Speed: %d%% | L, M, R: %d %d %d | [Vision] Status: %s\n", current_speed_a, L, M, R, current_status);
        // }

        /* Small delay between updates */
    

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