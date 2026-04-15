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
#include <signal.h>
#include <sys/wait.h>
#include <pigpio.h>
#include <pthread.h>
#include "MotorDriver.h"
#include "DEV_Config.h"
#include "Debug.h"

#define FIFO_PATH "/tmp/ai_camera_status"
#define CAUTION_SPEED 35
#define CLEAR_SPEED 100
#define CAMERA_APP "./ai_camera"

/* Global variables for thread communication */
volatile char current_status[32] = "INIT";
volatile int should_exit = 0;
volatile pid_t camera_pid = 0;

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
    motor_stop_all();
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
    pthread_t reader_thread;
    int target_speed, current_speed_a = 0, current_speed_b = 0;

    printf("========================================\n");
    printf("  AI-Driven Motor Control System\n");
    printf("  Dual Motor (A & B) Configuration\n");
    printf("  Running on Raspberry Pi 4\n");
    printf("========================================\n\n");


    /* Launch compiled camera application as background process */
    printf("[Main] Launching camera application...\n");
    camera_pid = launch_camera_process();
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
    printf("[Main] Starting vision reader thread...\n");
    if (pthread_create(&reader_thread, NULL, vision_reader_thread, NULL) != 0) {
        printf("[Main] ERROR: Failed to create vision reader thread\n");
        gpioTerminate();
        if (camera_pid > 0) kill(camera_pid, SIGTERM);
        return 1;
    }

    /* Give camera application time to initialize and create the FIFO */
    printf("[Main] Waiting for camera application to initialize...\n");
    while(!should_exit && strcmp((char*)current_status, "INIT") == 0)
    {
        usleep(100000);
    }
    printf("[Main] Camera Ready\n");

    printf("\n[Main] Motor control starting...\n");
    printf("[Main] Motor speed controlled by AI vision feedback\n");
    printf("[Main] Press Ctrl+C to exit\n\n");

    /* Main control loop */
    while (!should_exit) {
        target_speed = get_motor_speed_from_vision(current_status);

        /* Smooth speed transitions for Motor A */
        if (current_speed_a != target_speed) {
            if (target_speed > current_speed_a) {
                current_speed_a++;
            } else {
                if (current_speed_a - 5 < 0)
                {
                    current_speed_a = 0;
                }
                else{
                    current_speed_a -= 5;
                }
            }


            motor_run(MOTOR_A, current_speed_a, FORWARD);
            printf("[Motor A] Speed: %d%% | [Vision] Status: %s\n", current_speed_a, current_status);
            
        }

        /* Smooth speed transitions for Motor B */
        if (current_speed_b != target_speed) {
            if (target_speed > current_speed_b) {
                current_speed_b++;
            } else {
                if (current_speed_b - 5 < 0)
                {
                    current_speed_b = 0;
                }
                else{
                    current_speed_b -= 5;
                }
            }

            motor_run(MOTOR_B, current_speed_b, FORWARD);
            printf("[Motor B] Speed: %d%% | [Vision] Status: %s\n", current_speed_b, current_status);
            
        }

        /* Small delay between updates */
        gpioDelay(100000); 
    }

    printf("\n[Main] Vision feedback stopped. Stopping motors...\n");

    /* Cleanup */
    should_exit = 1;
    motor_stop_all();
    pthread_join(reader_thread, NULL);
    
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
