/**************************************************************
* Class:: CSC-615-01 Spring 2026
* Name:: Haibin Cao, Eric Ahsue, Kiran Khatri, John Tsiglieris
* Student ID:: 923756077, 922711514, 925750019, 923593954
* GitHub-Name:: haibinc, Jasuv, khatri5034, John-Tsiglieris
* Project:: CSC 615 Final Project - Self-Driving Car
*
* File:: main_ai_integrated.c
*
* Description:: Main entry point for the self-driving car. Launches
* ai_camera.py as a background process and reads its CLEAR/CAUTION/
* STOP vision status from a named pipe (FIFO) on a dedicated thread.
* Drives both Motor A and Motor B with smoothly ramped PWM speed
* based on that feedback:
* - CLEAR   = Full speed (100%)
* - CAUTION = Reduced speed
* - STOP    = Motors stop
* Also sets up GPIO for the ultrasonic and line sensors that are
* being integrated into the same control loop.
*
* Usage:
*   make
*   sudo ./motor_ai
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
#define PYTHON_SCRIPT "ai_camera.py"
#define ECHO_PIN 0                              // Set this up later
#define TRIG_PIN 0                              // Set this up later
#define LEFT_LINE_SENSOR_PIN                    // Set this up later
#define MIDDLE_LINE_SENSOR_PIN                  // Set this up later
#define RIGHT_LINE_SENSOR_PIN                   // Set this up later

/* Global variables for thread communication */
volatile char current_status[32] = "INIT";
volatile int should_exit = 0;
volatile pid_t python_pid = 0;

/*
 * Signal handler for graceful cleanup
 */
void signal_handler(int sig) {
    printf("\n[Main] Received signal %d, initiating shutdown...\n", sig);
    should_exit = 1;
    
    /* Kill the Python process if it's running */
    if (python_pid > 0) {
        printf("[Main] Terminating Python camera process (PID: %d)...\n", python_pid);
        kill(python_pid, SIGTERM);
        sleep(1);
        
        /* Force kill if still running */
        if (kill(python_pid, 0) == 0) {
            printf("[Main] Force killing Python process...\n");
            kill(python_pid, SIGKILL);
        }
    }
    motor_stop_all();
    gpioTerminate();
    exit(0);
}

/*
 * Launch Python AI camera script as background process
 */
pid_t launch_python_camera(void) {
    pid_t pid = fork();

    if (pid < 0) {
        printf("[Main] ERROR: Failed to fork process\n");
        return -1;
    } else if (pid == 0) {
        /* Child process: execute Python script */

        /* Close stdin so Python doesn't block on input */
        close(STDIN_FILENO);

        /* Redirect stdout/stderr to avoid interfering with C program output */
        int devnull = open("/dev/null", O_WRONLY);
        dup2(devnull, STDOUT_FILENO);
        dup2(devnull, STDERR_FILENO);
        close(devnull);

        /* Execute Python script from the current directory */
        execlp("python3", "python3", PYTHON_SCRIPT, (char*)NULL);

        /* If execlp fails, exit child */
        perror("[Python] ERROR: Failed to execute python3");
        exit(1);
    } else {
        /* Parent process: return child PID */
        printf("[Main] Python camera process launched (PID: %d)\n", pid);
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

void *ultrasonic_sensor_thread(void *arg)
{
    while(!kill_signal_recieved)
    {
        int value = gpioRead(IR_PIN);

        pthread_mutex_lock(&lock);
        obstacle_state = value;
        pthread_mutex_unlock(&lock);

        usleep(20000);
    }

    return(NULL);
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

	/* Initialize ultrasonic sensor GPIO pins*/
	printf("Setting GPIO pins to output\n");
	gpioSetMode(ECHO_PIN, PI_INPUT);
	gpioSetMode(TRIG_PIN, PI_OUTPUT);
	printf("Finished setting GPIO pins\n");

    /* Configure line sensor input pins */
    gpioSetMode(LINE_PIN, PI_INPUT);
    gpioSetPullUpDown(LINE_PIN, PI_PUD_UP);                     // wtf is this


    /* Launch Python camera script as background process */
    printf("[Main] Launching Python AI camera script...\n");
    python_pid = launch_python_camera();
    if (python_pid < 0) {
        printf("[Main] ERROR: Failed to launch Python script\n");
        return 1;
    }

    /* Initialize hardware */
    printf("[Main] Initializing pigpio...\n");
    if (gpioInitialise() < 0) {
        printf("[Main] ERROR: pigpio initialization failed\n");
        if (python_pid > 0) kill(python_pid, SIGTERM);
        return 1;
    }

    /* Setup signal handlers for graceful shutdown */
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    printf("[Main] Initializing DEV module...\n");
    if (DEV_ModuleInit() != 0) {
        printf("[Main] ERROR: DEV_ModuleInit failed\n");
        gpioTerminate();
        if (python_pid > 0) kill(python_pid, SIGTERM);
        return 1;
    }

    printf("[Main] Initializing motor...\n");
    motor_init();
    if (motor_stop_all() != 0) {
        printf("[Main] ERROR: Failed to stop motors at startup\n");
        gpioTerminate();
        if (python_pid > 0) kill(python_pid, SIGTERM);
        return 1;
    }

    /* Start vision reader thread */
    printf("[Main] Starting vision reader thread...\n");
    if (pthread_create(&reader_thread, NULL, vision_reader_thread, NULL) != 0) {
        printf("[Main] ERROR: Failed to create vision reader thread\n");
        gpioTerminate();
        if (python_pid > 0) kill(python_pid, SIGTERM);
        return 1;
    }

    /* Start ultrasonic sensor thread */


    /* Start line sensor threads */


    /* Give Python script time to initialize and create the FIFO */
    printf("[Main] Waiting for Python script to initialize...\n");
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
    
    /* Terminate Python process */
    if (python_pid > 0) {
        printf("[Main] Terminating Python camera process...\n");
        kill(python_pid, SIGTERM);
        sleep(1);
        
        /* Force kill if still running */
        if (kill(python_pid, 0) == 0) {
            kill(python_pid, SIGKILL);
        }
        
        /* Wait for child process to finish */
        waitpid(python_pid, NULL, 0);
    }
    
    gpioTerminate();

    printf("[Main] Motor control system shutdown complete.\n");
    return 0;
}
