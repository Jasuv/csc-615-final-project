/**************************************************************
 * File:: ai_camera.c
 *
 * Description:: Launches the rpicam-hello camera GUI with
 * MobileNet SSD object detection via the IMX500.
 * The GUI, bounding boxes, and preview are all handled by
 * rpicam-hello
 *
 * Build:
 *   gcc ai_camera.c -o ai_camera
 *
 * Run:
 *   ./ai_camera
 **************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/wait.h>

#define POST_PROCESS_JSON "imx500_model.json"

int main(void)
{
    printf("[AI Camera] Launching camera GUI with object detection...\n");

    pid_t pid = fork();

    if (pid < 0) {
        perror("[AI Camera] fork failed");
        return 1;
    }

    if (pid == 0) {
        // Child process: exec rpicam-hello with object detection post-processing 
        execlp("rpicam-hello", "rpicam-hello",
               "--timeout",            "0",          
               "--post-process-file",  POST_PROCESS_JSON,
               "--viewfinder-width",   "1920",
               "--viewfinder-height",  "1080",
               "--framerate",          "30",
               (char *)NULL);

        // Only reached if execlp fails
        perror("[AI Camera] execlp failed");
        exit(1);
    }

    // Parent: wait for the camera process to exit
    int status;
    waitpid(pid, &status, 0);
    printf("[AI Camera] Camera process exited.\n");

    return 0;
}
