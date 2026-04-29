// /**************************************************************
//  * File:: ai_camera.c
//  *
//  * Description:: Launches the rpicam-hello camera GUI with
//  * MobileNet SSD object detection via the IMX500.
//  * The GUI, bounding boxes, and preview are all handled by
//  * rpicam-hello
//  *
//  * Build:
//  *   gcc ai_camera.c -o ai_camera
//  *
//  * Run:
//  *   ./ai_camera
//  **************************************************************/

// #include <stdio.h>
// #include <stdlib.h>
// #include <unistd.h>
// #include <sys/wait.h>

// #define POST_PROCESS_JSON "imx500_model.json"

// int main(void)
// {
//     printf("[AI Camera] Launching camera GUI with object detection...\n");

//     pid_t pid = fork();

//     if (pid < 0) {
//         perror("[AI Camera] fork failed");
//         return 1;
//     }

//     if (pid == 0) {
//         // Child process: exec rpicam-hello with object detection post-processing 
//         execlp("rpicam-hello", "rpicam-hello",
//                "--timeout",            "0",          
//                "--post-process-file",  POST_PROCESS_JSON,
//                "--viewfinder-width",   "1920",
//                "--viewfinder-height",  "1080",
//                "--framerate",          "30",
//                (char *)NULL);

//         // Only reached if execlp fails
//         perror("[AI Camera] execlp failed");
//         exit(1);
//     }

//     // Parent: wait for the camera process to exit
//     int status;
//     waitpid(pid, &status, 0);
//     printf("[AI Camera] Camera process exited.\n");

//     return 0;
// }



#include <opencv2/opencv.hpp>
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <cstring>

#define FIFO_PATH "/tmp/ai_camera_status"
#define WIDTH 640
#define HEIGHT 480

using namespace std;
using namespace cv;

int main()
{
    cout << "=====================================\n";
    cout << " AI Camera — Stable Path Detection\n";
    cout << "=====================================\n";

    // Create FIFO if not exists
    if (access(FIFO_PATH, F_OK) == -1)
    {
        if (mkfifo(FIFO_PATH, 0666) != 0)
        {
            perror("mkfifo failed");
            return -1;
        }
    }

    int fifo_fd = open(FIFO_PATH, O_WRONLY);
    if (fifo_fd < 0)
    {
        perror("FIFO open failed");
        return -1;
    }

    // Start camera stream
    string command =
        "rpicam-vid -t 0 "
        "--width 640 --height 480 "
        "--framerate 30 "
        "--codec yuv420 "
        "--inline -o -";

    FILE* pipe = popen(command.c_str(), "r");
    if (!pipe)
    {
        cout << "Camera start failed\n";
        return -1;
    }

    int frameSize = WIDTH * HEIGHT * 3 / 2;
    unsigned char* buffer = new unsigned char[frameSize];

    while (true)
    {
        if (fread(buffer, 1, frameSize, pipe) != frameSize)
            continue;

        Mat yuv(HEIGHT + HEIGHT/2, WIDTH, CV_8UC1, buffer);
        Mat frame;
        cvtColor(yuv, frame, COLOR_YUV2BGR_I420);

        if (frame.empty())
            continue;

        // Convert to grayscale
        Mat gray;
        cvtColor(frame, gray, COLOR_BGR2GRAY);

        // Blur to reduce noise
        GaussianBlur(gray, gray, Size(5,5), 0);

        // Threshold (black path becomes white)
        Mat thresh;
        threshold(gray, thresh, 100, 255, THRESH_BINARY_INV);

        // Bottom 25% ROI
        int roiHeight = HEIGHT / 4;
        Mat roi = thresh(Rect(0, HEIGHT - roiHeight, WIDTH, roiHeight));

        // Count white pixels
        int whitePixels = countNonZero(roi);

        string status;

        // If almost no white pixels → NO PATH
        if (whitePixels < 2000)
        {
            status = "STOP";
            cout << "NO PATH → STOP\n";
        }
        else
        {
            Moments m = moments(roi, true);

            int cx = m.m10 / m.m00;
            int center = WIDTH / 2;
            int offset = cx - center;

            if (offset < -120)
                status = "HARD RIGHT";
            else if (offset < -40)
                status = "RIGHT";
            else if (offset > 120)
                status = "HARD LEFT";
            else if (offset > 40)
                status = "LEFT";
            else
                status = "CENTER";

            cout << "Bias: " << status << endl;
        }

        // Always write something
        write(fifo_fd, status.c_str(), status.length());
        write(fifo_fd, "\n", 1);

        usleep(100000);
    }

    close(fifo_fd);
    delete[] buffer;
    pclose(pipe);
    return 0;
}