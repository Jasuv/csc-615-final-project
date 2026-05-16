#include <opencv2/opencv.hpp>
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <cstring>
#include <cmath>
#include <deque>
#include <string>
#include <chrono>
#include <queue>
#include <numeric>

#define FIFO_PATH "/tmp/ai_camera_status"
#define WIDTH  640
#define HEIGHT 480

// ── Tuning ────────────────────────────────────────────────────────────────────
#define THRESHOLD_VAL       100   // grayscale threshold
#define LOST_FRAME_LIMIT    10   // frames before search kicks in
#define VOTE_HISTORY        50   // rolling window for search votes
#define QUEUE_DELAY_MS      500   // ms stable before recording vote

#define ANCHOR_SEARCH_PX    80   // search width at bottom center for anchor
#define MAX_STEP_DRIFT      65   // max px drift per row when tracing
#define SPINE_STEP_Y        7   // rows per spine step

// ── Steering thresholds ───────────────────────────────────────────────────────
// Bottom cx offset — where the anchor sits relative to center
#define HARD_TURN_PX        90
#define SOFT_TURN_PX        60

// Weight between bottom position and path curvature for final decision
// 0.0 = only bottom cx, 1.0 = only curvature
// 0.5 = equal mix — tune this for your track
#define CURVE_WEIGHT        0.5f
// ─────────────────────────────────────────────────────────────────────────────

using namespace std;
using namespace cv;
using namespace std::chrono;

string vote_search_direction(const deque<string>& hist)
{
    int lv = 0, rv = 0;
    for (const string& s : hist)
    {
        if (s == "LEFT" || s == "HARD LEFT")   lv++;
        if (s == "RIGHT" || s == "HARD RIGHT") rv++;
    }
    cout << "[VOTE] L=" << lv << " R=" << rv
         << " (" << hist.size() << " samples)\n";
    if (lv > rv) return "HARD LEFT";
    if (rv > lv) return "HARD RIGHT";
    return "CENTER";
}

Mat flood_connected(const Mat& thresh, int seedX, int seedY)
{
    Mat visited = Mat::zeros(thresh.size(), CV_8UC1);
    if (thresh.at<uint8_t>(seedY, seedX) != 255) return visited;

    queue<Point> q;
    q.push(Point(seedX, seedY));
    visited.at<uint8_t>(seedY, seedX) = 255;

    while (!q.empty())
    {
        Point p = q.front(); q.pop();
        const int dx[] = { 1,-1, 0, 0 };
        const int dy[] = { 0, 0, 1,-1 };
        for (int d = 0; d < 4; d++)
        {
            int nx = p.x + dx[d];
            int ny = p.y + dy[d];
            if (nx < 0 || nx >= thresh.cols) continue;
            if (ny < 0 || ny >= thresh.rows) continue;
            if (visited.at<uint8_t>(ny, nx)) continue;
            if (thresh.at<uint8_t>(ny, nx) != 255) continue;
            visited.at<uint8_t>(ny, nx) = 255;
            q.push(Point(nx, ny));
        }
    }
    return visited;
}

int row_midpoint_in_mask(const Mat& connMask, int y, int cx)
{
    if (y < 0 || y >= connMask.rows) return -1;

    int searchL = max(0,               cx - MAX_STEP_DRIFT);
    int searchR = min(connMask.cols-1, cx + MAX_STEP_DRIFT);

    int bestX    = -1;
    int bestDist = MAX_STEP_DRIFT + 1;

    for (int x = searchL; x <= searchR; x++)
    {
        if (connMask.at<uint8_t>(y, x) == 255)
        {
            int dist = abs(x - cx);
            if (dist < bestDist) { bestDist = dist; bestX = x; }
        }
    }
    if (bestX == -1) return -1;

    int left = bestX, right = bestX;
    for (int x = bestX-1; x >= searchL; x--)
    { if (connMask.at<uint8_t>(y,x)==255) left=x; else break; }
    for (int x = bestX+1; x <= searchR; x++)
    { if (connMask.at<uint8_t>(y,x)==255) right=x; else break; }

    return (left + right) / 2;
}

/*
 * Compute the "effective steering signal" by combining:
 *   1. Where the bottom anchor is (where car is on line now)
 *   2. Where the path curves to (what's coming ahead)
 *
 * Returns a signed pixel offset — positive = line is right of center,
 * negative = line is left of center.
 */
int compute_steering_offset(int bottom_cx, const vector<Point>& spine)
{
    // Bottom offset — current position error
    int bottom_offset = bottom_cx - (WIDTH / 2);

    if (spine.size() < 3)
        return bottom_offset;  // not enough spine to judge curve

    // Curvature signal — average x of top half of spine vs bottom half
    // Top half = farther ahead, bottom half = where car is now
    int half = (int)spine.size() / 2;

    // Bottom half average x
    float bottom_avg = 0;
    for (int i = 0; i < half; i++)
        bottom_avg += spine[i].x;
    bottom_avg /= half;

    // Top half average x
    float top_avg = 0;
    for (int i = half; i < (int)spine.size(); i++)
        top_avg += spine[i].x;
    top_avg /= ((int)spine.size() - half);

    // Curve drift: how much does the path veer ahead
    float curve_drift = top_avg - bottom_avg;

    // Blend: bottom_offset tells us where we are,
    // curve_drift tells us where we're heading
    float blended = (1.0f - CURVE_WEIGHT) * bottom_offset
                  + CURVE_WEIGHT          * curve_drift;

    cout << " [STEER] bottom_off=" << bottom_offset
         << " curve_drift=" << curve_drift
         << " blended=" << blended << "\n";

    return (int)blended;
}

int main()
{
    cout << "=====================================\n";
    cout << " AI Camera — Curvature-Aware Tracer\n";
    cout << "=====================================\n";

    if (access(FIFO_PATH, F_OK) == -1)
        if (mkfifo(FIFO_PATH, 0666) != 0) { perror("mkfifo"); return -1; }

    //int fifo_fd = open(FIFO_PATH, O_WRONLY | O_NONBLOCK);                 // apparently these two lines are fucking up our shit
    //if (fifo_fd < 0) { perror("FIFO open"); return -1; }

    int fifo_fd = -1;
    while (fifo_fd < 0) {
        fifo_fd = open(FIFO_PATH, O_WRONLY | O_NONBLOCK);
        if (fifo_fd < 0) {
            if (errno == ENXIO) {
                usleep(100000);  // no reader yet, wait 100ms and retry
                continue;
            }
            perror("FIFO open");  // real error, give up
            return -1;
        }
    }

    string command =
        "rpicam-vid -t 0 "
        "--width 640 --height 480 "
        "--framerate 30 "
        "--codec yuv420 "
        "--inline -o -";

    FILE* pipe = popen(command.c_str(), "r");
    if (!pipe) { cout << "Camera failed\n"; return -1; }

    size_t frameSize = (size_t)WIDTH * HEIGHT * 3 / 2;
    unsigned char* buf = new unsigned char[frameSize];

    int   last_cx     = WIDTH / 2;
    int   lost_frames = 0;
    deque<string> history;
    string pending_status = "";
    auto   pending_start  = steady_clock::now();
    bool   pending_active = false;

    while (true)
    {
        if (fread(buf, 1, frameSize, pipe) != frameSize) continue;

        Mat yuv(HEIGHT + HEIGHT/2, WIDTH, CV_8UC1, buf);
        Mat frame;
        cvtColor(yuv, frame, COLOR_YUV2BGR_I420);
        if (frame.empty()) continue;

        // ── Threshold ────────────────────────────────────────────────────────
        Mat gray;
        cvtColor(frame, gray, COLOR_BGR2GRAY);
        GaussianBlur(gray, gray, Size(5,5), 0);
        Mat thresh;
        threshold(gray, thresh, THRESHOLD_VAL, 255, THRESH_BINARY_INV);

        // ── Find anchor at bottom center ──────────────────────────────────────
        int anchor_cx = -1;
        int anchor_y  = -1;

        for (int y = HEIGHT-1; y >= HEIGHT-30 && anchor_cx == -1; y--)
        {
            int searchL = max(0,       WIDTH/2 - ANCHOR_SEARCH_PX);
            int searchR = min(WIDTH-1, WIDTH/2 + ANCHOR_SEARCH_PX);

            int bestX = -1, bestDist = ANCHOR_SEARCH_PX + 1;
            for (int x = searchL; x <= searchR; x++)
            {
                if (thresh.at<uint8_t>(y, x) == 255)
                {
                    int dist = abs(x - WIDTH/2);
                    if (dist < bestDist) { bestDist = dist; bestX = x; }
                }
            }
            if (bestX != -1)
            {
                int left = bestX, right = bestX;
                for (int x = bestX-1; x >= searchL; x--)
                { if (thresh.at<uint8_t>(y,x)==255) left=x; else break; }
                for (int x = bestX+1; x <= searchR; x++)
                { if (thresh.at<uint8_t>(y,x)==255) right=x; else break; }
                anchor_cx = (left + right) / 2;
                anchor_y  = y;
            }
        }

        bool   line_found = false;
        int    bottom_cx  = last_cx;
        vector<Point> spine;

        if (anchor_cx != -1)
        {
            Mat connMask = flood_connected(thresh, anchor_cx, anchor_y);

            bottom_cx   = anchor_cx;
            last_cx     = anchor_cx;
            line_found  = true;
            lost_frames = 0;

            int trace_cx = anchor_cx;
            for (int y = anchor_y; y >= HEIGHT/2; y -= SPINE_STEP_Y)
            {
                int mid = row_midpoint_in_mask(connMask, y, trace_cx);
                if (mid == -1) break;
                spine.push_back(Point(mid, y));
                trace_cx = mid;
            }

            // Green overlay — only connected component
            Mat maskColor = Mat::zeros(frame.size(), CV_8UC3);
            maskColor.setTo(Scalar(0, 160, 0), connMask);
            addWeighted(frame, 1.0, maskColor, 0.3, 0, frame);
        }
        else
        {
            lost_frames++;
            Mat maskColor = Mat::zeros(frame.size(), CV_8UC3);
            maskColor.setTo(Scalar(0, 60, 60), thresh);
            addWeighted(frame, 1.0, maskColor, 0.2, 0, frame);
        }

        // ── Draw spine ────────────────────────────────────────────────────────
        if (spine.size() >= 2)
        {
            for (int i = 0; i+1 < (int)spine.size(); i++)
            {
                float t = (float)i / (float)(spine.size()-1);
                Scalar col((int)(255*t), 255, (int)(255*(1.0f-t)));
                line(frame, spine[i], spine[i+1], col, 4, LINE_AA);
            }
            for (int i = 0; i < (int)spine.size(); i++)
            {
                float t = (float)i / max(1,(int)spine.size()-1);
                circle(frame, spine[i], 3,
                       Scalar((int)(255*t),255,(int)(255*(1-t))), -1);
            }
            circle(frame, spine.front(), 12, Scalar(0,255,255), -1);
            circle(frame, spine.front(), 12, Scalar(0,  0,  0),  2);
            circle(frame, spine.back(),  12, Scalar(255,0,255), -1);
            circle(frame, spine.back(),  12, Scalar(0,  0,  0),  2);

            // Draw line from bottom to top of spine to visualise curve drift
            line(frame, spine.front(), spine.back(),
                 Scalar(255, 255, 0), 1, LINE_AA);
        }
        else if (spine.size() == 1)
        {
            circle(frame, spine[0], 12, Scalar(0,255,255), -1);
        }

        if (anchor_cx != -1 && anchor_y != -1)
            drawMarker(frame, Point(anchor_cx, anchor_y),
                       Scalar(0,255,0), MARKER_CROSS, 20, 2);

        // Center and zone reference lines
        line(frame, Point(WIDTH/2, HEIGHT/2), Point(WIDTH/2, HEIGHT),
             Scalar(255,255,255), 1);
        line(frame, Point(0, HEIGHT*3/4), Point(WIDTH, HEIGHT*3/4),
             Scalar(255,255,0), 1);

        // ── Steering decision ─────────────────────────────────────────────────
        string status;

        if (lost_frames > LOST_FRAME_LIMIT)
        {
            status = vote_search_direction(history);
            cout << "[LOST] Voted: " << status << "\n";
            pending_active = false;

            line(frame, Point(0,HEIGHT/2),    Point(WIDTH,HEIGHT), Scalar(0,0,255),2);
            line(frame, Point(WIDTH,HEIGHT/2),Point(0,    HEIGHT), Scalar(0,0,255),2);
            putText(frame, "SEARCHING: " + status,
                    Point(10,35), FONT_HERSHEY_SIMPLEX, 1.0, Scalar(0,0,255), 3);

            write(fifo_fd, status.c_str(), status.length());
            write(fifo_fd, "\n", 1);
            //imshow("AI Camera - Path Trace", frame);
            //waitKey(1);
            usleep(100000);
            continue;
        }

        // Compute blended offset that accounts for both position and curvature
        int steering_offset = compute_steering_offset(bottom_cx, spine);

        if      (steering_offset < -HARD_TURN_PX) status = "HARD LEFT";
        else if (steering_offset < -SOFT_TURN_PX) status = "LEFT";
        else if (steering_offset >  HARD_TURN_PX) status = "HARD RIGHT";
        else if (steering_offset >  SOFT_TURN_PX) status = "RIGHT";
        else                                       status = "CENTER";

        cout << "Bias: " << status
             << " | cx=" << bottom_cx
             << " | steer_off=" << steering_offset
             << " | spine=" << spine.size()
             << " | lost=" << lost_frames << "\n";

        // ── Delayed vote recording ────────────────────────────────────────────
        if (lost_frames == 0 && status != "CENTER" && status != "STOP")
        {
            if (!pending_active || pending_status != status)
            {
                pending_status = status;
                pending_start  = steady_clock::now();
                pending_active = true;
            }
            else
            {
                auto ms = duration_cast<milliseconds>(
                    steady_clock::now() - pending_start).count();
                if (ms >= QUEUE_DELAY_MS)
                {
                    history.push_back(status);
                    if ((int)history.size() > VOTE_HISTORY)
                        history.pop_front();
                    pending_start = steady_clock::now();
                    cout << "[QUEUE] " << status
                         << " | hist=" << history.size() << "\n";
                }
            }
        }
        else
        {
            pending_active = false;
        }

        // ── Vote bar ──────────────────────────────────────────────────────────
        if (!history.empty())
        {
            int lv=0,rv=0;
            for (const string& s:history)
            {
                if (s=="LEFT"||s=="HARD LEFT")   lv++;
                if (s=="RIGHT"||s=="HARD RIGHT") rv++;
            }
            int tv=lv+rv;
            if (tv>0)
            {
                int bW=200,bX=WIDTH-210,bY=10,bH=15;
                int lPx=(lv*bW)/tv, rPx=(rv*bW)/tv;
                rectangle(frame,Rect(bX,     bY,lPx,bH),Scalar(255,100,0), -1);
                rectangle(frame,Rect(bX+lPx, bY,rPx,bH),Scalar(0,100,255), -1);
                rectangle(frame,Rect(bX,     bY,bW, bH),Scalar(255,255,255),1);
                putText(frame,"L",Point(bX-15,bY+bH),
                        FONT_HERSHEY_SIMPLEX,0.5,Scalar(255,100,0),1);
                putText(frame,"R",Point(bX+bW+3,bY+bH),
                        FONT_HERSHEY_SIMPLEX,0.5,Scalar(0,100,255),1);
            }
        }

        if (lost_frames > 0)
            putText(frame,
                    "LOST:"+to_string(lost_frames)+"/"+to_string(LOST_FRAME_LIMIT),
                    Point(10,70),FONT_HERSHEY_SIMPLEX,0.6,Scalar(0,0,255),2);

        // Draw steering offset bar at bottom so you can see the blended signal
        {
            int barCx  = WIDTH/2 + steering_offset;
            barCx = max(0, min(WIDTH-1, barCx));
            line(frame, Point(WIDTH/2, HEIGHT-5), Point(barCx, HEIGHT-5),
                 Scalar(0,255,255), 4);
            circle(frame, Point(barCx, HEIGHT-5), 6, Scalar(0,255,255), -1);
        }

        Scalar tc;
        if      (status=="CENTER")                           tc=Scalar(0,255,0);
        else if (status=="HARD LEFT"||status=="HARD RIGHT")  tc=Scalar(0,0,255);
        else                                                 tc=Scalar(0,165,255);

        putText(frame, status, Point(10,35),
                FONT_HERSHEY_SIMPLEX, 1.2, tc, 3);

        putText(frame,
                "cx:"+to_string(bottom_cx)+
                " off:"+to_string(steering_offset)+
                " spine:"+to_string((int)spine.size()),
                Point(10,HEIGHT-15),
                FONT_HERSHEY_SIMPLEX,0.5,Scalar(200,200,200),1);

        //imshow("AI Camera - Path Trace", frame);
        //if (waitKey(1) == 'q') break;

        write(fifo_fd, status.c_str(), status.length());
        write(fifo_fd, "\n", 1);

        usleep(100000);
    }

    close(fifo_fd);
    delete[] buf;
    pclose(pipe);
    //destroyAllWindows();
    return 0;
}