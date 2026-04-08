# AI Camera to C Translation Guide

## Overview
Your Python `ai_camera.py` script has been translated to C with support for running alongside your motor code. The implementation consists of:

1. **ai_camera.h** - Header file with camera interface
2. **ai_camera.c** - Camera AI detection implementation (runs in background thread)
3. **main_concurrent.c** - Updated main.c that integrates both systems
4. **Updated Makefile** - Compiles both original and new concurrent version

## Key Changes from Python

### Python Features → C Implementation

| Python Feature | C Implementation |
|---|---|
| `Picamera2` + `IMX500` | Camera thread with detection buffer |
| `deque(maxlen=BUFFER_SIZE)` | Fixed array buffer with circular index |
| `parse_detections()` | `read_detections_from_camera()` |
| `draw_and_pilot()` | `process_detections()` + motor control |
| Main loop with metadata | Camera thread runs continuously |
| Global state | Thread-safe mutexes + atomic operations |

## C API Reference

### Core Functions

```c
/* Initialize camera system */
int camera_init(void);

/* Start background camera thread */
int camera_start(void);

/* Stop camera thread and cleanup */
int camera_stop(void);

/* Get current pilot state */
pilot_state_t camera_get_pilot_state(void);

/* Get detection count */
int camera_get_detection_count(void);

/* Get specific detection */
int camera_get_detection(int index, detection_t *det);
```

### Pilot States (from Python)

```c
typedef enum {
    STATE_CLEAR = 0,           // Clear path
    STATE_CAUTION = 1,         // Slow down (bottom_edge > 0.50)
    STATE_EMERGENCY_STOP = 2   // Stop (bottom_edge > 0.85)
} pilot_state_t;
```

### Detection Structure

```c
typedef struct {
    float x, y;           /* Top-left bounding box coordinates */
    float width, height;  /* Box dimensions */
    float confidence;     /* Detection confidence score */
    int category;         /* Object category ID */
} detection_t;
```

## Motor Integration

The motor control automatically responds to camera states:

```c
STATE_EMERGENCY_STOP  → motor_stop()
STATE_CAUTION         → motor_run(30%, current_direction)  
STATE_CLEAR           → Motor operates normally
```

See `apply_motor_command()` in main_concurrent.c for details.

## Threading Model

```
┌─────────────────────────────────────────┐
│          Main Thread (motor demo)       │
│  - Runs motor sequence steps            │
│  - Polls camera state periodically      │
│  - Applies motor commands based on      │
│    camera input                         │
└─────────────────┬───────────────────────┘
                  │ pthread_mutex
┌─────────────────▼───────────────────────┐
│      Camera Thread (background)          │
│  - Continuously reads camera            │
│  - Processes detections                 │
│  - Updates pilot state                  │
│  - Runs at ~30 FPS                      │
└─────────────────────────────────────────┘
```

## Compilation

### Build Both Versions
```bash
make
```

This creates:
- `assignment3` - Original motor demo (unchanged)
- `motor_camera_control` - Motor + Camera integrated demo

### Build Only Concurrent Version
```bash
make motor_camera_control
```

### Cleanup
```bash
make clean
```

## Usage

```bash
# Run the integrated motor + camera system
sudo ./motor_camera_control
```

Then press the button to start the motor sequence. The camera will run in the background and override motor control based on detections.

## Integration with IMX500

The current implementation is a framework. To fully integrate your IMX500 camera:

### In `ai_camera.c`, modify `read_detections_from_camera()`:

```c
static void read_detections_from_camera(void) {
    /* TODO: Add libcamera frame capture here */
    /* libcamera_capture_frame(&frame); */
    
    /* TODO: Read IMX500 inference metadata */
    /* parse_imx500_metadata(frame.metadata); */
    
    /* TODO: Extract bounding boxes and scores */
    /* Store in global 'detections' array and update 'detection_count' */
    
    pthread_mutex_lock(&state_mutex);
    /* Update detections[] and detection_count */
    pthread_mutex_unlock(&state_mutex);
}
```

### Required Libraries for Full Integration:
- `libcamera` - Camera framework
- `libtiff`, `libjpeg` - Image processing
- `libssl` - For IMX500 firmware
- OpenCV optional - For advanced image processing

## Thread Safety

All camera state is protected by `pthread_mutex_t state_mutex`:

```c
pthread_mutex_lock(&state_mutex);
pilot_state_t state = current_pilot_state;
pthread_mutex_unlock(&state_mutex);
```

The API functions handle locking internally, so you can call them safely from any thread.

## Configuration Constants

In `ai_camera.c`:

```c
#define STOP_THRESHOLD 0.85f       /* Bottom edge threshold for STOP */
#define SLOW_THRESHOLD 0.50f       /* Bottom edge threshold for CAUTION */
#define PERSON_CATEGORY_ID 0       /* Skip persons in detection */
#define DETECTION_CONFIDENCE_THRESHOLD 0.4f  /* Minimum confidence */
#define BUFFER_SIZE 5              /* Frames for smoothing */
#define MAX_DETECTIONS 20          /* Max simultaneous detections */
```

Modify these in `ai_camera.c` before calling `camera_init()`.

## Debugging

The system prints status updates to stdout:

```
[CAMERA] STATUS: CLEAR | Detections: 0
[MOTOR] Reducing speed to 30% (CAUTION)
[MOTOR] EMERGENCY STOP triggered
```

Enable verbose logging by adding `DEBUG()` calls in MotorDriver.c and ai_camera.c.

## Next Steps

1. **Test basic compilation** - Ensure Makefile works
2. **Integrate libcamera** - Add camera frame capture
3. **Add IMX500 inference** - Use existing IMX500 libraries
4. **Tune thresholds** - Adjust STOP/SLOW thresholds for your setup
5. **Test concurrent operation** - Verify motor responds to camera states

## Differences from Original Python

| Aspect | Python | C |
|--------|--------|---|
| **Startup** | Immediate | Requires camera_init() + camera_start() |
| **Memory** | Automatic | Manual cleanup with camera_stop() |
| **Threading** | Single-threaded event loop | Explicit pthread |
| **FPS** | Configurable | ~30 FPS (hardcoded in sleep) |
| **Drawing** | OpenCV overlay | Status printing only |
| **State updates** | Real-time | ~33ms latency |

The C version sacrifices the visual overlay but gains:
- Non-blocking motor control
- Deterministic timing
- Ability to run on embedded Linux without X11
- No Python interpreter overhead
