# AI-Driven Motor Control System

This system uses your Raspberry Pi camera with AI object detection to automatically control motor speed based on what the camera sees.

## How It Works

- **Python script** (`ai_camera.py`): Runs AI inference on camera feed, writes status to a named pipe
- **C program** (`motor_ai`): Reads status from pipe, controls motor speed accordingly
- **Communication**: Named pipe at `/tmp/ai_camera_status`

```
┌─────────────────────┐
│   ai_camera.py      │
│  (AI Inference)     │
│  Detects obstacles  │
└──────────┬──────────┘
           │ writes status
           ▼
    /tmp/ai_camera_status (named pipe)
           │ reads status
┌──────────┴──────────┐
│    motor_ai         │
│  (Motor Control)    │
│  Adjusts speed      │
└─────────────────────┘
```

## Motor Speed Behavior

| Vision Status | Motor Speed | Use Case |
|---------------|-------------|----------|
| **CLEAR**     | 100%        | No obstacles detected → run at full speed |
| **CAUTION**   | 50%         | Object detected in mid-range → slow down |
| **STOP**      | 0%          | Object very close → emergency stop |

## Steps to Run

### Compile the C code

```bash
cd /home/pi70/dev/okay_we_ball
make clean
make
```

This creates three executables:
- `assignment3` - Original motor demo
- `motor_camera_control` - Concurrent version
- `motor_ai` - **AI-driven motor control** (this is what you want!)

### Run the AI Motor Control System (Everything in One!)

```bash
./motor_ai
```

**That's it!** The C program will automatically:
1. Launch the Python AI camera script as a background process
2. Wait for the camera to initialize 
3. Create the named pipe for communication
4. Wait for you to press the button
5. Start controlling the motor based on AI vision feedback

**Expected Output**:
```
========================================
  AI-Driven Motor Control System
  Running on Raspberry Pi 4
========================================

[Main] Launching Python AI camera script...
[Python] Starting ai_camera.py as background process...
[Main] Python camera process launched (PID: 1234)
[Main] Waiting for Python script to initialize (3 seconds)...
[Main] Initializing pigpio...
[Main] Initializing DEV module...
[Main] Initializing motor...
[Main] Starting vision reader thread...

[Main] Waiting for button press to start motor...
[Main] Note: Motor will be controlled by AI vision feedback
[Main] Press Ctrl+C to exit

[Vision] Waiting for FIFO... (is ai_camera.py running?)
[Vision] Connected to FIFO at /tmp/ai_camera_status
[Vision] Status updated: CLEAR
[Motor] Speed: 100% (Target: 100%, Vision: CLEAR)
[Motor] Speed: 70% (Target: 50%, Vision: CAUTION)
[Motor] Status: STOPPED (Vision: STOP)
```

You'll also see the camera preview pop up automatically.

## How to Test

1. **Run the program**: `./motor_ai` (Python starts automatically)
2. **Watch for the Python output** to appear in your terminal
3. **Wait for the camera preview** to show up
4. **Press the button** to start motor
5. **Move objects** in front of camera to trigger different statuses
6. **Watch motor** slow down when obstacles are detected
7. **Release button** to stop, or press **Ctrl+C** to exit completely

## Troubleshooting

### "Command not found: python3"
- Install Python: `sudo apt-get install python3`

### "ModuleNotFoundError: picamera2"
- Install picamera2: `sudo apt-get install -y python3-picamera2`

### Camera preview doesn't appear
- Make sure you're running on the Raspberry Pi with a connected camera
- Check if camera is enabled: `raspi-config`
- Verify camera is recognized: `libcamera-hello`

### Motor doesn't respond to vision changes
- Check the console output for Python error messages
- Verify `/tmp/ai_camera_status` was created
- Make sure to press the button to start motor control

### Named pipe permission issues
- The C program creates the pipe automatically
- Clean up old pipes if needed: `rm /tmp/ai_camera_status`

## Multiple Runs

The named pipe persists. To clean up between runs:
```bash
rm /tmp/ai_camera_status
```

The Python and C programs will recreate it automatically on startup.

## Key Features

✓ **Responsive** - Reads from pipe every 100ms  
✓ **Smooth transitions** - Motor speed doesn't jump, ramps gradually  
✓ **Multi-threaded** - Vision reader runs in separate thread  
✓ **Non-blocking** - Motor control doesn't freeze waiting for data  
✓ **Graceful shutdown** - Both programs can restart independently  

Enjoy your autonomous obstacle-avoiding robot!
