# CSC 615 Final Project — Self-Driving Car

A Raspberry Pi–powered, self-navigating robot car built for the CSC 615 (Embedded Systems) final project at SFSU. The car combines **PWM-driven dual-motor control**, **real-time AI vision obstacle avoidance** (Raspberry Pi AI Camera / IMX500), and **IR line/ultrasonic sensing** to drive forward autonomously and react to what's in front of it.

## How it works

The system is split into a Python AI vision process and a C motor-control process that talk to each other over a named pipe (FIFO), plus a WaveShare hardware abstraction layer shared by everything.

```
┌─────────────────────┐
│   ai_camera.py       │   Runs object detection on the IMX500 AI Camera,
│  (AI Inference)      │   classifies the scene as CLEAR / CAUTION / STOP
└──────────┬───────────┘   based on how close objects are to the bottom
           │ writes status  of the frame.
           ▼
   /tmp/ai_camera_status  (named pipe / FIFO)
           │ reads status
┌──────────┴───────────┐
│ main_ai_integrated.c  │   Launches the Python script, reads its status
│  (Motor Control)      │   on a background thread, and smoothly ramps
└───────────────────────┘   dual motor (A & B) speed up or down in response.
```

- **`CLEAR`** → drive forward at full speed
- **`CAUTION`** → slow down (object detected in the caution zone)
- **`STOP`** → emergency stop (object about to be hit)

The main control loop also has GPIO hooks laid out for an ultrasonic sensor and IR line sensors (`ECHO_PIN`, `TRIG_PIN`, line sensor pins, `ultrasonic_sensor_thread`), so the car can be extended to combine vision-based avoidance with direct proximity/line sensing — that integration is in progress alongside the AI camera pipeline.

## Project layout

| File | Purpose |
|---|---|
| `main_ai_integrated.c` | Main entry point: launches `ai_camera.py` as a background process, spawns a thread to read its FIFO status, and drives both motors with smooth speed ramps based on vision feedback |
| `ai_camera.py` | Runs IMX500 object detection via Picamera2, tracks how close each detection is to the camera, decides CLEAR/CAUTION/STOP, and streams that decision to the FIFO |
| `line_moto_main.c` | Standalone test program that drives Motor B on/off based on a single IR line sensor — used to validate line detection independently before integrating it into the main loop |
| `MotorDriver.c` / `.h` | Motor abstraction over the PCA9685 PWM controller: `motor_init`, `motor_run(motor, speed, direction)`, `motor_stop`, `motor_stop_all` for both Motor A and Motor B |
| `lib/` | WaveShare Motor Driver HAT hardware abstraction layer (`DEV_Config`, `PCA9685`, `dev_hardware_i2c`, `dev_hardware_SPI`, `sysfs_gpio`, `Debug`) — low-level I2C/GPIO access shared across the project |
| `AI_MOTOR_README.md` | Deep-dive on the AI-driven motor control system: architecture, speed behavior table, run steps, and troubleshooting |
| `CAMERA_C_MIGRATION.md` | Design notes for porting the AI camera pipeline from Python into a native C implementation (threading model, API reference, IMX500 integration plan) |

## Hardware

- Raspberry Pi 4/5 with `pigpio` and I2C enabled
- Raspberry Pi AI Camera (Sony IMX500) running an SSD MobileNetV2 object-detection model
- WaveShare Motor Driver HAT (PCA9685-based, I2C address `0x51`), driving two DC motors (Motor A / Motor B)
- IR line sensor(s) and ultrasonic/IR obstacle sensor (wiring in progress, pin placeholders in `main_ai_integrated.c`)

## Build & run

```bash
make            # builds motor_ai (and other targets defined in the Makefile)
sudo ./motor_ai
```

The C program automatically launches `ai_camera.py` as a background process, waits for the AI camera to initialize, creates the FIFO for communication, and starts controlling the motors once vision feedback is flowing. See `AI_MOTOR_README.md` for the full run-through and expected output, and `CAMERA_C_MIGRATION.md` for details on the AI camera pipeline and how it maps onto the C/threading model.

### Python dependencies (for `ai_camera.py`)

```bash
sudo apt-get install -y python3-picamera2
pip install opencv-python numpy
```

Run `make clean` to remove build artifacts.

## Team

Group project for CSC 615, San Francisco State University — Eric Ahsue, Haibin Cao, John Tsiglieris, Kiran Khatri
