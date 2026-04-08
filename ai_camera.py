import sys
import cv2
import os
import numpy as np
from collections import deque
from picamera2 import MappedArray, Picamera2
from picamera2.devices import IMX500
from picamera2.devices.imx500 import (NetworkIntrinsics,
                                      postprocess_nanodet_detection)

# Configuration
MODEL_PATH = "/usr/share/imx500-models/imx500_network_ssd_mobilenetv2_fpnlite_320x320_pp.rpk"
STOP_THRESHOLD = 0.85
SLOW_THRESHOLD = 0.50
PERSON_CATEGORY_ID = 0 
BUFFER_SIZE = 5  # Shorter buffer (5-8 frames) is usually better for responsiveness
state_buffer = deque(maxlen=BUFFER_SIZE)
last_results = None
FIFO_PATH = "/tmp/ai_camera_status"
fifo_fd = None

class Detection:
    def __init__(self, coords, category, conf, metadata):
        self.category = category
        self.conf = conf
        self.box = imx500.convert_inference_coords(coords, metadata, picam2)

def parse_detections(metadata: dict):
    threshold = 0.4 
    np_outputs = imx500.get_outputs(metadata, add_batch=True)
    input_w, input_h = imx500.get_input_size()
    
    if np_outputs is None:
        return []

    if intrinsics.postprocess == "nanodet":
        boxes, scores, classes = postprocess_nanodet_detection(
            outputs=np_outputs[0], conf=threshold, iou_thres=0.65, max_out_dets=10
        )[0]
        from picamera2.devices.imx500.postprocess import scale_boxes
        boxes = scale_boxes(boxes, 1, 1, input_h, input_w, False, False)
    else:
        boxes, scores, classes = np_outputs[0][0], np_outputs[1][0], np_outputs[2][0]

    return [
        Detection(box, category, score, metadata)
        for box, score, category in zip(boxes, scores, classes)
        if score > threshold and int(category) != PERSON_CATEGORY_ID
    ]

def draw_and_pilot(request, stream="main"):
    global last_results, fifo_fd
    detections = last_results
    if detections is None:
        return

    # 0 = Clear, 1 = Slow, 2 = Stop
    current_frame_severity = 0 

    with MappedArray(request, stream) as m:
        h, w, _ = m.array.shape
        cv2.line(m.array, (0, int(SLOW_THRESHOLD * h)), (w, int(SLOW_THRESHOLD * h)), (255, 255, 0), 2)
        cv2.line(m.array, (0, int(STOP_THRESHOLD * h)), (w, int(STOP_THRESHOLD * h)), (0, 0, 255), 2)

        for detection in detections:
            x, y, bw, bh = detection.box
            bottom_edge = (y + bh) / h 
            
            if bottom_edge > STOP_THRESHOLD:
                current_frame_severity = max(current_frame_severity, 2)
                color = (0, 0, 255) # Red
            elif bottom_edge > SLOW_THRESHOLD:
                current_frame_severity = max(current_frame_severity, 1)
                color = (0, 255, 255) # Cyan/Yellow
            else:
                color = (0, 255, 0) # Green
            
            cv2.rectangle(m.array, (x, y), (x + bw, y + bh), color, 2)

    # Update buffer
    state_buffer.append(current_frame_severity)
    
    # DETERMINE FINAL PILOT STATE
    # If any frame in our buffer says STOP, we stop immediately.
    # If no frame says STOP, but at least one says SLOW, we slow down.
    if 2 in state_buffer:
        final_state = "STOP"
    elif 1 in state_buffer:
        final_state = "CAUTION"
    else:
        final_state = "CLEAR"

    # Write status to named pipe for C motor code
    try:
        if fifo_fd is not None:
            fifo_fd.write(final_state + '\n')
            fifo_fd.flush()
    except (BrokenPipeError, IOError):
        pass  # C code not reading or disconnected; continue anyway

    # Draw status text in top-left corner of GUI
    with MappedArray(request, stream) as m:
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 1.0
        thickness = 2
        text_color = (0, 255, 0)  # Green by default
        
        # Change color based on state
        if final_state == "STOP":
            text_color = (0, 0, 255)  # Red
        elif final_state == "CAUTION":
            text_color = (0, 255, 255)  # Cyan/Yellow
        
        # Draw text box background
        text = f"STATUS: {final_state}"
        (text_width, text_height), baseline = cv2.getTextSize(text, font, font_scale, thickness)
        padding = 10
        cv2.rectangle(m.array, (5, 5), (text_width + padding + 5, text_height + padding + 5), 
                     (50, 50, 50), -1)  # Dark background
        
        # Draw text
        cv2.putText(m.array, text, (padding + 5, text_height + padding), 
                   font, font_scale, text_color, thickness)

# Initialization 
imx500 = IMX500(MODEL_PATH)
intrinsics = imx500.network_intrinsics
if not intrinsics:
    intrinsics = NetworkIntrinsics()
    intrinsics.task = "object detection"

intrinsics.update_with_defaults()

picam2 = Picamera2(imx500.camera_num)
config = picam2.create_preview_configuration(buffer_count=12)

imx500.show_network_fw_progress_bar()

# Create/open named pipe for motor control communication
try:
    import subprocess
    subprocess.run(["mkfifo", FIFO_PATH], stderr=subprocess.DEVNULL)  # Create if doesn't exist
except:
    pass

try:
    fifo_fd = open(FIFO_PATH, 'w', buffering=1)
except Exception as e:
    fifo_fd = None

picam2.start(config, show_preview=True)

picam2.pre_callback = draw_and_pilot

try:
    while True:
        last_results = parse_detections(picam2.capture_metadata())
except KeyboardInterrupt:
    picam2.stop()
