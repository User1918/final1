# -*- coding: utf-8 -*-
import cv2
import numpy as np
from picamera2 import Picamera2
import time
import serial
import struct
import ncnn  # Import the ncnn Python binding
import re    # Import regular expressions for parsing speed limits

# --- Configuration ---
# NCNN Object Detection Config
PARAM_PATH = "/home/labthayphuc/Desktop/yolov11ncnn/yolov11n_int8_final.param" # <<< UPDATE PATH IF NEEDED
BIN_PATH = "/home/labthayphuc/Desktop/yolov11ncnn/yolov11n_int8_final.bin"   # <<< UPDATE PATH IF NEEDED
NCNN_INPUT_SIZE = 640
NCNN_CONFIDENCE_THRESHOLD = 0.35 # Confidence threshold for final detection
NCNN_NMS_THRESHOLD = 0.35      # Non-Maximum Suppression threshold
NCNN_NUM_CLASSES = 21          # <<< UPDATED based on your list
NCNN_CLASS_NAMES = [           # <<< UPDATED based on your list
    'Pedestrian Crossing', 'Radar', 'Speed Limit -100-', 'Speed Limit -120-',
    'Speed Limit -20-', 'Speed Limit -30-', 'Speed Limit -40-', 'Speed Limit -50-',
    'Speed Limit -60-', 'Speed Limit -70-', 'Speed Limit -80-', 'Speed Limit -90-',
    'Stop Sign', 'Traffic Light -Green-', 'Traffic Light -Off-',
    'Traffic Light -Red-', 'Traffic Light -Yellow-','Pessoa','car',
    'dotted-line','line'
]
NCNN_INPUT_NAME = "in0"      # Default input layer name
NCNN_OUTPUT_NAME = "out0"     # Default output layer name
NCNN_MEAN_VALS = [0.0, 0.0, 0.0] # Mean values for normalization
NCNN_NORM_VALS = [1/255.0, 1/255.0, 1/255.0] # Normalization scale factors
MIN_SIGN_WIDTH_OR_HEIGHT = 100 # Size threshold for non-line objects

# Lane Keeping & Motor Control Config
SERIAL_PORT = '/dev/ttyUSB0'   # <<< ADJUST Serial Port
BAUD_RATE = 115200             # <<< MATCH ESP32 Baud Rate
PICAM_SIZE = (640, 480)        # Camera resolution
PICAM_FRAMERATE = 30           # Camera framerate
TARGET_LANE_OFFSET_FRACTION = 0.25 # Keep single model lane 1/4 of frame width from center

# Perspective Warp Points (Tune these based on camera mount)
leftTop = 160
heightTop = 150
rightTop = 160
leftBottom = 20
heightBottom = 380
rightBottom = 20

# Speed Control Config
DEFAULT_MAX_SPEED = 120 # Default speed limit if none detected
MANUAL_MAX_SPEED = 100  # Max speed allowed with manual 'w' key
MANUAL_ACCELERATION = 5 # Speed increment/decrement with 'w'/'s'
AUTO_MODE_SPEED_STRAIGHT = 30 # Target speed in auto mode ('a') on straight sections
AUTO_MODE_SPEED_CURVE = 20    # Target speed in auto mode ('a') on curved sections

# --- Global Variables (State and Shared Data) ---
# Image Data
frame = None           # Current BGR frame from camera
imgWarp = None         # Current warped binary image for traditional lane finding

# Hardware Interfaces (initialized later)
piCam = None           # Picamera2 object
net = None             # NCNN Net object
ser = None             # Serial object

# Vehicle State / Commands
steering = 90          # Current steering servo angle command (0-180, 90=center)
speed_motor_requested = 0 # Target speed requested by manual keys or auto mode (km/h)
flag = 1               # Legacy flag related to manual speed control
current_speed_limit = DEFAULT_MAX_SPEED # Current active speed limit

# Traditional Lane Finding State (modified by USER-PROVIDED SteeringAngle)
cte_f = 0.0            # Cross-Track Error calculated from warped image (using IIR filter)
left_point = -1        # X-coordinate of detected left lane edge (last row found in scan)
right_point = -1       # X-coordinate of detected right lane edge (last row found in scan)
lane_width = 500       # Estimated lane width from warp (dynamic, updated by SteeringAngle) - RE-ADDED
interested_line_y = 0  # Last Y-coordinate scanned in SteeringAngle
im_height = 0          # Height of warped image
im_width = 0           # Width of warped image

# Model-Based Lane Finding State (Persistent across frames)
last_known_innermost_left_x = -1.0 # Last known X-coord of left line from model
last_known_innermost_right_x = -1.0# Last known X-coord of right line from model
cte_source = "Initializing" # Tracks which method determined the current CTE

# Performance Monitoring State
frame_counter = 0
loop_start_time = 0.0
overall_fps = 0.0

# --- Helper Functions ---

def process_image():
    """ Captures frame, converts to BGR, preprocesses for lanes, and warps perspective.
        Updates global 'frame' and 'imgWarp'. Returns True on success, False on capture failure.
    """
    global frame, imgWarp, leftTop, heightTop, rightTop, leftBottom, heightBottom, rightBottom, piCam
    if piCam is None: return False # Camera not initialized

    try:
        rgb_frame = piCam.capture_array()
    except Exception as e:
        print(f"Capture Error: {e}")
        frame = None
        imgWarp = None
        return False # Indicate capture failure

    if rgb_frame is None:
        frame = None
        imgWarp = None
        return False

    # Convert to BGR *immediately* for NCNN detection compatibility
    frame = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2BGR)

    # --- Lane processing using grayscale for traditional method ---
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (11, 11), 0)
    edges = cv2.Canny(blur, 100, 150, apertureSize=3) # Canny edge detection
    h, w = edges.shape

    # Define perspective transform points
    points = np.float32([(leftTop, heightTop), (w - rightTop, heightTop),
                         (leftBottom, heightBottom), (w - rightBottom, heightBottom)])
    pts2 = np.float32([[0, 0], [w, 0], [0, h], [w, h]])

    # Apply perspective warp
    try:
        matrix = cv2.getPerspectiveTransform(points, pts2)
        imgWarp = cv2.warpPerspective(edges, matrix, (w, h)) # Output is global imgWarp
    except cv2.error as e:
        print(f"Warp Error: {e}")
        imgWarp = None # Set warp to None on error
        # Still return True because frame capture was successful

    return True # Indicate frame capture success

def ve_truc_anh(img, step=100):
    """ Draws coordinate axes and grid lines on an image. """
    if img is None: return None # Handle None input gracefully
    # Ensure image is BGR for drawing colors
    if len(img.shape) == 2: # If grayscale
        img_color = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    else:
        img_color = img.copy() # Work on a copy

    h, w = img_color.shape[:2]
    if h == 0 or w == 0: return img_color # Return if dimensions are invalid

    # Draw axes
    cv2.line(img_color, (0, h - 1), (w, h - 1), (200, 200, 200), 1) # X axis
    cv2.line(img_color, (0, 0), (0, h), (200, 200, 200), 1)       # Y axis
    # Draw grid markers and labels
    for x in range(0, w, step):
        cv2.line(img_color, (x, h - 10), (x, h), (200, 200, 200), 1)
        cv2.putText(img_color, str(x), (x + 2, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
    for y in range(0, h, step):
        cv2.line(img_color, (0, y), (10, y), (200, 200, 200), 1)
        cv2.putText(img_color, str(y), (12, y + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
    return img_color
def SteeringAngle():
    """TÃ­nh sai lá»‡ch lÃ¡i (CTE) dá»±a trÃªn quÃ©t cÃ¡c hÃ ng pixel cá»§a áº£nh warp."""
    global left_point, right_point, interested_line_y
    global im_height, im_width, cte_f
    im_height, im_width = imgWarp.shape[:2]
    left_point = -1
    right_point = -1
    center_img = im_width // 2
    set_point = 300
    cv2.line(imgWarp, (0, set_point), (im_width, set_point), (255, 0, 0), 2)
    pre_diff = 0
    step = -5
    u = (im_height - set_point) / abs(step)
    ki = (-u / im_height) + 1
    for i in range(im_height - 1, set_point, step):
        interested_line_y = int(i)
        interested_line = imgWarp[interested_line_y, :]
        for x in range(center_img, 0, -1):
            if interested_line[x] > 0:
                left_point = x
                break
        for x in range(center_img + 1, im_width):
            if interested_line[x] > 0:
                right_point = x
                break
        if left_point != -1 and right_point != -1:
            lane_width = right_point - left_point
        mid_point = (right_point + left_point) / 2
        diff = center_img - mid_point
        diff = diff * ki + pre_diff
        pre_diff = diff
    cte_f = diff / u
def signal_motor(key):
    """ Updates the global 'speed_motor_requested' based on key input. """
    global speed_motor_requested, flag, final_cte_for_pid # Uses final_cte_for_pid
    new_speed_req = speed_motor_requested # Start with current speed

    # Manual Speed Control
    if key == ord('w'): # Increase speed
        new_speed_req = min(speed_motor_requested + MANUAL_ACCELERATION, MANUAL_MAX_SPEED)
        flag = 1
    elif key == ord('s'): # Decrease speed
        new_speed_req = max(speed_motor_requested - MANUAL_ACCELERATION, 0) # Speed cannot be negative
    # Auto Speed Control ('a' key)
    elif key == ord('a'): # Set speed based on current curvature (estimated by FINAL CTE)
        CURVE_CTE_THRESHOLD = 25 # Example threshold - TUNE THIS
        target_speed = AUTO_MODE_SPEED_STRAIGHT if abs(final_cte_for_pid) < CURVE_CTE_THRESHOLD else AUTO_MODE_SPEED_CURVE
        new_speed_req = target_speed
        flag = 1 # Indicates auto mode was just activated/set
    # Stop ('x' key)
    elif key == ord('x'): # Emergency stop / manual stop
        new_speed_req = 0

    # Update global state only if changed
    if new_speed_req != speed_motor_requested:
        speed_motor_requested = new_speed_req

# --- PID Class (Unchanged) ---
class PID:
    """ Simple PID Controller Class with delta_time calculation. """
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.error_previous = 0.0 # Use float for errors
        self.integral = 0.0       # Use float for integral
        self.last_time = time.time() # Initialize time for first dt calculation

    def update(self, error_f):
        """ Calculates PID output based on error and elapsed time. """
        current_time = time.time()
        delta_time = current_time - self.last_time
        if delta_time <= 1e-6: delta_time = 1e-6
        self.last_time = current_time
        p_term = self.kp * error_f
        self.integral += error_f * delta_time
        i_term = self.ki * self.integral
        derivative = (error_f - self.error_previous) / delta_time
        d_term = self.kd * derivative
        self.error_previous = error_f
        pid_output = p_term + i_term + d_term
        steering_angle = 90.0 + pid_output
        steering_servo_angle = round(max(70, min(110, steering_angle))) # TUNED RANGE
        return steering_servo_angle

# --- NCNN Detection Function (Unchanged)----
def detect_signs_and_get_results(input_frame_bgr):
    """ Performs NCNN object detection. Returns list of detected object dictionaries. """
    global net, NCNN_INPUT_SIZE, NCNN_INPUT_NAME, NCNN_OUTPUT_NAME, NCNN_MEAN_VALS, NCNN_NORM_VALS
    global NCNN_NUM_CLASSES, NCNN_CLASS_NAMES, NCNN_CONFIDENCE_THRESHOLD, NCNN_NMS_THRESHOLD

    detections_results = []
    if input_frame_bgr is None or net is None: return detections_results
    original_height, original_width = input_frame_bgr.shape[:2]
    if original_height == 0 or original_width == 0: return detections_results

    try:
        target_size = NCNN_INPUT_SIZE
        scale = min(target_size / original_width, target_size / original_height)
        new_w, new_h = int(original_width * scale), int(original_height * scale)
        dw = (target_size - new_w) // 2
        dh = (target_size - new_h) // 2
        padded_img = np.full((target_size, target_size, 3), 114, dtype=np.uint8)
        if new_w > 0 and new_h > 0:
            resized_img = cv2.resize(input_frame_bgr, (new_w, new_h))
            padded_img[dh:dh+new_h, dw:dw+new_w, :] = resized_img
        else:
             padded_img = cv2.resize(input_frame_bgr, (target_size, target_size))
             print("Warning: Resize resulted in zero dimension. Using direct resize.")
             dw, dh, scale = 0, 0, target_size / max(original_width, original_height)

        mat_in = ncnn.Mat.from_pixels(padded_img, ncnn.Mat.PixelType.PIXEL_BGR, target_size, target_size)
        mat_in.substract_mean_normalize(NCNN_MEAN_VALS, NCNN_NORM_VALS)

    except Exception as e:
        print(f"NCNN Preprocessing Error: {e}")
        return detections_results

    try:
        ex = net.create_extractor()
        ex.input(NCNN_INPUT_NAME, mat_in)
        ret_extract, mat_out = ex.extract(NCNN_OUTPUT_NAME)
        if ret_extract != 0:
            print(f"NCNN Extract Error {ret_extract}")
            return detections_results
    except Exception as e:
        print(f"NCNN Inference Error: {e}")
        return detections_results

    output_data = np.array(mat_out)
    expected_output_dim = 4 + NCNN_NUM_CLASSES

    if len(output_data.shape) == 3 and output_data.shape[0] == 1:
        output_data = output_data[0]
    elif len(output_data.shape) == 2 and output_data.shape[0] == expected_output_dim:
        output_data = output_data.T

    if len(output_data.shape) != 2 or output_data.shape[1] != expected_output_dim:
        print(f"Unexpected NCNN output shape: {output_data.shape}. Expected (N, {expected_output_dim})")
        return detections_results

    num_detections = output_data.shape[0]
    boxes = []
    confidences = []
    class_ids = []

    for i in range(num_detections):
        detection = output_data[i]
        cx, cy, w_ncnn, h_ncnn = detection[:4]
        class_scores = detection[4:]
        class_id = np.argmax(class_scores)
        max_class_score = np.max(class_scores)
        final_confidence = max_class_score

        if final_confidence >= NCNN_CONFIDENCE_THRESHOLD:
            x1_resized = (cx - w_ncnn / 2)
            y1_resized = (cy - h_ncnn / 2)
            x2_resized = (cx + w_ncnn / 2)
            y2_resized = (cy + h_ncnn / 2)
            x1_nopad = x1_resized - dw
            y1_nopad = y1_resized - dh
            x2_nopad = x2_resized - dw
            y2_nopad = y2_resized - dh
            if scale > 1e-6:
                x1_orig = x1_nopad / scale
                y1_orig = y1_nopad / scale
                x2_orig = x2_nopad / scale
                y2_orig = y2_nopad / scale
            else:
                 x1_orig, y1_orig, x2_orig, y2_orig = 0, 0, 0, 0
            x1_orig = max(0, x1_orig)
            y1_orig = max(0, y1_orig)
            w_orig = x2_orig - x1_orig
            h_orig = y2_orig - y1_orig
            w_orig = max(0, min(w_orig, original_width - x1_orig - 1))
            h_orig = max(0, min(h_orig, original_height - y1_orig - 1))
            boxes.append([int(x1_orig), int(y1_orig), int(w_orig), int(h_orig)])
            confidences.append(float(final_confidence))
            class_ids.append(class_id)

    indices = []
    if boxes:
        indices = cv2.dnn.NMSBoxes(boxes, confidences, NCNN_CONFIDENCE_THRESHOLD, NCNN_NMS_THRESHOLD)

    if len(indices) > 0:
        if isinstance(indices, (list, tuple)) and len(indices) > 0 and isinstance(indices[0], (list, np.ndarray)):
             indices = indices.flatten()
        processed_indices = set()
        for idx in indices:
            i = int(idx)
            if 0 <= i < len(boxes) and i not in processed_indices:
                box = boxes[i]
                x, y, w, h = box
                confidence_nms = confidences[i]
                class_id_nms = class_ids[i]
                processed_indices.add(i)
                if 0 <= class_id_nms < len(NCNN_CLASS_NAMES):
                    class_name = NCNN_CLASS_NAMES[class_id_nms]
                else:
                    class_name = f"ID:{class_id_nms}"
                detections_results.append({
                    "name": class_name,
                    "confidence": confidence_nms,
                    "box": box,
                    "width": w,
                    "height": h
                })
    return detections_results

# --- Speed Limit Parsing Function (Unchanged) ---
def parse_speed_limit(class_name):
    match = re.search(r'Speed Limit -(\d+)-', class_name)
    if match:
        try: return int(match.group(1))
        except ValueError: return None
    return None

# --- Speed Transformation Function (Unchanged) ---
def transform_speed(velocity):
    if velocity < 0: velocity = 0
    f_velocity = velocity / 10.0
    motor_rpm = (f_velocity * 30 * 2.85) / (3.14 * 0.0475 * 3.6) # Check constants
    return int(round(motor_rpm))


# ==============================================================================
# ========================== MAIN EXECUTION BLOCK ============================
# ==============================================================================
if __name__ == "__main__":

    # --- Initializations ---
    # Serial Port
    print(f"Initializing Serial on {SERIAL_PORT}...")
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)
        print(f"Serial initialized on {SERIAL_PORT} at {BAUD_RATE} baud.")
    except serial.SerialException as e:
        print(f"CRITICAL: Serial Error: {e}. Port {SERIAL_PORT}.")
        ser = None
    except Exception as e:
        print(f"CRITICAL: Unexpected serial initialization error: {e}")
        ser = None

    # NCNN Model
    print("Initializing NCNN Net...")
    net = ncnn.Net()
    print("Checking for Vulkan GPU...")
    gpu_count = 0
    try: gpu_count = ncnn.get_gpu_count()
    except Exception as e: print(f"Warning: NCNN GPU count failed - {e}. CPU only.")

    if gpu_count > 0:
        print(f"NCNN: Vulkan GPU detected ({gpu_count} device(s)).")
        net.opt.use_vulkan_compute = False # <<< SET TO True TO TRY GPU
        if net.opt.use_vulkan_compute: print("NCNN: Vulkan acceleration enabled.")
    else:
        print("NCNN: Vulkan GPU not found. Using CPU threads.")
        net.opt.use_vulkan_compute = False
    if not net.opt.use_vulkan_compute:
        net.opt.num_threads = 4 # Adjust for Pi 5 cores
        print(f"NCNN: Using {net.opt.num_threads} CPU threads.")

    print("Loading NCNN model parameters and weights...")
    try:
        start_load_time = time.time()
        if net.load_param(PARAM_PATH) != 0: print(f"CRITICAL: Load Param Error: {PARAM_PATH}"); exit()
        if net.load_model(BIN_PATH) != 0: print(f"CRITICAL: Load Model Error: {BIN_PATH}"); exit()
        end_load_time = time.time()
        print(f"NCNN model loaded successfully in {end_load_time - start_load_time:.4f} seconds.")
    except Exception as e: print(f"CRITICAL: NCNN model loading exception: {e}"); exit()

    # PiCamera2 Setup
    print("Initializing Picamera2...")
    piCam = Picamera2()
    try:
        preview_config = piCam.create_preview_configuration(main={"size": PICAM_SIZE, "format": "RGB888"})
        piCam.configure(preview_config)
        piCam.set_controls({"FrameRate": float(PICAM_FRAMERATE)})
        piCam.start()
        time.sleep(1.0)
        print("Picamera2 started.")
    except Exception as e:
        print(f"CRITICAL: Picamera2 Error: {e}")
        if piCam.started: piCam.stop()
        exit()

    # PID Controllers Initialization
    pid_controller1 = PID(0.35, 0.001, 0.5) # High speed - TUNE
    pid_controller2 = PID(1.0, 0.001, 0.1)  # Medium speed - TUNE
    pid_controller3 = PID(1.0, 0.001, 0.1)  # Low speed/Stop - TUNE

    # Initialize persistent state variables before the loop
    last_known_innermost_left_x = -1.0
    last_known_innermost_right_x = -1.0
    current_speed_limit = DEFAULT_MAX_SPEED
    speed_motor_requested = 0
    steering = 90
    cte_source = "Initializing"
    final_cte_for_pid = 0.0
    lane_width = 500 # Initialize global lane_width RE-ADDED for SteeringAngle

    # Initialize performance counters
    frame_counter = 0
    loop_start_time = time.time()
    overall_fps = 0.0

    # --- Main Control Loop ---
    try:
        frame_width = PICAM_SIZE[0]
        frame_height = PICAM_SIZE[1]
        frame_center_x = frame_width // 2
        target_offset_pixels = frame_width * TARGET_LANE_OFFSET_FRACTION
        target_left_x_offset = frame_center_x - target_offset_pixels
        target_right_x_offset = frame_center_x + target_offset_pixels

        while True:
            frame_start_time_this = time.time()

            # --- Reset variables for CURRENT frame ---
            model_current_innermost_left_x = -1.0
            model_current_innermost_right_x = -1.0
            model_lane_detected_this_frame = False
            active_detections_this_frame = []

            # 1. Capture and Preprocess Image
            if not process_image():
                print("Warning: Frame capture/processing failed.")
                time.sleep(0.01)
                continue

            # 2. Traditional Lane Finding (using warped image - USER PROVIDED VERSION)
            # Updates global cte_f, left_point, right_point based on its internal logic
            SteeringAngle()
            # We will check left_point and right_point *after* this call in Step 7

            # 3. Model Detection (using original BGR frame)
            detected_objects = []
            if frame is not None:
                detected_objects = detect_signs_and_get_results(frame)
            else:
                print("Warning: Original frame is None, skipping detection.")

            # 4. Handle Key Presses
            key = cv2.waitKey(1) & 0xFF
            signal_motor(key)
            if key == ord('q'):
                print("'q' pressed, exiting loop.")
                break

            # 5. Process Detections (Filters, Lanes, Signs)
            stop_condition_met = False
            limit_sign_seen_this_frame = False
            new_limit_value = -1
            current_frame_left_segments = []
            current_frame_right_segments = []

            if detected_objects:
                for detection in detected_objects:
                    obj_name = detection['name']
                    box = detection['box']
                    x, y, w, h = box
                    obj_w = detection.get('width', 0)
                    obj_h = detection.get('height', 0)
                    process_this_detection = False

                    if obj_name in ["dotted-line", "line"]:
                        process_this_detection = True
                    else:
                        if obj_w > MIN_SIGN_WIDTH_OR_HEIGHT or obj_h > MIN_SIGN_WIDTH_OR_HEIGHT:
                            process_this_detection = True

                    if process_this_detection:
                        active_detections_this_frame.append(detection)
                        if obj_name in ["dotted-line", "line"]:
                            object_center_x = x + w / 2.0
                            segment_info = {'box': box, 'center_x': object_center_x, 'name': obj_name}
                            if object_center_x < frame_center_x:
                                current_frame_left_segments.append(segment_info)
                            elif object_center_x > frame_center_x:
                                current_frame_right_segments.append(segment_info)
                            model_lane_detected_this_frame = True
                        elif obj_name in ["Stop Sign", "Traffic Light -Red-"]:
                            stop_condition_met = True
                        else:
                            parsed_limit = parse_speed_limit(obj_name)
                            if parsed_limit is not None:
                                limit_sign_seen_this_frame = True
                                if new_limit_value == -1 or parsed_limit < new_limit_value:
                                    new_limit_value = parsed_limit

            # 6. Update Persistent Model Lane Positions & Handle Reset
            if current_frame_left_segments:
                innermost_left_segment = max(current_frame_left_segments, key=lambda item: item['center_x'])
                model_current_innermost_left_x = innermost_left_segment['center_x']
                last_known_innermost_left_x = model_current_innermost_left_x
            if current_frame_right_segments:
                innermost_right_segment = min(current_frame_right_segments, key=lambda item: item['center_x'])
                model_current_innermost_right_x = innermost_right_segment['center_x']
                last_known_innermost_right_x = model_current_innermost_right_x

            if not model_lane_detected_this_frame:
                last_known_innermost_left_x = -1.0
                last_known_innermost_right_x = -1.0

            # 7. Determine Final Steering CTE using Hierarchy (PRIORITIZING NEW SteeringAngle)
            final_cte_for_pid = 0.0 # Reset
            cte_source = "Default (0.0)" # Default

            # --- MODIFIED HIERARCHY ---
            # Priority 1: User-provided Traditional method found both lanes by end of scan
            # Check the *final* state of left_point/right_point AFTER SteeringAngle() ran
            if left_point != -1 and right_point != -1:
                final_cte_for_pid = cte_f # Use the CTE calculated by SteeringAngle (with IIR filter)
                cte_source = "Traditional"
            else:
                # Traditional method failed (or only found one point reliably in its scan)
                # Fallback to Model-based logic using persistent last known positions
                know_left = last_known_innermost_left_x > 0
                know_right = last_known_innermost_right_x > 0

                # Priority 2: Model knows both lanes
                if know_left and know_right:
                    detected_lane_center = (last_known_innermost_left_x + last_known_innermost_right_x) / 2.0
                    final_cte_for_pid = float(frame_center_x) - detected_lane_center
                    cte_source = "Model (Both Known)"
                # Priority 3: Model knows only Left lane -> Steer to offset
                elif know_left:
                    final_cte_for_pid = target_left_x_offset - last_known_innermost_left_x
                    cte_source = "Model (Offset L)"
                # Priority 4: Model knows only Right lane -> Steer to offset
                elif know_right:
                    final_cte_for_pid = target_right_x_offset - last_known_innermost_right_x
                    cte_source = "Model (Offset R)"
                # Priority 5: No lanes known by any reliable method (already defaulted above)

            # 8. Update Active Speed Limit
            if limit_sign_seen_this_frame and new_limit_value != -1:
                if new_limit_value != current_speed_limit:
                    print(f"** Speed Limit Updated: {new_limit_value} km/h **")
                    current_speed_limit = new_limit_value

            # 9. Determine Final Commanded Speed (km/h and RPM)
            final_speed_command_vel = speed_motor_requested
            final_speed_command_vel = min(final_speed_command_vel, current_speed_limit)
            if stop_condition_met:
                if final_speed_command_vel > 0: print("** Stop condition met! Speed = 0. **")
                final_speed_command_vel = 0
            final_speed_command_rpm = transform_speed(final_speed_command_vel)

            # 10. PID Calculation for Steering Angle
            current_pid = pid_controller3 # Default
            if final_speed_command_vel >= 20 : current_pid = pid_controller1
            elif final_speed_command_vel > 0: current_pid = pid_controller2
            steering = current_pid.update(final_cte_for_pid) # Use the final calculated CTE

            # 11. Send Commands to ESP32/Arduino via Serial
            if ser and ser.is_open:
                try:
                    data_to_send = struct.pack('<ii', steering, final_speed_command_rpm)
                    ser.write(b'<' + data_to_send + b'>')
                    ser.flush()
                except serial.SerialException as e:
                    print(f"Serial Write Error: {e}. Disabling.")
                    ser.close(); ser = None
                except Exception as e:
                    print(f"Unexpected serial send Error: {e}")
                    if ser: ser.close(); ser = None

            # 12. Calculate and Print FPS / Status
            frame_counter += 1
            frame_time_this = time.time() - frame_start_time_this
            instant_fps = 1.0 / frame_time_this if frame_time_this > 0 else 0
            elapsed_time_total = time.time() - loop_start_time
            if elapsed_time_total > 1: overall_fps = frame_counter / elapsed_time_total

            if frame_counter % 30 == 0:
                 # Check traditional success based on final points from SteeringAngle
                 trad_success_this_frame = (left_point != -1 and right_point != -1)
                 print(f"Status: Lim={current_speed_limit}|Req={speed_motor_requested}|Sent={final_speed_command_rpm}rpm|Steer={steering}|"
                       f"CTE={final_cte_for_pid:.2f}({cte_source})|TradOK={trad_success_this_frame}|"
                       f"L={last_known_innermost_left_x:.1f}|R={last_known_innermost_right_x:.1f}|"
                       f"FPS:{instant_fps:.1f}(Avg:{overall_fps:.1f})")

            # 13. Display Visualization Windows
            # --- Draw on Original Frame ---
            display_frame = frame.copy() if frame is not None else np.zeros((frame_height, frame_width, 3), dtype=np.uint8)
            cv2.putText(display_frame, f"CTE Source: {cte_source}", (10, frame_height - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            cv2.putText(display_frame, f"Final CTE: {final_cte_for_pid:.2f}", (10, frame_height - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            cv2.line(display_frame, (frame_center_x, 0), (frame_center_x, frame_height), (200, 200, 200), 1)
            cv2.line(display_frame, (int(target_left_x_offset), 0), (int(target_left_x_offset), frame_height), (0, 255, 0), 1)
            cv2.line(display_frame, (int(target_right_x_offset), 0), (int(target_right_x_offset), frame_height), (0, 255, 0), 1)

            for detection in active_detections_this_frame:
                d_x, d_y, d_w, d_h = detection['box']
                d_name = detection['name']
                d_conf = detection['confidence']
                label = f"{d_name}: {d_conf:.2f}"
                color = (0, 180, 0); thickness = 1
                if d_name in ["dotted-line", "line"]:
                    obj_center_x = d_x + d_w / 2.0
                    know_left = last_known_innermost_left_x > 0
                    know_right = last_known_innermost_right_x > 0
                    is_tracked_left = know_left and abs(obj_center_x - last_known_innermost_left_x) < 5
                    is_tracked_right = know_right and abs(obj_center_x - last_known_innermost_right_x) < 5
                    if obj_center_x < frame_center_x:
                        color = (255, 165, 0); label += " (L?)"
                        if is_tracked_left: thickness = 3; label = f"{d_name}: {d_conf:.2f} (TRACKED L)"
                    elif obj_center_x > frame_center_x:
                        color = (255, 0, 255); label += " (R?)"
                        if is_tracked_right: thickness = 3; label = f"{d_name}: {d_conf:.2f} (TRACKED R)"
                elif d_name in ["Stop Sign", "Traffic Light -Red-"]: color = (0, 0, 255); thickness = 2
                cv2.rectangle(display_frame, (d_x, d_y), (d_x + d_w, d_y + d_h), color, thickness)
                cv2.putText(display_frame, label, (d_x, d_y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, thickness)

            if last_known_innermost_left_x > 0:
                cv2.circle(display_frame, (int(last_known_innermost_left_x), frame_height - 20), 7, (255, 165, 0), -1)
                cv2.putText(display_frame, "Lk", (int(last_known_innermost_left_x) - 10, frame_height - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255,255,255), 1)
            if last_known_innermost_right_x > 0:
                cv2.circle(display_frame, (int(last_known_innermost_right_x), frame_height - 20), 7, (255, 0, 255), -1)
                cv2.putText(display_frame, "Rk", (int(last_known_innermost_right_x) - 10, frame_height - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255,255,255), 1)
            cv2.imshow("Object Detections & Lane Tracking", display_frame)

            # --- Display Warped Lane Image (Traditional Method Debug) ---
            if imgWarp is not None:
                display_img_warp = ve_truc_anh(imgWarp, step=50)
                if display_img_warp is not None:
                    # Check success based on final points AFTER SteeringAngle ran
                    trad_success_final = (left_point != -1 and right_point != -1)
                    cv2.putText(display_img_warp, f"Trad Final Points Found: {trad_success_final}", (10, im_height - 40 if im_height>40 else 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 1)
                    # Display the cte_f calculated by the IIR method
                    cv2.putText(display_img_warp, f"Trad CTE (cte_f - IIR): {cte_f:.2f}", (10, im_height - 20 if im_height>20 else 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 1)
                    # Final points are already drawn inside SteeringAngle in red
                    cv2.imshow("Traditional Lane Warp (IIR - Debug)", display_img_warp)
            # --- End of Loop ---

    # --- Error Handling & Cleanup ---
    except KeyboardInterrupt: print("\nCtrl+C detected. Exiting...")
    except Exception as e:
        print(f"\n--- UNHANDLED EXCEPTION IN MAIN LOOP ---")
        print(f"Error Type: {type(e).__name__}")
        print(f"Error Details: {e}")
        import traceback; traceback.print_exc()
        print(f"--- END OF TRACEBACK ---")
    finally:
        print("\nCleaning up resources...")
        if ser and ser.is_open:
            try:
                print("Sending stop command...")
                stop_data = struct.pack('<ii', 90, 0); ser.write(b'<' + stop_data + b'>'); ser.flush(); time.sleep(0.1); ser.close()
                print("Serial port closed.")
            except Exception as e: print(f"Error during serial cleanup: {e}")
        if piCam and piCam.started:
             try: piCam.stop(); print("Picamera2 stopped.")
             except Exception as e: print(f"Error stopping Picamera2: {e}")
        cv2.destroyAllWindows()
        print("OpenCV windows closed.")
        print("Cleanup complete. Exiting program.")
