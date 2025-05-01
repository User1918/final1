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
PICAM_SIZE = (640, 480)        # Camera resolution (MUST BE 640x480 for this NCNN preprocessing logic)
PICAM_FRAMERATE = 30           # Camera framerate
# LANE_WIDTH_ASSUMPTION_PIXELS = 380 # <<< REMOVED - No longer used for fallback
TARGET_LANE_OFFSET_FRACTION = 0.25 # Keep single lane 1/4 of frame width from center

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

# Traditional Lane Finding State (modified by SteeringAngle)
cte_f = 0.0            # Cross-Track Error calculated from warped image (ONLY IF BOTH LINES FOUND)
left_point = -1        # X-coordinate of detected left lane edge at bottom of warp
right_point = -1       # X-coordinate of detected right lane edge at bottom of warp
# pre_diff = 0           # Previous difference state (if using IIR filter in SteeringAngle) - Simplified logic below
# lane_width = 500       # Estimated lane width from warp (dynamic) - Less critical now
# lane_width_max = 500   # Max observed lane width from warp - No longer used for estimation
interested_line_y = 0  # Last Y-coordinate scanned in SteeringAngle
im_height = 0          # Height of warped image
im_width = 0           # Width of warped image

# Model-Based Lane Finding State (Persistent across frames)
# These now store the last known X coordinate of the *innermost* model-detected lanes
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
    """Calculates Cross-Track Error (CTE) based ONLY on finding BOTH left and right
       lane lines in the warped image by scanning pixel rows.
       Updates global left_point, right_point, and cte_f.
       cte_f is ONLY valid if both points are found.
    """
    global left_point, right_point, interested_line_y
    global im_height, im_width, cte_f # Removed lane_width, lane_width_max

    if imgWarp is None:
        left_point = -1
        right_point = -1
        cte_f = 0.0 # Cannot calculate
        return

    im_height, im_width = imgWarp.shape[:2]
    center_img = im_width // 2
    set_point = 300 # Y-level to scan from bottom up

    # Reset points for this frame's calculation
    current_frame_left = -1
    current_frame_right = -1
    diff_sum = 0.0
    scan_count = 0

    # Scan lines from bottom up to set_point
    for y in range(im_height - 1, set_point, -5): # Step upwards
        interested_line_y = y
        line_pixels = imgWarp[y, :]
        temp_left = -1
        temp_right = -1

        # Find left point near center
        for x in range(center_img, 0, -1):
            if line_pixels[x] > 0:
                temp_left = x
                break
        # Find right point near center
        for x in range(center_img + 1, im_width):
            if line_pixels[x] > 0:
                temp_right = x
                break

        # --- CRITICAL CHANGE: Only calculate diff if BOTH are found on THIS line ---
        if temp_left != -1 and temp_right != -1:
            mid_point = (temp_right + temp_left) / 2.0
            diff = center_img - mid_point
            diff_sum += diff
            scan_count += 1
            # Update the overall left/right points based on the lowest found valid pair
            # (or could average, but lowest is often more stable for control)
            current_frame_left = temp_left
            current_frame_right = temp_right
            # Visualization for this line (optional)
            cv2.circle(imgWarp, (temp_left, y), 2, (180, 180, 180), -1)
            cv2.circle(imgWarp, (temp_right, y), 2, (180, 180, 180), -1)

    # Update global state based on whether a valid pair was found *anywhere* in the scan
    left_point = current_frame_left
    right_point = current_frame_right

    # Calculate final CTE *only* if both points were found reliably
    if scan_count > 0 and left_point != -1 and right_point != -1:
        cte_f = diff_sum / scan_count # Average difference over valid scans
    else:
        cte_f = 0.0 # Indicate traditional method did not find both lanes reliably

    # Draw target line and center line on warp image for debug
    cv2.line(imgWarp, (0, set_point), (im_width, set_point), (255, 0, 0), 1)
    cv2.line(imgWarp, (center_img, set_point), (center_img, im_height), (0, 255, 255), 1)


def signal_motor(key):
    """ Updates the global 'speed_motor_requested' based on key input. """
    global speed_motor_requested, flag, final_cte_for_pid # Changed cte_f to final_cte_for_pid
    new_speed_req = speed_motor_requested # Start with current speed

    # Manual Speed Control
    if key == ord('w'): # Increase speed
        new_speed_req = min(speed_motor_requested + MANUAL_ACCELERATION, MANUAL_MAX_SPEED)
        flag = 1
    elif key == ord('s'): # Decrease speed
        new_speed_req = max(speed_motor_requested - MANUAL_ACCELERATION, 0) # Speed cannot be negative
    # Auto Speed Control ('a' key)
    elif key == ord('a'): # Set speed based on current curvature (estimated by FINAL CTE)
        # Simple logic: slower on curves (high |final_cte_for_pid|), faster on straights (low |final_cte_for_pid|)
        # Adjust threshold as needed, this is just an example
        CURVE_CTE_THRESHOLD = 25 # Example threshold for CTE magnitude indicating a curve
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
        # Prevent unrealistic derivative with tiny or zero delta_time
        if delta_time <= 1e-6:
            delta_time = 1e-6 # Use a small minimum timestep
        self.last_time = current_time

        # Proportional term
        p_term = self.kp * error_f

        # Integral term (consider adding integral limits/anti-windup for robustness)
        self.integral += error_f * delta_time
        i_term = self.ki * self.integral

        # Derivative term
        derivative = (error_f - self.error_previous) / delta_time
        d_term = self.kd * derivative

        # Store error for next iteration
        self.error_previous = error_f

        # Calculate total PID output (this is the steering deviation)
        pid_output = p_term + i_term + d_term

        # Convert PID output to servo angle (90 is center)
        # Add limits to prevent servo damage or over-steering
        steering_angle = 90.0 + pid_output # Apply deviation to center angle
        # Clamp the servo angle to a safe operating range (e.g., 70-110)
        steering_servo_angle = round(max(70, min(110, steering_angle))) # TUNED RANGE

        return steering_servo_angle


# --- NCNN Detection Function (Unchanged from previous provided version)----
def detect_signs_and_get_results(input_frame_bgr):
    """ Performs NCNN object detection. Returns list of detected object dictionaries. """
    global net, NCNN_INPUT_SIZE, NCNN_INPUT_NAME, NCNN_OUTPUT_NAME, NCNN_MEAN_VALS, NCNN_NORM_VALS
    global NCNN_NUM_CLASSES, NCNN_CLASS_NAMES, NCNN_CONFIDENCE_THRESHOLD, NCNN_NMS_THRESHOLD

    detections_results = []
    # Basic checks for valid input
    if input_frame_bgr is None or net is None: return detections_results
    original_height, original_width = input_frame_bgr.shape[:2]
    if original_height == 0 or original_width == 0: return detections_results

    try:
        # Preprocessing: Resize with padding and normalize
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
             # Fallback if resize fails - unlikely but handles edge case
             padded_img = cv2.resize(input_frame_bgr, (target_size, target_size))
             print("Warning: Resize resulted in zero dimension. Using direct resize.")
             dw, dh, scale = 0, 0, target_size / max(original_width, original_height) # Re-estimate scale approx

        mat_in = ncnn.Mat.from_pixels(padded_img, ncnn.Mat.PixelType.PIXEL_BGR, target_size, target_size)
        mat_in.substract_mean_normalize(NCNN_MEAN_VALS, NCNN_NORM_VALS)

    except Exception as e:
        print(f"NCNN Preprocessing Error: {e}")
        return detections_results

    # --- NCNN Inference ---
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

    # --- Post-processing YOLO Output ---
    output_data = np.array(mat_out)
    expected_output_dim = 4 + NCNN_NUM_CLASSES # cx, cy, w, h + classes

    # Handle potential shapes: (1, 8400, 25) or (25, 8400)
    if len(output_data.shape) == 3 and output_data.shape[0] == 1:
        output_data = output_data[0]
    elif len(output_data.shape) == 2 and output_data.shape[0] == expected_output_dim:
        output_data = output_data.T

    if len(output_data.shape) != 2 or output_data.shape[1] != expected_output_dim:
        print(f"Unexpected NCNN output shape after processing: {output_data.shape}. Expected (N, {expected_output_dim})")
        return detections_results

    num_detections = output_data.shape[0]
    boxes = []
    confidences = []
    class_ids = []

    # Decode detections
    for i in range(num_detections):
        detection = output_data[i]
        cx, cy, w_ncnn, h_ncnn = detection[:4]
        class_scores = detection[4:]
        class_id = np.argmax(class_scores)
        max_class_score = np.max(class_scores)
        final_confidence = max_class_score

        if final_confidence >= NCNN_CONFIDENCE_THRESHOLD:
            # Scale box back to ORIGINAL image coordinates
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
            else: # Avoid division by zero/small scale
                 x1_orig, y1_orig, x2_orig, y2_orig = 0, 0, 0, 0

            # Ensure coordinates are within original image bounds
            x1_orig = max(0, x1_orig)
            y1_orig = max(0, y1_orig)
            # Calculate width/height *after* scaling back, then clamp bottom-right
            w_orig = x2_orig - x1_orig
            h_orig = y2_orig - y1_orig
            w_orig = max(0, min(w_orig, original_width - x1_orig - 1))
            h_orig = max(0, min(h_orig, original_height - y1_orig - 1))

            boxes.append([int(x1_orig), int(y1_orig), int(w_orig), int(h_orig)])
            confidences.append(float(final_confidence))
            class_ids.append(class_id)

    # Apply Non-Maximum Suppression (NMS)
    indices = []
    if boxes:
        indices = cv2.dnn.NMSBoxes(boxes, confidences, NCNN_CONFIDENCE_THRESHOLD, NCNN_NMS_THRESHOLD)

    # Process final detections after NMS
    if len(indices) > 0:
        # Handle case where indices might be nested list/tuple
        if isinstance(indices, (list, tuple)) and len(indices) > 0 and isinstance(indices[0], (list, np.ndarray)):
             indices = indices.flatten() # Flatten if it's like [[0], [2]]

        processed_indices = set()
        for idx in indices:
            i = int(idx) # Ensure index is integer
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
                    "box": box, # [x, y, w, h] in original coordinates
                    "width": w,
                    "height": h
                })
    return detections_results


# --- Speed Limit Parsing Function (Unchanged) ---
def parse_speed_limit(class_name):
    """ Extracts the numerical speed value from 'Speed Limit -VALUE-' class names. """
    match = re.search(r'Speed Limit -(\d+)-', class_name)
    if match:
        try:
            return int(match.group(1))
        except ValueError:
            return None # Handle potential conversion error
    return None # Return None if pattern doesn't match

# --- Speed Transformation Function (Unchanged) ---
def transform_speed(velocity):
    """ Transforms desired velocity (km/h) to motor RPM. """
    # Adjust formula based on your specific motor, gearbox, and wheel size
    # Example: velocity (km/h) -> m/s -> wheel rad/s -> motor rad/s -> motor RPM
    # velocity_mps = velocity * (1000 / 3600)
    # wheel_circumference_m = 3.14159 * (0.0475 * 2) # Example wheel diameter 9.5cm
    # wheel_rpm = (velocity_mps / wheel_circumference_m) * 60
    # gearbox_ratio = 30 * 2.85 # Example combined ratio
    # motor_rpm = wheel_rpm * gearbox_ratio
    # This simplified formula seems to incorporate the constants directly:
    if velocity < 0: velocity = 0 # Ensure non-negative
    f_velocity = velocity / 10.0 # Original scaling factor? Keep if it worked.
    # Original formula:
    motor_rpm = (f_velocity * 30 * 2.85) / (3.14 * 0.0475 * 3.6)
    # Make sure this formula correctly reflects your setup. Check units.
    return int(round(motor_rpm))


# ==============================================================================
# ========================== MAIN EXECUTION BLOCK ============================
# ==============================================================================
if __name__ == "__main__":

    # --- Initializations ---
    # Serial Port
    print(f"Initializing Serial on {SERIAL_PORT}...")
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) # Assign to global 'ser'
        time.sleep(2) # Allow time for Arduino/ESP32 to reset
        print(f"Serial initialized on {SERIAL_PORT} at {BAUD_RATE} baud.")
    except serial.SerialException as e:
        print(f"CRITICAL: Serial Error: {e}. Could not open port {SERIAL_PORT}.")
        ser = None # Ensure ser is None if failed
    except Exception as e:
        print(f"CRITICAL: An unexpected error occurred during serial initialization: {e}")
        ser = None

    # NCNN Model
    print("Initializing NCNN Net...")
    net = ncnn.Net() # Assign to global 'net'
    print("Checking for Vulkan GPU...")
    gpu_count = 0
    try:
        gpu_count = ncnn.get_gpu_count()
    except Exception as e:
        print(f"Warning: Could not query NCNN GPU count - {e}. Assuming CPU only.")

    # Configure NCNN compute options
    if gpu_count > 0:
        print(f"NCNN: Vulkan GPU detected ({gpu_count} device(s)).")
        net.opt.use_vulkan_compute = False # <<< SET TO True TO TRY GPU
        if net.opt.use_vulkan_compute: print("NCNN: Vulkan acceleration enabled.")
    else:
        print("NCNN: Vulkan GPU not found or check failed. Using CPU threads.")
        net.opt.use_vulkan_compute = False
    # Set CPU threads if not using GPU
    if not net.opt.use_vulkan_compute:
        net.opt.num_threads = 4 # Adjust based on your Pi 5's cores
        print(f"NCNN: Using {net.opt.num_threads} CPU threads.")

    # Load NCNN model files
    print("Loading NCNN model parameters and weights...")
    try:
        start_load_time = time.time()
        if net.load_param(PARAM_PATH) != 0:
            print(f"CRITICAL: Load NCNN Param Error: {PARAM_PATH}"); exit()
        if net.load_model(BIN_PATH) != 0:
            print(f"CRITICAL: Load NCNN Model Error: {BIN_PATH}"); exit()
        end_load_time = time.time()
        print(f"NCNN model loaded successfully in {end_load_time - start_load_time:.4f} seconds.")
    except Exception as e:
        print(f"CRITICAL: Exception during NCNN model loading: {e}"); exit()

    # PiCamera2 Setup
    print("Initializing Picamera2...")
    piCam = Picamera2() # Assign to global 'piCam'
    try:
        preview_config = piCam.create_preview_configuration(main={"size": PICAM_SIZE, "format": "RGB888"})
        piCam.configure(preview_config)
        piCam.set_controls({"FrameRate": float(PICAM_FRAMERATE)})
        piCam.start()
        time.sleep(1.0) # Allow camera to stabilize
        print("Picamera2 started.")
    except Exception as e:
        print(f"CRITICAL: Picamera2 Error: {e}")
        if piCam.started: piCam.stop()
        exit()

    # PID Controllers Initialization
    pid_controller1 = PID(0.35, 0.001, 0.5) # PID for higher speeds (e.g., >= 20) - TUNE THESE
    pid_controller2 = PID(1.0, 0.001, 0.1)  # PID for medium speeds (e.g., > 0 and < 20) - TUNE THESE (Increased P, Added small D)
    pid_controller3 = PID(1.0, 0.001, 0.1)  # PID potentially for very low speed/stop - TUNE THESE

    # Initialize persistent state variables before the loop
    last_known_innermost_left_x = -1.0 # <<< MODEL STATE
    last_known_innermost_right_x = -1.0 # <<< MODEL STATE
    current_speed_limit = DEFAULT_MAX_SPEED
    speed_motor_requested = 0
    steering = 90
    cte_source = "Initializing"
    final_cte_for_pid = 0.0 # Initialize the final CTE value used by PID

    # Initialize performance counters
    frame_counter = 0
    loop_start_time = time.time()
    overall_fps = 0.0

    # --- Main Control Loop ---
    try:
        frame_width = PICAM_SIZE[0]
        frame_height = PICAM_SIZE[1]
        frame_center_x = frame_width // 2
        # Calculate target offsets based on frame width
        target_offset_pixels = frame_width * TARGET_LANE_OFFSET_FRACTION
        target_left_x_offset = frame_center_x - target_offset_pixels
        target_right_x_offset = frame_center_x + target_offset_pixels

        while True:
            frame_start_time_this = time.time()

            # --- Reset variables for CURRENT frame ---
            # Model detection results for this frame
            model_current_innermost_left_x = -1.0
            model_current_innermost_right_x = -1.0
            model_lane_detected_this_frame = False
            active_detections_this_frame = [] # Objects passing filters for drawing/logic

            # 1. Capture and Preprocess Image
            if not process_image():
                print("Warning: Frame capture/processing failed. Skipping loop iteration.")
                time.sleep(0.01)
                continue

            # 2. Traditional Lane Finding (using warped image)
            # Updates global cte_f, left_point, right_point. cte_f only valid if both found.
            SteeringAngle()
            # Determine if traditional method was successful in finding *both* lines
            traditional_found_both = (left_point != -1 and right_point != -1)

            # 3. Model Detection (using original BGR frame)
            detected_objects = []
            if frame is not None:
                detected_objects = detect_signs_and_get_results(frame)
            else:
                print("Warning: Original frame is None, skipping detection.")

            # 4. Handle Key Presses
            key = cv2.waitKey(1) & 0xFF
            signal_motor(key) # Updates global speed_motor_requested based on key and final_cte_for_pid
            if key == ord('q'):
                print("'q' pressed, exiting loop.")
                break

            # 5. Process Detections (Apply Filters, Identify Lanes, Signs)
            stop_condition_met = False
            limit_sign_seen_this_frame = False
            new_limit_value = -1 # Reset for this frame

            # Temporary lists to hold model's lane segments *for this frame*
            current_frame_left_segments = []
            current_frame_right_segments = []

            if detected_objects:
                for detection in detected_objects:
                    obj_name = detection['name']
                    box = detection['box']
                    x, y, w, h = box
                    obj_w = detection.get('width', 0)
                    obj_h = detection.get('height', 0)

                    process_this_detection = False # Flag to check if detection should be processed

                    # Apply filter logic: Lines always processed, others need size check
                    if obj_name in ["dotted-line", "line"]:
                        process_this_detection = True # Process lines regardless of size
                    else:
                        # Apply size filter ONLY to non-line objects
                        if obj_w > MIN_SIGN_WIDTH_OR_HEIGHT or obj_h > MIN_SIGN_WIDTH_OR_HEIGHT:
                            process_this_detection = True

                    # --- Process if flagged ---
                    if process_this_detection:
                        active_detections_this_frame.append(detection) # Add for drawing later

                        # A. Lane Lines ("dotted-line", "line") -> Categorize for *this frame*
                        if obj_name in ["dotted-line", "line"]:
                            object_center_x = x + w / 2.0
                            segment_info = {'box': box, 'center_x': object_center_x, 'name': obj_name}
                            # Classify as left/right based on position relative to frame center
                            if object_center_x < frame_center_x:
                                current_frame_left_segments.append(segment_info)
                            elif object_center_x > frame_center_x:
                                current_frame_right_segments.append(segment_info)
                            model_lane_detected_this_frame = True # Mark that model found at least one lane

                        # B. Stop Signs / Red Lights
                        elif obj_name in ["Stop Sign", "Traffic Light -Red-"]:
                            stop_condition_met = True

                        # C. Speed Limits (Parse only non-line, non-stop objects that passed size filter)
                        else:
                            parsed_limit = parse_speed_limit(obj_name)
                            if parsed_limit is not None:
                                limit_sign_seen_this_frame = True
                                # Track the lowest speed limit seen in the frame
                                if new_limit_value == -1 or parsed_limit < new_limit_value:
                                    new_limit_value = parsed_limit

                        # D. Could add logic for 'Pessoa', 'car' here (e.g., obstacle avoidance)

            # 6. Update Persistent Model Lane Positions & Handle Reset
            # Find innermost left line segment detected THIS frame by the model
            if current_frame_left_segments:
                innermost_left_segment = max(current_frame_left_segments, key=lambda item: item['center_x'])
                model_current_innermost_left_x = innermost_left_segment['center_x']
                last_known_innermost_left_x = model_current_innermost_left_x # PERSIST this value

            # Find innermost right line segment detected THIS frame by the model
            if current_frame_right_segments:
                innermost_right_segment = min(current_frame_right_segments, key=lambda item: item['center_x'])
                model_current_innermost_right_x = innermost_right_segment['center_x']
                last_known_innermost_right_x = model_current_innermost_right_x # PERSIST this value

            # --- Reset persistent state ONLY if NO model lanes were detected this frame ---
            if not model_lane_detected_this_frame:
                last_known_innermost_left_x = -1.0
                last_known_innermost_right_x = -1.0
                # Optional: Add a small delay/counter before resetting if needed

            # 7. Determine Final Steering CTE using NEW Hierarchy Logic
            final_cte_for_pid = 0.0 # Reset for this frame's calculation
            cte_source = "Default (0.0)" # Default source

            # Priority 1: Traditional method found both lanes reliably
            if traditional_found_both:
                final_cte_for_pid = cte_f # Use CTE from warped image edge scan
                cte_source = "Traditional (Both)"
            else:
                # Traditional failed or found only one lane, use Model-based logic with persistence
                # Check last known states (which were updated in step 6)
                know_left = last_known_innermost_left_x > 0
                know_right = last_known_innermost_right_x > 0

                # Priority 2: Model knows both lanes
                if know_left and know_right:
                    detected_lane_center = (last_known_innermost_left_x + last_known_innermost_right_x) / 2.0
                    final_cte_for_pid = float(frame_center_x) - detected_lane_center
                    cte_source = "Model (Both Known)"

                # Priority 3: Model knows only Left lane -> Steer to offset
                elif know_left:
                    # Error = Target Position - Actual Position
                    final_cte_for_pid = target_left_x_offset - last_known_innermost_left_x
                    cte_source = "Model (Offset L)"

                # Priority 4: Model knows only Right lane -> Steer to offset
                elif know_right:
                    # Error = Target Position - Actual Position
                    final_cte_for_pid = target_right_x_offset - last_known_innermost_right_x
                    cte_source = "Model (Offset R)"

                # Priority 5: No lanes known by any reliable method (already defaulted above)

            # 8. Update Active Speed Limit
            if limit_sign_seen_this_frame and new_limit_value != -1:
                if new_limit_value != current_speed_limit:
                    print(f"** Speed Limit Updated: {new_limit_value} km/h **")
                    current_speed_limit = new_limit_value
            # Optional: Add logic to reset speed limit if no sign seen for X seconds/frames

            # 9. Determine Final Commanded Speed (km/h and RPM)
            final_speed_command_vel = speed_motor_requested # Start with requested
            final_speed_command_vel = min(final_speed_command_vel, current_speed_limit) # Apply limit
            if stop_condition_met:
                if final_speed_command_vel > 0:
                    print("** Stop condition met! Setting speed to 0. **")
                final_speed_command_vel = 0 # Force stop
            final_speed_command_rpm = transform_speed(final_speed_command_vel) # Convert to RPM

            # 10. PID Calculation for Steering Angle
            current_pid = pid_controller3 # Default PID
            if final_speed_command_vel >= 20 :
                current_pid = pid_controller1 # High speed PID
            elif final_speed_command_vel > 0:
                current_pid = pid_controller2 # Medium speed PID

            # Calculate steering command using the selected PID and the FINAL calculated CTE
            steering = current_pid.update(final_cte_for_pid)

            # 11. Send Commands to ESP32/Arduino via Serial
            if ser and ser.is_open:
                try:
                    # Pack steering angle (int) and RPM (int) as two 4-byte integers ('<ii')
                    data_to_send = struct.pack('<ii', steering, final_speed_command_rpm)
                    ser.write(b'<' + data_to_send + b'>') # Use markers
                    ser.flush()
                except serial.SerialException as e:
                    print(f"Serial Write Error: {e}. Disabling serial.")
                    ser.close()
                    ser = None
                except Exception as e:
                    print(f"Unexpected Error during serial send: {e}")
                    if ser: ser.close()
                    ser = None

            # 12. Calculate and Print FPS / Status
            frame_counter += 1
            frame_time_this = time.time() - frame_start_time_this
            instant_fps = 1.0 / frame_time_this if frame_time_this > 0 else 0
            elapsed_time_total = time.time() - loop_start_time
            if elapsed_time_total > 1:
                overall_fps = frame_counter / elapsed_time_total

            if frame_counter % 30 == 0: # Print status periodically
                 print(f"Status: Limit={current_speed_limit} | Req={speed_motor_requested} vel | Sent={final_speed_command_rpm} rpm | Steer={steering} | "
                       f"CTE={final_cte_for_pid:.2f} ({cte_source}) | TradOK={traditional_found_both} | "
                       f"L={last_known_innermost_left_x:.1f} | R={last_known_innermost_right_x:.1f} | "
                       f"FPS: {instant_fps:.1f} (Avg: {overall_fps:.1f})")


            # 13. Display Visualization Windows
            # --- Draw on Original Frame ---
            display_frame = frame.copy() if frame is not None else np.zeros((frame_height, frame_width, 3), dtype=np.uint8)

            # Draw CTE source and value
            cv2.putText(display_frame, f"CTE Source: {cte_source}", (10, frame_height - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            cv2.putText(display_frame, f"Final CTE: {final_cte_for_pid:.2f}", (10, frame_height - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

            # Draw frame center line
            cv2.line(display_frame, (frame_center_x, 0), (frame_center_x, frame_height), (200, 200, 200), 1)

            # Draw Target Offset Lines (only visible when model uses offset steering)
            cv2.line(display_frame, (int(target_left_x_offset), 0), (int(target_left_x_offset), frame_height), (0, 255, 0), 1) # Green Target Left
            cv2.line(display_frame, (int(target_right_x_offset), 0), (int(target_right_x_offset), frame_height), (0, 255, 0), 1) # Green Target Right

            # Draw all active detections that passed filters
            for detection in active_detections_this_frame:
                d_x, d_y, d_w, d_h = detection['box']
                d_name = detection['name']
                d_conf = detection['confidence']
                label = f"{d_name}: {d_conf:.2f}"
                color = (0, 180, 0) # Default Greenish
                thickness = 1

                # Customize drawing for Lane Lines
                if d_name in ["dotted-line", "line"]:
                    obj_center_x = d_x + d_w / 2.0
                    is_tracked_left = know_left and abs(obj_center_x - last_known_innermost_left_x) < 5 # Check if it matches the tracked left
                    is_tracked_right = know_right and abs(obj_center_x - last_known_innermost_right_x) < 5 # Check if it matches the tracked right

                    if obj_center_x < frame_center_x:
                        label += " (L?)"
                        color = (255, 165, 0) # Orange
                        if is_tracked_left:
                            thickness = 3 # Highlight tracked left
                            label = f"{d_name}: {d_conf:.2f} (TRACKED L)"
                    elif obj_center_x > frame_center_x:
                        label += " (R?)"
                        color = (255, 0, 255) # Magenta
                        if is_tracked_right:
                            thickness = 3 # Highlight tracked right
                            label = f"{d_name}: {d_conf:.2f} (TRACKED R)"

                # Highlight stop conditions
                elif d_name in ["Stop Sign", "Traffic Light -Red-"]:
                    color = (0, 0, 255) # Red
                    thickness = 2

                # Draw the rectangle and label
                cv2.rectangle(display_frame, (d_x, d_y), (d_x + d_w, d_y + d_h), color, thickness)
                cv2.putText(display_frame, label, (d_x, d_y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, thickness)

            # Draw markers for the LAST KNOWN model line positions at bottom (if known)
            if last_known_innermost_left_x > 0:
                cv2.circle(display_frame, (int(last_known_innermost_left_x), frame_height - 20), 7, (255, 165, 0), -1) # Orange Circle
                cv2.putText(display_frame, "Lk", (int(last_known_innermost_left_x) - 10, frame_height - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255,255,255), 1)
            if last_known_innermost_right_x > 0:
                cv2.circle(display_frame, (int(last_known_innermost_right_x), frame_height - 20), 7, (255, 0, 255), -1) # Magenta Circle
                cv2.putText(display_frame, "Rk", (int(last_known_innermost_right_x) - 10, frame_height - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255,255,255), 1)

            # Show the frame with detections
            cv2.imshow("Object Detections & Lane Tracking", display_frame)

            # --- Display Warped Lane Image (Traditional Method Debug) ---
            if imgWarp is not None:
                display_img_warp = ve_truc_anh(imgWarp, step=50) # Add grid lines
                if display_img_warp is not None:
                    # Add relevant text overlays for debugging traditional method
                    cv2.putText(display_img_warp, f"Trad Found Both: {traditional_found_both}", (10, im_height - 40 if im_height>40 else 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 1)
                    cv2.putText(display_img_warp, f"Trad CTE (cte_f): {cte_f:.2f} {'(VALID)' if traditional_found_both else '(INVALID)'}", (10, im_height - 20 if im_height>20 else 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 1)

                    # Draw traditional left/right points IF FOUND (at their detected y-coord)
                    if left_point != -1: cv2.circle(display_img_warp, (left_point, interested_line_y), 5, (0,0,255), -1) # Red circle where found
                    if right_point != -1: cv2.circle(display_img_warp, (right_point, interested_line_y), 5, (0,0,255), -1) # Red circle where found

                    # Show the warped image window
                    cv2.imshow("Traditional Lane Warp (Debug)", display_img_warp)
            # --- End of Loop ---

    # --- Error Handling & Cleanup ---
    except KeyboardInterrupt:
        print("\nCtrl+C detected. Exiting...")
    except Exception as e:
        print(f"\n--- UNHANDLED EXCEPTION IN MAIN LOOP ---")
        print(f"Error Type: {type(e).__name__}")
        print(f"Error Details: {e}")
        import traceback
        traceback.print_exc()
        print(f"--- END OF TRACEBACK ---")
    finally:
        # This block executes regardless of how the loop exits
        print("\nCleaning up resources...")

        # Stop the motor gracefully
        if ser and ser.is_open:
            try:
                print("Sending stop command to motor...")
                stop_data = struct.pack('<ii', 90, 0) # Center steer, zero RPM
                ser.write(b'<' + stop_data + b'>')
                ser.flush()
                time.sleep(0.1)
                ser.close()
                print("Serial port closed.")
            except Exception as e:
                print(f"Error sending stop command or closing serial port: {e}")

        # Stop the camera
        if piCam and piCam.started:
             try:
                 piCam.stop()
                 print("Picamera2 stopped.")
             except Exception as e:
                 print(f"Error stopping Picamera2: {e}")

        # Close OpenCV display windows
        cv2.destroyAllWindows()
        print("OpenCV windows closed.")
        print("Cleanup complete. Exiting program.")
