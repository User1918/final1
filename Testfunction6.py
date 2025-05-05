# -*- coding: utf-8 -*-
import cv2
import numpy as np
from picamera2 import Picamera2
import time
import serial
import struct
import math # Added from script 2
import ncnn  # Import the ncnn Python binding
import re    # Import regular expressions for parsing speed limits

# --- Configuration ---
# NCNN Object Detection Config (From Script 1/2)
PARAM_PATH = "/home/labthayphuc/Desktop/yolov11ncnn/yolov11n_int8_final.param" # <<< UPDATE PATH IF NEEDED
BIN_PATH = "/home/labthayphuc/Desktop/yolov11ncnn/yolov11n_int8_final.bin"   # <<< UPDATE PATH IF NEEDED
NCNN_INPUT_SIZE = 640
NCNN_CONFIDENCE_THRESHOLD = 0.1 # Using lower threshold from Script 2
NCNN_NMS_THRESHOLD = 0.3     # Using threshold from Script 2
NCNN_NUM_CLASSES = 21          # <<< UPDATED based on list in script 2
NCNN_CLASS_NAMES = [           # <<< UPDATED based on list in script 2
    "Pedestrian Crossing", "Radar", "Speed Limit -100-", "Speed Limit -120-",
    "Speed Limit -20-", "Speed Limit -30-", "Speed Limit -40-", "Speed Limit -50-",
    "Speed Limit -60-", "Speed Limit -70-", "Speed Limit -80-", "Speed Limit -90-",
    "Stop Sign", "Traffic Light -Green-", "Traffic Light -Off-",
    "Traffic Light -Red-", "Traffic Light -Yellow-", "Person", "Car","dot_line","line"
]
NCNN_INPUT_NAME = "in0"      # Default input layer name (Consistent)
NCNN_OUTPUT_NAME = "out0"     # Default output layer name (Consistent)
NCNN_MEAN_VALS = [0.0, 0.0, 0.0] # Mean values for normalization (Consistent)
NCNN_NORM_VALS = [1/255.0, 1/255.0, 1/255.0] # Normalization scale factors (Consistent)
MIN_SIGN_SIZE = 100 # px tối thiểu để xét biển (Renamed from MIN_SIGN_WIDTH_OR_HEIGHT, using Script 2 value/name)

# Lane Keeping & Motor Control Config (From Script 1, SERIAL/BAUD from Script 2)
SERIAL_PORT = '/dev/ttyUSB0'   # <<< ADJUST Serial Port (From Script 2)
BAUD_RATE = 115200             # <<< MATCH ESP32 Baud Rate (From Script 2)
PICAM_SIZE = (640, 480)        # Camera resolution (Consistent)
PICAM_FRAMERATE = 30           # Camera framerate (Consistent)
TARGET_LANE_OFFSET_FRACTION = 0.25 # RE-ADDED: Keep single model lane 1/4 of frame width from center

# Perspective Warp Points (From Script 2)
leftTop, heightTop, rightTop     = 160, 150, 160
leftBottom, heightBottom, rightBottom = 20, 380, 20

# Speed Control Config (From Script 1, adjusted names to match script 2 where applicable)
DEFAULT_MAX_SPEED = 120       # km/h
MANUAL_MAX_SPEED = 100        # km/h khi điều khiển tay
MANUAL_ACCELERATION = 5       # km/h mỗi nhấn phím
AUTO_MODE_SPEED_STRAIGHT = 30 # km/h khi đường thẳng
AUTO_MODE_SPEED_CURVE = 20    # km/h khi vào cua

# --- Global Variables (State and Shared Data) ---
# Image Data
frame = None             # Current BGR frame from camera
imgWarp = None           # Current warped binary image for traditional lane finding

# Hardware Interfaces (initialized later)
piCam = None             # Picamera2 object
net = None               # NCNN Net object
ser = None               # Serial object

# Vehicle State / Commands
steering = 90            # Current steering servo angle command (0-180, 90=center)
speed_motor_requested = 0 # Target speed requested by manual keys or auto mode (km/h)
current_speed_limit = DEFAULT_MAX_SPEED # Current active speed limit

# Traditional Lane Finding State (modified by NEW SteeringAngle from Script 2)
cte_f = 0.0              # Cross-Track Error calculated from warped image (now a scaled angle)
left_point = -1          # X-coordinate of detected left lane edge (last row found in scan)
right_point = -1         # X-coordinate of detected right lane edge (last row found in scan)
interested_line_y = 0    # Last Y-coordinate scanned in SteeringAngle
im_height = 0            # Height of warped image (Set inside SteeringAngle)
im_width = 0             # Width of warped image (Set inside SteeringAngle)

# Variables from Script 2's SteeringAngle
pre_diff = 0.0
lane_width = lane_width_max = 500 # Initial estimate, max used for fallback
prev_cte_ema = 0.0         # Giá trị EMA CTE của khung trước
alpha_cte = 0.5            # Hệ số làm mịn EMA (0 < alpha_cte <= 1)

# Model-Based Lane Finding State (RE-ADDED for Fallback Logic)
last_known_innermost_left_x = -1.0 # Last known X-coord of left line from model
last_known_innermost_right_x = -1.0# Last known X-coord of right line from model
cte_source = "Initializing" # Tracks which method determined the current CTE

# Performance Monitoring State
frame_counter = 0
loop_start_time = 0.0
overall_fps = 0.0
prev_time = 0.0 # From script 2 (used for dt in its main loop, now used for FPS)

# --- Helper Functions ---

# PROCESS_IMAGE FUNCTION FROM SCRIPT 2
def process_image():
    """- Đọc ảnh, chuyển sang BGR, Canny, warp phối cảnh, draw contours/lines"""
    global frame, imgWarp
    if piCam is None: return False

    try:
        rgb = piCam.capture_array()
    except Exception as e:
        print(f"Capture Error: {e}")
        frame = None
        imgWarp = None
        return False

    if rgb is None:
        frame = None
        imgWarp = None
        return False

    frame = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (11,11), 0)
    edges = cv2.Canny(blur, 70, 120)
    h, w = edges.shape

    pts1 = np.float32([(leftTop, heightTop), (w-rightTop, heightTop),
                       (leftBottom, heightBottom), (w-rightBottom, heightBottom)])
    pts2 = np.float32([[0,0],[w,0],[0,h],[w,h]])

    try:
        M = cv2.getPerspectiveTransform(pts1, pts2)
        imgWarp = cv2.warpPerspective(edges, M, (w,h))
        contours, hierarchy = cv2.findContours(imgWarp, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(imgWarp, contours, -1, (255, 255, 255), 3)
        lines = cv2.HoughLinesP(imgWarp, 1, np.pi/180, threshold=150, minLineLength=70, maxLineGap=0)
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(imgWarp, (x1, y1), (x2, y2), (255, 255, 255), 3)
    except cv2.error as e:
        print(f"Warp Error: {e}")
        imgWarp = None
    return True


# VE_TRUC_ANH FUNCTION FROM SCRIPT 2
def ve_truc_anh(img, step=50):
    """Vẽ trục và lưới debug"""
    if img is None: return None
    if len(img.shape) == 2:
        viz = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    else:
        viz = img.copy()

    h, w = viz.shape[:2]
    if h == 0 or w == 0: return viz

    cv2.line(viz, (0,h-1), (w,h-1), (200,200,200), 1)
    cv2.line(viz, (0,0), (0,h), (200,200,200), 1)
    for x in range(0, w, step): cv2.line(viz, (x,h-10),(x,h),(200,200,200),1)
    for y in range(0, h, step): cv2.line(viz, (0,y),(10,y),(200,200,200),1)
    return viz


# STEERINGANGLE FUNCTION FROM SCRIPT 2
def SteeringAngle():
    """Tính CTE bằng quét hàng ngang"""
    global cte_f, pre_diff, lane_width, lane_width_max, left_point, right_point, interested_line_y, prev_cte_ema, alpha_cte

    prev_cte_ema = cte_f
    pre_diff = 0
    h, w = imgWarp.shape
    left_point = right_point = -1
    center = w//2
    set_y = 401
    step = -5
    u = (h-set_y)/abs(step)
    ki = (-u/h) + 1
    for i in range(h-1, set_y, step):
        interested_line_y = i
        row = imgWarp[i]
        for x in range(center, -1, -1):
            if row[x]>0: left_point=x; break
        for x in range(center+1, w):
            if row[x]>0: right_point=x; break
        if left_point!=-1 and right_point!=-1: lane_width = right_point-left_point
        if left_point!=-1 and right_point==-1: right_point=left_point+lane_width_max
        if right_point!=-1 and left_point==-1: left_point=right_point-lane_width_max
        mid = (left_point+right_point)/2
        diff = center-mid
        diff = diff*ki + pre_diff
        pre_diff = diff        
    raw_cte = pre_diff 
    # Lọc EMA qua các khung để mượt CTE
    cte_filter = alpha_cte * raw_cte + (1.0 - alpha_cte) * prev_cte_ema
    cte_rad = math.atan((cte_filter-center) / (set_y))
    cte_deg = math.degrees(cte_rad)
    cte_f = round(cte_deg*0.45)

# Signal Motor function - USING SCRIPT 1 VERSION
# Adapting 'a' mode check for angle CTE
def signal_motor(key):
    """ Updates the global 'speed_motor_requested' based on key input. """
    global speed_motor_requested, final_cte_for_pid # Use final_cte_for_pid

    new_speed_req = speed_motor_requested

    if key == ord('w'):
        new_speed_req = min(speed_motor_requested + MANUAL_ACCELERATION, MANUAL_MAX_SPEED)
    elif key == ord('s'):
        new_speed_req = max(speed_motor_requested - MANUAL_ACCELERATION, 0)
    elif key == ord('a'):
        # Use final_cte_for_pid which is always a scaled angle
        CURVE_CTE_THRESHOLD_ANGLE = 10 # Threshold for scaled angle - TUNE THIS
        target_speed = AUTO_MODE_SPEED_STRAIGHT if abs(final_cte_for_pid) < CURVE_CTE_THRESHOLD_ANGLE else AUTO_MODE_SPEED_CURVE
        new_speed_req = target_speed
    elif key == ord('x'):
        new_speed_req = 0

    if new_speed_req != speed_motor_requested:
        speed_motor_requested = new_speed_req


# --- PID Class (USING SCRIPT 1 VERSION) ---
class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp; self.ki = ki; self.kd = kd
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_derivative = 0.0
        self.integrator_min = -50.0
        self.integrator_max = 50.0
        self.alpha = 0.1  # lọc IIR cho D

    def update(self, error, dt):
        """Tính P, I, D và trả về (output, góc servo)"""
        if dt <= 0: dt = 1e-3
        
        # tính P
        p = self.kp * error
        # tính I
        self.integral += error * dt
        self.integral = max(self.integrator_min, min(self.integral, self.integrator_max))
        i = self.ki * self.integral
        # tính D
        derivative = (error - self.prev_error) / dt
        derivative_filtered = self.alpha * derivative + (1 - self.alpha) * self.prev_derivative #làm mượt bằng EMA
        d = self.kd * derivative_filtered
        # cập nhật trạng thái
        self.prev_error = error
        self.prev_derivative = derivative
        # tổng output và góc servo (90 trung tâm)
        output = round(p + i + d)
        servo_angle = 90 + output
        
        return output ,servo_angle

# --- NCNN Detection Function (Unchanged from previous merged version)----
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
        print(f"Unexpected NCNN output shape after reshape: {output_data.shape}. Expected (N, {expected_output_dim})")
        return detections_results

    num_detections = output_data.shape[0]
    boxes = []; confidences = []; class_ids = []

    for i in range(num_detections):
        detection = output_data[i]
        cx, cy, w_ncnn, h_ncnn = detection[:4]
        class_scores = detection[4:]
        class_id = np.argmax(class_scores)
        final_confidence = np.max(class_scores) # Use max score

        if final_confidence >= NCNN_CONFIDENCE_THRESHOLD:
            x1_resized = (cx - w_ncnn / 2); y1_resized = (cy - h_ncnn / 2)
            x2_resized = (cx + w_ncnn / 2); y2_resized = (cy + h_ncnn / 2)
            x1_nopad = x1_resized - dw; y1_nopad = y1_resized - dh
            x2_nopad = x2_resized - dw; y2_nopad = y2_resized - dh
            if scale > 1e-6:
                x1_orig = x1_nopad / scale; y1_orig = y1_nopad / scale
                x2_orig = x2_nopad / scale; y2_orig = y2_nopad / scale
            else:
                 x1_orig, y1_orig, x2_orig, y2_orig = 0, 0, 0, 0
            x1_orig = max(0, x1_orig); y1_orig = max(0, y1_orig)
            w_orig = x2_orig - x1_orig; h_orig = y2_orig - y1_orig
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
                box = boxes[i]; x, y, w, h = box
                confidence_nms = confidences[i]; class_id_nms = class_ids[i]
                processed_indices.add(i)
                class_name = NCNN_CLASS_NAMES[class_id_nms] if 0 <= class_id_nms < len(NCNN_CLASS_NAMES) else f"ID:{class_id_nms}"
                detections_results.append({
                    "name": class_name, "confidence": confidence_nms, "box": box,
                    "width": w, "height": h })
    return detections_results

# --- Speed Limit Parsing Function (Unchanged) ---
def parse_speed_limit(class_name):
    match = re.search(r'Speed Limit -(\d+)-', class_name)
    return int(match.group(1)) if match else None

# --- Speed Transformation Function (Unchanged) ---
def transform_speed(velocity):
    f = velocity / 10.0
    rpm = (f * 30 * 2.85) / (math.pi * 0.0475 * 3.6)
    return int(round(rpm))

# --- Helper Function to Convert Pixel CTE to Angle CTE ---
def pixel_cte_to_angle_cte(pixel_cte, baseline_y = 401, angle_scale = 0.45):
    """Converts a pixel offset CTE to a scaled angle CTE."""
    if baseline_y <= 0: baseline_y = 1 # Avoid division by zero
    rad = math.atan(pixel_cte / baseline_y)
    deg = math.degrees(rad)
    return round(deg * angle_scale)

# ==============================================================================
# ========================== MAIN EXECUTION BLOCK ============================
# ==============================================================================
if __name__ == "__main__":

    # --- Initializations ---
    print(f"Initializing Serial on {SERIAL_PORT}...")
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(0.1)
        print(f"Serial initialized on {SERIAL_PORT} at {BAUD_RATE} baud.")
    except Exception as e:
        print(f"CRITICAL: Serial Error: {e}. Port {SERIAL_PORT}."); ser = None

    print("Initializing NCNN Net...")
    net = ncnn.Net()
    # GPU/CPU settings (unchanged)
    gpu_count = 0
    try: gpu_count = ncnn.get_gpu_count()
    except Exception as e: print(f"Warning: NCNN GPU count failed - {e}. CPU only.")
    net.opt.use_vulkan_compute = False # Keep False
    if gpu_count > 0 and net.opt.use_vulkan_compute:
        print(f"NCNN: Vulkan GPU detected. Acceleration enabled.")
    else:
        print("NCNN: Using CPU threads.")
        net.opt.num_threads = 4 # Use 4 threads
        print(f"NCNN: Using {net.opt.num_threads} CPU threads.")
    # Load model (unchanged)
    print("Loading NCNN model parameters and weights...")
    try:
        start_load_time = time.time()
        if net.load_param(PARAM_PATH) != 0: print(f"CRITICAL: Load Param Error: {PARAM_PATH}"); exit()
        if net.load_model(BIN_PATH) != 0: print(f"CRITICAL: Load Model Error: {BIN_PATH}"); exit()
        print(f"NCNN model loaded successfully in {time.time() - start_load_time:.4f} seconds.")
    except Exception as e: print(f"CRITICAL: NCNN model loading exception: {e}"); exit()

    print("Initializing Picamera2...")
    piCam = Picamera2()
    try:
        preview_config = piCam.create_preview_configuration(main={"size": PICAM_SIZE, "format": "RGB888"})
        piCam.configure(preview_config)
        piCam.set_controls({"FrameRate": float(PICAM_FRAMERATE)})
        piCam.start(); time.sleep(1.0)
        print("Picamera2 started.")
    except Exception as e:
        print(f"CRITICAL: Picamera2 Error: {e}"); exit()

    # PID Controllers Initialization (Unchanged - Gains need tuning!)
    pid_controller1 = PID(0.35, 0.008, 0.25) # High speed - TUNE
    pid_controller2 = PID(1.0, 0.001, 0.1)  # Medium speed - TUNE
    pid_controller3 = PID(1.0, 0.001, 0.1)  # Low speed/Stop - TUNE

    # Initialize persistent state variables
    last_known_innermost_left_x = -1.0 # RE-ADDED
    last_known_innermost_right_x = -1.0# RE-ADDED
    current_speed_limit = DEFAULT_MAX_SPEED
    speed_motor_requested = 0
    steering = 90
    cte_source = "Initializing"
    final_cte_for_pid = 0.0 # Will hold the final scaled angle CTE
    prev_cte_ema = 0.0      # For SteeringAngle's filter
    lane_width_max = 500    # For SteeringAngle's estimation

    # Initialize performance counters
    frame_counter = 0; loop_start_time = time.time(); prev_time = loop_start_time; overall_fps = 0.0

    # --- Main Control Loop ---
    try:
        frame_width = PICAM_SIZE[0]; frame_height = PICAM_SIZE[1]
        frame_center_x = frame_width // 2
        # Calculate target offsets for model-based fallback (RE-ADDED)
        target_offset_pixels = frame_width * TARGET_LANE_OFFSET_FRACTION
        target_left_x_offset = frame_center_x - target_offset_pixels
        target_right_x_offset = frame_center_x + target_offset_pixels

        while True:
            now = time.time(); dt = now - prev_time if now != prev_time else 1e-3; prev_time = now
            frame_start_time_this = time.time()

            # --- Reset variables for CURRENT frame ---
            active_detections_this_frame = []
            # RE-ADDED: For tracking model lanes within this frame
            model_current_innermost_left_x = -1.0
            model_current_innermost_right_x = -1.0
            model_lane_detected_this_frame = False
            current_frame_left_segments = []
            current_frame_right_segments = []


            # 1. Capture and Preprocess Image (Using Script 2's process_image)
            if not process_image(): time.sleep(0.01); continue

            # 2. Traditional Lane Finding (Using Script 2's SteeringAngle)
            # Updates global cte_f, left_point, right_point
            SteeringAngle()
            # Check if the traditional method found both points *without estimation*
            # Note: left_point/right_point hold the *last* found point during the scan
            # A more robust check might involve checking if estimation was used inside SteeringAngle
            traditional_success = (left_point != -1 and right_point != -1)

            # 3. Model Detection (Unchanged)
            detected_objects = detect_signs_and_get_results(frame) if frame is not None else []

            # 4. Handle Key Presses (Unchanged)
            key = cv2.waitKey(1) & 0xFF
            signal_motor(key)
            if key == ord('q'): break

            # 5. Process Detections (Signs AND Model Lanes - RE-ADDED Lane Logic)
            stop_condition_met = False
            limit_sign_seen_this_frame = False
            new_limit_value = -1

            if detected_objects:
                for detection in detected_objects:
                    obj_name = detection['name']
                    box = detection['box']; x, y, w, h = box
                    obj_w = detection.get('width', 0); obj_h = detection.get('height', 0)
                    process_this_detection = False

                    # Filter criteria (same as before)
                    if obj_name in ["dot_line", "line"]: process_this_detection = True
                    else:
                        if obj_w > MIN_SIGN_SIZE or obj_h > MIN_SIGN_SIZE: process_this_detection = True

                    if process_this_detection:
                        active_detections_this_frame.append(detection)

                        # RE-ADDED: Process Model Lane Detections
                        if obj_name in ["dot_line", "line"]:
                            model_lane_detected_this_frame = True
                            object_center_x = x + w / 2.0
                            segment_info = {'box': box, 'center_x': object_center_x, 'name': obj_name}
                            if object_center_x < frame_center_x:
                                current_frame_left_segments.append(segment_info)
                            elif object_center_x > frame_center_x:
                                current_frame_right_segments.append(segment_info)
                        # END RE-ADDED Lane Logic

                        # Process Signs (unchanged)
                        elif obj_name in ["Stop Sign", "Traffic Light -Red-"]:
                            stop_condition_met = True
                        else:
                            parsed_limit = parse_speed_limit(obj_name)
                            if parsed_limit is not None:
                                limit_sign_seen_this_frame = True
                                if new_limit_value == -1 or parsed_limit < new_limit_value:
                                    new_limit_value = parsed_limit

            # 6. Update Persistent Model Lane Positions (RE-ADDED)
            if current_frame_left_segments:
                innermost_left_segment = max(current_frame_left_segments, key=lambda item: item['center_x'])
                model_current_innermost_left_x = innermost_left_segment['center_x']
                last_known_innermost_left_x = model_current_innermost_left_x # Update persistent state
            if current_frame_right_segments:
                innermost_right_segment = min(current_frame_right_segments, key=lambda item: item['center_x'])
                model_current_innermost_right_x = innermost_right_segment['center_x']
                last_known_innermost_right_x = model_current_innermost_right_x # Update persistent state

            # Reset persistent state if no model lanes detected this frame
            if not model_lane_detected_this_frame:
                 # Gradually fade? Or reset instantly? Resetting for now.
                 last_known_innermost_left_x = -1.0
                 last_known_innermost_right_x = -1.0
            # END RE-ADDED

            # 7. Determine Final Steering CTE (Hybrid Hierarchy - REVISED)
            final_cte_for_pid = 0.0 # Default
            cte_source = "Default (0.0)"

            know_model_left = last_known_innermost_left_x > 0
            know_model_right = last_known_innermost_right_x > 0

            # Priority 1: Traditional method succeeded (found both points during scan)
            if traditional_success:
                final_cte_for_pid = cte_f # Use the scaled angle from SteeringAngle
                cte_source = "Traditional"
            # Priority 2: Model knows BOTH lanes
            elif know_model_left and know_model_right:
                 model_mid_point = (last_known_innermost_left_x + last_known_innermost_right_x) / 2.0
                 pixel_error = frame_center_x - model_mid_point
                 final_cte_for_pid = pixel_cte_to_angle_cte(pixel_error) # Convert pixel error to angle
                 cte_source = "Model (Center)"
            # Priority 3: Model knows only LEFT lane
            elif know_model_left:
                 pixel_error = target_left_x_offset - last_known_innermost_left_x # Error relative to target offset
                 final_cte_for_pid = pixel_cte_to_angle_cte(pixel_error) # Convert pixel error to angle
                 cte_source = "Model (Offset L)"
            # Priority 4: Model knows only RIGHT lane
            elif know_model_right:
                 pixel_error = target_right_x_offset - last_known_innermost_right_x # Error relative to target offset
                 final_cte_for_pid = pixel_cte_to_angle_cte(pixel_error) # Convert pixel error to angle
                 cte_source = "Model (Offset R)"
            # Priority 5: No reliable data (already defaulted to 0.0)

            # 8. Update Active Speed Limit (Unchanged)
            if limit_sign_seen_this_frame and new_limit_value != -1:
                if new_limit_value != current_speed_limit:
                    print(f"** Speed Limit Updated: {new_limit_value} km/h **")
                    current_speed_limit = new_limit_value

            # 9. Determine Final Commanded Speed (Unchanged)
            final_speed_command_vel = min(speed_motor_requested, current_speed_limit)
            if stop_condition_met:
                if final_speed_command_vel > 0: print("** Stop condition met! Speed = 0. **")
                final_speed_command_vel = 0
            final_speed_command_rpm = transform_speed(final_speed_command_vel)

            # 10. PID Calculation for Steering Angle (Unchanged - uses final_cte_for_pid)
            current_pid = pid_controller3
            if final_speed_command_vel >= 20 : current_pid = pid_controller1
            elif final_speed_command_vel > 0: current_pid = pid_controller2
            steering = current_pid.update(final_cte_for_pid) # Input is always scaled angle

            # 11. Send Commands to ESP32/Arduino via Serial (Unchanged)
            if ser and ser.is_open:
                try:
                    data_to_send = struct.pack('<ii', steering, final_speed_command_rpm)
                    ser.write(b'<' + data_to_send + b'>'); ser.flush()
                except Exception as e: # Catch generic Exception is okay here for serial write
                    print(f"Serial Write Error: {e}. Disabling.")
                    if ser: ser.close(); ser = None

            # 12. Calculate and Print FPS / Status
            frame_counter += 1
            frame_time_this = time.time() - frame_start_time_this
            instant_fps = 1.0 / frame_time_this if frame_time_this > 0 else 0
            elapsed_time_total = time.time() - loop_start_time
            if elapsed_time_total > 1: overall_fps = frame_counter / elapsed_time_total

            if frame_counter % 30 == 0:
                print(f"Status: Lim={current_speed_limit}|Req={speed_motor_requested}|Sent={final_speed_command_rpm}rpm|Steer={steering}|"
                      f"CTE={final_cte_for_pid:.2f}deg|Src={cte_source}|TradOK={traditional_success}|" # Show Trad success
                      f"L={last_known_innermost_left_x:.1f}|R={last_known_innermost_right_x:.1f}|" # Show L/R state again
                      f"FPS:{instant_fps:.1f}(Avg:{overall_fps:.1f})")

            # 13. Display Visualization Windows (Re-added Model elements)
            display_frame = frame.copy() if frame is not None else np.zeros((frame_height, frame_width, 3), dtype=np.uint8)
            cv2.putText(display_frame, f"CTE Source: {cte_source}", (10, frame_height - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            cv2.putText(display_frame, f"Final CTE (Angle): {final_cte_for_pid:.2f}", (10, frame_height - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            cv2.line(display_frame, (frame_center_x, 0), (frame_center_x, frame_height), (200, 200, 200), 1) # Center line
            # RE-ADDED: Target offset lines for model fallback visualization
            cv2.line(display_frame, (int(target_left_x_offset), 0), (int(target_left_x_offset), frame_height), (0, 255, 0), 1)
            cv2.line(display_frame, (int(target_right_x_offset), 0), (int(target_right_x_offset), frame_height), (0, 255, 0), 1)

            # Draw active detections
            for detection in active_detections_this_frame:
                d_x, d_y, d_w, d_h = detection['box']
                d_name = detection['name']
                d_conf = detection['confidence']
                label = f"{d_name}: {d_conf:.2f}"
                color = (0, 180, 0); thickness = 1

                # RE-ADDED: Highlight tracked model lines
                if d_name in ["dot_line", "line"]:
                    obj_center_x = d_x + d_w / 2.0
                    # Check if this detection corresponds to the persistently tracked line
                    is_tracked_left = know_model_left and abs(obj_center_x - last_known_innermost_left_x) < 10 # Tolerance
                    is_tracked_right = know_model_right and abs(obj_center_x - last_known_innermost_right_x) < 10 # Tolerance

                    if obj_center_x < frame_center_x:
                         color = (255, 165, 0); # Orange for leftish lines
                         if is_tracked_left: thickness = 3; label += " (TRACKED L)" # Highlight if tracked
                    elif obj_center_x > frame_center_x:
                         color = (255, 0, 255); # Magenta for rightish lines
                         if is_tracked_right: thickness = 3; label += " (TRACKED R)" # Highlight if tracked
                elif d_name in ["Stop Sign", "Traffic Light -Red-"]: color = (0, 0, 255); thickness = 2
                elif d_name in ["Person", "Car"]: color = (0, 255, 0); thickness = 2

                cv2.rectangle(display_frame, (d_x, d_y), (d_x + d_w, d_y + d_h), color, thickness)
                cv2.putText(display_frame, label, (d_x, d_y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, thickness)

            # RE-ADDED: Draw circles for last known model lane positions
            if last_known_innermost_left_x > 0:
                cv2.circle(display_frame, (int(last_known_innermost_left_x), frame_height - 20), 7, (255, 165, 0), -1)
                cv2.putText(display_frame, "Lk", (int(last_known_innermost_left_x) - 10, frame_height - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255,255,255), 1)
            if last_known_innermost_right_x > 0:
                cv2.circle(display_frame, (int(last_known_innermost_right_x), frame_height - 20), 7, (255, 0, 255), -1)
                cv2.putText(display_frame, "Rk", (int(last_known_innermost_right_x) - 10, frame_height - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255,255,255), 1)

            cv2.imshow("Object Detections & Lane Tracking", display_frame)

            # Display Warped Lane Image (Unchanged from previous merged)
            if imgWarp is not None:
                display_img_warp = ve_truc_anh(imgWarp, step=50)
                if display_img_warp is not None:
                    cv2.putText(display_img_warp, f"Trad CTE (Scaled Angle): {cte_f:.2f}", (10, im_height - 20 if im_height>20 else 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 1)
                    cv2.putText(display_img_warp, f"Trad Success (Scan): {traditional_success}", (10, im_height - 40 if im_height>40 else 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 1)
                    cv2.imshow("Traditional Lane Warp (Script 2 - Debug)", display_img_warp)
            # --- End of Loop ---

    # --- Error Handling & Cleanup (Unchanged) ---
    except KeyboardInterrupt: print("\nCtrl+C detected. Exiting...")
    except Exception as e:
        print(f"\n--- UNHANDLED EXCEPTION IN MAIN LOOP ---")
        print(f"Error Type: {type(e).__name__}"); print(f"Error Details: {e}")
        import traceback; traceback.print_exc(); print(f"--- END OF TRACEBACK ---")
    finally:
        print("\nCleaning up resources...")
        if ser and ser.is_open:
            try:
                print("Sending stop command (Steer=90, RPM=0)...")
                ser.write(b'<' + struct.pack('<ii', 90, 0) + b'>'); ser.flush(); time.sleep(0.1)
                ser.close(); print("Serial port closed.")
            except Exception as e: print(f"Error during serial cleanup: {e}")
        if piCam and piCam.started:
             try: piCam.stop(); print("Picamera2 stopped.")
             except Exception as e: print(f"Error stopping Picamera2: {e}")
        cv2.destroyAllWindows(); print("OpenCV windows closed.")
        print("Cleanup complete. Exiting program.")