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
# ... (Keep all configuration sections: NCNN, Lane Keeping, Speed Control, etc. as they were) ...
# NCNN Traffic Sign Detection Config
PARAM_PATH = "/home/labthayphuc/Desktop/yolov11ncnn/yolov11n_int8.param" # <<< UPDATE
BIN_PATH = "/home/labthayphuc/Desktop/yolov11ncnn/yolov11n_int8.bin"     # <<< UPDATE
NCNN_INPUT_SIZE = 640
NCNN_CONFIDENCE_THRESHOLD = 0.35
NCNN_NMS_THRESHOLD = 0.45
NCNN_NUM_CLASSES = 17
NCNN_CLASS_NAMES = [
    "Pedestrian Crossing", "Radar", "Speed Limit -100-", "Speed Limit -120-",
    "Speed Limit -20-", "Speed Limit -30-", "Speed Limit -40-", "Speed Limit -50-",
    "Speed Limit -60-", "Speed Limit -70-", "Speed Limit -80-", "Speed Limit -90-",
    "Stop Sign", "Traffic Light -Green-", "Traffic Light -Off-",
    "Traffic Light -Red-", "Traffic Light -Yellow-"
]
NCNN_INPUT_NAME = "in0"
NCNN_OUTPUT_NAME = "out0" # Make sure this is the correct output layer name for YOLOv11n
NCNN_MEAN_VALS = [0.0, 0.0, 0.0]
NCNN_NORM_VALS = [1/255.0, 1/255.0, 1/255.0]

# Lane Keeping & Motor Control Config
SERIAL_PORT = '/dev/ttyUSB0'      # <<< ADJUST
BAUD_RATE = 115200                # <<< MATCH ESP32
PICAM_SIZE = (640, 480)           # MUST BE 640x480 for NCNN preprocessing logic below
PICAM_FRAMERATE = 60

# Perspective Warp Points (from Script 2)
leftTop = 137
heightTop = 196
rightTop = 177
leftBottom = 0
heightBottom = 380
rightBottom = 40

# Speed Control Config
DEFAULT_MAX_SPEED = 120
MANUAL_MAX_SPEED = 100
MANUAL_ACCELERATION = 5
AUTO_MODE_SPEED_STRAIGHT = 30
AUTO_MODE_SPEED_CURVE = 20
MIN_SIGN_WIDTH_OR_HEIGHT = 200 # <<< NEW: Minimum size threshold for activation

# --- Global Variables ---
# ... (Keep all global variables as they were) ...
frame = None
imgWarp = None
net = None
ser = None
steering = 90
speed_motor_requested = 0
current_speed_limit = DEFAULT_MAX_SPEED
flag = 1
cte_f = 0.0
left_point = -1
right_point = -1
lane_width = 500 # Keep initial estimate
lane_width_max = 0
interested_line_y = 0 # Will be updated in SteeringAngle
im_height = 0         # Will be updated
im_width = 0          # Will be updated
frame_counter = 0
loop_start_time = 0
overall_fps = 0.0
# No need for global x,y,w,h anymore, they'll be in the detection results

# --- Initializations ---
# ... (Keep Serial, NCNN, Picamera2 initializations as they were) ...
# Serial bus setup
print(f"Initializing Serial on {SERIAL_PORT}...")
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)
    print(f"Serial initialized on {SERIAL_PORT} at {BAUD_RATE} baud.")
except serial.SerialException as e:
    print(f"CRITICAL: Serial Error: {e}. Could not open port {SERIAL_PORT}.")
    ser = None
except Exception as e:
    print(f"CRITICAL: An unexpected error occurred during serial initialization: {e}")
    ser = None

# NCNN Initialization
print("Initializing NCNN Net...")
net = ncnn.Net()
print("Checking for Vulkan GPU...")
gpu_count = 0
try: gpu_count = ncnn.get_gpu_count()
except Exception as e: print(f"Warning: Could not query GPU count - {e}. Assuming CPU only.")
if gpu_count > 0:
    print(f"Vulkan GPU detected ({gpu_count} device(s)). Enabling Vulkan acceleration.")
    net.opt.use_vulkan_compute = False # Set True if using Vulkan
else:
    print("Vulkan GPU not found or check failed. Using CPU threads.")
    net.opt.use_vulkan_compute = False
    net.opt.num_threads = 4 # Adjust based on your CPU
print("Loading NCNN model parameters and weights...")
try:
    start_load_time = time.time()
    if net.load_param(PARAM_PATH) != 0: print(f"CRITICAL: Load Param Error: {PARAM_PATH}"); exit()
    if net.load_model(BIN_PATH) != 0: print(f"CRITICAL: Load Model Error: {BIN_PATH}"); exit()
    end_load_time = time.time()
    print(f"NCNN model loaded successfully in {end_load_time - start_load_time:.4f} seconds.")
except Exception as e: print(f"CRITICAL: Exception during NCNN model loading: {e}"); exit()

# PiCamera2 setup
print("Initializing Picamera2...")
piCam = Picamera2()
try:
    piCam.preview_configuration.main.size = PICAM_SIZE
    piCam.preview_configuration.main.format = 'RGB888' # NCNN needs BGR, convert later
    piCam.preview_configuration.controls.FrameRate = float(PICAM_FRAMERATE)
    piCam.preview_configuration.align(); piCam.configure('preview'); piCam.start()
    time.sleep(1.0); print("Picamera2 started.")
except Exception as e: print(f"CRITICAL: Picamera2 Error: {e}"); exit()


# --- Helper Functions ---

def process_image():
    """ Captures frame, converts to BGR, preprocesses for lanes, and warps perspective. """
    global frame, imgWarp # Make frame global to be used by detection
    try:
        rgb_frame = piCam.capture_array()
    except Exception as e:
        print(f"Capture Error: {e}")
        frame = None
        imgWarp = None
        return False # Indicate failure

    if rgb_frame is None:
        frame = None
        imgWarp = None
        return False

    # Convert to BGR *immediately* for NCNN detection compatibility
    # Keep a reference to the BGR frame for detection
    frame = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2BGR)

    # Continue with lane processing using grayscale on the same frame data
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (11, 11), 0)
    edges = cv2.Canny(blur, 80, 130, apertureSize=3) # Settings from script 2
    h, w = edges.shape

    # Use perspective points from Script 2
    points = np.float32([(leftTop, heightTop), (w - rightTop, heightTop),
                         (leftBottom, heightBottom), (w - rightBottom, heightBottom)])
    pts2 = np.float32([[0, 0], [w, 0], [0, h], [w, h]])

    try:
        matrix = cv2.getPerspectiveTransform(points, pts2)
        imgWarp = cv2.warpPerspective(edges, matrix, (w, h))
    except cv2.error as e:
        print(f"Warp Error: {e}")
        imgWarp = None
        return False # Still return True as frame is valid, but warp failed
    return True # Indicate success (frame is valid)


# --- Restored SteeringAngle Function from Script 2 ---
def SteeringAngle():
    """ Calculates steering angle based on detected lanes using the multi-line iterative method from Script 2. """
    global left_point, right_point, interested_line_y, im_height, im_width, lane_width, lane_width_max, cte_f

    if imgWarp is None or imgWarp.size == 0:
        cte_f = 0.0
        # Reset points if warp failed
        left_point = -1
        right_point = -1
        return

    # Update image dimensions if needed (use imgWarp dimensions)
    current_h, current_w = imgWarp.shape[:2]
    if im_height != current_h or im_width != current_w:
        im_height, im_width = current_h, current_w
        if im_height == 0 or im_width == 0: # Check again after update
            cte_f = 0.0
            left_point = -1
            right_point = -1
            return
        # Reinitialize lane width guesses based on new width
        lane_width = int(im_width * 0.8)
        lane_width_max = int(im_width * 0.95)


    # Reset points for this frame's calculation
    center_img = im_width // 2
    set_point = 350 # Y-coordinate target line (from Script 2)
    # Make sure set_point is within image bounds
    set_point = max(0, min(set_point, im_height - 1))

    step = -5    # Step size for iterating upwards from bottom
    # Calculate normalization factor 'u' and weighting factor 'ki' based on Script 2 logic
    u = (im_height - set_point) / abs(step) if abs(step) > 0 else 1 # Avoid division by zero
    u = max(u, 1) # Ensure u is at least 1

    diff_accumulator = 0.0 # Accumulate weighted differences
    lines_processed = 0 # Count lines where calculation happened

    # Iterate from bottom up to set_point
    for i in range(im_height - 1, set_point + step, step): # Include set_point if step allows
        current_y = int(i)
        if not (0 <= current_y < im_height): # Bounds check for safety
            continue

        interested_line_y = current_y # Store last processed Y
        interested_line = imgWarp[interested_line_y, :]

        # Find left point by searching from center to left
        current_left = -1
        indices_left = np.where(interested_line[:center_img] > 0)[0]
        if len(indices_left) > 0:
            current_left = indices_left[-1]
            if i == (im_height -1): left_point = current_left # Store the bottom-most point

        # Find right point by searching from center to right
        current_right = -1
        indices_right = np.where(interested_line[center_img:] > 0)[0]
        if len(indices_right) > 0:
            current_right = center_img + indices_right[0]
            if i == (im_height -1): right_point = current_right # Store the bottom-most point


        # Update lane width estimate based on points found *at this Y level*
        if current_left != -1 and current_right != -1:
            current_lane_width = current_right - current_left
            # Simple filter/update for lane_width_max (can be improved)
            if current_lane_width > 50 and current_lane_width < im_width * 0.95:
                 lane_width_max = max(lane_width_max, current_lane_width) # Grow max width seen
                 lane_width = current_lane_width # Use current width for this level's midpoint
        elif lane_width_max == 0: # Initialize max width if not set
             lane_width_max = int(im_width * 0.8) # Default guess


        # Estimate missing points based on the max observed width
        final_left = current_left
        final_right = current_right
        # Use lane_width_max for estimation ONLY if one point is missing
        if current_left != -1 and current_right == -1:
            final_right = min(current_left + lane_width_max, im_width - 1) # Clamp to image edge
        elif current_right != -1 and current_left == -1:
            final_left = max(current_right - lane_width_max, 0) # Clamp to image edge


        # Calculate midpoint and difference *for this line* if we have at least one original point
        # or two estimated points that make sense
        mid_point = -1
        if final_left != -1 and final_right != -1: # Both points (original or estimated) available
            mid_point = (final_right + final_left) / 2
        elif final_left != -1: # Only left known (original or estimated)
            mid_point = final_left + lane_width_max / 2 # Estimate center based on left
        elif final_right != -1: # Only right known (original or estimated)
            mid_point = final_right - lane_width_max / 2 # Estimate center based on right

        if mid_point != -1: # Proceed if we could calculate a midpoint
            diff = center_img - mid_point

            # Apply weighting (ki) based on how close to the set_point this line is
            weight_denominator = (im_height - set_point)
            if weight_denominator == 0: weight_denominator = 1 # Avoid division by zero
            ki = max(0, (i - set_point) / weight_denominator)
            ki = 1 - ki # Invert: lines closer to set_point have higher weight (closer to 1)

            # Accumulate the weighted difference
            diff_accumulator += diff * ki
            lines_processed += 1

            # Draw circles for visualization (optional, can be done later)
            # if final_left != -1: cv2.circle(imgWarp, (int(final_left), interested_line_y), 3, (150, 150, 150), -1)
            # if final_right != -1: cv2.circle(imgWarp, (int(final_right), interested_line_y), 3, (150, 150, 150), -1)

    # Calculate final CTE as the average weighted difference across processed lines
    # Use 'lines_processed' instead of 'u' for a potentially more stable average
    if lines_processed > 0:
        cte_f = diff_accumulator / lines_processed
    else:
        cte_f = 0.0 # No lines processed, zero error

    # Clamp CTE to prevent extreme values if needed (e.g., if lanes are lost)
    cte_f = max(min(cte_f, im_width / 2), -im_width / 2)


def ve_truc_anh(img, step=100):
    """ Draws coordinate axes and grid lines on the image (from Script 2). """
    # ... (Keep ve_truc_anh function exactly as it was) ...
    h, w = img.shape[:2]
    img_color = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR) if len(img.shape) == 2 else img.copy()
    cv2.line(img_color, (0, h - 1), (w, h - 1), (200, 200, 200), 1) # X axis
    cv2.line(img_color, (0, 0), (0, h), (200, 200, 200), 1)       # Y axis
    for x in range(0, w, step):
        cv2.line(img_color, (x, h - 10), (x, h), (200, 200, 200), 1)
        cv2.putText(img_color, str(x), (x + 2, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
    for y in range(0, h, step):
        cv2.line(img_color, (0, y), (10, y), (200, 200, 200), 1)
        cv2.putText(img_color, str(y), (12, y + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
    return img_color


def signal_motor(key):
    """ Updates the *requested* motor speed based on key input. """
    # ... (Keep signal_motor function exactly as it was) ...
    global speed_motor_requested, flag
    new_speed_req = speed_motor_requested
    if key == ord('w'):
        new_speed_req = min(speed_motor_requested + MANUAL_ACCELERATION, MANUAL_MAX_SPEED)
        flag = 1
    elif key == ord('s'):
        new_speed_req = max(speed_motor_requested - MANUAL_ACCELERATION, 0)
    elif key == ord('a'): # Auto mode speed based on curve
        # Slightly less sensitive turn detection for speed reduction
        target_speed = AUTO_MODE_SPEED_STRAIGHT if abs(cte_f) < 25 else AUTO_MODE_SPEED_CURVE
        new_speed_req = target_speed
        flag = 1
    elif key == ord('x'): # Stop
        new_speed_req = 0
    if new_speed_req != speed_motor_requested:
        speed_motor_requested = new_speed_req


# --- PID Class (Using Script 1's version for robustness with delta_time) ---
class PID:
    """ Simple PID Controller Class. """
    # ... (Keep PID class exactly as it was) ...
    def __init__(self, kp, ki, kd, integral_limit=500, output_limit=(-60, 60)):
        self.kp=kp; self.ki=ki; self.kd=kd; self.cte_previous=0.0; self.integral=0.0
        self.integral_limit=integral_limit; self.output_min, self.output_max=output_limit; self.last_time=time.time()
    def update(self, cte):
        current_time=time.time()
        delta_time=current_time-self.last_time
        if delta_time<=1e-6: delta_time=1e-6 # Avoid division by zero or near-zero
        p_term=self.kp*cte
        # Anti-windup: Only integrate if the output is not already saturated or if error sign prevents saturation
        if (self.output_min < (p_term + self.ki*(self.integral + cte*delta_time)) < self.output_max) or \
           (cte*self.integral < 0): # Allow integration if error reduces integral magnitude
             self.integral+=cte*delta_time
             self.integral=max(min(self.integral,self.integral_limit),-self.integral_limit)

        i_term=self.ki*self.integral
        derivative=(cte-self.cte_previous)/delta_time
        d_term=self.kd*derivative
        self.cte_previous=cte; self.last_time=current_time
        pid_output=p_term+i_term+d_term
        steering_deviation=max(min(pid_output,self.output_max),self.output_min)
        # Map deviation (-limit to +limit) to servo angle (0-180, centered at 90)
        # Example: If limit is 50, deviation -50 maps to 40, +50 maps to 140
        # Adjust center if needed (e.g., 90 might not be perfectly straight)
        steering_servo_angle=round(90 + steering_deviation)
        steering_servo_angle=max(min(steering_servo_angle,180),0) # Clamp to 0-180
        return steering_servo_angle


# --- NCNN Detection Function ---
def detect_signs_and_get_results(input_frame_bgr):
    """ Performs NCNN traffic sign detection. Returns list of results including width/height. """
    detections_results = []
    if input_frame_bgr is None or net is None: return detections_results

    original_height, original_width = input_frame_bgr.shape[:2]

    # --- Preprocessing ---
    # Ensure the input size matches NCNN expected input
    if original_width != PICAM_SIZE[0] or original_height != PICAM_SIZE[1]:
         print(f"Warning: Input frame size ({original_width}x{original_height}) doesn't match PICAM_SIZE ({PICAM_SIZE[0]}x{PICAM_SIZE[1]}). Resizing.")
         input_frame_bgr = cv2.resize(input_frame_bgr, PICAM_SIZE)
         original_height, original_width = input_frame_bgr.shape[:2] # Update after resize

    # Calculate padding to make it square (letterboxing/pillarboxing)
    target_size = NCNN_INPUT_SIZE
    scale = min(target_size / original_width, target_size / original_height)
    new_w, new_h = int(original_width * scale), int(original_height * scale)
    dw = (target_size - new_w) // 2
    dh = (target_size - new_h) // 2

    try:
        # Create padded image
        resized_img = cv2.resize(input_frame_bgr, (new_w, new_h))
        padded_img = np.full((target_size, target_size, 3), 114, dtype=np.uint8) # Pad color (e.g., gray 114)
        padded_img[dh:dh+new_h, dw:dw+new_w, :] = resized_img

        # Convert to NCNN Mat
        mat_in = ncnn.Mat.from_pixels(padded_img, ncnn.Mat.PixelType.PIXEL_BGR, target_size, target_size)
        mat_in.substract_mean_normalize(NCNN_MEAN_VALS, NCNN_NORM_VALS)
    except Exception as e:
        print(f"NCNN Preprocessing Error: {e}")
        return detections_results

    # --- Inference ---
    try:
        ex = net.create_extractor()
        ex.input(NCNN_INPUT_NAME, mat_in)
        ret_extract, mat_out = ex.extract(NCNN_OUTPUT_NAME) # Make sure NCNN_OUTPUT_NAME is correct!
        if ret_extract != 0:
            print("NCNN Extract Error")
            return detections_results
    except Exception as e:
        print(f"NCNN Inference Error: {e}")
        return detections_results

    # --- Postprocessing ---
    # The output format of YOLO models can vary slightly.
    # Typically it's [batch, num_detections, box_coords + confidence + class_scores]
    # or sometimes transposed. We need to adapt based on the exact model.
    # Assuming output is [num_detections, 4_box_coords + num_classes] AFTER potential transpose
    # where box coords are cx, cy, w, h relative to NCNN_INPUT_SIZE

    output_data = np.array(mat_out)

    # Reshape or transpose if necessary based on observed output shape
    # Example handling (adjust based on your model's specific output):
    if len(output_data.shape) == 3 and output_data.shape[0] == 1:
        output_data = output_data[0] # Remove batch dimension if present
    # Check if transpose is needed (e.g., if shape is [num_classes+4, num_detections])
    if len(output_data.shape) == 2 and output_data.shape[0] == (NCNN_NUM_CLASSES + 4) and output_data.shape[1] > (NCNN_NUM_CLASSES + 4):
         output_data = output_data.T

    # Final check for expected shape [num_detections, 4 + num_classes]
    if len(output_data.shape) != 2 or output_data.shape[1] != (4 + NCNN_NUM_CLASSES):
        print(f"Unexpected NCNN output shape after processing: {output_data.shape}")
        return detections_results

    num_detections = output_data.shape[0]
    boxes = []
    confidences = []
    class_ids = []

    # Parse detections
    for i in range(num_detections):
        detection = output_data[i]
        class_scores = detection[4:]
        confidence = np.max(class_scores)
        class_id = np.argmax(class_scores)

        if confidence >= NCNN_CONFIDENCE_THRESHOLD:
            # Extract box coordinates (cx, cy, w, h) relative to padded NCNN_INPUT_SIZE
            cx, cy, w_ncnn, h_ncnn = detection[:4]

            # Scale box back to original image coordinates
            # 1. Convert NCNN box (relative to padded 640x640) to corners
            x1_ncnn = cx - w_ncnn / 2
            y1_ncnn = cy - h_ncnn / 2
            x2_ncnn = cx + w_ncnn / 2
            y2_ncnn = cy + h_ncnn / 2

            # 2. Remove padding offset
            x1_resized = x1_ncnn - dw
            y1_resized = y1_ncnn - dh
            x2_resized = x2_ncnn - dw
            y2_resized = y2_ncnn - dh

            # 3. Scale back to original image size (using the inverse of the resize scale)
            # scale = min(target_size / original_width, target_size / original_height)
            x1_orig = x1_resized / scale
            y1_orig = y1_resized / scale
            x2_orig = x2_resized / scale
            y2_orig = y2_resized / scale

            # 4. Calculate width and height in original image pixels
            w_orig = x2_orig - x1_orig
            h_orig = y2_orig - y1_orig

            # 5. Clamp coordinates to original image bounds
            x1_orig = max(0, x1_orig)
            y1_orig = max(0, y1_orig)
            w_orig = min(original_width - x1_orig, w_orig) # Adjust width based on clamped x1
            h_orig = min(original_height - y1_orig, h_orig) # Adjust height based on clamped y1

            # Store the box in (x, y, w, h) format for NMS and results
            boxes.append([int(x1_orig), int(y1_orig), int(w_orig), int(h_orig)])
            confidences.append(float(confidence))
            class_ids.append(class_id)

    # Apply Non-Maximum Suppression (NMS)
    if boxes:
        # Use the actual boxes for NMS now
        indices = cv2.dnn.NMSBoxes(boxes, confidences, NCNN_CONFIDENCE_THRESHOLD, NCNN_NMS_THRESHOLD)

        if len(indices) > 0:
             # Flatten indices if necessary (it can be returned as a column vector)
             if isinstance(indices, (list, tuple)) and len(indices) > 0 and isinstance(indices[0], (list, np.ndarray)):
                 indices = indices.flatten()

             processed_indices = set() # Ensure we process each index only once
             for idx in indices:
                 i = int(idx) # Get the index from NMS result
                 # Basic check to prevent index out of bounds after NMS
                 if 0 <= i < len(boxes) and i not in processed_indices:
                     box = boxes[i]
                     x, y, w, h = box # Unpack the original image coordinates box
                     confidence_nms = confidences[i]
                     class_id_nms = class_ids[i]
                     processed_indices.add(i) # Mark as processed

                     if 0 <= class_id_nms < len(NCNN_CLASS_NAMES):
                         class_name = NCNN_CLASS_NAMES[class_id_nms]
                     else:
                         class_name = f"ID:{class_id_nms}"

                     # Append result with width and height
                     detections_results.append({
                         "name": class_name,
                         "confidence": confidence_nms,
                         "box": box, # Store the box [x, y, w, h]
                         "width": w, # Store width
                         "height": h # Store height
                     })

    return detections_results


# --- Speed Limit Parsing Function (Copied from Script 1) ---
def parse_speed_limit(class_name):
    """ Extracts the speed value from 'Speed Limit -VALUE-' strings. """
    # ... (Keep parse_speed_limit function exactly as it was) ...
    match = re.search(r'Speed Limit -(\d+)-', class_name)
    if match:
        try: return int(match.group(1))
        except ValueError: return None
    return None

# --- Speed Transformation Function (Copied from Script 1) ---
def transform_speed(velocity):
    f_velocity = velocity / 10
    rpm = (f_velocity *30*2.85)/(3.14*0.0475*3.6)
    return int(round(rpm))


# --- Main Execution ---
def main():
    # Remove display_frame from globals as it's no longer needed
    global steering, speed_motor_requested, flag, current_speed_limit, net, ser, frame_counter, loop_start_time, overall_fps, frame, imgWarp # Added imgWarp back for safety

    # Ensure globals are accessible
    if net is None: print("CRITICAL: NCNN Net object not initialized!"); return
    if ser is None: print("WARNING: Serial port not available. Motor commands will not be sent.")

    # --- PID Tuning - START HERE ---
    kp = 0.45 # Proportional gain
    ki = 0.001 # Integral gain
    kd = 0.15 # Derivative gain
    pid_integral_limit = 300 # Max/min value the integral term can reach
    pid_output_limit = (-55, 55) # Max steering deviation (+/- degrees from center 90)
    # --- PID Tuning - END HERE ---

    pid_controller = PID(kp, ki, kd, integral_limit=pid_integral_limit, output_limit=pid_output_limit)
    frame_counter = 0
    loop_start_time = time.time()
    overall_fps = 0.0
    current_speed_limit = DEFAULT_MAX_SPEED
    speed_motor_requested = 0 # Start stopped
    print("\nStarting main loop...")
    print(f"Default Speed Limit: {current_speed_limit}. Use W/A/S/X for manual control.")
    print(f"NCNN Inference using: {'Vulkan GPU' if net.opt.use_vulkan_compute else 'CPU'}")
    print(f"Traffic Sign Activation Size Threshold: W > {MIN_SIGN_WIDTH_OR_HEIGHT} or H > {MIN_SIGN_WIDTH_OR_HEIGHT}")
    print("Press 'q' in the display window to exit.")
    # No need for display_frame variable anymore

    try:
        while True:
            frame_start_time = time.time() # Start FPS timer

            # 1. Capture and Preprocess for Lanes
            if not process_image(): # Updates global 'frame' (BGR) and 'imgWarp' (edges)
                time.sleep(0.01)
                continue

            # 'frame' (BGR) is ready here if needed for detection
            # 'imgWarp' (Edges) is ready here if needed for lane finding

            # 2. Calculate Steering Angle
            # Ensure imgWarp is valid before calling SteeringAngle
            if imgWarp is None:
                 print("Warning: imgWarp is None, skipping SteeringAngle calculation.")
                 cte_f = 0.0 # Reset CTE if warp failed
                 # Maybe reset points too? Depends on desired behavior
                 left_point = -1
                 right_point = -1
            else:
                 SteeringAngle() # Updates cte_f based on imgWarp

            # 3. Traffic Sign Detection (on the BGR 'frame')
            # Ensure frame is valid before detection
            if frame is None:
                print("Warning: frame is None, skipping detection.")
                detected_signs = []
            else:
                detected_signs = detect_signs_and_get_results(frame) # Uses 'frame', returns width/height

            # 4. Handle Key Presses
            key = cv2.waitKey(1) & 0xFF
            signal_motor(key) # Updates speed_motor_requested
            if key == ord('q'):
                print("'q' pressed, exiting loop.")
                break

            # 5. Process Detections & Update Speed Rules (with size check)
            stop_condition_met = False
            limit_sign_seen_this_frame = False
            new_limit_value = -1 # Reset for this frame
            active_signs_this_frame = [] # Store signs that meet the size criteria

            if detected_signs:
                # active_signs_this_frame = [] # Moved declaration up
                for sign in detected_signs:
                    sign_w = sign.get('width', 0)
                    sign_h = sign.get('height', 0)

                    # --- SIZE CHECK ---
                    if sign_w > MIN_SIGN_WIDTH_OR_HEIGHT or sign_h > MIN_SIGN_WIDTH_OR_HEIGHT:
                        active_signs_this_frame.append(sign) # Add to list for drawing later
                        sign_name = sign['name']
                        # print(f"  -> Active Sign Detected: {sign_name} (W:{sign_w}, H:{sign_h})") # Debug print

                        if sign_name in ["Stop Sign", "Traffic Light -Red-"]:
                            stop_condition_met = True
                            continue # Process next large sign

                        parsed_limit = parse_speed_limit(sign_name)
                        if parsed_limit is not None:
                            limit_sign_seen_this_frame = True
                            if new_limit_value == -1 or parsed_limit < new_limit_value:
                                new_limit_value = parsed_limit
                    # else: # Optional: Print ignored signs
                    #    print(f"  -> Ignoring small sign: {sign['name']} (W:{sign_w}, H:{sign_h})")

            # Update speed limit only if a *large enough* limit sign was seen
            if limit_sign_seen_this_frame:
                if new_limit_value != current_speed_limit:
                    print(f"** Speed Limit Updated (due to large sign): {new_limit_value} **")
                    current_speed_limit = new_limit_value

            # 6. Determine Final Commanded Speed (RPM)
            final_speed_command_vel = speed_motor_requested
            final_speed_command_vel = min(final_speed_command_vel, current_speed_limit)
            if stop_condition_met:
                if final_speed_command_vel > 0:
                     print("** Stop condition met (large sign)! Setting speed to 0. **")
                final_speed_command_vel = 0
            final_speed_command_rpm = transform_speed(final_speed_command_vel)

            # 7. PID Calculation for Steering
            steering = pid_controller.update(cte_f)

            # 8. Send Data via Serial (if available)
            if ser:
                try:
                    data_to_send = struct.pack('<ii', int(steering), int(final_speed_command_rpm))
                    ser.write(b'<' + data_to_send + b'>')
                    ser.flush()
                except serial.SerialException as e:
                    print(f"Serial Write Error: {e}"); ser.close(); ser = None; print("Serial disabled.")
                except Exception as e: print(f"Unexpected error during serial send: {e}")

            # 9. Calculate and Print FPS / Status
            frame_counter += 1
            frame_time = time.time() - frame_start_time
            instant_fps = 1.0 / frame_time if frame_time > 0 else 0
            elapsed_time_total = time.time() - loop_start_time
            if elapsed_time_total > 1:
                overall_fps = frame_counter / elapsed_time_total

            if frame_counter % 30 == 0:
                print(f" Status: Limit={current_speed_limit} | Req={speed_motor_requested} vel | Sent={final_speed_command_rpm} rpm | Steer={steering} | CTE={cte_f:.2f}")
                print(f" FPS: {instant_fps:.1f} (Avg: {overall_fps:.1f})")

            # 10. Display Image (ONLY Warp View)
            # Ensure imgWarp is valid before trying to display
            if imgWarp is not None:
                display_warp = ve_truc_anh(imgWarp, step=50) # Use ve_truc_anh for grid/color

                im_h, im_w = display_warp.shape[:2]
                center_img_viz = im_w // 2
                display_y = interested_line_y if interested_line_y > 0 else im_h // 2 # Fallback Y

                # Draw center line, lane points, target position
                cv2.line(display_warp, (center_img_viz, 0), (center_img_viz, im_h-1), (0, 0, 255), 1)
                if left_point != -1: cv2.circle(display_warp, (left_point, im_h - 1), 5, (0, 255, 255), -1)
                if right_point != -1: cv2.circle(display_warp, (right_point, im_h - 1), 5, (0, 255, 255), -1)
                mid_point_viz = int(center_img_viz - cte_f)
                cv2.line(display_warp, (mid_point_viz, display_y - 20), (mid_point_viz, display_y + 20), (255, 0, 0), 2)

                # Add text overlays (FPS, CTE, Commands, Limit, Active Signs)
                cv2.putText(display_warp, f"FPS: {instant_fps:.1f}", (im_w - 120, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
                cv2.putText(display_warp, f"CTE: {cte_f:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
                cv2.putText(display_warp, f"Steer Cmd: {steering}", (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
                cv2.putText(display_warp, f"RPM Cmd: {final_speed_command_rpm} RPM", (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
                cv2.putText(display_warp, f"(Speed {speed_motor_requested} km/h)", (10, 105), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 0), 1)
                cv2.putText(display_warp, f"Limit: {current_speed_limit}", (10, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1)

                sign_y_offset = 160
                for sign in active_signs_this_frame: # Use the filtered list
                    sign_text = f"ACTIVE: {sign['name']} ({sign['confidence']:.2f})"
                    cv2.putText(display_warp, sign_text, (10, sign_y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                    sign_y_offset += 20
                if stop_condition_met: cv2.putText(display_warp, "STOP ACTIVE", (im_w // 2 - 100 , im_h // 2), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)

                cv2.imshow("Lane Detection Warp", display_warp) # Show the warp window

    except KeyboardInterrupt:
        print("\nCtrl+C detected. Exiting...")
    finally:
        # --- Cleanup ---
        print("Cleaning up resources...")
        if ser and ser.is_open:
            try:
                print("Sending stop command to motor...")
                stop_data = struct.pack('<ii', 90, 0)
                ser.write(b'<' + stop_data + b'>')
                ser.flush(); time.sleep(0.1)
                ser.close(); print("Serial port closed.")
            except Exception as e: print(f"Error closing serial port: {e}")
        if 'piCam' in globals() and piCam.started:
            try: piCam.stop(); print("Picamera2 stopped.")
            except Exception as e: print(f"Error stopping Picamera2: {e}")
        cv2.destroyAllWindows(); print("OpenCV windows closed.")
        print("Cleanup complete.")

# Keep the if __name__ == "__main__": block as it was
if __name__ == "__main__":
    # Make sure all necessary imports and function definitions are above this point
    main()