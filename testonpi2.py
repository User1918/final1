# -*- coding: utf-8 -*-
import numpy as np
import cv2
import time
import ncnn # Import the ncnn Python binding
from picamera2 import Picamera2 # Import Picamera2 library
import serial
import struct # No longer needed for string-based serial
import re # Import regular expressions for parsing speed limits

# --- Configuration ---
# NCNN Traffic Sign Detection Config
PARAM_PATH = "/home/labthayphuc/Desktop/yolov11ncnn/model.ncnn.param" # <<< UPDATE THIS PATH
BIN_PATH = "/home/labthayphuc/Desktop/yolov11ncnn/model.ncnn.bin"     # <<< UPDATE THIS PATH
NCNN_INPUT_SIZE = 640           # Input size for NCNN model (MUST BE 640 for modified preprocessing)
NCNN_CONFIDENCE_THRESHOLD = 0.35 # Min confidence for NCNN detection (Adjust if needed)
NCNN_NMS_THRESHOLD = 0.45       # NMS threshold for NCNN detection
NCNN_NUM_CLASSES = 17           # Number of classes for NCNN model
NCNN_CLASS_NAMES = [            # Class names for NCNN model
    "Pedestrian Crossing", "Radar", "Speed Limit -100-", "Speed Limit -120-",
    "Speed Limit -20-", "Speed Limit -30-", "Speed Limit -40-", "Speed Limit -50-",
    "Speed Limit -60-", "Speed Limit -70-", "Speed Limit -80-", "Speed Limit -90-",
    "Stop Sign", "Traffic Light -Green-", "Traffic Light -Off-",
    "Traffic Light -Red-", "Traffic Light -Yellow-"
]
NCNN_INPUT_NAME = "in0"         # Input layer name for NCNN
NCNN_OUTPUT_NAME = "out0"       # Output layer name for NCNN
NCNN_MEAN_VALS = [0.0, 0.0, 0.0] # Mean values for NCNN normalization
NCNN_NORM_VALS = [1/255.0, 1/255.0, 1/255.0] # Norm values for NCNN normalization

# Lane Keeping & Motor Control Config
# --- IMPORTANT: ADJUST THESE ROI CROP VALUES FOR YOUR CAMERA SETUP ---
LANE_ROI_CROP = (100, 380, 540, 480) # Example: (x1, y1, x2, y2) -> Must be within 640x480 of warped image
LANE_RESIZE_DIM = (640, 480)         # resize_dim for warpImg
SERIAL_PORT = '/dev/ttyUSB0'         # <<< ADJUST THIS PORT AS NEEDED
BAUD_RATE = 115200                   # <<< MUST MATCH ESP32 BAUD RATE
PICAM_SIZE = (640, 480)              # MUST BE 640x480 for modified preprocessing
PICAM_FRAMERATE = 60                 # Note: Actual FPS might be lower due to processing
# TRANSMISSION_INTERVAL = 0.001      # Note: Original script sends every loop, not timed interval

# Speed Control Config
DEFAULT_MAX_SPEED = 40           # Default speed limit if no sign detected (used for reset)
MANUAL_MAX_SPEED = 50            # Absolute max speed achievable with 'w' key (caps requested speed)
MANUAL_ACCELERATION = 5
AUTO_MODE_SPEED_STRAIGHT = 30    # Speed for 'a' when straight
AUTO_MODE_SPEED_CURVE = 20       # Speed for 'a' when turning


# --- Global Variables ---
canny_edge = None
frame = None
gray = None
imgWarp = None
lines = None
roi = None
roi_haha = None
points = None
left_point = -1
right_point = -1
interested_line_y = 0
im_height = 0
im_width = 0
lane_width = 500
lane_width_max = 0
cte_f = 0.0
speed_motor_requested = 0
last_speed_sent = 0
current_speed_limit = DEFAULT_MAX_SPEED
flag = 1
steering = 90
net = None # Initialize net globally
ser = None # Initialize ser globally

# --- Initializations ---

# <<< --- MODIFIED Serial bus setup (String Format) --- >>>
print(f"Initializing Serial on {SERIAL_PORT}...")
try:
    # Use timeout=1 like original script, can help prevent blocking if ESP32 isn't responding
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2) # Keep the delay to allow ESP32 to initialize
    print(f"Serial initialized on {SERIAL_PORT} at {BAUD_RATE} baud.")
except serial.SerialException as e:
    print(f"CRITICAL: Serial Error: {e}. Could not open port {SERIAL_PORT}. Check connection and permissions.")
    ser = None # Ensure ser is None if initialization fails
    # Consider exiting if serial is critical: exit()
except Exception as e:
    print(f"CRITICAL: An unexpected error occurred during serial initialization: {e}")
    ser = None
    # Consider exiting: exit()
# <<< --- End MODIFIED Serial bus setup --- >>>

# <<< --- MODIFIED NCNN Initialization with Vulkan Check --- >>>
print("Initializing NCNN Net...")
net = ncnn.Net() # Create the Net object

print("Checking for Vulkan GPU...")
gpu_count = 0
try:
    # Check how many Vulkan-capable GPUs NCNN can detect at runtime
    gpu_count = ncnn.get_gpu_count()
except Exception as e:
    print(f"Warning: Could not query GPU count - {e}. Assuming CPU only.")

if gpu_count > 0: # Use > 0 check for clarity
    # If GPUs are found, configure NCNN to use Vulkan
    print(f"Vulkan GPU detected ({gpu_count} device(s)). Enabling Vulkan acceleration.")
    net.opt.use_vulkan_compute = True
    # net.opt.vulkan_compute_device_index = 0 # Usually defaults to device 0 (Pi's GPU)
else:
    # If no GPU found or error occurred, configure for CPU multithreading
    print("Vulkan GPU not found or check failed. Using CPU threads.")
    net.opt.use_vulkan_compute = False
    net.opt.num_threads = 4 # Use 4 threads for Pi 5 CPU

# Load model parameters and weights *after* setting options
print("Loading NCNN model parameters and weights...")
try:
    start_load_time = time.time()
    if net.load_param(PARAM_PATH) != 0:
        print(f"CRITICAL: Load Param Error: {PARAM_PATH}"); exit()
    if net.load_model(BIN_PATH) != 0:
        print(f"CRITICAL: Load Model Error: {BIN_PATH}"); exit()
    end_load_time = time.time()
    print(f"NCNN model loaded successfully in {end_load_time - start_load_time:.4f} seconds.")
except Exception as e:
    print(f"CRITICAL: Exception during NCNN model loading: {e}")
    exit()
# <<< --- End MODIFIED NCNN Initialization --- >>>

# PiCamera2 setup
print("Initializing Picamera2...")
piCam = Picamera2()
try:
    piCam.preview_configuration.main.size = PICAM_SIZE
    piCam.preview_configuration.main.format = 'RGB888'
    piCam.preview_configuration.controls.FrameRate = float(PICAM_FRAMERATE)
    piCam.preview_configuration.align(); piCam.configure('preview'); piCam.start()
    time.sleep(1.0); print("Picamera2 started.")
except Exception as e: print(f"CRITICAL: Picamera2 Error: {e}"); exit()


# --- Helper Functions ---
# (All helper functions: picam, getPoints, warpImg, drawPoints, SteeringAngle,
#  handle_key_input, PID, detect_signs_and_get_results, parse_speed_limit
#  remain unchanged from the previous version)





def picam():
    """Captures frame, converts to BGR & gray, applies blur and Canny edge."""
    global canny_edge, frame, gray
    try: rgb_frame = piCam.capture_array()
    except Exception as e: print(f"Capture Error: {e}"); frame=None;gray=None;canny_edge=None; return
    if rgb_frame is None: frame=None;gray=None;canny_edge=None; return
    frame = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2BGR)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (11, 11), 0)
    thresh_low = 100; thresh_high = 175
    canny_edge = cv2.Canny(blur, thresh_low, thresh_high, apertureSize=3)



def getPoints(wT=640, hT=480):
    """Defines perspective transform points."""
    global points
    widthTop = 70; heightTop = 95; widthBottom = -750; heightBottom = 380 # Check these
 
 



def warpImg(roi_crop=LANE_ROI_CROP, resize_dim=LANE_RESIZE_DIM):
    """Applies perspective warp, crops ROI, resizes."""
    global imgWarp, lines, roi, roi_haha
    if canny_edge is None: 
        return None
    h, w = canny_edge.shape
    if points is None or len(points) != 4: 
        getPoints(w,h)
    pts1 = np.float32(points)
    pts2 = np.float32([[0, 0], [w, 0], [0, h], [w, h]])
    try: 
        matrix = cv2.getPerspectiveTransform(pts1, pts2)
        imgWarp = cv2.warpPerspective(canny_edge, matrix, (w, h))
    except cv2.error as e: 
        print(f"Warp Error: {e}")
        roi_haha = None; return None
    x1, y1, x2, y2 = roi_crop
    y1_clamped=max(0,min(y1,h))
    y2_clamped=max(y1_clamped,min(y2,h))
    x1_clamped=max(0,min(x1,w))
    x2_clamped=max(x1_clamped,min(x2,w))
    if y1_clamped >= y2_clamped or x1_clamped >= x2_clamped: 
        print(f"ROI Clamp Error")
        roi_haha=None
        return None
    roi = imgWarp[y1_clamped:y2_clamped, x1_clamped:x2_clamped]
    if roi.size == 0: 
        print("ROI empty")
        roi_haha=None
        return None
    try: 
        roi_haha = cv2.resize(roi, resize_dim)
    except cv2.error as e: 
        print(f"Resize Error: {e}")
        roi_haha=None
        return None
    lines = cv2.HoughLinesP(roi_haha, 1, np.pi/180, threshold=170, minLineLength=100, maxLineGap=20)
    return roi_haha




def drawPoints():
    """Optional: Draws perspective points."""
    if frame is None or points is None: return
    display_frame = frame.copy(); color = (0,0,255)
    for x in range(4):
        try: cv2.circle(display_frame, (int(points[x][0]), int(points[x][1])), 10, color, cv2.FILLED)
        except IndexError: pass
    cv2.imshow("Perspective Points", display_frame)





def SteeringAngle():
    """Calculates CTE."""
    global left_point, right_point, interested_line_y, im_height, im_width, lane_width, lane_width_max, cte_f

    if roi_haha is None or roi_haha.size == 0: 
        cte_f = 0.0
        return
    im_height, im_width = roi_haha.shape[:2]
    if im_height == 0 or im_width == 0: 
        cte_f = 0.0
        return
    left_point = -1
    right_point = -1
    center_img = im_width // 2
    set_point = max(0, min(int(im_height * 0.85), im_height - 1))
    if lane_width_max <= 0: 
        lane_width_max = int(im_width * 0.8)
    interested_line_y = set_point; interested_line = roi_haha[interested_line_y, :]
    left_indices = np.where(interested_line[:center_img]>0)[0]
    left_point = left_indices[-1] if len(left_indices)>0 else -1
    right_indices = np.where(interested_line[center_img:]>0)[0]
    right_point = (center_img + right_indices[0]) if len(right_indices)>0 else -1
    if left_point != -1 and right_point == -1: 
        right_point = left_point + lane_width_max
    elif right_point != -1 and left_point == -1: 
        left_point = right_point - lane_width_max
    elif left_point != -1 and right_point != -1:
        current_lane_width = right_point - left_point
        lane_width_max = max(current_lane_width, 50)
        lane_width_max = min(lane_width_max, im_width)
    if left_point != -1 or right_point != -1:
        if left_point == -1: 
            mid_point = right_point - lane_width_max / 2
        elif right_point == -1: 
            mid_point = left_point + lane_width_max / 2
        else: mid_point = (right_point + left_point) / 2
        cte_f = center_img - mid_point
    else: cte_f = 0.0





def handle_key_input(key):
    """Updates the *requested* motor speed based on key input."""
    global speed_motor_requested, flag
    new_speed_req = speed_motor_requested
    if key == ord('w'): 
        new_speed_req = min(speed_motor_requested + MANUAL_ACCELERATION, MANUAL_MAX_SPEED)
        flag = 1
    elif key == ord('s'): 
        new_speed_req = max(speed_motor_requested - MANUAL_ACCELERATION, 0)
        flag = 0 if new_speed_req == 0 else 1
    elif key == ord('a'): 
        target_speed = AUTO_MODE_SPEED_STRAIGHT if abs(cte_f) < 10 else AUTO_MODE_SPEED_CURVE
        new_speed_req = target_speed; flag = 1
    elif key == ord('x'): 
        new_speed_req = 0; flag = 0
    if new_speed_req != speed_motor_requested: 
        speed_motor_requested = new_speed_req






class PID:
    """Simple PID Controller Class."""
    def __init__(self, kp, ki, kd, integral_limit=500, output_limit=(-60, 60)):
        self.kp=kp; self.ki=ki; self.kd=kd; self.cte_previous=0.0; self.integral=0.0
        self.integral_limit=integral_limit; self.output_min, self.output_max=output_limit; self.last_time=time.time()
    def update(self, cte):
        current_time=time.time()
        delta_time=current_time-self.last_time
        if delta_time<=1e-6: 
            delta_time=1e-6
        p_term=self.kp*cte
        self.integral+=cte*delta_time
        self.integral=max(min(self.integral,self.integral_limit),-self.integral_limit)
        i_term=self.ki*self.integral
        derivative=(cte-self.cte_previous)/delta_time if delta_time > 1e-6 else 0.0
        d_term=self.kd*derivative
        self.cte_previous=cte; self.last_time=current_time
        pid_output=p_term+i_term+d_term
        steering_deviation=max(min(pid_output,self.output_max),self.output_min)
        steering_servo_angle=round(90+steering_deviation)
        steering_servo_angle=max(min(steering_servo_angle,180),0)
        return steering_servo_angle



def parse_speed_limit(class_name):
    """Extracts the speed value from 'Speed Limit -VALUE-' strings."""
    match = re.search(r'Speed Limit -(\d+)-', class_name)
    if match:
        try: return int(match.group(1))
        except ValueError: 
            return None
    return None






def detect_signs_and_get_results(input_frame_bgr):
    """Performs NCNN traffic sign detection. Returns list of results."""
    # (Function unchanged - still uses global 'net')
    detections_results = []
    if input_frame_bgr is None: 
        return detections_results
    original_height, original_width = input_frame_bgr.shape[:2]
    if original_width != 640 or original_height != 480 or NCNN_INPUT_SIZE != 640:
            print(f"ERROR: detect_signs (no scale) expects 640x480 input, 640 NCNN size.")
            return detections_results
    try:
        padded_img = np.full((NCNN_INPUT_SIZE, NCNN_INPUT_SIZE, 3), 114, dtype=np.uint8)
        dw = (NCNN_INPUT_SIZE - original_width) // 2; dh = (NCNN_INPUT_SIZE - original_height) // 2
        padded_img[dh:dh+original_height, dw:dw+original_width, :] = input_frame_bgr
        mat_in = ncnn.Mat.from_pixels(padded_img, ncnn.Mat.PixelType.PIXEL_BGR, NCNN_INPUT_SIZE, NCNN_INPUT_SIZE)
        mat_in.substract_mean_normalize(NCNN_MEAN_VALS, NCNN_NORM_VALS)
    except Exception as e: 
        print(f"Preproc Error: {e}")
        return detections_results
    try:
        ex = net.create_extractor()
        ex.input(NCNN_INPUT_NAME, mat_in)
        ret_extract, mat_out = ex.extract(NCNN_OUTPUT_NAME)
        if ret_extract != 0: 
            return detections_results
    except Exception as e: 
        print(f"Inference Error: {e}")
        return detections_results
    output_data = np.array(mat_out)
    if len(output_data.shape) == 3 and output_data.shape[0] == 1: 
        output_data = output_data[0]
    if len(output_data.shape) == 2 and output_data.shape[0] == (NCNN_NUM_CLASSES + 4): 
        output_data = output_data.T
    if len(output_data.shape) != 2: 
        return detections_results
    num_detections, detection_size = output_data.shape
    expected_size = 4 + NCNN_NUM_CLASSES
    if detection_size != expected_size: 
        return detections_results
    boxes = []
    confidences = []
    class_ids = []
    for i in range(num_detections):
        detection = output_data[i]
        class_scores = detection[4:]
        confidence = np.max(class_scores)
        if confidence >= NCNN_CONFIDENCE_THRESHOLD:
            class_id = np.argmax(class_scores)
            boxes.append([0,0,1,1])
            confidences.append(float(confidence))
            class_ids.append(class_id) # Dummy box for NMS
    if boxes:
        # Use dummy boxes for NMS, actual box coordinates not needed here, only filtering by confidence/class
        dummy_boxes_for_nms = [[0, 0, 1, 1]] * len(confidences) # Create one dummy box per detection above threshold
        indices = cv2.dnn.NMSBoxes(dummy_boxes_for_nms, confidences, NCNN_CONFIDENCE_THRESHOLD, NCNN_NMS_THRESHOLD)

        if len(indices) > 0:
            # Flatten indices if necessary (it might return [[0], [2], ...])
            if isinstance(indices[0], (list, np.ndarray)):
                indices = indices.flatten()

            processed_indices = set() # Keep track to avoid duplicates if NMS outputs same index
            for idx in indices:
                i = int(idx)
                # Check bounds and ensure we haven't processed this index already
                if 0 <= i < len(class_ids) and i not in processed_indices:
                    confidence_nms = confidences[i]
                    class_id_nms = class_ids[i]
                    processed_indices.add(i) # Mark index as processed

                    if 0 <= class_id_nms < len(NCNN_CLASS_NAMES):
                        class_name = NCNN_CLASS_NAMES[class_id_nms]
                    else:
                        class_name = f"ID:{class_id_nms}" # Fallback if class ID is out of range

                    # Add the filtered detection result
                    detections_results.append({"name": class_name, "confidence": confidence_nms})
    return detections_results


# --- Main Execution ---

def main():
    global steering, last_speed_sent, speed_motor_requested, flag, current_speed_limit, net, ser # Add ser to globals
    if net is None:
        print("CRITICAL: NCNN Net object not initialized!")
        return
    if ser is None: # Check if serial was initialized successfully
        print("WARNING: Serial port not available. Motor commands will not be sent.")
        # Decide if you want to continue without serial or exit
        # return # Uncomment this line to exit if serial is mandatory

    kp = 0.4
    ki = 0.01
    kd = 0.05 # --- TUNE PID GAINS ---
    pid_controller = PID(kp, ki, kd)

    frame_counter = 0
    loop_start_time = time.time()
    overall_fps = 0.0
    current_speed_limit = DEFAULT_MAX_SPEED
    speed_motor_requested = 0
    last_speed_sent = 0

    print("\nStarting main loop...")
    print(f"Default Speed Limit: {current_speed_limit}. Use W/A/S/X for manual control.")
    print(f"NCNN Inference using: {'Vulkan GPU' if net.opt.use_vulkan_compute else 'CPU'}") # Confirm which backend is active
    print("Press 'q' in the display window to exit.")

    try:
        while True:
            frame_start_time = time.time()

            # 1. Capture and basic processing
            picam()
            if frame is None: time.sleep(0.01); continue # Short sleep if capture fails

            # 2. Lane Detection Processing
            getPoints(frame.shape[1], frame.shape[0])
            warp_result = warpImg()

            # 3. Steering Angle Calculation
            SteeringAngle()

            # 4. Traffic Sign Detection (Uses global 'net' object)
            detect_start_time = time.time()
            detected_signs = detect_signs_and_get_results(frame)
            detect_end_time = time.time()
            # print(f" Sign Detection Time: {detect_end_time - detect_start_time:.4f} s") # Optional timing print

            # 5. Handle Key Presses (updates speed_motor_requested)
            key = cv2.waitKey(1) & 0xFF
            handle_key_input(key)
            if key == ord('q'): 
                print("'q' pressed, exiting loop.")
                break

            # 6. Process Detections & Update Speed Rules
            stop_condition_met = False
            limit_sign_seen_this_frame = False
            new_limit_value = -1
            # print("\n--- Frame Cycle ---") # Less verbose
            if detected_signs:
                # print(" Signs Detected:") # Less verbose
                for sign in detected_signs:
                    # print(f"  - {sign['name']}: {sign['confidence']:.2f}") # Less verbose
                    stop_signs = ["Stop Sign"] # Add Red Light to stop conditions
                    if sign['name'] in stop_signs: 
                        stop_condition_met = True
                    parsed_limit = parse_speed_limit(sign['name'])
                    if parsed_limit is not None:
                        limit_sign_seen_this_frame = True
                        # If multiple limit signs are seen, maybe take the lowest? Or the first?
                        # Current logic takes the last one processed in the loop.
                        new_limit_value = parsed_limit

            # Update Speed Limit Rule
            if limit_sign_seen_this_frame:
                if new_limit_value != current_speed_limit:
                    print(f"** Speed Limit Updated: {new_limit_value} **")
                    current_speed_limit = new_limit_value
            # Optional: Reset to default if no sign seen for a while (more complex logic)
            # else:
            #     if current_speed_limit != DEFAULT_MAX_SPEED:
            #         print(f"** No speed limit sign detected, resetting limit to default: {DEFAULT_MAX_SPEED} **")
            #         current_speed_limit = DEFAULT_MAX_SPEED


            # 7. Determine Final Commanded Speed
            final_speed_command = speed_motor_requested
            final_speed_command = min(final_speed_command, current_speed_limit) # Apply speed limit
            if stop_condition_met:
                final_speed_command = 0 # Override speed to 0 if stop sign/red light detected
                print("** Stop condition met! Setting speed to 0. **")

            last_speed_sent = final_speed_command # Update the value that will be sent

            # 8. PID Calculation
            steering = pid_controller.update(cte_f) # steering is likely float here

            # <<< --- MODIFIED: Send Data via Serial (String Format) --- >>>
            steering_angle = round(steering)
            print(f"Steering angle: {steering_angle}")
            last_speed = round(last_speed_sent)
            data = b'<' + struct.pack('<ii', steering_angle, last_speed) + b'>'
            ser.write(data)
            ser.flush()
            # <<< --- End MODIFIED Serial Sending --- >>>


            # 10. Calculate and Print FPS / Status to Console
            frame_counter += 1
            current_loop_time = time.time()
            frame_time = current_loop_time - frame_start_time
            instant_fps = 1.0 / frame_time if frame_time > 0 else 0
            elapsed_time_total = current_loop_time - loop_start_time
            if elapsed_time_total > 1: # Update average FPS calculation less frequently
                 overall_fps = frame_counter / elapsed_time_total

            # Print status less frequently or conditionally to avoid spamming
            if frame_counter % 10 == 0: # Print every 10 frames
                print(f" Status: Limit={current_speed_limit} | Req={speed_motor_requested} | Sent={last_speed_sent} | Steer={int(round(steering))} | CTE={cte_f:.2f}")
                print(f" FPS: {instant_fps:.1f} (Avg: {overall_fps:.1f})")
                # print("-" * 30)

            # 11. Display Lane Finding Image (Needed for waitKey)
            if warp_result is not None:
                display_roi = cv2.cvtColor(roi_haha, cv2.COLOR_GRAY2BGR)
                # Draw calculated center line and actual center for visualization
                center_img = im_width // 2
                cv2.line(display_roi, (center_img, im_height), (center_img, im_height - 50), (0, 0, 255), 1) # Center line marker red
                if left_point != -1 or right_point != -1:
                     if left_point == -1: mid_point_viz = int(right_point - lane_width_max / 2)
                     elif right_point == -1: mid_point_viz = int(left_point + lane_width_max / 2)
                     else: mid_point_viz = int((right_point + left_point) / 2)
                     cv2.line(display_roi, (mid_point_viz, im_height), (mid_point_viz, im_height - 50), (0, 255, 0), 1) # Mid point marker green

                # Add text overlays
                cv2.putText(display_roi, f"FPS: {instant_fps:.1f}", (display_roi.shape[1] - 120, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
                cv2.putText(display_roi, f"CTE: {cte_f:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
                cv2.putText(display_roi, f"Steer: {int(round(steering))}", (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
                cv2.putText(display_roi, f"Speed Cmd: {last_speed_sent}", (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
                cv2.putText(display_roi, f"Limit: {current_speed_limit}", (10, 105), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1)
                if stop_condition_met: cv2.putText(display_roi, "STOP", (display_roi.shape[1] // 2 - 40 , display_roi.shape[0] // 2), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 3)

                # Display detected signs on the ROI image
                sign_y_offset = 130
                for sign in detected_signs:
                     sign_text = f"{sign['name']} ({sign['confidence']:.2f})"
                     cv2.putText(display_roi, sign_text, (10, sign_y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                     sign_y_offset += 20

                cv2.imshow("Lane Detection ROI", display_roi)

            # Optional: Display original frame with bounding boxes (requires modification of detect_signs)
            # cv2.imshow("Original Frame", frame)

    except KeyboardInterrupt: print("\nCtrl+C detected. Exiting...")
    finally:
        # --- Cleanup ---
        print("Cleaning up resources...")
        # <<< --- MODIFIED: Cleanup with String Format --- >>>
        data = b'<' + struct.pack('<ii', 90, 0) + b'>'
        ser.write(data)
        ser.flush()
        # <<< --- End MODIFIED Cleanup --- >>>

        if 'piCam' in globals() and piCam.started:
            try: piCam.stop(); print("Picamera2 stopped.")
            except Exception as e: print(f"Error stopping Picamera2: {e}")
        cv2.destroyAllWindows(); print("OpenCV windows closed."); print("Cleanup complete.")

if __name__ == "__main__":
    main()