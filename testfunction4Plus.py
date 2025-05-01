import cv2
import numpy as np
from picamera2 import Picamera2
import time
import serial
import struct
import ncnn  # Import the ncnn Python binding
import re    # Import regular expressions for parsing speed limits


# NCNN Traffic Sign Detection Config
PARAM_PATH = "/home/labthayphuc/Desktop/yolov11ncnn/yolov11n_int8_final.param" 
BIN_PATH = "/home/labthayphuc/Desktop/yolov11ncnn/yolov11n_int8_final.bin"     
NCNN_INPUT_SIZE = 640
NCNN_CONFIDENCE_THRESHOLD = 0.35
NCNN_NMS_THRESHOLD = 0.35
NCNN_NUM_CLASSES = 19
NCNN_CLASS_NAMES = [
    "Pedestrian Crossing", "Radar", "Speed Limit -100-", "Speed Limit -120-",
    "Speed Limit -20-", "Speed Limit -30-", "Speed Limit -40-", "Speed Limit -50-",
    "Speed Limit -60-", "Speed Limit -70-", "Speed Limit -80-", "Speed Limit -90-",
    "Stop Sign", "Traffic Light -Green-", "Traffic Light -Off-",
    "Traffic Light -Red-", "Traffic Light -Yellow-","Person","Car"
]
NCNN_INPUT_NAME = "in0"
NCNN_OUTPUT_NAME = "out0"
NCNN_MEAN_VALS = [0.0, 0.0, 0.0]
NCNN_NORM_VALS = [1/255.0, 1/255.0, 1/255.0]
MIN_SIGN_WIDTH_OR_HEIGHT = 100

# Lane Keeping & Motor Control Config
SERIAL_PORT = '/dev/ttyUSB0'      # <<< ADJUST
BAUD_RATE = 115200                # <<< MATCH ESP32
PICAM_SIZE = (640, 480)           # MUST BE 640x480 for NCNN preprocessing
PICAM_FRAMERATE = 30

# Perspective Warp Points (from Script 2)
leftTop = 160
heightTop = 150
rightTop = 160
leftBottom = 20
heightBottom = 380
rightBottom = 20

# Speed Control Config
DEFAULT_MAX_SPEED = 120
MANUAL_MAX_SPEED = 100
MANUAL_ACCELERATION = 5
AUTO_MODE_SPEED_STRAIGHT = 30
AUTO_MODE_SPEED_CURVE = 20

# --- Global Variables ---
# ... (Keep all global variables as they were) ...
frame = None
imgWarp = None
net = None
ser = None
new_limit_value = -1
steering = 90
speed_motor_requested = 0
current_speed_limit = DEFAULT_MAX_SPEED
flag = 1
#cte_f = 0.0
#left_point = -1
#right_point = -1
# Khá»Ÿi táº¡o biáº¿n lÆ°u sai lá»‡ch trÆ°á»›c Ä‘Ã³ cho lá»c IIR
pre_diff = 0
lane_width = 500 # Keep initial estimate
lane_width_max = 500
interested_line_y = 0 # Will be updated in SteeringAngle
#im_height = 0         # Will be updated
#im_width = 0          # Will be updated
frame_counter = 0
loop_start_time = 0
overall_fps = 0.0


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
    piCam.preview_configuration.main.format = 'RGB888'
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
    frame = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2BGR)

    # Continue with lane processing using grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (11, 11), 0)
    edges = cv2.Canny(blur, 100, 150, apertureSize=3) # Settings from script 2
    h, w = edges.shape
    # Use perspective points from Script 2
    points = np.float32([(leftTop, heightTop), (w - rightTop, heightTop),(leftBottom, heightBottom), (w - rightBottom, heightBottom)])
    pts2 = np.float32([[0, 0], [w, 0], [0, h], [w, h]])

    try:
        matrix = cv2.getPerspectiveTransform(points, pts2)
        imgWarp = cv2.warpPerspective(edges, matrix, (w, h))
    except cv2.error as e:
        print(f"Warp Error: {e}")
        imgWarp = None
        return False
    contours, hierarchy = cv2.findContours(imgWarp, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(imgWarp, contours, -1, (255, 255, 255), 3) 
    lines = cv2.HoughLinesP(imgWarp, 1, np.pi/180, threshold=150, minLineLength=70, maxLineGap=0)
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(imgWarp, (x1, y1), (x2, y2), (255, 255, 255), 3)  
    return True # Indicate success
    
# --- Applying values of column and row
def ve_truc_anh(img, step=100):
    """ Váº½ trá»¥c tá»a Ä‘á»™ vÃ  cÃ¡c má»‘c tá»a Ä‘á»™ lÃªn áº£nh """
    h, w = img.shape[:2]
    
    img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR) 


    cv2.line(img, (0, h - 1), (w, h - 1), (200, 200, 200), 1)
    cv2.line(img, (0, 0), (0, h), (200, 200, 200), 1)

    for x in range(0, w, step):
        cv2.line(img, (x, h - 10), (x, h), (200, 200, 200), 1)
        cv2.putText(img, str(x), (x + 2, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)


    for y in range(0, h, step):
        cv2.line(img, (0, y), (10, y), (200, 200, 200), 1)
        cv2.putText(img, str(y), (12, y + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
    return img


# --- Restored SteeringAngle Function from Script 2 ---
def SteeringAngle():
    """TÃ­nh sai lá»‡ch lÃ¡i (CTE) dá»±a trÃªn quÃ©t cÃ¡c hÃ ng pixel cá»§a áº£nh warp."""
    global left_point, right_point, interested_line_y
    global im_height, im_width, lane_width, lane_width_max, cte_f
    # Láº¥y kÃ­ch thÆ°á»›c hÃ¬nh warp hiá»‡n táº¡i
    im_height, im_width = imgWarp.shape[:2]
    # Khá»Ÿi táº¡o láº¡i giÃ¡ trá»‹ Ä‘iá»ƒm trÃ¡i vÃ  pháº£i cho má»—i frame
    left_point = -1
    right_point = -1
    # TÃ­nh tÃ¢m áº£nh theo chiá»u ngang
    center_img = im_width // 2
    # Äáº·t má»©c quÃ©t Y (tá»« dÆ°á»›i lÃªn) Ä‘á»ƒ tÃ­nh CTE
    set_point = 300
    # Váº½ Ä‘Æ°á»ng target line lÃªn áº£nh Ä‘á»ƒ debug
    cv2.line(imgWarp, (0, set_point), (im_width, set_point), (255, 0, 0), 2)
    # Khá»Ÿi táº¡o biáº¿n lÆ°u sai lá»‡ch trÆ°á»›c Ä‘Ã³ cho lá»c IIR
    pre_diff = 0
    # BÆ°á»›c nháº£y má»—i láº§n quÃ©t hÃ ng ngang (pixel)
    step = -5
    # TÃ­nh sá»‘ bÆ°á»›c quÃ©t u = khoáº£ng cÃ¡ch quÃ©t / |step|
    u = (im_height - set_point) / abs(step)
    # TÃ­nh há»‡ sá»‘ trá»ng sá»‘ cho má»—i hÃ ng quÃ©t
    ki = (-u / im_height) + 1
    # VÃ²ng láº·p quÃ©t cÃ¡c hÃ ng ngang tá»« Ä‘Ã¡y hÃ¬nh lÃªn set_point
    for i in range(im_height - 1, set_point, step):
        #  GÃ¡n chá»‰ sá»‘ hÃ ng hiá»‡n táº¡i
        interested_line_y = int(i)
        # Láº¥y dÃ²ng pixel táº¡i y
        interested_line = imgWarp[interested_line_y, :]
        #  TÃ¬m Ä‘iá»ƒm lane bÃªn trÃ¡i gáº§n tÃ¢m (center_img)
        for x in range(center_img, 0, -1):
            if interested_line[x] > 0:
                left_point = x
                break
        # TÃ¬m Ä‘iá»ƒm lane bÃªn pháº£i
        for x in range(center_img + 1, im_width):
            if interested_line[x] > 0:
                right_point = x
                break
        # Cáº­p nháº­t Ä‘á»™ rá»™ng lane náº¿u cáº£ hai Ä‘iá»ƒm Ä‘á»u tÃ¬m Ä‘Æ°á»£c
        if left_point != -1 and right_point != -1:
            lane_width = right_point - left_point
        # TÃ­nh midpoint cá»§a lane
        mid_point = (right_point + left_point) / 2
        # Sai lá»‡ch so vá»›i center
        diff = center_img - mid_point
        # Ãp dá»¥ng lá»c IIR: current_diff = diff * ki + pre_diff
        diff = diff * ki + pre_diff
        # Cáº­p nháº­t pre_diff cho vÃ²ng láº·p káº¿ tiáº¿
        pre_diff = diff
    # TÃ­nh Cross-Track Error (CTE) chuáº©n hÃ³a
    cte_f = diff / u

def signal_motor(key):
    """ Updates the *requested* motor speed based on key input. """
    global speed_motor_requested, flag
    new_speed_req = speed_motor_requested
    if key == ord('w'):
        new_speed_req = min(speed_motor_requested + MANUAL_ACCELERATION, MANUAL_MAX_SPEED)
        flag = 1
    elif key == ord('s'):
        new_speed_req = max(speed_motor_requested - MANUAL_ACCELERATION, 0)
    elif key == ord('a'): # Auto mode speed based on curve
        target_speed = AUTO_MODE_SPEED_STRAIGHT if abs(cte_f) < 10 else AUTO_MODE_SPEED_CURVE
        new_speed_req = target_speed
        flag = 1
    elif key == ord('x'): # Stop
        new_speed_req = 0
    if new_speed_req != speed_motor_requested:
        speed_motor_requested = new_speed_req


# --- PID Class (Using Script 1's version for robustness with delta_time) ---
class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.error_previous = 0
        self.integral = 0
        
    def update(self, error_f):
        sampling_time = 0.05
        inv_sampling_time = 1 / sampling_time
        derivative = (  error_f - self.error_previous) * inv_sampling_time
        self.integral = self.integral + error_f* sampling_time
        self.error_previous = error_f
        steering_angle = round((self.kp * error_f +self.ki * self.integral +self.kd * derivative))
        steering = round(steering_angle + 90) #VÃ¬ setup gÃ³c á»Ÿ giá»¯a cá»§a servo lÃ  90 nÃªn cá»™ng vá»›i 90 trÆ°á»›c khi gá»­i vá» arduino
        steering = max(70, min(110, steering))  # Giá»›i háº¡n gÃ³c lÃ¡i tá»« 0 Ä‘áº¿n 180 Ä‘á»™
        steering_servo_angle = steering
        return steering_servo_angle


# --- NCNN Detection Function (Copied from Script 1) ---
def detect_signs_and_get_results(input_frame_bgr):
    """ Performs NCNN traffic sign detection. Returns list of results. """
    # ... (Keep detect_signs_and_get_results function exactly as it was) ...
    detections_results = []
    if input_frame_bgr is None or net is None: return detections_results
    original_height, original_width = input_frame_bgr.shape[:2]
    if original_width != 640 or original_height != 480 or NCNN_INPUT_SIZE != 640:
        print(f"ERROR: detect_signs expects 640x480 input, 640 NCNN size for this preprocessing.")
        return detections_results
    try:
        padded_img = np.full((NCNN_INPUT_SIZE, NCNN_INPUT_SIZE, 3), 114, dtype=np.uint8)
        scale = min(NCNN_INPUT_SIZE / original_width, NCNN_INPUT_SIZE / original_height)
        new_w, new_h = int(original_width * scale), int(original_height * scale)
        dw = (NCNN_INPUT_SIZE - original_width) // 2
        dh = (NCNN_INPUT_SIZE - original_height) // 2
        padded_img[dh:dh+original_height, dw:dw+original_width, :] = input_frame_bgr
        mat_in = ncnn.Mat.from_pixels(padded_img, ncnn.Mat.PixelType.PIXEL_BGR, NCNN_INPUT_SIZE, NCNN_INPUT_SIZE)
        mat_in.substract_mean_normalize(NCNN_MEAN_VALS, NCNN_NORM_VALS)
    except Exception as e:
        print(f"NCNN Preprocessing Error: {e}")
        return detections_results
    try:
        ex = net.create_extractor()
        ex.input(NCNN_INPUT_NAME, mat_in)
        ret_extract, mat_out = ex.extract(NCNN_OUTPUT_NAME)
        if ret_extract != 0: print("NCNN Extract Error"); return detections_results
    except Exception as e: print(f"NCNN Inference Error: {e}"); return detections_results
    output_data = np.array(mat_out)
    if len(output_data.shape) == 3 and output_data.shape[0] == 1: 
        output_data = output_data[0]
    if len(output_data.shape) == 2 and output_data.shape[0] == (NCNN_NUM_CLASSES + 4): 
        output_data = output_data.T
    if len(output_data.shape) != 2: 
        print("Unexpected output shape"); return detections_results
    num_detections, detection_size = output_data.shape
    expected_size = 4 + NCNN_NUM_CLASSES
    if detection_size != expected_size: print("Unexpected detection size"); return detections_results
    boxes = []
    confidences = []
    class_ids = []
    for i in range(num_detections):
        detection = output_data[i]
        class_scores = detection[4:]
        confidence = np.max(class_scores)
        class_id = np.argmax(class_scores)
        if confidence >= NCNN_CONFIDENCE_THRESHOLD:
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




def main():
    # Khai bÃ¡o sá»­ dá»¥ng biáº¿n toÃ n cá»¥c Ä‘Ã£ Ä‘á»‹nh nghÄ©a tá»« trÆ°á»›c
    global steering, speed_motor_requested, flag, current_speed_limit, net, ser, frame_counter, loop_start_time, overall_fps

    # Kiá»ƒm tra mÃ´ hÃ¬nh NCNN Ä‘Ã£ Ä‘Æ°á»£c khá»Ÿi táº¡o hay chÆ°a
    if net is None:
        print("CRITICAL: NCNN Net object not initialized!")  # In lá»—i nghiÃªm trá»ng
        return  # Dá»«ng chÆ°Æ¡ng trÃ¬nh
    # Kiá»ƒm tra cá»•ng Serial (ESP32) cÃ³ sáºµn hay khÃ´ng
    if ser is None:
        print("WARNING: Serial port not available. Motor commands will not be sent.")  # Cáº£nh bÃ¡o

    # Khá»Ÿi táº¡o 3 bá»™ PID vá»›i há»‡ sá»‘ khÃ¡c nhau cho cÃ¡c tráº¡ng thÃ¡i di chuyá»ƒn
    pid_controller1 = PID(0.35, 0.001, 0.5)  
    pid_controller2 = PID(1, 0.001, 0)      
    pid_controller3 = PID(1, 0.001, 0)       

    # Khá»Ÿi táº¡o bá»™ Ä‘áº¿m, thá»i gian báº¯t Ä‘áº§u Ä‘á»ƒ tÃ­nh FPS
    frame_counter = 0
    loop_start_time = time.time()
    overall_fps = 0.0
    

    try:
        # VÃ²ng láº·p chÃ­nh
        while True:
            # ÄÃ¡nh dáº¥u thá»i gian báº¯t Ä‘áº§u frame Ä‘á»ƒ tÃ­nh FPS
            frame_start_time = time.time()

            # Capture vÃ  preprocess áº£nh cho lane
            if not process_image():
                time.sleep(0.01)  # Äá»£i 10ms náº¿u capture lá»—i rá»“i tiáº¿p tá»¥c
                continue

            # TÃ­nh sai lá»‡ch lÃ¡i (CTE)
            SteeringAngle()  # Cáº­p nháº­t biáº¿n cte_f toÃ n cá»¥c

            # PhÃ¡t hiá»‡n biá»ƒn bÃ¡o giao thÃ´ng
            detected_signs = detect_signs_and_get_results(frame)

            # Xá»­ lÃ½ phÃ­m nháº¥n tá»« ngÆ°á»i dÃ¹ng
            key = cv2.waitKey(1) & 0xFF
            signal_motor(key)  # Cáº­p nháº­t speed_motor_requested
            if key == ord('q'):
                print("'q' pressed, exiting loop.")
                break  # ThoÃ¡t vÃ²ng láº·p

            # Xá»­ lÃ½ káº¿t quáº£ phÃ¡t hiá»‡n biá»ƒn bÃ¡o
            stop_condition_met = False
            limit_sign_seen_this_frame = False
            new_limit_value = -1
            active_signs_this_frame = []
            if detected_signs:
                for sign in detected_signs:
                    sign_w = sign.get('width', 0)
                    sign_h = sign.get('height', 0)
                    # Kiá»ƒm tra kÃ­ch thÆ°á»›c biá»ƒn bÃ¡o cÃ³ Ä‘á»§ lá»›n khÃ´ng
                    if sign_w > MIN_SIGN_WIDTH_OR_HEIGHT or sign_h > MIN_SIGN_WIDTH_OR_HEIGHT:
                        active_signs_this_frame.append(sign)
                        sign_name = sign['name']
                        # Náº¿u lÃ  biá»ƒn STOP hoáº·c Ä‘á», dá»«ng ngay
                        if sign_name in ["Stop Sign", "Traffic Light -Red-"]:
                            stop_condition_met = True
                            continue
                        # Náº¿u lÃ  biá»ƒn giá»›i háº¡n tá»‘c Ä‘á»™, parse giÃ¡ trá»‹
                        parsed_limit = parse_speed_limit(sign_name)
                        limit_sign_seen_this_frame = True
                        if new_limit_value == -1 or parsed_limit < new_limit_value:
                            new_limit_value = parsed_limit

            # Cáº­p nháº­t giá»›i háº¡n tá»‘c Ä‘á»™ náº¿u cÃ³ biá»ƒn má»›i
            if limit_sign_seen_this_frame and new_limit_value != current_speed_limit:
                current_speed_limit = new_limit_value

            # Determine Final Commanded Speed (RPM)
            final_speed_command_vel = speed_motor_requested
            final_speed_command_vel = min(final_speed_command_vel, current_speed_limit)
            if stop_condition_met:
                if final_speed_command_vel > 0:
                     print("** Stop condition met (large sign)! Setting speed to 0. **")
                final_speed_command_vel = 0
            final_speed_command_rpm = transform_speed(final_speed_command_vel)

            # TÃ­nh gÃ³c lÃ¡i qua PID dá»±a trÃªn rpm
            if final_speed_command_vel >= 20 :
                steering = pid_controller1.update(cte_f)
            elif 35 > final_speed_command_vel > 20:
                steering = pid_controller2.update(cte_f)
            else:
                steering = pid_controller3.update(cte_f)

            # Gá»­i lá»‡nh lÃªn ESP32 qua Serial
            if ser:
                try:
                    data_to_send = struct.pack('<ii', steering, final_speed_command_rpm)
                    ser.write(b'<' + data_to_send + b'>')
                    ser.flush()
                except Exception as e:
                    print(f"Serial Write Error: {e}")
                    ser.close()
                    ser = None
                    print("Serial disabled.")

            # TÃ­nh vÃ  in FPS má»—i 30 frame
            frame_counter += 1
            frame_time = time.time() - frame_start_time
            instant_fps = 1.0 / frame_time if frame_time > 0 else 0
            elapsed_time_total = time.time() - loop_start_time
            if elapsed_time_total > 1:
                overall_fps = frame_counter / elapsed_time_total
            if frame_counter % 30 == 0:
                print(f"Status: Limit={current_speed_limit} | Req={speed_motor_requested} vel | "
                      f"Sent={final_speed_command_rpm} rpm | Steer={steering} | CTE={cte_f:.2f}")
                print(f"FPS: {instant_fps:.1f} (Avg: {overall_fps:.1f})")

            # Hiá»ƒn thá»‹ áº£nh debug lane vÃ  overlay thÃ´ng sá»‘
            if imgWarp is not None:
                display_img = ve_truc_anh(imgWarp, step=50)
                im_h, im_w = display_img.shape[:2]
                center_img_viz = im_w // 2
                display_y = interested_line_y if interested_line_y > 0 else im_h // 2
                # Váº½ tÃ¢m áº£nh
                cv2.line(display_img, (center_img_viz, display_y-20), (center_img_viz, display_y+20), (0,0,255), 1)
                # Váº½ Ä‘iá»ƒm lane hai bÃªn
                if left_point != -1:
                    cv2.circle(display_img, (left_point, im_h-1), 5, (0,255,255), -1)
                if right_point != -1:
                    cv2.circle(display_img, (right_point, im_h-1), 5, (0,255,255), -1)
                # Váº½ Ä‘Æ°á»ng mid-point theo CTE
                if left_point != -1 or right_point != -1:
                    mid_point_viz = int(center_img_viz - cte_f)
                    cv2.line(display_img, (mid_point_viz, display_y-20), (mid_point_viz, display_y+20), (255,0,0), 2)
                # Overlay text thÃ´ng sá»‘
                cv2.putText(display_img, f"FPS: {instant_fps:.1f}", (im_w-120,30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255),1)
                cv2.putText(display_img, f"CTE: {cte_f:.2f}",   (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0),1)
                cv2.putText(display_img, f"Steer Cmd: {steering}", (10,55), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0),1)
                cv2.putText(display_img, f"RPM Cmd: {final_speed_command_rpm} RPM", (10,80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0),1)
                cv2.putText(display_img, f"(Speed {speed_motor_requested} km/h)", (10,105), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,200,0),1)
                cv2.putText(display_img, f"Limit: {current_speed_limit}", (10,130), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255),1)
                # Hiá»ƒn thá»‹ cÃ¡c biá»ƒn bÃ¡o phÃ¡t hiá»‡n
                sign_y_offset = 160
                for sign in detected_signs:
                    sign_text = f"{sign['name']} ({sign['confidence']:.2f})"
                    cv2.putText(display_img, sign_text, (10, sign_y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,0),1)
                    sign_y_offset += 20
                if stop_condition_met:
                    cv2.putText(display_img, "STOP", (im_w//2-50, im_h//2), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0,0,255),3)
                cv2.imshow("Lane Detection Warp", display_img)
                #cv2.imshow("frame",frame)

    except KeyboardInterrupt:
        print("\nCtrl+C detected. Exiting...")
    finally:
        # Dá»n dáº¹p khi thoÃ¡t
        print("Cleaning up resources...")
        if ser and ser.is_open:
            stop_data = struct.pack('<ii', 90, 0)
            ser.write(b'<' + stop_data + b'>')
            ser.flush(); time.sleep(0.1)
            ser.close(); print("Serial port closed.")
        if 'piCam' in globals() and piCam.started:
            piCam.stop(); print("Picamera2 stopped.")
        cv2.destroyAllWindows(); print("OpenCV windows closed.")
        print("Cleanup complete.")
if __name__ == "__main__":
    main()
