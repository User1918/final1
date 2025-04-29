# -*- coding: utf-8 -*-
import cv2
import numpy as np
from picamera2 import Picamera2
import time
import ncnn # Import the ncnn Python binding
import os   # To check if model files exist

# --- Configuration ---
# NCNN Model Details
PARAM_PATH = "/home/labthayphuc/Desktop/yolov11ncnn/yolov11n_int8_final2.param"
BIN_PATH = "/home/labthayphuc/Desktop/yolov11ncnn/yolov11n_int8_final2.bin"
NCNN_INPUT_SIZE = 640          # Input size the model expects
NCNN_NUM_CLASSES = 21          # Number of classes your model detects
NCNN_CLASS_NAMES = [           # List of class names in order
    'Pedestrian Crossing', 'Radar', 'Speed Limit -100-', 'Speed Limit -120-',
    'Speed Limit -20-', 'Speed Limit -30-', 'Speed Limit -40-', 'Speed Limit -50-',
    'Speed Limit -60-', 'Speed Limit -70-', 'Speed Limit -80-', 'Speed Limit -90-',
    'Stop Sign', 'Traffic Light -Green-', 'Traffic Light -Off-',
    'Traffic Light -Red-', 'Traffic Light -Yellow-','Pessoa','car',
    'dotted-line','line'
]

# Inference Parameters
NCNN_CONFIDENCE_THRESHOLD = 0.35 # Minimum confidence to consider a detection
NCNN_NMS_THRESHOLD = 0.35      # Threshold for Non-Maximum Suppression

# NCNN Input/Output Layer Names (Usually default, check if different)
NCNN_INPUT_NAME = "in0"
NCNN_OUTPUT_NAME = "out0"

# Preprocessing Parameters (Usually standard for YOLO models)
NCNN_MEAN_VALS = [0.0, 0.0, 0.0]
NCNN_NORM_VALS = [1/255.0, 1/255.0, 1/255.0]

# Camera Configuration
PICAM_SIZE = (640, 480) # Capture resolution (W, H)
PICAM_FRAMERATE = 30    # Target framerate

# --- Helper Functions ---

def preprocess(frame_bgr, target_size):
    """
    Prepares the input frame for NCNN inference.
    Resizes with padding, normalizes, and returns ncnn.Mat and scale info.
    """
    original_height, original_width = frame_bgr.shape[:2]
    if original_height == 0 or original_width == 0:
        print("Error: Invalid frame dimensions received in preprocess.")
        return None, 0, 0, 0 # Return None for mat and zeros for scale/padding

    # Calculate scaling factor and new dimensions
    scale = min(target_size / original_width, target_size / original_height)
    new_w = int(original_width * scale)
    new_h = int(original_height * scale)

    # Ensure new dimensions are valid
    if new_w <= 0 or new_h <= 0:
        print(f"Error: Invalid resized dimensions ({new_w}x{new_h}).")
        return None, 0, 0, 0

    # Resize the image
    resized_img = cv2.resize(frame_bgr, (new_w, new_h))

    # Create padded image (gray background)
    padded_img = np.full((target_size, target_size, 3), 114, dtype=np.uint8)
    dw = (target_size - new_w) // 2
    dh = (target_size - new_h) // 2
    padded_img[dh:dh+new_h, dw:dw+new_w, :] = resized_img

    # Create NCNN Mat from the padded image
    mat_in = ncnn.Mat.from_pixels(padded_img, ncnn.Mat.PixelType.PIXEL_BGR, target_size, target_size)

    # Apply normalization
    mat_in.substract_mean_normalize(NCNN_MEAN_VALS, NCNN_NORM_VALS)

    return mat_in, scale, dw, dh

def postprocess(mat_out, scale, dw, dh, original_width, original_height):
    """
    Processes the NCNN output Mat to get final detections.
    Handles shape checking, decodes boxes, applies NMS, scales boxes back.
    """
    detections_results = []
    output_data = np.array(mat_out)

    # --- Adjust expected dimension based on model output (4 coords + 21 classes) ---
    expected_output_dim = 4 + NCNN_NUM_CLASSES # 4 + 21 = 25

    # --- Handle potential output shapes ---
    # Shape might be (1, N, 25) -> Reshape to (N, 25)
    if len(output_data.shape) == 3 and output_data.shape[0] == 1:
         output_data = output_data[0]
    # Shape might be (25, N) -> Transpose to (N, 25)
    elif len(output_data.shape) == 2 and output_data.shape[0] == expected_output_dim:
         output_data = output_data.T

    # --- Final shape check ---
    if len(output_data.shape) != 2 or output_data.shape[1] != expected_output_dim:
        print(f"Error: Unexpected NCNN output shape after processing: {output_data.shape}. Expected (N, {expected_output_dim})")
        return detections_results # Return empty list if shape is wrong

    num_detections = output_data.shape[0]
    boxes = []
    confidences = []
    class_ids = []

    # Decode each potential detection
    for i in range(num_detections):
        detection = output_data[i] # Shape (25,)
        # Class scores are the elements after the box coordinates
        class_scores = detection[4:] # Shape (21,)

        # Find the class with the highest score and its score value
        class_id = np.argmax(class_scores)
        max_class_score = np.max(class_scores)

        # Use the highest class score as the confidence for filtering
        final_confidence = max_class_score

        # Filter based on confidence threshold
        if final_confidence >= NCNN_CONFIDENCE_THRESHOLD:
            # Extract box coordinates (relative to NCNN_INPUT_SIZE)
            cx, cy, w_ncnn, h_ncnn = detection[:4]

            # --- Scale box back to ORIGINAL image coordinates ---
            x1_resized = (cx - w_ncnn / 2)
            y1_resized = (cy - h_ncnn / 2)
            x2_resized = (cx + w_ncnn / 2)
            y2_resized = (cy + h_ncnn / 2)

            # Remove padding offset
            x1_nopad = x1_resized - dw
            y1_nopad = y1_resized - dh
            x2_nopad = x2_resized - dw
            y2_nopad = y2_resized - dh

            # Rescale to original image size (check scale to avoid division by zero)
            if scale > 1e-6:
                x1_orig = x1_nopad / scale
                y1_orig = y1_nopad / scale
                x2_orig = x2_nopad / scale
                y2_orig = y2_nopad / scale
            else:
                 x1_orig, y1_orig, x2_orig, y2_orig = 0, 0, 0, 0

            # Calculate final width/height and clamp coordinates
            w_orig = x2_orig - x1_orig
            h_orig = y2_orig - y1_orig
            x1_orig = max(0, x1_orig)
            y1_orig = max(0, y1_orig)
            w_orig = min(original_width - x1_orig - 1, w_orig)
            h_orig = min(original_height - y1_orig - 1, h_orig)
            w_orig = max(0, w_orig)
            h_orig = max(0, h_orig)

            # Store data needed for NMS
            boxes.append([int(x1_orig), int(y1_orig), int(w_orig), int(h_orig)])
            confidences.append(float(final_confidence))
            class_ids.append(class_id)

    # Apply Non-Maximum Suppression
    indices = []
    if boxes: # Only run NMS if there are candidate boxes
        indices = cv2.dnn.NMSBoxes(boxes, confidences, NCNN_CONFIDENCE_THRESHOLD, NCNN_NMS_THRESHOLD)

    # Create the final list of detections after NMS
    if len(indices) > 0:
        if isinstance(indices, (list, tuple)) and len(indices) > 0 and isinstance(indices[0], (list, np.ndarray)):
            indices = indices.flatten() # Flatten if needed
        processed_indices = set()
        for idx in indices:
            i = int(idx)
            if 0 <= i < len(boxes) and i not in processed_indices:
                box = boxes[i]
                x, y, w, h = box
                confidence_nms = confidences[i]
                class_id_nms = class_ids[i]
                processed_indices.add(i)

                # Get class name
                if 0 <= class_id_nms < len(NCNN_CLASS_NAMES):
                    class_name = NCNN_CLASS_NAMES[class_id_nms]
                else:
                    class_name = f"ID:{class_id_nms}" # Fallback

                detections_results.append({
                    "name": class_name,
                    "confidence": confidence_nms,
                    "box": box, # [x, y, w, h]
                })
    return detections_results

def draw_detections(frame, detections):
    """Draws bounding boxes and labels on the frame."""
    for det in detections:
        box = det['box']
        x, y, w, h = box
        label = f"{det['name']}: {det['confidence']:.2f}"
        color = (0, 255, 0) # Green for all detections in this test script

        # Draw rectangle
        cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)

        # Draw label background
        (label_width, label_height), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        label_y = max(y, label_height + 5) # Ensure label is within frame top
        cv2.rectangle(frame, (x, label_y - label_height - baseline), (x + label_width, label_y), color, cv2.FILLED)
        # Draw label text (black text on green background)
        cv2.putText(frame, label, (x, label_y - baseline // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)

# ==============================================================================
# ========================== MAIN EXECUTION BLOCK ============================
# ==============================================================================
if __name__ == "__main__":

    # --- Check if model files exist ---
    if not os.path.exists(PARAM_PATH):
        print(f"CRITICAL ERROR: Param file not found at {PARAM_PATH}")
        exit()
    if not os.path.exists(BIN_PATH):
        print(f"CRITICAL ERROR: Bin file not found at {BIN_PATH}")
        exit()

    # --- Initialize NCNN Net ---
    print("Initializing NCNN Net...")
    net = ncnn.Net()
    # Configure compute options (CPU is generally safer/easier on Pi)
    net.opt.use_vulkan_compute = False # Set to True ONLY if Vulkan is properly set up
    if not net.opt.use_vulkan_compute:
        net.opt.num_threads = 4 # Use 4 threads for Pi 5 CPU
        print(f"NCNN using {net.opt.num_threads} CPU threads.")
    else:
         print("NCNN attempting to use Vulkan GPU.")
    # Load model
    print("Loading NCNN model...")
    if net.load_param(PARAM_PATH) != 0:
        print(f"CRITICAL ERROR: Failed to load NCNN param file: {PARAM_PATH}"); exit()
    if net.load_model(BIN_PATH) != 0:
        print(f"CRITICAL ERROR: Failed to load NCNN model file: {BIN_PATH}"); exit()
    print("NCNN model loaded successfully.")

    # --- Initialize Picamera2 ---
    print("Initializing Picamera2...")
    piCam = Picamera2()
    try:
        preview_config = piCam.create_preview_configuration(main={"size": PICAM_SIZE, "format": "RGB888"})
        piCam.configure(preview_config)
        piCam.set_controls({"FrameRate": float(PICAM_FRAMERATE)})
        piCam.start()
        time.sleep(1.0) # Allow camera to settle
        print("Picamera2 started.")
    except Exception as e:
        print(f"CRITICAL ERROR: Picamera2 initialization failed: {e}")
        if 'piCam' in locals() and piCam.started: piCam.stop()
        exit()

    # --- Main Inference Loop ---
    print("\nStarting inference loop... Press 'q' in OpenCV window to exit.")
    frame_count = 0
    start_time = time.time()
    fps = 0

    try:
        while True:
            loop_start = time.time()

            # 1. Capture Frame
            try:
                frame_bgr = piCam.capture_array()
                if frame_bgr is None:
                    print("Warning: Failed to capture frame.")
                    time.sleep(0.1) # Wait a bit before retrying
                    continue
            except Exception as e:
                 print(f"Error during frame capture: {e}")
                 time.sleep(0.1)
                 continue

            # Ensure frame is BGR
            # (Picamera2 with RGB888 format should already be BGR compatible with OpenCV)
            # frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR) # Only needed if format was RGB

            # 2. Preprocess Frame for NCNN
            mat_in, scale, dw, dh = preprocess(frame_bgr, NCNN_INPUT_SIZE)
            if mat_in is None:
                print("Error: Preprocessing failed.")
                continue # Skip inference if preprocessing fails

            # 3. Run NCNN Inference
            try:
                ex = net.create_extractor()
                ex.input(NCNN_INPUT_NAME, mat_in)
                ret, mat_out = ex.extract(NCNN_OUTPUT_NAME)
                if ret != 0:
                    print(f"Error: NCNN extraction failed with code {ret}")
                    continue # Skip postprocessing if extraction fails
            except Exception as e:
                 print(f"Error during NCNN inference: {e}")
                 continue

            # 4. Postprocess Output
            original_h, original_w = frame_bgr.shape[:2]
            detections = postprocess(mat_out, scale, dw, dh, original_w, original_h)

            # 5. Draw Detections
            draw_detections(frame_bgr, detections)

            # 6. Calculate and Display FPS
            frame_count += 1
            loop_end = time.time()
            loop_time = loop_end - loop_start
            if loop_time > 0:
                 instant_fps = 1.0 / loop_time
            else:
                 instant_fps = float('inf')

            # Calculate average FPS over a window (e.g., last 1 second)
            if time.time() - start_time >= 1.0:
                fps = frame_count / (time.time() - start_time)
                frame_count = 0
                start_time = time.time()

            # Display FPS on frame
            fps_text = f"FPS: {fps:.1f} (Inst: {instant_fps:.1f})"
            cv2.putText(frame_bgr, fps_text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

            # 7. Show Frame
            cv2.imshow("NCNN Detection Test (Pi 5)", frame_bgr)

            # 8. Check for Exit Key
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("Exit key 'q' pressed.")
                break

    except KeyboardInterrupt:
        print("\nCtrl+C detected. Exiting...")
    finally:
        # --- Cleanup ---
        print("Cleaning up...")
        if 'piCam' in locals() and piCam.started:
            piCam.stop()
            print("Picamera2 stopped.")
        cv2.destroyAllWindows()
        print("OpenCV windows closed.")
        print("Test script finished.")
