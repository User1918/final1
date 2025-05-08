import cv2
import numpy as np
from picamera2 import Picamera2
import time
import serial
import struct
import math

speed_motor_requested = 0
lane_width = 500
# Warp phá»‘i cáº£nh
leftTop, heightTop, rightTop       = 160, 150, 160
leftBottom, heightBottom, rightBottom = 20, 380, 20

# Tham sá»‘ EMA smoothing (ná»™i tuyáº¿n)
prev_cte_ema = 0.0    # GiÃ¡ trá»‹ EMA CTE cá»§a khung trÆ°á»›c
alpha_cte =  1     # Há»‡ sá»‘ lÃ m má»‹n EMA (0 < alpha_cte <= 1)

#thiáº¿t láº­p serial bus
ser = serial.Serial('/dev/ttyUSB0', 115200,timeout=1)
time.sleep(1)  # Äá»£i cá»•ng serial á»•n Ä‘á»‹nh

#Thiáº¿t láº­p thÃ´ng sá»‘ ban Ä‘áº§u cá»§a Pi camera1
piCam = Picamera2()
piCam.preview_configuration.main.size=(640,480) #Äá»™ phÃ¢n giáº£i 640x480
piCam.preview_configuration.main.format='RGB888'
piCam.preview_configuration.controls.FrameRate=60 #Tá»‘c Ä‘á»™ khung hÃ¬nh
piCam.preview_configuration.align()
piCam.configure('preview')
piCam.start()
#-------------------------------------------------------------------------------

#HÃ m láº¥y hÃ¬nh áº£nh tá»« camera vÃ  lÆ°u vÃ o biáº¿n "frame"
def picam():
    global canny_edge
    global gray
    frame = piCam.capture_array()
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY) #chuyá»ƒn qua áº£nh xÃ¡m
    blur = cv2.GaussianBlur(gray, (11,11), 0)
    thresh_low = 85
    thresh_high = 120
    canny_edge = cv2.Canny(blur, thresh_low, thresh_high)

#-------------------------------------------------------------------------------
def warpImg():
    global imgWarp
    h, w = canny_edge.shape
    pts1 = np.float32([(leftTop, heightTop), (w-rightTop, heightTop), (leftBottom, heightBottom), (w-rightBottom, heightBottom)])
    pts2 = np.float32([[0,0],[w,0],[0,h],[w,h]])
    M = cv2.getPerspectiveTransform(pts1, pts2)
    imgWarp = cv2.warpPerspective(canny_edge, M, (w,h))
    contours, hierarchy = cv2.findContours(imgWarp, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(imgWarp, contours, -1, (255, 255, 255), 3) 
    lines = cv2.HoughLinesP(imgWarp, 1, np.pi/180, threshold=100, minLineLength=70, maxLineGap=0)
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(imgWarp, (x1, y1), (x2, y2), (255, 255, 255), 3)  
    return True


def SteeringAngle():
    """TÃ­nh CTE báº±ng quÃ©t hÃ ng ngang"""
    global  prev_cte_ema, raw_cte, pre_diff, lane_width, left_point, right_point, alpha_cte, cte_filter
    h, w = imgWarp.shape
    center = w//2
    set_y = 351
    step = -5
    u = (h-set_y)/abs(step)
    ki = (-u/h) + 1
    diff = 0
    pre_diff = 0
    # quÃ©t tá»«ng hÃ ng ngang tá»« Ä‘Ã¡y lÃªn Ä‘áº¿n set_y
    for i in range(h - 1, set_y, step):
        row = imgWarp[i]
        # tÃ¬m Ä‘iá»ƒm trÃ¡i
        left_point = next((x for x in range(center, -1, -1) if row[x] > 0), -1)
        # tÃ¬m Ä‘iá»ƒm pháº£i
        right_point = next((x for x in range(center + 1, w) if row[x] > 0), -1)

        # náº¿u chá»‰ tÃ¬m tháº¥y má»™t bÃªn thÃ¬ dÃ¹ng lane_width_max Ä‘á»ƒ bÃ¹
        if left_point !=-1 and right_point !=-1:
            lane_width = right_point - left_point
        elif  left_point !=-1 and right_point ==-1:
            right_point =  left_point + lane_width
        elif right_point !=-1 and left_point ==-1:
            left_point = right_point - lane_width
        else:
            continue
        
        mid = (left_point + right_point)/2
        diff = center-mid
        diff = diff*ki + pre_diff
        pre_diff = diff        

    raw_cte = diff / u
        # Lá»c EMA qua cÃ¡c khung Ä‘á»ƒ mÆ°á»£t CTE
    cte_filter = alpha_cte * raw_cte + (1.0 - alpha_cte) * prev_cte_ema

    prev_cte_ema = cte_filter
    cte_rad = math.atan((cte_filter-center) / (set_y))
    cte_deg = math.degrees(cte_rad)
    cte_f = round(cte_deg*0.45)
    return cte_f

    # Lá»c EMA qua cÃ¡c khung Ä‘á»ƒ mÆ°á»£t CTE
    #cte_filter = alpha_cte * raw_cte + (1.0 - alpha_cte) * prev_cte_ema
    #cte_rad = math.atan((cte_filter-center) / (set_y))
    #cte_deg = math.degrees(cte_rad)
    #cte_f = round(cte_deg*0.45)

def signal_motor(key):
    """Äiá»u khiá»ƒn tá»‘c Ä‘á»™ qua phÃ­m w/s/x"""
    global speed_motor_requested
    if key == ord('w'): speed_motor_requested = min(speed_motor_requested+5, 50)
    if key == ord('s'): speed_motor_requested = max(speed_motor_requested-5, 0)
    if key == ord('x'): speed_motor_requested = 0
def transform_speed(vel):
    """Chuyá»ƒn km/h -> RPM Ä‘á»™ng cÆ¡"""
    f=vel/10
    rpm=(f*30*2.85)/(3.14*0.0475*3.6)
    return int(round(rpm))

class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.cte_previous = 0
        self.integral = 0

    def update(self, cte):
        self.cte_previous = cte
        sampling_time = 0.0005
        
        inv_sampling_time = 1 / sampling_time

        derivative = (cte - self.cte_previous) * inv_sampling_time
        
        self.integral = self.integral + cte * sampling_time
        
        steering_angle = self.kp * cte +self.ki * self.integral +self.kd * derivative

        steering = round(steering_angle + 90) #VÃ¬ setup gÃ³c á»Ÿ giá»¯a cá»§a servo lÃ  90 nÃªn cá»™ng vá»›i 90 trÆ°á»›c khi gá»­i vá» arduino
        
        return  steering

def main():

    kp = 0.5
    ki = 0.0008
    kd = 0.2
    prev_time = 0
    while True:
        now=time.time(); dt=now-prev_time if now!=prev_time else 1e-3; prev_time=now
        picam()
        warpImg()
        pid_controller = PID(kp, ki, kd)
        cte_f = SteeringAngle()
        steering = pid_controller.update(cte_f)
        steering_angle = round(steering)
        rpm_cmd=transform_speed(speed_motor_requested)
        #sen data thru USB
        if ser:
            data= struct.pack('<ii', steering_angle, rpm_cmd)
            ser.write(b'<'+data+b'>'); ser.flush()

        print(f"CTE: { steering_angle:.2f}Ä‘á»™, Speed: {speed_motor_requested} km/h, Steering: { steering_angle}, dt: {dt} ")
        cv2.imshow("Original", imgWarp)
        key=cv2.waitKey(1)&0xFF; signal_motor(key)
        if key==ord('q'): break
    piCam.stop()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
