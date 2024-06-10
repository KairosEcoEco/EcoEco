import cv2
import numpy as np
from pymycobot.myagv import MyAgv
import threading
import time
from queue import Queue

agv = MyAgv("/dev/ttyAMA2", 115200)
def process_frame(frame):
    height, width, _ = frame.shape
    
    roi_height = int(height / 5)
    roi_top = height - roi_height
    roi = frame[roi_top:, :]
    cv2.rectangle(roi, (0, 0), (width, roi_height), (255, 255, 0), 2)
    cv2.line(frame, (width // 2, 0), (width // 2, height), (0, 255, 0), 2)
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    lower_yellow = np.array([15, 150, 20], dtype=np.uint8)
    upper_yellow = np.array([35, 255, 255], dtype=np.uint8)
    yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    yellow_detected = cv2.countNonZero(yellow_mask) > 100

    # 占쏙옙占쏙옙 LED 占쏙옙占쏙옙
    lower_red = np.array([0, 50, 100])
    upper_red = np.array([10, 255, 255])
    red_mask1 = cv2.inRange(hsv, lower_red, upper_red)
    lower_red = np.array([170, 100, 100])
    upper_red = np.array([180, 255, 255])
    red_mask2 = cv2.inRange(hsv, lower_red, upper_red)
    red_mask = cv2.bitwise_or(red_mask1, red_mask2)
    red_detected = cv2.countNonZero(red_mask) > 100
    
    if red_detected:
        return "STOP"
    elif yellow_detected:
        return "RIGHT"
    else:
        return "GO"
   
    
def control_thread(frame_queue):
    while True:
        if not frame_queue.empty():
            frame = frame_queue.get()
            result = process_frame(frame)
            if result == "GO":
                print("GO")
                agv.go_ahead(1)
                time.sleep(0.5)
                agv.stop()
            elif result == "RIGHT":
                agv.stop()
                time.sleep(0.5)
                print("RIGHT")
                agv.clockwise_rotation(1)
                time.sleep(2.15)
                agv.stop()
            elif result == "STOP":
                time.sleep(1)
                print("STOP")
                agv.stop()

def camera_thread(frame_queue):
    cap = cv2.VideoCapture(0)
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            print("camera error")
            break
        if not frame_queue.full():
            frame_queue.put(frame)
        frame_s = cv2.resize(frame, None, fx=0.2, fy=0.2, interpolation=cv2.INTER_AREA)
        cv2.imshow("ORG", frame_s)
        if cv2.waitKey(25) & 0xFF == ord('q'):
            agv.stop()
            break
        

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    frame_queue = Queue(maxsize=1)

    control_thread = threading.Thread(target=control_thread, args=(frame_queue,))
    control_thread.start()

    camera_thread = threading.Thread(target=camera_thread, args=(frame_queue,))
    camera_thread.start()

    control_thread.join()
    camera_thread.join()