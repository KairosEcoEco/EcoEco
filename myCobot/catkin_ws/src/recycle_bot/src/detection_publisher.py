#!/usr/bin/env python

import rospy
import cv2
import time
from ultralytics import YOLO
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Point
from recycle_bot.msg import TrashDetection
from cv_bridge import CvBridge

# 모델 경로 및 ROI 설정
MODEL_PATH = "/home/karios/catkin_ws/src/recycle_bot/model/best.pt"
ROI = (70, 70, 400, 300)

# 쓰레기별 색상 딕셔너리
trash_color = {
    "can": (0, 255, 255),        # 노란색
    "paper": (128, 0, 128),      # 보라색
    "plastic": (0, 0, 255),      # 빨간색
    "plastic-bag": (255, 255, 102) # 밝은 노란색
}

bridge = CvBridge()

def process_frame(frame, roi, model):
    x, y, w, h = roi
    roi_frame = frame[y:y+h, x:x+w]
    results = model(roi_frame)
    
    highest_confidence = 0
    best_trash = None
    best_box = None

    for result in results:
        boxes = result.boxes
        for box in boxes:
            confidence = box.conf[0]
            if confidence > 0.75 and confidence > highest_confidence:
                highest_confidence = confidence
                best_trash = model.names[int(box.cls[0])]
                best_box = box

    if best_trash and best_box:
        x1, y1, x2, y2 = map(int, best_box.xyxy[0])
        x1 += x
        y1 += y
        x2 += x
        y2 += y
        color = trash_color.get(best_trash, (255, 255, 255))
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        cv2.putText(frame, f"{best_trash} {highest_confidence:.2f}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
    
    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
    
    return frame, best_trash, best_box

def main():
    rospy.init_node('detection_publisher', anonymous=True)
    detection_pub = rospy.Publisher('detected_trash', TrashDetection, queue_size=10)
    image_pub = rospy.Publisher('detection_image', Image, queue_size=10)
    rate = rospy.Rate(10)
    
    model = YOLO(MODEL_PATH).cuda()
    cap = cv2.VideoCapture("/dev/video0")
    cap.set(3, 640)
    cap.set(4, 480)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    
    if not cap.isOpened():
        rospy.logerr("Error: Could not open camera.")
        return

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            rospy.logerr("Cam Error!!!")
            break

        annotated_frame, detected_trash, detected_box = process_frame(frame, ROI, model)
        
        if detected_trash and detected_box:
            detection_msg = TrashDetection()
            detection_msg.trash_name = detected_trash
            detection_msg.box_coords.x = (detected_box.xyxy[0][0] + detected_box.xyxy[0][2]) / 2
            detection_msg.box_coords.y = (detected_box.xyxy[0][1] + detected_box.xyxy[0][3]) / 2
            detection_msg.box_coords.z = 0  # z 값은 사용하지 않으므로 0으로 설정
            detection_pub.publish(detection_msg)
        
        image_message = bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
        image_pub.publish(image_message)
        
        rate.sleep()

    cap.release()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
