import cv2
import time
import threading
from ultralytics import YOLO
from pymycobot.mycobot import MyCobot

# 초기 각도 설정
INITIAL_ANGLES = [0, -35, -10, -40, 95, 90]
WIDTH_ANGLES = [-15, -50, 13, -50, 90, -10]
LENGTH_ANGLES= [-15, -50, 13, -50, 90, 60]

# 모델 경로 및 ROI 설정
MODEL_PATH = "C:/Mywork/P3/datasets/runs/detect/train/weights/best.pt"
ROI = (70, 70, 400, 300)

# 코봇 객체 초기화
cobot = MyCobot("COM9", 115200)

# 쓰레기별 색상 딕셔너리
trash_color = {
    "can": (0, 255, 255),
    "paper": (128, 0, 128),
    "plastic": (0, 0, 255),
    "plastic-bag": (255, 255, 102)
}

# 쓰레기별 위치 딕셔너리
trash_angles = {
    "can": [-55, -40, 20, -40, 90, 90],
    "paper": [-45, -45, 8, -20, 90, 90],
    "plastic": [-70, -30, 8, -40, 90, 90],
    "plastic-bag": [-30, 40, -90, -10, 90, 90]
}

cobot_moving = False
SPEED = 60

def get_object_dimensions(box_coords):
    """
    바운딩 박스 좌표로부터 객체의 중심 좌표와 너비, 높이를 계산합니다.

    Args:
        box_coords (list): 바운딩 박스 좌표 [x1, y1, x2, y2]

    Returns:
        center_x (int): 객체의 중심 x 좌표
        center_y (int): 객체의 중심 y 좌표
        width (int): 객체의 너비
        height (int): 객체의 높이
    """
    x1, y1, x2, y2 = map(int, box_coords)
    center_x = (x1 + x2) // 2
    center_y = (y1 + y2) // 2
    width = abs(x2 - x1)
    height = abs(y2 - y1)
    return center_x, center_y, width, height

def select_shorter_dimension(box_coords):
    """
    객체의 너비와 높이를 비교하여 더 짧은 쪽에 해당하는 각도 배열을 반환합니다.

    Args:
        box_coords (list): 바운딩 박스 좌표 [x1, y1, x2, y2]

    Returns:
        list: 더 짧은 쪽에 해당하는 로봇 관절 각도 배열
    """
    _, _, width, height = get_object_dimensions(box_coords)
    return WIDTH_ANGLES if width > height else LENGTH_ANGLES

def control_robot(trash_name, box_coords):
    global cobot_moving
    #print("control_robot")

    cobot_moving = True
    color = trash_color.get(trash_name)
    
    # Extract coordinates if box_coords is not None
    if box_coords is not None:
        box_coords = box_coords.xyxy[0].tolist()
        adjust_angles = select_shorter_dimension(box_coords)
    else:
        adjust_angles = INITIAL_ANGLES

    cobot.set_color(*color)
    time.sleep(2)
    
    cobot.send_angle(1, -15, 30)
    time.sleep(2)
    cobot.set_gripper_value(45, 20, 1)
    time.sleep(1)

    cobot.send_angles(adjust_angles, 30)
    time.sleep(2)

    cobot.set_gripper_value(0, 20, 1)
    time.sleep(1)

    cobot.send_angle(2, 0, 30)
    time.sleep(2)

    cobot.send_angles([0, 0, 0, 0, 0, 0], SPEED)
    time.sleep(2)

    print(f"{trash_angles[trash_name]}, {trash_name}")
    cobot.send_angles(trash_angles[trash_name], SPEED)
    time.sleep(2)
        
    cobot.set_gripper_value(100, 20, 1)
    time.sleep(1)

    cobot.send_angles(INITIAL_ANGLES, SPEED)  # 로봇을 초기 상태로 되돌림
    time.sleep(3)
    cobot.set_color(255, 255, 255)
    time.sleep(1)
    cobot_moving = False

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
        
        # 사각형 그리기
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        # 감지한 객체 이름
        cv2.putText(frame, f"{best_trash} {highest_confidence:.2f}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
    
    # ROI 사각형 그리기
    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
    
    return frame, best_trash, best_box

def main(model_path=MODEL_PATH, roi=ROI):
    model = YOLO(model_path).cuda()
    cap = cv2.VideoCapture(0)
    cap.set(3, 640)
    cap.set(4, 480)
    
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    control_thread = None

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Cam Error!!!")
            break

        annotated_frame, detected_trash, detected_box = process_frame(frame, roi, model)
        #print(f"Start Robot {detected_trash} {detected_box}")
        
        #print("start Robot1")
        if detected_trash and detected_box and (control_thread is None or not control_thread.is_alive()):
            #print("start Robot2")
            control_thread = threading.Thread(target=control_robot, args=(detected_trash, detected_box))
            control_thread.start()
                

        cv2.imshow("Detection Result", annotated_frame)
        if cv2.waitKey(1) == ord('q'):
            break
       

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    cobot.send_angles([0, 0, 0, 0, 0, 0], 30)
    time.sleep(3)
    cobot.send_angles(INITIAL_ANGLES, 30)
    time.sleep(0.5)
    cobot.set_gripper_mode(0)
    cobot.init_eletric_gripper()
    
    main_thread = threading.Thread(target=main)
    main_thread.start()
