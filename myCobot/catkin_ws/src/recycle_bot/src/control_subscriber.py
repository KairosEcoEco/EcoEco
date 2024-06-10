#!/usr/bin/env python

import rospy
import time
import threading
from pymycobot.mycobot import MyCobot
from recycle_bot.msg import TrashDetection

# 초기 각도 설정
INITIAL_ANGLES = [0, -35, -10, -40, 95, 90]

# 너비 기준 각도 설정
WIDTH_ANGLES = [-15, -50, 13, -50, 90, -10]

# 높이 기준 각도 설정
LENGTH_ANGLES = [-15, -50, 13, -50, 90, 60]

# 쓰레기별 위치 딕셔너리 (로봇 관절 각도 설정)
trash_angles = {
    "can": [-55, -40, 20, -40, 90, 90],
    "paper": [-45, -45, 8, -20, 90, 90],
    "plastic": [-70, -30, 8, -40, 90, 90],
    "plastic-bag": [-30, 40, -90, -10, 90, 90]
}

# 쓰레기별 색상
trash_color = {
    "can": (0, 255, 255),
    "paper": (128, 0, 128),
    "plastic": (0, 0, 255),
    "plastic-bag": (255, 255, 102)
}

cobot_moving = False
SPEED = 60
cobot = MyCobot("/dev/ttyACM0", 115200)

def get_object_dimensions(box_coords):
    """
    바운딩 박스 좌표로부터 객체의 중심 좌표와 너비, 높이를 계산합니다.
    Args:
        box_coords (list): 바운딩 박스 좌표 [center_x, center_y]
    Returns:
        center_x (int): 객체의 중심 x 좌표
        center_y (int): 객체의 중심 y 좌표
        width (int): 객체의 너비
        height (int): 객체의 높이
    """
    center_x, center_y = map(int, box_coords)
    width = 50  # 임의의 너비 값 설정
    height = 50  # 임의의 높이 값 설정
    return center_x, center_y, width, height

def select_shorter_dimension(box_coords):
    """
    객체의 너비와 높이를 비교하여 더 짧은 쪽에 해당하는 각도 배열을 반환합니다.
    Args:
        box_coords (list): 바운딩 박스 좌표 [center_x, center_y]

    Returns:
        list: 더 짧은 쪽에 해당하는 로봇 관절 각도 배열
    """
    _, _, width, height = get_object_dimensions(box_coords)
    return WIDTH_ANGLES if width > height else LENGTH_ANGLES

def control_robot(trash_name, box_coords):
    global cobot_moving

    cobot_moving = True
    color = trash_color.get(trash_name)
    cobot.set_color(*color)
    time.sleep(2)

    if box_coords is not None:
        adjust_angles = select_shorter_dimension(box_coords)
    else:
        adjust_angles = INITIAL_ANGLES
    
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

    cobot.send_angles(trash_angles[trash_name], SPEED)
    time.sleep(2)
        
    cobot.set_gripper_value(100, 20, 1)
    time.sleep(1)

    cobot.send_angles(INITIAL_ANGLES, SPEED)
    time.sleep(3)
    cobot.set_color(255, 255, 255)
    time.sleep(1)
    cobot_moving = False

def detection_callback(data):
    if not cobot_moving:
        trash_name = data.trash_name
        box_coords = [data.box_coords.x, data.box_coords.y]
        control_thread = threading.Thread(target=control_robot, args=(trash_name, box_coords))
        control_thread.start()

def main():
    rospy.init_node('control_subscriber', anonymous=True)
    rospy.Subscriber('detected_trash', TrashDetection, detection_callback)
    rospy.spin()

if __name__ == '__main__':
    # 로봇 초기 설정
    cobot.send_angles([0, 0, 0, 0, 0, 0], 30)
    time.sleep(3)
    cobot.send_angles(INITIAL_ANGLES, 30)
    time.sleep(0.5)
    cobot.set_gripper_mode(0)
    cobot.init_eletric_gripper()
    
    main()
