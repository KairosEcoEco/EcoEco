# Recycle Bot 문서

이 문서는 YOLO 모델을 사용하여 다양한 종류의 쓰레기를 감지한 후 로봇 팔을 제어하여 분류하는 Recycle Bot 시스템의 기능과 작동에 대한 개요를 제공합니다. 시스템은 `detection_publisher.py`와 `control_subscriber.py`라는 두 개의 주요 Python 스크립트로 구성됩니다.

## detection_publisher.py

`detection_publisher.py` 스크립트는 비디오 입력을 캡처하고, 프레임을 처리하여 쓰레기를 감지하고, 감지 결과를 게시하는 역할을 합니다.

### 주요 구성 요소

- **라이브러리 및 ROS 통합:**
  - `rospy`를 사용하여 ROS와 통합하고, `cv2`를 사용하여 비디오를 처리하며, `ultralytics`를 사용하여 YOLO 모델을 불러옵니다.
  - `sensor_msgs.msg.Image` 및 `std_msgs.msg.String`을 사용하여 메시지를 게시합니다.
  - `geometry_msgs.msg.Point` 및 커스텀 메시지 `TrashDetection`을 사용하여 감지 결과를 게시합니다.

- **모델 및 ROI 설정:**
  - `MODEL_PATH`는 YOLO 모델의 경로를 지정합니다.
  - `ROI`는 캡처된 비디오 프레임에서 관심 영역을 정의합니다.

- **쓰레기 색상:**
  - 딕셔너리 `trash_color`는 다양한 쓰레기 종류를 주석 처리를 위한 특정 색상과 매핑합니다.

- **함수 정의:**
  - `process_frame(frame, roi, model)`: 지정된 ROI 내에서 YOLO 모델을 사용하여 비디오 프레임을 처리하여 쓰레기를 감지합니다. 감지된 쓰레기의 경계 상자와 레이블로 프레임에 주석을 추가합니다.
  - `main()`: ROS 노드를 초기화하고, 퍼블리셔를 설정하며, YOLO 모델을 불러오고, 카메라로부터 비디오 프레임을 캡처합니다. 각 프레임을 지속적으로 처리하고, 감지 결과와 주석이 추가된 이미지를 게시합니다.

### 작업 흐름

1. ROS 노드를 초기화합니다.
2. YOLO 모델을 불러옵니다.
3. 카메라로부터 비디오 프레임을 캡처합니다.
4. 각 프레임을 처리하여 쓰레기를 감지합니다.
5. 감지 결과로 프레임에 주석을 추가합니다.
6. 감지 결과와 주석이 추가된 이미지를 게시합니다.
7. 노드가 종료될 때까지 이 과정을 반복합니다.

## control_subscriber.py

`control_subscriber.py` 스크립트는 감지 결과를 구독하고, 감지된 쓰레기의 종류에 따라 로봇 팔을 제어하여 분류합니다.

### 주요 구성 요소

- **라이브러리 및 ROS 통합:**
  - `rospy`를 사용하여 ROS와 통합하고, `threading`을 사용하여 동시 작업을 관리합니다.
  - `pymycobot` 라이브러리를 사용하여 MyCobot 로봇 팔을 제어합니다.
  - 커스텀 메시지 `TrashDetection`을 사용하여 감지 결과를 수신합니다.

- **로봇 각도 및 설정:**
  - `INITIAL_ANGLES`, `WIDTH_ANGLES`, 및 `LENGTH_ANGLES`는 로봇 관절의 다양한 초기 각도를 정의합니다.
  - `trash_angles` 딕셔너리는 다양한 쓰레기 종류를 분류하기 위한 특정 각도로 매핑합니다.
  - `trash_color` 딕셔너리는 로봇 팔의 LED 색상을 다양한 쓰레기 종류와 매핑합니다.

- **함수 정의:**
  - `get_object_dimensions(box_coords)`: 바운딩 박스 좌표로부터 객체의 크기를 계산합니다.
  - `select_shorter_dimension(box_coords)`: 객체의 너비와 높이를 비교하여 더 짧은 쪽의 각도 배열을 선택합니다.
  - `control_robot(trash_name, box_coords)`: 감지된 쓰레기의 종류와 위치에 따라 로봇 팔을 제어합니다.
  - `detection_callback(data)`: 감지 결과를 수신할 때 로봇 제어 프로세스를 트리거하는 콜백 함수입니다.

### 작업 흐름

1. ROS 노드와 로봇 팔을 초기화합니다.
2. `detected_trash` 토픽을 구독합니다.
3. 감지 결과를 수신하면:
   - 감지된 쓰레기의 종류와 위치를 결정합니다.
   - 로봇 팔을 제어하여 쓰레기를 집어 분류합니다.
4. 로봇 팔이 분류 작업을 수행하고 초기 위치로 돌아갑니다.

## msg/TrashDetection.msg

```plaintext
string trash_name
geometry_msgs/Point box_coords
```

- string trash_name: 이 필드는 감지된 쓰레기의 이름 또는 유형(예: "플라스틱 병", "알루미늄 캔")을 포함합니다.
- geometry_msgs/Point box_coords: 이 필드는 감지된 쓰레기 주위의 경계 상자의 좌표를 포함합니다. geometry_msgs/Point 메시지는 일반적으로 3D 공간에서 위치를 나타내는 x, y, z 필드를 포함합니다.

## launch/recycle_launch.launch
```
<launch>
    <node pkg="recycle_bot" type="detection_publisher.py" name="detection_publisher" output="screen" />
    <node pkg="recycle_bot" type="control_subscriber.py" name="control_subscriber" output="screen" />
</launch>
```

  - detection_publisher
    - 패키지: recycle_bot

    - 타입: detection_publisher.py

    - 이름: detection_publisher

    - 출력: screen

    - 이 노드는 쓰레기를 감지하고 TrashDetection 메시지를 발행하는 역할을 합니다. 센서나 이미지 데이터를 처리하여 쓰레기 객체와 그 좌표를 식별한 후, 이 정보를 시스템의 다른 노드에 방송합니다.

  - control_subscriber

    - 패키지: recycle_bot
   
    - 타입: control_subscriber.py
   
    - 이름: control_subscriber
   
    - 출력: screen
   
    - 이 노드는 TrashDetection 메시지를 구독합니다. 감지된 쓰레기와 좌표에 대한 정보를 수신한 후, 로봇의 동작을 제어하는 데 이 데이터를 사용합니다. 여기에는 쓰레기 쪽으로 이동하고, 쓰레기를 집어 올리며, 적절한 재활용 통에 분류하는 작업이 포함될 수 있습니다.
   

## 전체 시스템 동작
1. 감지: detection_publisher 노드는 detection_publisher.py 스크립트를 실행하여 쓰레기를 감지하고 이름과 좌표를 포함한 TrashDetection 메시지를 발행합니다.

2. 제어: control_subscriber 노드는 control_subscriber.py 스크립트를 실행하여 TrashDetection 메시지를 구독합니다. 감지 데이터를 수신하고, 수집 및 재활용 작업을 수행하는 데 필요한 동작을 실행합니다. 



   
