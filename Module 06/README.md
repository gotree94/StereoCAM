# Module 06: ROS2 연동 개발

[![난이도](https://img.shields.io/badge/난이도-⭐⭐⭐⭐_고급-red.svg)]()
[![예상시간](https://img.shields.io/badge/예상시간-12--16시간-blue.svg)]()
[![선수지식](https://img.shields.io/badge/선수지식-Module_04,_Linux_기초-orange.svg)]()

---

## 📋 모듈 개요

| 항목 | 내용 |
|------|------|
| **학습 목표** | 스테레오 카메라 시스템을 ROS2와 연동하여 로봇 플랫폼에 적용 |
| **핵심 키워드** | ROS2 Humble, sensor_msgs, cv_bridge, PointCloud2, TF2 |
| **산출물** | 스테레오 카메라 ROS2 패키지, 실시간 깊이/포인트클라우드 퍼블리셔 |

---

## 📚 목차

1. [ROS2 개요](#1-ros2-개요) : 아키텍처, ROS1 vs ROS2 비교, 배포판
2. [ROS2 설치 및 환경 설정](#2-ros2-설치-및-환경-설정) : Humble 설치, 워크스페이스 생성, 필수 패키지
3. [ROS2 기본 개념](#3-ros2-기본-개념) : Node, Topic, Publisher/Subscriber, 메시지 타입
4. [스테레오 카메라 노드 개발](#4-스테레오-카메라-노드-개발) : 패키지 생성, 구조, package.xml, setup.py
5. [이미지 메시지 퍼블리시](#5-이미지-메시지-퍼블리시) : StereoCameraNode 전체 구현
6. [깊이 이미지 퍼블리시](#6-깊이-이미지-퍼블리시) : StereoProcessorNode, 시간 동기화
7. [포인트 클라우드 퍼블리시](#7-포인트-클라우드-퍼블리시) : PointCloud2 메시지 생성
8. [TF2 프레임 설정](#8-tf2-프레임-설정) : Static TF 브로드캐스터, 프레임 구조
9. [Launch 파일 작성](#9-launch-파일-작성) : Python Launch 파일, 실행 방법
10. [Rviz2 시각화](#10-rviz2-시각화) : Rviz 설정, 토픽 확인

📁 포함된 코드
   * minimal_node.py - 최소 ROS2 노드 예제
   * topic_example.py - Publisher/Subscriber 예제
   * stereo_camera_node.py - 스테레오 카메라 캡처 노드
   * stereo_processor.py - 스테레오 매칭 및 깊이 추정 노드
   * pointcloud_publisher.py - 포인트 클라우드 퍼블리셔
   * tf_broadcaster.py - TF2 브로드캐스터
   * stereo_camera.launch.py - Launch 파일
   * stereo_camera.rviz - Rviz2 설정 파일

---

## 1. ROS2 개요

### 1.1 ROS2란?

ROS2 (Robot Operating System 2)는 로봇 소프트웨어 개발을 위한 오픈소스 프레임워크입니다.

```
┌─────────────────────────────────────────────────────────────┐
│                    ROS2 아키텍처                             │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│   ┌─────────┐    Topic    ┌─────────┐                      │
│   │  Node   │────────────→│  Node   │                      │
│   │(Camera) │  /image     │ (SLAM)  │                      │
│   └─────────┘             └─────────┘                      │
│        │                       │                           │
│        │ /depth                │ /map                      │
│        ▼                       ▼                           │
│   ┌─────────┐             ┌─────────┐                      │
│   │  Node   │             │  Node   │                      │
│   │(Obstacle│             │ (Nav2)  │                      │
│   │Detect)  │             │         │                      │
│   └─────────┘             └─────────┘                      │
│                                                             │
│   DDS (Data Distribution Service)                          │
│   ─────────────────────────────────────────────────────    │
│   실시간 통신, QoS, 분산 시스템                              │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### 1.2 ROS1 vs ROS2

| 특징 | ROS1 | ROS2 |
|------|------|------|
| **미들웨어** | 자체 (TCPROS) | DDS (표준) |
| **실시간성** | 제한적 | 지원 |
| **보안** | 없음 | SROS2 |
| **멀티플랫폼** | Linux 중심 | Linux/Windows/macOS |
| **Python** | 2.7/3 | 3.x 전용 |
| **빌드** | catkin | colcon |
| **마스터 노드** | 필요 (roscore) | 불필요 |

### 1.3 주요 배포판

| 배포판 | 출시일 | 지원 종료 | Ubuntu |
|--------|--------|----------|--------|
| **Humble Hawksbill** | 2022.05 | 2027.05 | 22.04 LTS |
| Iron Irwini | 2023.05 | 2024.11 | 22.04 |
| **Jazzy Jalisco** | 2024.05 | 2029.05 | 24.04 LTS |

> 💡 **권장**: Humble (LTS, 안정적)

---

## 2. ROS2 설치 및 환경 설정

### 2.1 ROS2 Humble 설치 (Ubuntu 22.04)

```bash
# 1. 로케일 설정
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 2. 저장소 설정
sudo apt install software-properties-common
sudo add-apt-repository universe

# 3. ROS2 GPG 키 추가
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# 4. 저장소 추가
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 5. 패키지 설치
sudo apt update
sudo apt upgrade

# Desktop 설치 (Rviz2 포함)
sudo apt install ros-humble-desktop

# 또는 기본 설치만
# sudo apt install ros-humble-ros-base

# 6. 개발 도구
sudo apt install ros-dev-tools
```

### 2.2 환경 설정

```bash
# ~/.bashrc에 추가
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 확인
ros2 --version
```

### 2.3 워크스페이스 생성

```bash
# 워크스페이스 디렉토리 생성
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# 빌드 (빈 워크스페이스)
colcon build

# 워크스페이스 소싱
source install/setup.bash

# ~/.bashrc에 추가 (선택)
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

### 2.4 필수 패키지 설치

```bash
# 이미지 관련
sudo apt install ros-humble-cv-bridge
sudo apt install ros-humble-image-transport
sudo apt install ros-humble-image-transport-plugins

# 카메라 관련
sudo apt install ros-humble-camera-info-manager
sudo apt install ros-humble-camera-calibration

# 포인트 클라우드 관련
sudo apt install ros-humble-pcl-conversions
sudo apt install ros-humble-pcl-ros

# TF2
sudo apt install ros-humble-tf2-ros
sudo apt install ros-humble-tf2-geometry-msgs

# 시각화
sudo apt install ros-humble-rviz2

# Python 의존성
pip install opencv-python numpy pyyaml
```

---

## 3. ROS2 기본 개념

### 3.1 노드 (Node)

```python
"""
minimal_node.py
최소 ROS2 노드 예제
"""

import rclpy
from rclpy.node import Node


class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')
        self.get_logger().info('노드가 시작되었습니다!')
        
        # 타이머 생성 (1초마다 콜백)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.count = 0
    
    def timer_callback(self):
        self.count += 1
        self.get_logger().info(f'타이머 콜백: {self.count}')


def main(args=None):
    rclpy.init(args=args)
    
    node = MinimalNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 3.2 토픽 (Topic) - Publisher/Subscriber

```python
"""
topic_example.py
Publisher/Subscriber 예제
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        
        # Publisher 생성
        self.publisher_ = self.create_publisher(
            String,           # 메시지 타입
            'topic',          # 토픽 이름
            10                # QoS (큐 크기)
        )
        
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.i = 0
    
    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1


class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        
        # Subscriber 생성
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10
        )
    
    def listener_callback(self, msg):
        self.get_logger().info(f'Received: "{msg.data}"')
```

### 3.3 주요 메시지 타입 (스테레오 카메라 관련)

```
┌─────────────────────────────────────────────────────────────┐
│              스테레오 카메라 관련 메시지 타입                  │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  sensor_msgs/msg/Image                                      │
│  ├─ header (std_msgs/Header)                               │
│  ├─ height, width (uint32)                                 │
│  ├─ encoding (string): "bgr8", "mono8", "32FC1" 등         │
│  ├─ is_bigendian (uint8)                                   │
│  ├─ step (uint32): 한 행의 바이트 수                        │
│  └─ data (uint8[]): 이미지 데이터                          │
│                                                             │
│  sensor_msgs/msg/CameraInfo                                │
│  ├─ header                                                 │
│  ├─ height, width                                          │
│  ├─ distortion_model (string)                              │
│  ├─ D (float64[]): 왜곡 계수                               │
│  ├─ K (float64[9]): 내부 파라미터                          │
│  ├─ R (float64[9]): 정류 행렬                              │
│  ├─ P (float64[12]): 투영 행렬                             │
│  └─ binning_x, binning_y                                   │
│                                                             │
│  sensor_msgs/msg/PointCloud2                               │
│  ├─ header                                                 │
│  ├─ height, width                                          │
│  ├─ fields (PointField[]): x, y, z, rgb 등                 │
│  ├─ is_bigendian                                           │
│  ├─ point_step, row_step                                   │
│  ├─ data (uint8[])                                         │
│  └─ is_dense (bool)                                        │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

---

## 4. 스테레오 카메라 노드 개발

### 4.1 패키지 생성

```bash
cd ~/ros2_ws/src

# Python 패키지 생성
ros2 pkg create --build-type ament_python stereo_camera \
    --dependencies rclpy sensor_msgs cv_bridge image_transport

# 또는 C++ 패키지
# ros2 pkg create --build-type ament_cmake stereo_camera \
#     --dependencies rclcpp sensor_msgs cv_bridge image_transport
```

### 4.2 패키지 구조

```
stereo_camera/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── stereo_camera
├── stereo_camera/
│   ├── __init__.py
│   ├── stereo_camera_node.py      # 메인 노드
│   ├── stereo_processor.py        # 스테레오 처리
│   └── utils.py                   # 유틸리티
├── config/
│   ├── stereo_params.yaml         # 캘리브레이션
│   └── camera_config.yaml         # 카메라 설정
├── launch/
│   └── stereo_camera.launch.py    # 런치 파일
└── rviz/
    └── stereo_camera.rviz         # Rviz 설정
```

### 4.3 package.xml

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>stereo_camera</name>
  <version>1.0.0</version>
  <description>USB 스테레오 카메라 ROS2 패키지</description>
  <maintainer email="your@email.com">Your Name</maintainer>
  <license>MIT</license>

  <!-- 빌드 의존성 -->
  <buildtool_depend>ament_python</buildtool_depend>
  
  <!-- 실행 의존성 -->
  <exec_depend>rclpy</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>cv_bridge</exec_depend>
  <exec_depend>image_transport</exec_depend>
  <exec_depend>tf2_ros</exec_depend>
  <exec_depend>tf2_geometry_msgs</exec_depend>
  
  <!-- 테스트 -->
  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### 4.4 setup.py

```python
from setuptools import setup
import os
from glob import glob

package_name = 'stereo_camera'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch 파일
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # Config 파일
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        # Rviz 설정
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='USB 스테레오 카메라 ROS2 패키지',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stereo_camera_node = stereo_camera.stereo_camera_node:main',
            'stereo_processor_node = stereo_camera.stereo_processor:main',
        ],
    },
)
```

---

## 5. 이미지 메시지 퍼블리시

### 5.1 스테레오 카메라 노드

```python
"""
stereo_camera_node.py
스테레오 카메라 캡처 및 퍼블리시 노드
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import yaml


class StereoCameraNode(Node):
    def __init__(self):
        super().__init__('stereo_camera_node')
        
        # 파라미터 선언
        self.declare_parameter('left_camera_id', 0)
        self.declare_parameter('right_camera_id', 2)
        self.declare_parameter('frame_rate', 30.0)
        self.declare_parameter('image_width', 1920)
        self.declare_parameter('image_height', 1080)
        self.declare_parameter('calibration_file', '')
        self.declare_parameter('frame_id', 'stereo_camera')
        
        # 파라미터 가져오기
        self.left_id = self.get_parameter('left_camera_id').value
        self.right_id = self.get_parameter('right_camera_id').value
        self.frame_rate = self.get_parameter('frame_rate').value
        self.width = self.get_parameter('image_width').value
        self.height = self.get_parameter('image_height').value
        self.calibration_file = self.get_parameter('calibration_file').value
        self.frame_id = self.get_parameter('frame_id').value
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # 카메라 초기화
        self.init_cameras()
        
        # 캘리브레이션 로드
        self.load_calibration()
        
        # QoS 설정 (센서 데이터용)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Publishers
        self.pub_left_image = self.create_publisher(
            Image, '/stereo/left/image_raw', sensor_qos)
        self.pub_right_image = self.create_publisher(
            Image, '/stereo/right/image_raw', sensor_qos)
        self.pub_left_info = self.create_publisher(
            CameraInfo, '/stereo/left/camera_info', sensor_qos)
        self.pub_right_info = self.create_publisher(
            CameraInfo, '/stereo/right/camera_info', sensor_qos)
        
        # 정류된 이미지 (캘리브레이션 있는 경우)
        if self.calibration_loaded:
            self.pub_left_rect = self.create_publisher(
                Image, '/stereo/left/image_rect', sensor_qos)
            self.pub_right_rect = self.create_publisher(
                Image, '/stereo/right/image_rect', sensor_qos)
        
        # 타이머 (캡처 루프)
        timer_period = 1.0 / self.frame_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info(f'스테레오 카메라 노드 시작')
        self.get_logger().info(f'  왼쪽 카메라: {self.left_id}')
        self.get_logger().info(f'  오른쪽 카메라: {self.right_id}')
        self.get_logger().info(f'  해상도: {self.width}x{self.height}')
        self.get_logger().info(f'  프레임 레이트: {self.frame_rate} Hz')
    
    def init_cameras(self):
        """카메라 초기화"""
        
        self.cap_left = cv2.VideoCapture(self.left_id)
        self.cap_right = cv2.VideoCapture(self.right_id)
        
        for cap in [self.cap_left, self.cap_right]:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            cap.set(cv2.CAP_PROP_FPS, self.frame_rate)
        
        if not self.cap_left.isOpened() or not self.cap_right.isOpened():
            self.get_logger().error('카메라 열기 실패!')
            raise RuntimeError('카메라 초기화 실패')
    
    def load_calibration(self):
        """캘리브레이션 파라미터 로드"""
        
        self.calibration_loaded = False
        
        if not self.calibration_file:
            self.get_logger().warn('캘리브레이션 파일이 지정되지 않았습니다.')
            return
        
        try:
            with open(self.calibration_file, 'r') as f:
                params = yaml.safe_load(f)
            
            self.img_size = tuple(params['image_size'])
            self.K1 = np.array(params['K1'])
            self.D1 = np.array(params['D1'])
            self.K2 = np.array(params['K2'])
            self.D2 = np.array(params['D2'])
            self.R = np.array(params['R'])
            self.T = np.array(params['T'])
            self.R1 = np.array(params['R1'])
            self.R2 = np.array(params['R2'])
            self.P1 = np.array(params['P1'])
            self.P2 = np.array(params['P2'])
            self.Q = np.array(params['Q'])
            
            # 정류 맵 생성
            self.map1_left, self.map2_left = cv2.initUndistortRectifyMap(
                self.K1, self.D1, self.R1, self.P1, self.img_size, cv2.CV_32FC1
            )
            self.map1_right, self.map2_right = cv2.initUndistortRectifyMap(
                self.K2, self.D2, self.R2, self.P2, self.img_size, cv2.CV_32FC1
            )
            
            self.calibration_loaded = True
            self.get_logger().info(f'캘리브레이션 로드 완료: {self.calibration_file}')
            
        except Exception as e:
            self.get_logger().error(f'캘리브레이션 로드 실패: {e}')
    
    def create_camera_info(self, K, D, R, P, is_left=True):
        """CameraInfo 메시지 생성"""
        
        info = CameraInfo()
        info.header.frame_id = f'{self.frame_id}_{"left" if is_left else "right"}'
        info.height = self.height
        info.width = self.width
        info.distortion_model = 'plumb_bob'
        info.d = D.flatten().tolist()
        info.k = K.flatten().tolist()
        info.r = R.flatten().tolist()
        info.p = P.flatten().tolist()
        
        return info
    
    def timer_callback(self):
        """주기적 캡처 콜백"""
        
        # 이미지 캡처
        ret_l, frame_left = self.cap_left.read()
        ret_r, frame_right = self.cap_right.read()
        
        if not ret_l or not ret_r:
            self.get_logger().warn('프레임 캡처 실패')
            return
        
        # 타임스탬프
        now = self.get_clock().now().to_msg()
        
        # 원본 이미지 퍼블리시
        left_msg = self.bridge.cv2_to_imgmsg(frame_left, 'bgr8')
        left_msg.header.stamp = now
        left_msg.header.frame_id = f'{self.frame_id}_left'
        self.pub_left_image.publish(left_msg)
        
        right_msg = self.bridge.cv2_to_imgmsg(frame_right, 'bgr8')
        right_msg.header.stamp = now
        right_msg.header.frame_id = f'{self.frame_id}_right'
        self.pub_right_image.publish(right_msg)
        
        # CameraInfo 퍼블리시
        if self.calibration_loaded:
            left_info = self.create_camera_info(self.K1, self.D1, self.R1, self.P1, True)
            left_info.header.stamp = now
            self.pub_left_info.publish(left_info)
            
            right_info = self.create_camera_info(self.K2, self.D2, self.R2, self.P2, False)
            right_info.header.stamp = now
            self.pub_right_info.publish(right_info)
            
            # 정류된 이미지 퍼블리시
            rect_left = cv2.remap(frame_left, self.map1_left, self.map2_left, cv2.INTER_LINEAR)
            rect_right = cv2.remap(frame_right, self.map1_right, self.map2_right, cv2.INTER_LINEAR)
            
            rect_left_msg = self.bridge.cv2_to_imgmsg(rect_left, 'bgr8')
            rect_left_msg.header.stamp = now
            rect_left_msg.header.frame_id = f'{self.frame_id}_left'
            self.pub_left_rect.publish(rect_left_msg)
            
            rect_right_msg = self.bridge.cv2_to_imgmsg(rect_right, 'bgr8')
            rect_right_msg.header.stamp = now
            rect_right_msg.header.frame_id = f'{self.frame_id}_right'
            self.pub_right_rect.publish(rect_right_msg)
    
    def destroy_node(self):
        """노드 종료 시 정리"""
        self.cap_left.release()
        self.cap_right.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = StereoCameraNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## 6. 깊이 이미지 퍼블리시

### 6.1 스테레오 프로세서 노드

```python
"""
stereo_processor.py
스테레오 매칭 및 깊이 추정 노드
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import message_filters
import yaml


class StereoProcessorNode(Node):
    def __init__(self):
        super().__init__('stereo_processor_node')
        
        # 파라미터 선언
        self.declare_parameter('calibration_file', '')
        self.declare_parameter('num_disparities', 128)
        self.declare_parameter('block_size', 5)
        self.declare_parameter('min_depth', 100.0)   # mm
        self.declare_parameter('max_depth', 10000.0) # mm
        self.declare_parameter('frame_id', 'stereo_camera')
        
        # 파라미터 가져오기
        self.calibration_file = self.get_parameter('calibration_file').value
        self.num_disparities = self.get_parameter('num_disparities').value
        self.block_size = self.get_parameter('block_size').value
        self.min_depth = self.get_parameter('min_depth').value
        self.max_depth = self.get_parameter('max_depth').value
        self.frame_id = self.get_parameter('frame_id').value
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # 캘리브레이션 로드
        self.load_calibration()
        
        # SGBM 매처 생성
        self.stereo = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=self.num_disparities,
            blockSize=self.block_size,
            P1=8 * 3 * self.block_size ** 2,
            P2=32 * 3 * self.block_size ** 2,
            disp12MaxDiff=1,
            uniquenessRatio=10,
            speckleWindowSize=100,
            speckleRange=2,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
        )
        
        # QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Publishers
        self.pub_disparity = self.create_publisher(
            Image, '/stereo/disparity', sensor_qos)
        self.pub_depth = self.create_publisher(
            Image, '/stereo/depth', sensor_qos)
        
        # Synchronized Subscribers
        self.sub_left = message_filters.Subscriber(
            self, Image, '/stereo/left/image_rect')
        self.sub_right = message_filters.Subscriber(
            self, Image, '/stereo/right/image_rect')
        
        # 시간 동기화
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.sub_left, self.sub_right],
            queue_size=5,
            slop=0.1  # 100ms 허용
        )
        self.ts.registerCallback(self.stereo_callback)
        
        self.get_logger().info('스테레오 프로세서 노드 시작')
        self.get_logger().info(f'  시차 범위: 0-{self.num_disparities}')
        self.get_logger().info(f'  깊이 범위: {self.min_depth/1000:.1f}m - {self.max_depth/1000:.1f}m')
    
    def load_calibration(self):
        """캘리브레이션 로드"""
        
        if not self.calibration_file:
            self.get_logger().error('캘리브레이션 파일이 필요합니다!')
            raise RuntimeError('캘리브레이션 파일 없음')
        
        with open(self.calibration_file, 'r') as f:
            params = yaml.safe_load(f)
        
        self.Q = np.array(params['Q'])
        self.baseline = params['baseline_mm']
        self.P1 = np.array(params['P1'])
        self.focal_length = self.P1[0, 0]
        
        self.get_logger().info(f'캘리브레이션 로드: baseline={self.baseline:.1f}mm')
    
    def stereo_callback(self, left_msg, right_msg):
        """동기화된 스테레오 이미지 콜백"""
        
        # ROS 메시지 → OpenCV
        left_image = self.bridge.imgmsg_to_cv2(left_msg, 'bgr8')
        right_image = self.bridge.imgmsg_to_cv2(right_msg, 'bgr8')
        
        # 시차 계산
        disparity = self.stereo.compute(left_image, right_image)
        disparity = disparity.astype(np.float32) / 16.0
        
        # 깊이 계산
        depth = np.zeros_like(disparity)
        valid_mask = disparity > 0
        depth[valid_mask] = (self.focal_length * self.baseline) / disparity[valid_mask]
        
        # 깊이 범위 제한
        depth[(depth < self.min_depth) | (depth > self.max_depth)] = 0
        
        # 타임스탬프
        stamp = left_msg.header.stamp
        
        # 시차 맵 퍼블리시 (시각화용 정규화)
        disp_normalized = cv2.normalize(disparity, None, 0, 255, cv2.NORM_MINMAX)
        disp_normalized = disp_normalized.astype(np.uint8)
        disp_color = cv2.applyColorMap(disp_normalized, cv2.COLORMAP_JET)
        
        disp_msg = self.bridge.cv2_to_imgmsg(disp_color, 'bgr8')
        disp_msg.header.stamp = stamp
        disp_msg.header.frame_id = f'{self.frame_id}_left'
        self.pub_disparity.publish(disp_msg)
        
        # 깊이 맵 퍼블리시 (32FC1: 미터 단위)
        depth_meters = depth / 1000.0  # mm → m
        depth_msg = self.bridge.cv2_to_imgmsg(depth_meters.astype(np.float32), '32FC1')
        depth_msg.header.stamp = stamp
        depth_msg.header.frame_id = f'{self.frame_id}_left'
        self.pub_depth.publish(depth_msg)


def main(args=None):
    rclpy.init(args=args)
    
    node = StereoProcessorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## 7. 포인트 클라우드 퍼블리시

### 7.1 포인트 클라우드 생성 노드

```python
"""
pointcloud_publisher.py
깊이 이미지를 PointCloud2로 변환하여 퍼블리시
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, PointCloud2, PointField
from cv_bridge import CvBridge
import numpy as np
import struct
import yaml


class PointCloudPublisher(Node):
    def __init__(self):
        super().__init__('pointcloud_publisher')
        
        # 파라미터
        self.declare_parameter('calibration_file', '')
        self.declare_parameter('max_depth', 5.0)  # meters
        self.declare_parameter('downsample', 2)   # 다운샘플 비율
        self.declare_parameter('frame_id', 'stereo_camera_left')
        
        self.calibration_file = self.get_parameter('calibration_file').value
        self.max_depth = self.get_parameter('max_depth').value
        self.downsample = self.get_parameter('downsample').value
        self.frame_id = self.get_parameter('frame_id').value
        
        # 캘리브레이션 로드
        self.load_calibration()
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Publisher
        self.pub_pointcloud = self.create_publisher(
            PointCloud2, '/stereo/points', sensor_qos)
        
        # Subscriber
        self.sub_depth = self.create_subscription(
            Image, '/stereo/depth', self.depth_callback, sensor_qos)
        self.sub_color = self.create_subscription(
            Image, '/stereo/left/image_rect', self.color_callback, sensor_qos)
        
        self.latest_color = None
        
        self.get_logger().info('포인트 클라우드 퍼블리셔 시작')
    
    def load_calibration(self):
        """캘리브레이션 로드"""
        
        with open(self.calibration_file, 'r') as f:
            params = yaml.safe_load(f)
        
        self.P1 = np.array(params['P1'])
        self.fx = self.P1[0, 0]
        self.fy = self.P1[1, 1]
        self.cx = self.P1[0, 2]
        self.cy = self.P1[1, 2]
        
        self.get_logger().info(f'카메라 파라미터: fx={self.fx:.1f}, fy={self.fy:.1f}')
    
    def color_callback(self, msg):
        """컬러 이미지 저장"""
        self.latest_color = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
    
    def depth_callback(self, msg):
        """깊이 이미지 → 포인트 클라우드 변환"""
        
        # 깊이 이미지 변환
        depth = self.bridge.imgmsg_to_cv2(msg, '32FC1')
        
        h, w = depth.shape
        
        # 다운샘플링
        step = self.downsample
        
        # 좌표 그리드 생성
        u = np.arange(0, w, step)
        v = np.arange(0, h, step)
        u, v = np.meshgrid(u, v)
        
        # 깊이 추출
        z = depth[::step, ::step]
        
        # 유효한 깊이 마스크
        valid = (z > 0) & (z < self.max_depth)
        
        # 3D 좌표 계산
        x = (u - self.cx) * z / self.fx
        y = (v - self.cy) * z / self.fy
        
        # 유효한 점만 추출
        points_x = x[valid]
        points_y = y[valid]
        points_z = z[valid]
        
        # 색상 추출
        if self.latest_color is not None:
            color_downsampled = self.latest_color[::step, ::step]
            colors = color_downsampled[valid]
            # BGR → RGB로 패킹
            rgb = np.zeros(len(colors), dtype=np.uint32)
            rgb = (colors[:, 2].astype(np.uint32) << 16 |
                   colors[:, 1].astype(np.uint32) << 8 |
                   colors[:, 0].astype(np.uint32))
        else:
            # 색상 없으면 흰색
            rgb = np.full(len(points_x), 0xFFFFFF, dtype=np.uint32)
        
        # PointCloud2 메시지 생성
        cloud_msg = self.create_pointcloud2(
            points_x, points_y, points_z, rgb,
            msg.header.stamp
        )
        
        self.pub_pointcloud.publish(cloud_msg)
    
    def create_pointcloud2(self, x, y, z, rgb, stamp):
        """PointCloud2 메시지 생성"""
        
        # 포인트 수
        n_points = len(x)
        
        # 필드 정의
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
        ]
        
        # 데이터 패킹
        point_step = 16  # 4*4 bytes
        data = np.zeros(n_points * point_step, dtype=np.uint8)
        
        # 구조화된 배열로 변환
        points_arr = np.zeros(n_points, dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('rgb', np.uint32)
        ])
        
        points_arr['x'] = x.astype(np.float32)
        points_arr['y'] = y.astype(np.float32)
        points_arr['z'] = z.astype(np.float32)
        points_arr['rgb'] = rgb
        
        data = points_arr.tobytes()
        
        # 메시지 생성
        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        
        msg.height = 1
        msg.width = n_points
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = point_step
        msg.row_step = point_step * n_points
        msg.data = data
        msg.is_dense = True
        
        return msg


def main(args=None):
    rclpy.init(args=args)
    
    node = PointCloudPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## 8. TF2 프레임 설정

### 8.1 TF2 브로드캐스터

```python
"""
tf_broadcaster.py
스테레오 카메라 TF 프레임 브로드캐스트
"""

import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import yaml
import numpy as np


class StereoCameraTFBroadcaster(Node):
    def __init__(self):
        super().__init__('stereo_camera_tf_broadcaster')
        
        # 파라미터
        self.declare_parameter('calibration_file', '')
        self.declare_parameter('parent_frame', 'base_link')
        self.declare_parameter('camera_frame', 'stereo_camera')
        
        self.calibration_file = self.get_parameter('calibration_file').value
        self.parent_frame = self.get_parameter('parent_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        
        # Static TF Broadcaster
        self.static_broadcaster = StaticTransformBroadcaster(self)
        
        # 캘리브레이션 로드 및 TF 퍼블리시
        self.load_and_broadcast()
        
        self.get_logger().info('TF 브로드캐스터 시작')
    
    def load_and_broadcast(self):
        """캘리브레이션 로드 및 Static TF 퍼블리시"""
        
        # 캘리브레이션 로드
        if self.calibration_file:
            with open(self.calibration_file, 'r') as f:
                params = yaml.safe_load(f)
            baseline = params['baseline_mm'] / 1000.0  # mm → m
        else:
            baseline = 0.085  # 기본값 85mm
        
        transforms = []
        
        # 1. base_link → stereo_camera (카메라 마운트 위치)
        t1 = TransformStamped()
        t1.header.stamp = self.get_clock().now().to_msg()
        t1.header.frame_id = self.parent_frame
        t1.child_frame_id = self.camera_frame
        # 카메라가 로봇 앞쪽 0.1m, 높이 0.3m에 위치한다고 가정
        t1.transform.translation.x = 0.1
        t1.transform.translation.y = 0.0
        t1.transform.translation.z = 0.3
        t1.transform.rotation.x = 0.0
        t1.transform.rotation.y = 0.0
        t1.transform.rotation.z = 0.0
        t1.transform.rotation.w = 1.0
        transforms.append(t1)
        
        # 2. stereo_camera → stereo_camera_left
        t2 = TransformStamped()
        t2.header.stamp = self.get_clock().now().to_msg()
        t2.header.frame_id = self.camera_frame
        t2.child_frame_id = f'{self.camera_frame}_left'
        t2.transform.translation.x = 0.0
        t2.transform.translation.y = baseline / 2  # 왼쪽으로
        t2.transform.translation.z = 0.0
        t2.transform.rotation.w = 1.0
        transforms.append(t2)
        
        # 3. stereo_camera → stereo_camera_right
        t3 = TransformStamped()
        t3.header.stamp = self.get_clock().now().to_msg()
        t3.header.frame_id = self.camera_frame
        t3.child_frame_id = f'{self.camera_frame}_right'
        t3.transform.translation.x = 0.0
        t3.transform.translation.y = -baseline / 2  # 오른쪽으로
        t3.transform.translation.z = 0.0
        t3.transform.rotation.w = 1.0
        transforms.append(t3)
        
        # 4. stereo_camera_left → stereo_camera_left_optical
        # 광학 프레임: Z가 앞쪽, X가 오른쪽, Y가 아래쪽
        t4 = TransformStamped()
        t4.header.stamp = self.get_clock().now().to_msg()
        t4.header.frame_id = f'{self.camera_frame}_left'
        t4.child_frame_id = f'{self.camera_frame}_left_optical'
        # 90도 회전 (ROS 좌표계 → 광학 좌표계)
        # R = Rz(-90) * Ry(-90)
        t4.transform.rotation.x = -0.5
        t4.transform.rotation.y = 0.5
        t4.transform.rotation.z = -0.5
        t4.transform.rotation.w = 0.5
        transforms.append(t4)
        
        # Static TF 퍼블리시
        self.static_broadcaster.sendTransform(transforms)
        
        self.get_logger().info(f'Static TF 퍼블리시 완료 (베이스라인: {baseline*1000:.1f}mm)')


def main(args=None):
    rclpy.init(args=args)
    
    node = StereoCameraTFBroadcaster()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 8.2 TF 프레임 구조

```
┌─────────────────────────────────────────────────────────────┐
│                    TF 프레임 구조                            │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│                    base_link                                │
│                        │                                    │
│                        │ (x: 0.1, y: 0, z: 0.3)            │
│                        ▼                                    │
│                  stereo_camera                              │
│                    /       \                                │
│     (y: +B/2)    /         \    (y: -B/2)                  │
│                 ▼           ▼                               │
│    stereo_camera_left    stereo_camera_right               │
│              │                                              │
│              │ (rotation)                                   │
│              ▼                                              │
│    stereo_camera_left_optical                              │
│                                                             │
│    ROS 좌표계:      광학 좌표계:                             │
│         Z↑              Z→ (앞)                             │
│         │              /                                    │
│         │            /                                      │
│    X←───┼───→Y      Y↓                                     │
│                     X→ (오른쪽)                              │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

---

## 9. Launch 파일 작성

### 9.1 Python Launch 파일

```python
"""
stereo_camera.launch.py
스테레오 카메라 시스템 런치 파일
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 패키지 경로
    pkg_share = FindPackageShare('stereo_camera')
    
    # Launch Arguments
    calibration_file_arg = DeclareLaunchArgument(
        'calibration_file',
        default_value=PathJoinSubstitution([pkg_share, 'config', 'stereo_params.yaml']),
        description='스테레오 캘리브레이션 파일 경로'
    )
    
    left_camera_arg = DeclareLaunchArgument(
        'left_camera_id',
        default_value='0',
        description='왼쪽 카메라 인덱스'
    )
    
    right_camera_arg = DeclareLaunchArgument(
        'right_camera_id',
        default_value='2',
        description='오른쪽 카메라 인덱스'
    )
    
    # 노드 정의
    stereo_camera_node = Node(
        package='stereo_camera',
        executable='stereo_camera_node',
        name='stereo_camera_node',
        output='screen',
        parameters=[{
            'left_camera_id': LaunchConfiguration('left_camera_id'),
            'right_camera_id': LaunchConfiguration('right_camera_id'),
            'frame_rate': 30.0,
            'image_width': 1920,
            'image_height': 1080,
            'calibration_file': LaunchConfiguration('calibration_file'),
            'frame_id': 'stereo_camera',
        }]
    )
    
    stereo_processor_node = Node(
        package='stereo_camera',
        executable='stereo_processor_node',
        name='stereo_processor_node',
        output='screen',
        parameters=[{
            'calibration_file': LaunchConfiguration('calibration_file'),
            'num_disparities': 128,
            'block_size': 5,
            'min_depth': 100.0,
            'max_depth': 10000.0,
            'frame_id': 'stereo_camera',
        }]
    )
    
    pointcloud_publisher_node = Node(
        package='stereo_camera',
        executable='pointcloud_publisher',
        name='pointcloud_publisher',
        output='screen',
        parameters=[{
            'calibration_file': LaunchConfiguration('calibration_file'),
            'max_depth': 5.0,
            'downsample': 2,
            'frame_id': 'stereo_camera_left',
        }]
    )
    
    tf_broadcaster_node = Node(
        package='stereo_camera',
        executable='tf_broadcaster',
        name='stereo_tf_broadcaster',
        output='screen',
        parameters=[{
            'calibration_file': LaunchConfiguration('calibration_file'),
            'parent_frame': 'base_link',
            'camera_frame': 'stereo_camera',
        }]
    )
    
    # Rviz2 (선택)
    rviz_config = PathJoinSubstitution([pkg_share, 'rviz', 'stereo_camera.rviz'])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )
    
    return LaunchDescription([
        calibration_file_arg,
        left_camera_arg,
        right_camera_arg,
        stereo_camera_node,
        stereo_processor_node,
        pointcloud_publisher_node,
        tf_broadcaster_node,
        rviz_node,
    ])
```

### 9.2 실행 방법

```bash
# 빌드
cd ~/ros2_ws
colcon build --packages-select stereo_camera
source install/setup.bash

# 실행
ros2 launch stereo_camera stereo_camera.launch.py

# 파라미터 변경
ros2 launch stereo_camera stereo_camera.launch.py \
    left_camera_id:=0 \
    right_camera_id:=2 \
    calibration_file:=/path/to/calibration.yaml
```

---

## 10. Rviz2 시각화

### 10.1 Rviz2 설정 파일

```yaml
# stereo_camera.rviz
Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Views
    Name: Views

Visualization Manager:
  Displays:
    # TF 프레임
    - Class: rviz_default_plugins/TF
      Name: TF
      Enabled: true
      Frame Timeout: 15
      Marker Scale: 0.5
      Show Arrows: true
      Show Axes: true
      Show Names: true

    # 왼쪽 이미지
    - Class: rviz_default_plugins/Image
      Name: Left Image
      Enabled: true
      Topic:
        Value: /stereo/left/image_rect
        Depth: 5
      Normalize Range: true

    # 시차 맵
    - Class: rviz_default_plugins/Image
      Name: Disparity
      Enabled: true
      Topic:
        Value: /stereo/disparity
        Depth: 5
      Normalize Range: true

    # 포인트 클라우드
    - Class: rviz_default_plugins/PointCloud2
      Name: PointCloud
      Enabled: true
      Topic:
        Value: /stereo/points
        Depth: 5
      Style: Points
      Size (Pixels): 2
      Color Transformer: RGB8
      Decay Time: 0

    # 그리드
    - Class: rviz_default_plugins/Grid
      Name: Grid
      Enabled: true
      Cell Size: 1
      Plane Cell Count: 10

  Global Options:
    Fixed Frame: stereo_camera_left
    Frame Rate: 30

  Tools:
    - Class: rviz_default_plugins/Interact
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select

  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 5
      Pitch: 0.5
      Yaw: 3.14
      Focal Point:
        X: 0
        Y: 0
        Z: 0
```

### 10.2 토픽 확인

```bash
# 토픽 목록
ros2 topic list

# 예상 출력:
# /stereo/left/image_raw
# /stereo/left/image_rect
# /stereo/left/camera_info
# /stereo/right/image_raw
# /stereo/right/image_rect
# /stereo/right/camera_info
# /stereo/disparity
# /stereo/depth
# /stereo/points
# /tf
# /tf_static

# 토픽 주파수 확인
ros2 topic hz /stereo/left/image_rect

# 토픽 정보
ros2 topic info /stereo/points

# 이미지 보기 (rqt)
ros2 run rqt_image_view rqt_image_view
```

### 10.3 시각화 화면

```
┌─────────────────────────────────────────────────────────────┐
│                        Rviz2                                │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────┐ │
│  │ Left Image  │  │  Disparity  │  │                     │ │
│  │             │  │             │  │                     │ │
│  │   📷        │  │   🌈        │  │     PointCloud     │ │
│  │             │  │             │  │        · · ·       │ │
│  │             │  │             │  │       · · · ·      │ │
│  └─────────────┘  └─────────────┘  │      · · · · ·     │ │
│                                     │       · · · ·      │ │
│  ┌─────────────────────────────┐   │        · · ·       │ │
│  │ Displays     [+][-]         │   │                     │ │
│  │  ☑ TF                       │   │     ↑Z             │ │
│  │  ☑ Left Image               │   │     │   Y          │ │
│  │  ☑ Disparity                │   │     │  /           │ │
│  │  ☑ PointCloud               │   │     │ /            │ │
│  │  ☑ Grid                     │   │     └────→X        │ │
│  └─────────────────────────────┘   └─────────────────────┘ │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

---

## 📝 학습 체크리스트

### 이론 이해

- [ ] ROS2의 토픽, 서비스, 액션 개념을 이해했다
- [ ] QoS (Quality of Service) 설정의 의미를 알고 있다
- [ ] TF2 프레임의 필요성을 설명할 수 있다
- [ ] sensor_msgs/Image와 PointCloud2 구조를 이해했다

### 실습 완료

- [ ] ROS2 Humble 설치
- [ ] 워크스페이스 및 패키지 생성
- [ ] 스테레오 카메라 노드 구현
- [ ] 깊이 이미지 퍼블리시
- [ ] 포인트 클라우드 퍼블리시
- [ ] TF2 프레임 설정
- [ ] Launch 파일 작성
- [ ] Rviz2에서 시각화

---

## ➡️ 다음 모듈

**[Module 07: 임베디드 구현 (Jetson/STM32H7)](../Module_07_Embedded/README.md)**

다음 모듈에서는:
- Jetson Nano/Xavier에서의 구현
- STM32H7을 이용한 저수준 구현
- 최적화 기법
- 실시간 처리

를 학습합니다.

---

## 📄 라이선스

MIT License - 자유롭게 사용, 수정, 배포 가능
