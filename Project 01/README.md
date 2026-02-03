# Project 01: 실시간 거리 측정기

[![난이도](https://img.shields.io/badge/난이도-⭐⭐⭐_중급-yellow.svg)]()
[![예상시간](https://img.shields.io/badge/예상시간-8--12시간-blue.svg)]()
[![선수지식](https://img.shields.io/badge/선수지식-Module_01--03_완료-orange.svg)]()

---

## 🎯 프로젝트 개요

| 항목 | 내용 |
|------|------|
| **목표** | USB 웹캠 스테레오 카메라로 실시간 거리 측정기 구현 |
| **기능** | 화면 중앙/클릭 지점 거리 측정, 다중 포인트 측정, GUI |
| **출력** | 거리 표시 오버레이, 측정 로그, CSV 저장 |

---

## 📋 목차

1. [프로젝트 구조](#1-프로젝트-구조)
2. [요구사항](#2-요구사항)
3. [시스템 아키텍처](#3-시스템-아키텍처)
4. [핵심 모듈 구현](#4-핵심-모듈-구현)
5. [GUI 구현](#5-gui-구현)
6. [측정 모드](#6-측정-모드)
7. [정확도 향상 기법](#7-정확도-향상-기법)
8. [결과 저장 및 로깅](#8-결과-저장-및-로깅)
9. [전체 코드](#9-전체-코드)
10. [테스트 및 검증](#10-테스트-및-검증)

---

## 1. 프로젝트 구조

```
Project_01_Distance_Meter/
├── README.md
├── requirements.txt
├── config/
│   ├── stereo_params.yaml      # 캘리브레이션 파라미터
│   └── app_config.yaml         # 앱 설정
├── src/
│   ├── __init__.py
│   ├── main.py                 # 메인 실행 파일
│   ├── stereo_camera.py        # 스테레오 카메라 클래스
│   ├── depth_estimator.py      # 깊이 추정 클래스
│   ├── distance_meter.py       # 거리 측정 클래스
│   ├── gui.py                  # GUI 클래스
│   └── utils.py                # 유틸리티 함수
├── logs/
│   └── measurements.csv        # 측정 로그
└── screenshots/
    └── ...                     # 스크린샷
```

---

## 2. 요구사항

### 2.1 하드웨어

```
┌─────────────────────────────────────────────────────────────┐
│                    하드웨어 요구사항                         │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  📷 USB 웹캠 2대                                            │
│     - 동일 모델 권장                                        │
│     - 해상도: 720p 이상                                     │
│     - 프레임: 30fps 이상                                    │
│                                                             │
│  📐 스테레오 마운트                                          │
│     - 베이스라인: 60-120mm                                  │
│     - 평행 정렬 필수                                        │
│                                                             │
│  💻 PC                                                      │
│     - CPU: Intel i5 이상                                    │
│     - RAM: 8GB 이상                                         │
│     - USB 3.0 포트 2개                                      │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### 2.2 소프트웨어

```bash
# requirements.txt
opencv-python>=4.5.0
opencv-contrib-python>=4.5.0
numpy>=1.20.0
PyYAML>=6.0
tkinter  # Python 기본 포함
matplotlib>=3.4.0
pandas>=1.3.0
```

### 2.3 설치

```bash
# 가상환경 생성
python -m venv venv
source venv/bin/activate  # Linux/Mac
# venv\Scripts\activate   # Windows

# 패키지 설치
pip install -r requirements.txt
```

---

## 3. 시스템 아키텍처

```
┌─────────────────────────────────────────────────────────────┐
│                   실시간 거리 측정기 아키텍처                  │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│   ┌─────────┐     ┌─────────┐                              │
│   │ Camera  │     │ Camera  │                              │
│   │  Left   │     │  Right  │                              │
│   └────┬────┘     └────┬────┘                              │
│        │               │                                    │
│        └───────┬───────┘                                    │
│                ▼                                            │
│        ┌──────────────┐                                    │
│        │ StereoCamera │                                    │
│        │    Class     │                                    │
│        └──────┬───────┘                                    │
│                │                                            │
│        ┌───────┴───────┐                                   │
│        ▼               ▼                                    │
│   ┌─────────┐    ┌─────────┐                              │
│   │Rectify  │    │Rectify  │                              │
│   │ Left    │    │ Right   │                              │
│   └────┬────┘    └────┬────┘                              │
│        │               │                                    │
│        └───────┬───────┘                                    │
│                ▼                                            │
│        ┌──────────────┐                                    │
│        │   Stereo     │                                    │
│        │   Matcher    │                                    │
│        │   (SGBM)     │                                    │
│        └──────┬───────┘                                    │
│                │                                            │
│                ▼                                            │
│        ┌──────────────┐                                    │
│        │  Disparity   │                                    │
│        │     Map      │                                    │
│        └──────┬───────┘                                    │
│                │                                            │
│                ▼                                            │
│        ┌──────────────┐                                    │
│        │    Depth     │                                    │
│        │     Map      │                                    │
│        └──────┬───────┘                                    │
│                │                                            │
│        ┌───────┴───────┐                                   │
│        ▼               ▼                                    │
│   ┌─────────┐    ┌──────────┐                             │
│   │Distance │    │   GUI    │                             │
│   │ Meter   │    │ Display  │                             │
│   └─────────┘    └──────────┘                             │
│        │               │                                    │
│        ▼               ▼                                    │
│   ┌─────────┐    ┌──────────┐                             │
│   │  Log    │    │  User    │                             │
│   │ Output  │    │Interface │                             │
│   └─────────┘    └──────────┘                             │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

---

## 4. 핵심 모듈 구현

### 4.1 스테레오 카메라 클래스

```python
"""
stereo_camera.py
스테레오 카메라 캡처 및 정류 클래스
"""

import cv2
import numpy as np
import yaml
from typing import Tuple, Optional


class StereoCamera:
    """스테레오 카메라 관리 클래스"""
    
    def __init__(self, calibration_file: str, 
                 left_id: int = 0, right_id: int = 2,
                 width: int = 1280, height: int = 720):
        """
        Parameters:
        - calibration_file: 캘리브레이션 YAML 파일 경로
        - left_id, right_id: 카메라 인덱스
        - width, height: 캡처 해상도
        """
        
        self.width = width
        self.height = height
        
        # 캘리브레이션 로드
        self._load_calibration(calibration_file)
        
        # 카메라 초기화
        self._init_cameras(left_id, right_id)
        
        # 상태
        self.is_running = False
        
    def _load_calibration(self, filename: str):
        """캘리브레이션 파라미터 로드"""
        
        with open(filename, 'r') as f:
            params = yaml.safe_load(f)
        
        self.img_size = tuple(params['image_size'])
        self.K1 = np.array(params['K1'])
        self.D1 = np.array(params['D1'])
        self.K2 = np.array(params['K2'])
        self.D2 = np.array(params['D2'])
        self.R1 = np.array(params['R1'])
        self.R2 = np.array(params['R2'])
        self.P1 = np.array(params['P1'])
        self.P2 = np.array(params['P2'])
        self.Q = np.array(params['Q'])
        self.baseline = params['baseline_mm']
        self.focal_length = self.P1[0, 0]
        
        # 정류 맵 생성
        self.map1_left, self.map2_left = cv2.initUndistortRectifyMap(
            self.K1, self.D1, self.R1, self.P1, self.img_size, cv2.CV_32FC1
        )
        self.map1_right, self.map2_right = cv2.initUndistortRectifyMap(
            self.K2, self.D2, self.R2, self.P2, self.img_size, cv2.CV_32FC1
        )
        
        print(f"✅ 캘리브레이션 로드 완료")
        print(f"   이미지 크기: {self.img_size}")
        print(f"   베이스라인: {self.baseline:.1f} mm")
        print(f"   초점거리: {self.focal_length:.1f} px")
    
    def _init_cameras(self, left_id: int, right_id: int):
        """카메라 초기화"""
        
        self.cap_left = cv2.VideoCapture(left_id)
        self.cap_right = cv2.VideoCapture(right_id)
        
        # 설정
        for cap in [self.cap_left, self.cap_right]:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            cap.set(cv2.CAP_PROP_FPS, 30)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # 버퍼 최소화
        
        if not self.cap_left.isOpened() or not self.cap_right.isOpened():
            raise RuntimeError("카메라 열기 실패")
        
        self.is_running = True
        print(f"✅ 카메라 초기화 완료 (Left: {left_id}, Right: {right_id})")
    
    def capture(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """
        프레임 캡처
        
        Returns:
        - frame_left, frame_right: BGR 이미지 또는 None
        """
        
        if not self.is_running:
            return None, None
        
        ret_l, frame_left = self.cap_left.read()
        ret_r, frame_right = self.cap_right.read()
        
        if not ret_l or not ret_r:
            return None, None
        
        return frame_left, frame_right
    
    def rectify(self, frame_left: np.ndarray, 
                frame_right: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        이미지 정류
        
        Returns:
        - rect_left, rect_right: 정류된 이미지
        """
        
        rect_left = cv2.remap(frame_left, self.map1_left, self.map2_left, 
                              cv2.INTER_LINEAR)
        rect_right = cv2.remap(frame_right, self.map1_right, self.map2_right,
                               cv2.INTER_LINEAR)
        
        return rect_left, rect_right
    
    def capture_rectified(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """
        캡처 및 정류 동시 수행
        
        Returns:
        - rect_left, rect_right: 정류된 이미지 또는 None
        """
        
        frame_left, frame_right = self.capture()
        
        if frame_left is None or frame_right is None:
            return None, None
        
        return self.rectify(frame_left, frame_right)
    
    def release(self):
        """카메라 리소스 해제"""
        
        self.is_running = False
        self.cap_left.release()
        self.cap_right.release()
        print("카메라 리소스 해제됨")
    
    def __del__(self):
        self.release()
```

### 4.2 깊이 추정 클래스

```python
"""
depth_estimator.py
스테레오 매칭 및 깊이 추정 클래스
"""

import cv2
import numpy as np
from typing import Tuple, Optional


class DepthEstimator:
    """깊이 추정 클래스"""
    
    def __init__(self, focal_length: float, baseline: float,
                 num_disparities: int = 128, block_size: int = 5):
        """
        Parameters:
        - focal_length: 초점거리 (픽셀)
        - baseline: 베이스라인 (mm)
        - num_disparities: 시차 범위 (16의 배수)
        - block_size: 블록 크기 (홀수)
        """
        
        self.focal_length = focal_length
        self.baseline = baseline
        
        # SGBM 매처 생성
        self.stereo = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=num_disparities,
            blockSize=block_size,
            P1=8 * 3 * block_size ** 2,
            P2=32 * 3 * block_size ** 2,
            disp12MaxDiff=1,
            uniquenessRatio=10,
            speckleWindowSize=100,
            speckleRange=2,
            preFilterCap=63,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
        )
        
        # WLS 필터 (선택적)
        self.use_wls = True
        if self.use_wls:
            self.right_matcher = cv2.ximgproc.createRightMatcher(self.stereo)
            self.wls_filter = cv2.ximgproc.createDisparityWLSFilter(
                matcher_left=self.stereo
            )
            self.wls_filter.setLambda(8000)
            self.wls_filter.setSigmaColor(1.5)
        
        print(f"✅ 깊이 추정기 초기화")
        print(f"   시차 범위: 0-{num_disparities}")
        print(f"   WLS 필터: {'활성화' if self.use_wls else '비활성화'}")
    
    def compute_disparity(self, rect_left: np.ndarray, 
                          rect_right: np.ndarray) -> np.ndarray:
        """
        시차 맵 계산
        
        Returns:
        - disparity: 시차 맵 (float32)
        """
        
        # 왼쪽 시차
        disp_left = self.stereo.compute(rect_left, rect_right)
        
        if self.use_wls:
            # 오른쪽 시차
            disp_right = self.right_matcher.compute(rect_right, rect_left)
            
            # WLS 필터 적용
            disp_left = self.wls_filter.filter(
                disp_left, rect_left, 
                disparity_map_right=disp_right
            )
        
        # 실제 값으로 변환 (16으로 나눔)
        disparity = disp_left.astype(np.float32) / 16.0
        
        return disparity
    
    def disparity_to_depth(self, disparity: np.ndarray) -> np.ndarray:
        """
        시차를 깊이로 변환
        
        Z = f × B / d
        
        Returns:
        - depth: 깊이 맵 (mm)
        """
        
        depth = np.zeros_like(disparity)
        valid = disparity > 0
        
        depth[valid] = (self.focal_length * self.baseline) / disparity[valid]
        
        return depth
    
    def compute_depth(self, rect_left: np.ndarray,
                      rect_right: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        깊이 맵 계산 (시차 + 깊이 변환)
        
        Returns:
        - disparity: 시차 맵
        - depth: 깊이 맵 (mm)
        """
        
        disparity = self.compute_disparity(rect_left, rect_right)
        depth = self.disparity_to_depth(disparity)
        
        return disparity, depth
    
    def get_depth_at_point(self, depth: np.ndarray, 
                           x: int, y: int, 
                           window_size: int = 5) -> Tuple[float, float]:
        """
        특정 좌표의 깊이 (윈도우 평균)
        
        Parameters:
        - depth: 깊이 맵
        - x, y: 좌표
        - window_size: 평균 윈도우 크기
        
        Returns:
        - mean_depth: 평균 깊이 (mm)
        - std_depth: 표준편차 (mm)
        """
        
        h, w = depth.shape
        half = window_size // 2
        
        # 경계 체크
        x1 = max(0, x - half)
        x2 = min(w, x + half + 1)
        y1 = max(0, y - half)
        y2 = min(h, y + half + 1)
        
        # 윈도우 추출
        window = depth[y1:y2, x1:x2]
        
        # 유효한 값만
        valid = window[window > 0]
        
        if len(valid) == 0:
            return 0.0, 0.0
        
        return float(np.mean(valid)), float(np.std(valid))
```

### 4.3 거리 측정 클래스

```python
"""
distance_meter.py
거리 측정 및 관리 클래스
"""

import numpy as np
from typing import List, Tuple, Optional
from dataclasses import dataclass
from datetime import datetime
import csv
import os


@dataclass
class MeasurementPoint:
    """측정 포인트 데이터"""
    x: int
    y: int
    distance_mm: float
    std_mm: float
    timestamp: datetime
    label: str = ""


class DistanceMeter:
    """거리 측정 관리 클래스"""
    
    def __init__(self, depth_estimator, 
                 min_depth: float = 100, 
                 max_depth: float = 10000):
        """
        Parameters:
        - depth_estimator: DepthEstimator 인스턴스
        - min_depth: 최소 측정 거리 (mm)
        - max_depth: 최대 측정 거리 (mm)
        """
        
        self.depth_estimator = depth_estimator
        self.min_depth = min_depth
        self.max_depth = max_depth
        
        # 측정 포인트 목록
        self.measurement_points: List[MeasurementPoint] = []
        
        # 현재 깊이 맵
        self.current_depth: Optional[np.ndarray] = None
        
        # 측정 히스토리 (시간 평균용)
        self.history_size = 5
        self.depth_history: List[np.ndarray] = []
    
    def update(self, rect_left: np.ndarray, rect_right: np.ndarray):
        """
        새 프레임으로 깊이 맵 업데이트
        """
        
        disparity, depth = self.depth_estimator.compute_depth(rect_left, rect_right)
        
        # 범위 제한
        depth[(depth < self.min_depth) | (depth > self.max_depth)] = 0
        
        # 히스토리에 추가
        self.depth_history.append(depth.copy())
        if len(self.depth_history) > self.history_size:
            self.depth_history.pop(0)
        
        # 시간 평균 (노이즈 감소)
        if len(self.depth_history) >= 3:
            stacked = np.stack(self.depth_history, axis=0)
            # 유효한 값만 평균
            with np.errstate(invalid='ignore'):
                self.current_depth = np.nanmean(
                    np.where(stacked > 0, stacked, np.nan), axis=0
                )
            self.current_depth = np.nan_to_num(self.current_depth, nan=0)
        else:
            self.current_depth = depth
        
        return disparity, self.current_depth
    
    def measure_point(self, x: int, y: int, 
                      window_size: int = 7,
                      label: str = "") -> Optional[MeasurementPoint]:
        """
        특정 좌표의 거리 측정
        
        Returns:
        - MeasurementPoint 또는 None
        """
        
        if self.current_depth is None:
            return None
        
        mean_depth, std_depth = self.depth_estimator.get_depth_at_point(
            self.current_depth, x, y, window_size
        )
        
        if mean_depth <= 0:
            return None
        
        point = MeasurementPoint(
            x=x,
            y=y,
            distance_mm=mean_depth,
            std_mm=std_depth,
            timestamp=datetime.now(),
            label=label
        )
        
        return point
    
    def measure_center(self, image_shape: Tuple[int, int]) -> Optional[MeasurementPoint]:
        """
        이미지 중앙 거리 측정
        """
        
        h, w = image_shape[:2]
        return self.measure_point(w // 2, h // 2, label="Center")
    
    def add_measurement(self, point: MeasurementPoint):
        """측정 포인트 추가"""
        self.measurement_points.append(point)
    
    def clear_measurements(self):
        """측정 포인트 초기화"""
        self.measurement_points.clear()
    
    def remove_last_measurement(self):
        """마지막 측정 포인트 삭제"""
        if self.measurement_points:
            self.measurement_points.pop()
    
    def get_distance_between(self, idx1: int, idx2: int) -> Optional[float]:
        """
        두 측정 포인트 간 3D 거리 계산
        
        Returns:
        - 3D 거리 (mm) 또는 None
        """
        
        if idx1 >= len(self.measurement_points) or idx2 >= len(self.measurement_points):
            return None
        
        p1 = self.measurement_points[idx1]
        p2 = self.measurement_points[idx2]
        
        # 2D 픽셀 거리
        dx_px = p2.x - p1.x
        dy_px = p2.y - p1.y
        
        # 3D 근사 (같은 깊이 평면 가정)
        avg_depth = (p1.distance_mm + p2.distance_mm) / 2
        
        # 픽셀 → mm 변환 (간단한 근사)
        scale = avg_depth / self.depth_estimator.focal_length
        dx_mm = dx_px * scale
        dy_mm = dy_px * scale
        dz_mm = p2.distance_mm - p1.distance_mm
        
        distance_3d = np.sqrt(dx_mm**2 + dy_mm**2 + dz_mm**2)
        
        return distance_3d
    
    def save_measurements(self, filename: str):
        """측정 결과를 CSV로 저장"""
        
        os.makedirs(os.path.dirname(filename), exist_ok=True)
        
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Label', 'X', 'Y', 'Distance_mm', 'Std_mm', 'Timestamp'])
            
            for p in self.measurement_points:
                writer.writerow([
                    p.label, p.x, p.y, 
                    f"{p.distance_mm:.1f}", 
                    f"{p.std_mm:.1f}",
                    p.timestamp.isoformat()
                ])
        
        print(f"✅ 측정 결과 저장: {filename}")
    
    def get_statistics(self) -> dict:
        """측정 통계"""
        
        if not self.measurement_points:
            return {}
        
        distances = [p.distance_mm for p in self.measurement_points]
        
        return {
            'count': len(distances),
            'min_mm': min(distances),
            'max_mm': max(distances),
            'mean_mm': np.mean(distances),
            'std_mm': np.std(distances)
        }
```

---

## 5. GUI 구현

### 5.1 메인 GUI 클래스

```python
"""
gui.py
거리 측정기 GUI
"""

import cv2
import numpy as np
from typing import Optional, Callable
import time


class DistanceMeterGUI:
    """거리 측정기 GUI"""
    
    def __init__(self, window_name: str = "Distance Meter"):
        """
        Parameters:
        - window_name: 윈도우 이름
        """
        
        self.window_name = window_name
        
        # 마우스 콜백
        self.mouse_callback: Optional[Callable] = None
        self.mouse_x = 0
        self.mouse_y = 0
        self.clicked = False
        self.click_x = 0
        self.click_y = 0
        
        # 표시 설정
        self.show_disparity = False
        self.show_depth_map = True
        self.show_crosshair = True
        self.show_measurements = True
        
        # 색상
        self.color_text = (0, 255, 0)
        self.color_crosshair = (0, 255, 255)
        self.color_point = (0, 0, 255)
        self.color_distance = (255, 255, 0)
        
        # FPS 계산
        self.fps_history = []
        self.last_time = time.time()
        
        # 윈도우 생성
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(self.window_name, self._mouse_callback)
    
    def _mouse_callback(self, event, x, y, flags, param):
        """마우스 콜백"""
        
        self.mouse_x = x
        self.mouse_y = y
        
        if event == cv2.EVENT_LBUTTONDOWN:
            self.clicked = True
            self.click_x = x
            self.click_y = y
    
    def check_click(self) -> Optional[tuple]:
        """
        클릭 이벤트 확인
        
        Returns:
        - (x, y) 또는 None
        """
        
        if self.clicked:
            self.clicked = False
            return (self.click_x, self.click_y)
        return None
    
    def update_fps(self):
        """FPS 업데이트"""
        
        current_time = time.time()
        fps = 1.0 / (current_time - self.last_time + 1e-6)
        self.last_time = current_time
        
        self.fps_history.append(fps)
        if len(self.fps_history) > 30:
            self.fps_history.pop(0)
    
    def get_fps(self) -> float:
        """평균 FPS 반환"""
        return np.mean(self.fps_history) if self.fps_history else 0
    
    def draw_crosshair(self, image: np.ndarray, x: int, y: int, size: int = 20):
        """십자선 그리기"""
        
        cv2.line(image, (x - size, y), (x + size, y), self.color_crosshair, 2)
        cv2.line(image, (x, y - size), (x, y + size), self.color_crosshair, 2)
        cv2.circle(image, (x, y), 5, self.color_crosshair, -1)
    
    def draw_measurement_point(self, image: np.ndarray, 
                               x: int, y: int, 
                               distance_mm: float,
                               label: str = "",
                               idx: int = -1):
        """측정 포인트 표시"""
        
        # 마커
        cv2.drawMarker(image, (x, y), self.color_point, 
                      cv2.MARKER_CROSS, 15, 2)
        cv2.circle(image, (x, y), 8, self.color_point, 2)
        
        # 거리 텍스트
        if distance_mm > 0:
            text = f"{distance_mm/1000:.2f}m"
            if label:
                text = f"{label}: {text}"
            if idx >= 0:
                text = f"[{idx}] {text}"
            
            cv2.putText(image, text, (x + 15, y - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, self.color_point, 2)
    
    def draw_info_panel(self, image: np.ndarray, 
                        center_distance: float,
                        fps: float,
                        num_points: int):
        """정보 패널 그리기"""
        
        h, w = image.shape[:2]
        
        # 반투명 배경
        overlay = image.copy()
        cv2.rectangle(overlay, (10, 10), (300, 130), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.6, image, 0.4, 0, image)
        
        # 텍스트
        y_offset = 35
        
        cv2.putText(image, f"FPS: {fps:.1f}", (20, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, self.color_text, 2)
        y_offset += 30
        
        if center_distance > 0:
            cv2.putText(image, f"Center: {center_distance/1000:.2f} m", 
                       (20, y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, self.color_distance, 2)
        else:
            cv2.putText(image, "Center: --", (20, y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (128, 128, 128), 2)
        y_offset += 30
        
        cv2.putText(image, f"Points: {num_points}", (20, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, self.color_text, 2)
    
    def draw_help(self, image: np.ndarray):
        """도움말 표시"""
        
        h, w = image.shape[:2]
        
        help_text = [
            "[Click] Add point",
            "[C] Clear points",
            "[Z] Remove last",
            "[D] Toggle depth",
            "[S] Save",
            "[Q] Quit"
        ]
        
        # 반투명 배경
        overlay = image.copy()
        cv2.rectangle(overlay, (w - 180, 10), (w - 10, 180), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.6, image, 0.4, 0, image)
        
        for i, text in enumerate(help_text):
            cv2.putText(image, text, (w - 170, 35 + i * 25),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
    
    def colorize_depth(self, depth: np.ndarray, 
                       max_depth: float = 5000) -> np.ndarray:
        """깊이 맵 컬러화"""
        
        # 정규화 (가까울수록 밝게)
        depth_clipped = np.clip(depth, 0, max_depth)
        depth_norm = (1 - depth_clipped / max_depth) * 255
        depth_norm = depth_norm.astype(np.uint8)
        
        # 컬러맵 적용
        depth_color = cv2.applyColorMap(depth_norm, cv2.COLORMAP_TURBO)
        
        # 유효하지 않은 영역 검정색
        depth_color[depth <= 0] = [0, 0, 0]
        
        return depth_color
    
    def render(self, rect_left: np.ndarray,
               disparity: Optional[np.ndarray],
               depth: Optional[np.ndarray],
               center_distance: float,
               measurement_points: list) -> np.ndarray:
        """
        화면 렌더링
        
        Returns:
        - display: 렌더링된 이미지
        """
        
        # FPS 업데이트
        self.update_fps()
        fps = self.get_fps()
        
        # 메인 이미지 복사
        display = rect_left.copy()
        h, w = display.shape[:2]
        
        # 십자선 (화면 중앙)
        if self.show_crosshair:
            self.draw_crosshair(display, w // 2, h // 2)
        
        # 마우스 위치 십자선
        if 0 <= self.mouse_x < w and 0 <= self.mouse_y < h:
            cv2.circle(display, (self.mouse_x, self.mouse_y), 3, 
                      (255, 255, 255), -1)
        
        # 측정 포인트 표시
        if self.show_measurements:
            for i, point in enumerate(measurement_points):
                self.draw_measurement_point(
                    display, point.x, point.y, 
                    point.distance_mm, point.label, i
                )
        
        # 정보 패널
        self.draw_info_panel(display, center_distance, fps, 
                            len(measurement_points))
        
        # 도움말
        self.draw_help(display)
        
        # 깊이 맵 표시 (선택)
        if self.show_depth_map and depth is not None:
            depth_color = self.colorize_depth(depth)
            depth_color = cv2.resize(depth_color, (w // 3, h // 3))
            
            # 오른쪽 하단에 표시
            x_offset = w - depth_color.shape[1] - 10
            y_offset = h - depth_color.shape[0] - 10
            display[y_offset:y_offset+depth_color.shape[0],
                   x_offset:x_offset+depth_color.shape[1]] = depth_color
        
        return display
    
    def show(self, image: np.ndarray):
        """이미지 표시"""
        cv2.imshow(self.window_name, image)
    
    def wait_key(self, delay: int = 1) -> int:
        """키 입력 대기"""
        return cv2.waitKey(delay) & 0xFF
    
    def close(self):
        """윈도우 닫기"""
        cv2.destroyWindow(self.window_name)
```

---

## 6. 측정 모드

### 6.1 측정 모드 정의

```python
"""
measurement_modes.py
다양한 측정 모드 구현
"""

import numpy as np
from enum import Enum, auto
from typing import List, Optional, Tuple
from distance_meter import MeasurementPoint, DistanceMeter


class MeasurementMode(Enum):
    """측정 모드"""
    SINGLE_POINT = auto()      # 단일 포인트
    CONTINUOUS = auto()        # 연속 측정 (중앙)
    MULTI_POINT = auto()       # 다중 포인트
    LINE_MEASURE = auto()      # 두 점 사이 거리
    AREA_MEASURE = auto()      # 영역 평균


class MeasurementController:
    """측정 모드 컨트롤러"""
    
    def __init__(self, distance_meter: DistanceMeter):
        self.meter = distance_meter
        self.mode = MeasurementMode.CONTINUOUS
        
        # 라인 측정용 임시 포인트
        self.line_start: Optional[MeasurementPoint] = None
        
        # 영역 측정용 좌표
        self.area_start: Optional[Tuple[int, int]] = None
        self.area_end: Optional[Tuple[int, int]] = None
    
    def set_mode(self, mode: MeasurementMode):
        """모드 설정"""
        self.mode = mode
        self._reset_temp()
    
    def _reset_temp(self):
        """임시 데이터 초기화"""
        self.line_start = None
        self.area_start = None
        self.area_end = None
    
    def process_click(self, x: int, y: int) -> Optional[dict]:
        """
        클릭 처리
        
        Returns:
        - 결과 딕셔너리 또는 None
        """
        
        if self.mode == MeasurementMode.SINGLE_POINT:
            return self._single_point_measure(x, y)
        
        elif self.mode == MeasurementMode.MULTI_POINT:
            return self._multi_point_measure(x, y)
        
        elif self.mode == MeasurementMode.LINE_MEASURE:
            return self._line_measure(x, y)
        
        elif self.mode == MeasurementMode.AREA_MEASURE:
            return self._area_measure(x, y)
        
        return None
    
    def _single_point_measure(self, x: int, y: int) -> Optional[dict]:
        """단일 포인트 측정"""
        
        point = self.meter.measure_point(x, y, label="Single")
        
        if point:
            return {
                'type': 'single',
                'point': point,
                'distance_mm': point.distance_mm
            }
        return None
    
    def _multi_point_measure(self, x: int, y: int) -> Optional[dict]:
        """다중 포인트 측정 (저장됨)"""
        
        idx = len(self.meter.measurement_points)
        point = self.meter.measure_point(x, y, label=f"P{idx}")
        
        if point:
            self.meter.add_measurement(point)
            return {
                'type': 'multi',
                'point': point,
                'index': idx,
                'total': len(self.meter.measurement_points)
            }
        return None
    
    def _line_measure(self, x: int, y: int) -> Optional[dict]:
        """두 점 사이 거리 측정"""
        
        point = self.meter.measure_point(x, y)
        
        if point is None:
            return None
        
        if self.line_start is None:
            # 첫 번째 점
            self.line_start = point
            return {
                'type': 'line_start',
                'point': point
            }
        else:
            # 두 번째 점 - 거리 계산
            line_end = point
            
            # 3D 거리 계산
            dx = line_end.x - self.line_start.x
            dy = line_end.y - self.line_start.y
            avg_depth = (self.line_start.distance_mm + line_end.distance_mm) / 2
            
            scale = avg_depth / self.meter.depth_estimator.focal_length
            distance_3d = np.sqrt(
                (dx * scale) ** 2 + 
                (dy * scale) ** 2 + 
                (line_end.distance_mm - self.line_start.distance_mm) ** 2
            )
            
            result = {
                'type': 'line_end',
                'start': self.line_start,
                'end': line_end,
                'distance_3d_mm': distance_3d
            }
            
            # 리셋
            self.line_start = None
            
            return result
        
        return None
    
    def _area_measure(self, x: int, y: int) -> Optional[dict]:
        """영역 평균 깊이 측정"""
        
        if self.area_start is None:
            self.area_start = (x, y)
            return {
                'type': 'area_start',
                'position': (x, y)
            }
        else:
            self.area_end = (x, y)
            
            # 영역 좌표 정리
            x1 = min(self.area_start[0], self.area_end[0])
            y1 = min(self.area_start[1], self.area_end[1])
            x2 = max(self.area_start[0], self.area_end[0])
            y2 = max(self.area_start[1], self.area_end[1])
            
            # 영역 깊이 추출
            if self.meter.current_depth is not None:
                region = self.meter.current_depth[y1:y2, x1:x2]
                valid = region[region > 0]
                
                if len(valid) > 0:
                    result = {
                        'type': 'area_end',
                        'rect': (x1, y1, x2, y2),
                        'mean_mm': np.mean(valid),
                        'std_mm': np.std(valid),
                        'min_mm': np.min(valid),
                        'max_mm': np.max(valid),
                        'valid_ratio': len(valid) / region.size
                    }
                    
                    self._reset_temp()
                    return result
            
            self._reset_temp()
            return None
```

---

## 7. 정확도 향상 기법

### 7.1 정확도 향상 유틸리티

```python
"""
accuracy_utils.py
측정 정확도 향상 유틸리티
"""

import numpy as np
from scipy import ndimage
from typing import Tuple, Optional


class AccuracyEnhancer:
    """측정 정확도 향상 도구"""
    
    @staticmethod
    def temporal_filter(depth_history: list, 
                        method: str = 'median') -> np.ndarray:
        """
        시간 필터링 (프레임 간 평균/중간값)
        
        Parameters:
        - depth_history: 깊이 맵 리스트
        - method: 'mean', 'median', 'weighted'
        
        Returns:
        - filtered_depth: 필터링된 깊이 맵
        """
        
        if len(depth_history) == 0:
            return None
        
        if len(depth_history) == 1:
            return depth_history[0]
        
        stacked = np.stack(depth_history, axis=0)
        
        # 0을 NaN으로 변환
        stacked = np.where(stacked > 0, stacked, np.nan)
        
        if method == 'mean':
            result = np.nanmean(stacked, axis=0)
        
        elif method == 'median':
            result = np.nanmedian(stacked, axis=0)
        
        elif method == 'weighted':
            # 최신 프레임에 더 높은 가중치
            weights = np.array([1, 2, 3, 4, 5][-len(depth_history):])
            weights = weights / weights.sum()
            
            weighted_sum = np.zeros_like(stacked[0])
            weight_sum = np.zeros_like(stacked[0])
            
            for i, d in enumerate(stacked):
                valid = ~np.isnan(d)
                weighted_sum[valid] += d[valid] * weights[i]
                weight_sum[valid] += weights[i]
            
            result = np.where(weight_sum > 0, weighted_sum / weight_sum, 0)
        
        else:
            raise ValueError(f"Unknown method: {method}")
        
        return np.nan_to_num(result, nan=0)
    
    @staticmethod
    def spatial_filter(depth: np.ndarray, 
                       method: str = 'bilateral',
                       **kwargs) -> np.ndarray:
        """
        공간 필터링 (노이즈 제거)
        
        Parameters:
        - depth: 깊이 맵
        - method: 'bilateral', 'median', 'gaussian'
        
        Returns:
        - filtered_depth: 필터링된 깊이 맵
        """
        
        if method == 'bilateral':
            # 깊이를 8비트로 정규화
            valid = depth > 0
            if not np.any(valid):
                return depth
            
            d_min, d_max = depth[valid].min(), depth[valid].max()
            depth_norm = np.zeros_like(depth, dtype=np.uint8)
            depth_norm[valid] = ((depth[valid] - d_min) / (d_max - d_min + 1e-6) * 255).astype(np.uint8)
            
            # Bilateral 필터
            d = kwargs.get('d', 9)
            sigma_color = kwargs.get('sigma_color', 75)
            sigma_space = kwargs.get('sigma_space', 75)
            
            filtered_norm = cv2.bilateralFilter(depth_norm, d, sigma_color, sigma_space)
            
            # 원래 스케일로 복원
            filtered = np.zeros_like(depth)
            filtered[valid] = filtered_norm[valid].astype(np.float32) / 255 * (d_max - d_min) + d_min
            
            return filtered
        
        elif method == 'median':
            ksize = kwargs.get('ksize', 5)
            return ndimage.median_filter(depth, size=ksize)
        
        elif method == 'gaussian':
            sigma = kwargs.get('sigma', 1.0)
            return ndimage.gaussian_filter(depth, sigma=sigma)
        
        else:
            raise ValueError(f"Unknown method: {method}")
    
    @staticmethod
    def confidence_weighted_measurement(depth: np.ndarray,
                                        disparity: np.ndarray,
                                        x: int, y: int,
                                        window_size: int = 9) -> Tuple[float, float]:
        """
        신뢰도 가중 측정
        
        시차가 높을수록 (가까울수록) 신뢰도 높음
        
        Returns:
        - weighted_depth: 가중 평균 깊이
        - confidence: 신뢰도 (0-1)
        """
        
        h, w = depth.shape
        half = window_size // 2
        
        x1, x2 = max(0, x - half), min(w, x + half + 1)
        y1, y2 = max(0, y - half), min(h, y + half + 1)
        
        depth_window = depth[y1:y2, x1:x2]
        disp_window = disparity[y1:y2, x1:x2]
        
        valid = (depth_window > 0) & (disp_window > 0)
        
        if not np.any(valid):
            return 0.0, 0.0
        
        # 시차를 가중치로 사용 (높을수록 신뢰)
        weights = disp_window[valid]
        weights = weights / weights.sum()
        
        weighted_depth = np.sum(depth_window[valid] * weights)
        
        # 신뢰도 계산 (분산 기반)
        variance = np.average(
            (depth_window[valid] - weighted_depth) ** 2,
            weights=weights
        )
        
        # 분산이 작을수록, 유효 비율이 높을수록 신뢰도 높음
        valid_ratio = np.sum(valid) / valid.size
        variance_score = 1 / (1 + variance / 1000)  # 정규화
        confidence = valid_ratio * variance_score
        
        return weighted_depth, confidence
    
    @staticmethod
    def outlier_rejection(values: np.ndarray, 
                          method: str = 'iqr',
                          **kwargs) -> np.ndarray:
        """
        이상치 제거
        
        Parameters:
        - values: 값 배열
        - method: 'iqr', 'zscore', 'mad'
        
        Returns:
        - filtered_values: 이상치 제거된 값
        """
        
        if len(values) == 0:
            return values
        
        if method == 'iqr':
            q1, q3 = np.percentile(values, [25, 75])
            iqr = q3 - q1
            factor = kwargs.get('factor', 1.5)
            lower = q1 - factor * iqr
            upper = q3 + factor * iqr
            mask = (values >= lower) & (values <= upper)
        
        elif method == 'zscore':
            threshold = kwargs.get('threshold', 2.0)
            mean = np.mean(values)
            std = np.std(values)
            if std == 0:
                return values
            z_scores = np.abs((values - mean) / std)
            mask = z_scores < threshold
        
        elif method == 'mad':
            # Median Absolute Deviation
            threshold = kwargs.get('threshold', 3.5)
            median = np.median(values)
            mad = np.median(np.abs(values - median))
            if mad == 0:
                return values
            modified_z = 0.6745 * (values - median) / mad
            mask = np.abs(modified_z) < threshold
        
        else:
            raise ValueError(f"Unknown method: {method}")
        
        return values[mask]
```

---

## 8. 결과 저장 및 로깅

### 8.1 로깅 유틸리티

```python
"""
logging_utils.py
결과 저장 및 로깅 유틸리티
"""

import csv
import json
import os
from datetime import datetime
from typing import List, Optional
import numpy as np


class MeasurementLogger:
    """측정 결과 로거"""
    
    def __init__(self, log_dir: str = "logs"):
        self.log_dir = log_dir
        os.makedirs(log_dir, exist_ok=True)
        
        # 세션 ID
        self.session_id = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # CSV 파일
        self.csv_file = os.path.join(log_dir, f"measurements_{self.session_id}.csv")
        self._init_csv()
    
    def _init_csv(self):
        """CSV 파일 초기화"""
        with open(self.csv_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'Timestamp', 'Type', 'Label',
                'X', 'Y', 'Distance_mm', 'Std_mm',
                'Confidence', 'Notes'
            ])
    
    def log_measurement(self, measurement_type: str,
                        label: str,
                        x: int, y: int,
                        distance_mm: float,
                        std_mm: float = 0,
                        confidence: float = 1.0,
                        notes: str = ""):
        """측정 로그 기록"""
        
        with open(self.csv_file, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                datetime.now().isoformat(),
                measurement_type,
                label,
                x, y,
                f"{distance_mm:.1f}",
                f"{std_mm:.1f}",
                f"{confidence:.2f}",
                notes
            ])
    
    def save_session_summary(self, measurements: list,
                             statistics: dict):
        """세션 요약 저장"""
        
        summary = {
            'session_id': self.session_id,
            'timestamp': datetime.now().isoformat(),
            'total_measurements': len(measurements),
            'statistics': statistics,
            'measurements': [
                {
                    'label': m.label,
                    'x': m.x,
                    'y': m.y,
                    'distance_mm': m.distance_mm,
                    'std_mm': m.std_mm,
                    'timestamp': m.timestamp.isoformat()
                }
                for m in measurements
            ]
        }
        
        summary_file = os.path.join(
            self.log_dir, 
            f"summary_{self.session_id}.json"
        )
        
        with open(summary_file, 'w') as f:
            json.dump(summary, f, indent=2)
        
        print(f"✅ 세션 요약 저장: {summary_file}")


class ScreenshotManager:
    """스크린샷 관리"""
    
    def __init__(self, screenshot_dir: str = "screenshots"):
        self.screenshot_dir = screenshot_dir
        os.makedirs(screenshot_dir, exist_ok=True)
    
    def save_screenshot(self, image: np.ndarray,
                        depth: Optional[np.ndarray] = None,
                        prefix: str = "screenshot") -> str:
        """스크린샷 저장"""
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # 메인 이미지
        filename = f"{prefix}_{timestamp}.png"
        filepath = os.path.join(self.screenshot_dir, filename)
        cv2.imwrite(filepath, image)
        
        # 깊이 맵 (있으면)
        if depth is not None:
            depth_filename = f"{prefix}_{timestamp}_depth.npy"
            depth_filepath = os.path.join(self.screenshot_dir, depth_filename)
            np.save(depth_filepath, depth)
        
        print(f"📸 스크린샷 저장: {filepath}")
        return filepath
```

---

## 9. 전체 코드

### 9.1 메인 실행 파일

```python
"""
main.py
실시간 거리 측정기 메인 실행 파일
"""

import argparse
import yaml
import cv2
import sys
from datetime import datetime

from stereo_camera import StereoCamera
from depth_estimator import DepthEstimator
from distance_meter import DistanceMeter
from gui import DistanceMeterGUI
from measurement_modes import MeasurementMode, MeasurementController
from logging_utils import MeasurementLogger, ScreenshotManager


class DistanceMeterApp:
    """실시간 거리 측정기 앱"""
    
    def __init__(self, config_file: str):
        """
        Parameters:
        - config_file: 설정 파일 경로
        """
        
        # 설정 로드
        with open(config_file, 'r') as f:
            self.config = yaml.safe_load(f)
        
        # 컴포넌트 초기화
        self._init_components()
        
        print("\n" + "="*60)
        print("🎯 실시간 거리 측정기")
        print("="*60)
    
    def _init_components(self):
        """컴포넌트 초기화"""
        
        # 스테레오 카메라
        self.camera = StereoCamera(
            calibration_file=self.config['calibration_file'],
            left_id=self.config['camera']['left_id'],
            right_id=self.config['camera']['right_id'],
            width=self.config['camera']['width'],
            height=self.config['camera']['height']
        )
        
        # 깊이 추정기
        self.depth_estimator = DepthEstimator(
            focal_length=self.camera.focal_length,
            baseline=self.camera.baseline,
            num_disparities=self.config['stereo']['num_disparities'],
            block_size=self.config['stereo']['block_size']
        )
        
        # 거리 측정기
        self.distance_meter = DistanceMeter(
            depth_estimator=self.depth_estimator,
            min_depth=self.config['measurement']['min_depth'],
            max_depth=self.config['measurement']['max_depth']
        )
        
        # 측정 컨트롤러
        self.measurement_controller = MeasurementController(self.distance_meter)
        
        # GUI
        self.gui = DistanceMeterGUI(window_name="Distance Meter")
        
        # 로거
        self.logger = MeasurementLogger(log_dir="logs")
        self.screenshot_manager = ScreenshotManager(screenshot_dir="screenshots")
    
    def run(self):
        """메인 루프"""
        
        print("\n조작 방법:")
        print("  [Click] - 포인트 측정/추가")
        print("  [C]     - 포인트 초기화")
        print("  [Z]     - 마지막 포인트 삭제")
        print("  [M]     - 측정 모드 변경")
        print("  [D]     - 깊이맵 표시 토글")
        print("  [S]     - 스크린샷 저장")
        print("  [R]     - 결과 저장")
        print("  [Q]     - 종료")
        print("="*60 + "\n")
        
        current_mode_idx = 0
        modes = list(MeasurementMode)
        
        while True:
            # 프레임 캡처 및 정류
            rect_left, rect_right = self.camera.capture_rectified()
            
            if rect_left is None or rect_right is None:
                print("프레임 캡처 실패")
                continue
            
            # 깊이 업데이트
            disparity, depth = self.distance_meter.update(rect_left, rect_right)
            
            # 중앙 거리 측정
            center_point = self.distance_meter.measure_center(rect_left.shape)
            center_distance = center_point.distance_mm if center_point else 0
            
            # 마우스 클릭 처리
            click = self.gui.check_click()
            if click:
                result = self.measurement_controller.process_click(click[0], click[1])
                if result:
                    self._handle_measurement_result(result)
            
            # 렌더링
            display = self.gui.render(
                rect_left=rect_left,
                disparity=disparity,
                depth=depth,
                center_distance=center_distance,
                measurement_points=self.distance_meter.measurement_points
            )
            
            # 현재 모드 표시
            mode_text = f"Mode: {modes[current_mode_idx].name}"
            cv2.putText(display, mode_text, (10, display.shape[0] - 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            self.gui.show(display)
            
            # 키 입력 처리
            key = self.gui.wait_key(1)
            
            if key == ord('q'):
                break
            
            elif key == ord('c'):
                self.distance_meter.clear_measurements()
                print("측정 포인트 초기화")
            
            elif key == ord('z'):
                self.distance_meter.remove_last_measurement()
                print("마지막 포인트 삭제")
            
            elif key == ord('m'):
                current_mode_idx = (current_mode_idx + 1) % len(modes)
                self.measurement_controller.set_mode(modes[current_mode_idx])
                print(f"모드 변경: {modes[current_mode_idx].name}")
            
            elif key == ord('d'):
                self.gui.show_depth_map = not self.gui.show_depth_map
                print(f"깊이맵 표시: {'ON' if self.gui.show_depth_map else 'OFF'}")
            
            elif key == ord('s'):
                self.screenshot_manager.save_screenshot(display, depth)
            
            elif key == ord('r'):
                self._save_results()
        
        self._cleanup()
    
    def _handle_measurement_result(self, result: dict):
        """측정 결과 처리"""
        
        result_type = result.get('type', '')
        
        if result_type == 'single':
            point = result['point']
            print(f"📍 단일 측정: ({point.x}, {point.y}) = {point.distance_mm/1000:.2f}m")
            self.logger.log_measurement(
                'single', 'Single', point.x, point.y,
                point.distance_mm, point.std_mm
            )
        
        elif result_type == 'multi':
            point = result['point']
            print(f"📍 포인트 #{result['index']}: ({point.x}, {point.y}) = {point.distance_mm/1000:.2f}m")
            self.logger.log_measurement(
                'multi', point.label, point.x, point.y,
                point.distance_mm, point.std_mm
            )
        
        elif result_type == 'line_start':
            point = result['point']
            print(f"📐 라인 시작: ({point.x}, {point.y})")
        
        elif result_type == 'line_end':
            dist_3d = result['distance_3d_mm']
            print(f"📐 라인 끝: 3D 거리 = {dist_3d/1000:.2f}m")
            self.logger.log_measurement(
                'line', 'Line Distance', 0, 0,
                dist_3d, 0, notes="3D distance between two points"
            )
        
        elif result_type == 'area_start':
            print(f"🔲 영역 시작: {result['position']}")
        
        elif result_type == 'area_end':
            rect = result['rect']
            print(f"🔲 영역 측정: 평균={result['mean_mm']/1000:.2f}m, "
                  f"범위={result['min_mm']/1000:.2f}-{result['max_mm']/1000:.2f}m")
    
    def _save_results(self):
        """결과 저장"""
        
        stats = self.distance_meter.get_statistics()
        self.logger.save_session_summary(
            self.distance_meter.measurement_points,
            stats
        )
        
        if self.distance_meter.measurement_points:
            filename = f"logs/measurements_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
            self.distance_meter.save_measurements(filename)
    
    def _cleanup(self):
        """정리"""
        
        self._save_results()
        self.camera.release()
        self.gui.close()
        print("\n프로그램 종료")


def main():
    parser = argparse.ArgumentParser(description='실시간 거리 측정기')
    parser.add_argument('--config', type=str, default='config/app_config.yaml',
                        help='설정 파일 경로')
    args = parser.parse_args()
    
    try:
        app = DistanceMeterApp(args.config)
        app.run()
    except KeyboardInterrupt:
        print("\n사용자 중단")
    except Exception as e:
        print(f"오류: {e}")
        raise


if __name__ == "__main__":
    main()
```

### 9.2 설정 파일

```yaml
# config/app_config.yaml
# 실시간 거리 측정기 설정

# 캘리브레이션 파일
calibration_file: "config/stereo_params.yaml"

# 카메라 설정
camera:
  left_id: 0
  right_id: 2
  width: 1280
  height: 720
  fps: 30

# 스테레오 매칭 설정
stereo:
  num_disparities: 128
  block_size: 5
  use_wls_filter: true

# 측정 설정
measurement:
  min_depth: 200      # mm
  max_depth: 5000     # mm
  window_size: 7      # 평균 윈도우
  temporal_frames: 5  # 시간 평균 프레임 수

# GUI 설정
gui:
  show_depth_map: true
  show_crosshair: true
  depth_colormap: "TURBO"
```

---

## 10. 테스트 및 검증

### 10.1 정확도 테스트

```python
"""
test_accuracy.py
거리 측정 정확도 테스트
"""

import numpy as np
from distance_meter import DistanceMeter
from depth_estimator import DepthEstimator


def test_known_distances():
    """알려진 거리로 정확도 테스트"""
    
    # 테스트 거리 (실제 측정값)
    known_distances = [
        {'actual_mm': 500, 'label': '50cm'},
        {'actual_mm': 1000, 'label': '1m'},
        {'actual_mm': 2000, 'label': '2m'},
        {'actual_mm': 3000, 'label': '3m'},
    ]
    
    print("\n" + "="*60)
    print("거리 측정 정확도 테스트")
    print("="*60)
    print(f"{'거리':<10} {'측정값':<12} {'오차':<12} {'오차율':<10}")
    print("-"*50)
    
    results = []
    
    for test in known_distances:
        actual = test['actual_mm']
        label = test['label']
        
        # 여기서 실제 측정 수행
        # measured = measure_at_distance(actual)
        measured = actual * (1 + np.random.normal(0, 0.02))  # 시뮬레이션
        
        error = measured - actual
        error_percent = abs(error) / actual * 100
        
        print(f"{label:<10} {measured:.1f}mm{'':<4} {error:+.1f}mm{'':<4} {error_percent:.1f}%")
        
        results.append({
            'actual': actual,
            'measured': measured,
            'error': error,
            'error_percent': error_percent
        })
    
    # 통계
    errors = [r['error'] for r in results]
    error_percents = [r['error_percent'] for r in results]
    
    print("-"*50)
    print(f"평균 오차: {np.mean(np.abs(errors)):.1f}mm")
    print(f"최대 오차: {np.max(np.abs(errors)):.1f}mm")
    print(f"평균 오차율: {np.mean(error_percents):.1f}%")
    print("="*60)


if __name__ == "__main__":
    test_known_distances()
```

### 10.2 성능 벤치마크

```
┌─────────────────────────────────────────────────────────────┐
│                    예상 성능 (1280x720)                      │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  하드웨어        │  FPS   │  지연시간  │  정확도 (1m)       │
│  ──────────────────────────────────────────────────────────│
│  Intel i5       │  15-20 │  50-70ms  │  ±2-3cm            │
│  Intel i7       │  25-30 │  35-45ms  │  ±2-3cm            │
│  + WLS 필터     │  -5fps │  +20ms    │  향상               │
│                                                             │
│  권장: WLS 필터 사용, 720p, 30fps 목표                      │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

---

## 📝 학습 체크리스트

### 구현 완료

- [ ] 스테레오 카메라 클래스 구현
- [ ] 깊이 추정기 구현
- [ ] 거리 측정기 구현
- [ ] GUI 구현
- [ ] 다중 측정 모드 구현
- [ ] 결과 저장/로깅 구현

### 테스트 완료

- [ ] 카메라 캡처 테스트
- [ ] 정류 확인 (에피폴라 라인)
- [ ] 시차 맵 품질 확인
- [ ] 거리 측정 정확도 테스트
- [ ] 성능 (FPS) 확인

---

## ➡️ 다음 프로젝트

**[Project 02: 장애물 감지 시스템](../Project_02_Obstacle_Detection/README.md)**

다음 프로젝트에서는:
- 깊이 기반 장애물 감지
- 거리별 경고 시스템
- 영역 분할
- 실시간 알림

을 구현합니다.

---

## 📄 라이선스

MIT License - 자유롭게 사용, 수정, 배포 가능
