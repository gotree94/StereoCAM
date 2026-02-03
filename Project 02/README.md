# Project 02: 장애물 감지 시스템

[![난이도](https://img.shields.io/badge/난이도-⭐⭐⭐⭐_고급-red.svg)]()
[![예상시간](https://img.shields.io/badge/예상시간-10--15시간-blue.svg)]()
[![선수지식](https://img.shields.io/badge/선수지식-Project_01_완료-orange.svg)]()

---

## 🎯 프로젝트 개요

| 항목 | 내용 |
|------|------|
| **목표** | 스테레오 카메라 기반 실시간 장애물 감지 및 경고 시스템 |
| **기능** | 거리별 위험 영역 감지, 충돌 경고, 안전 영역 시각화 |
| **응용** | 로봇 네비게이션, 자율주행, 드론 장애물 회피, 시각 보조 |

---

## 📋 목차

1. [프로젝트 구조](#1-프로젝트-구조) : 디렉토리 구조, 모듈 구성
2. [시스템 아키텍처](#2-시스템-아키텍처) : 전체 파이프라인 (캡처→감지→추적→경고)
3. [장애물 감지 알고리즘](#3-장애물-감지-알고리즘) : ObstacleDetector, GroundPlaneRemover, U-Disparity
4. [위험 영역 분류](#4-위험-영역-분류) : 5단계 (SAFE→CRITICAL), 거리 임계값
5. [경고 시스템](#5-경고-시스템) : 시각/청각 경고, 거리 바, 경고 오버레이
6. [영역 분할](#6-영역-분할) : ZoneManager, 사다리꼴 영역, 점유 그리드
7. [추적 및 예측](#7-추적-및-예측) : SimpleTracker, 충돌 예측 (TTC)
8. [GUI 구현](#8-gui-구현) : 바운딩 박스, 추적 경로, 미니맵
9. [전체 코드](#9-전체-코드) : main.py, 설정 파일
10. [성능 최적화](#10-성능-최적화) : 최적화 가이드

📁 포함된 완전한 코드
   * obstacle_detector.py - 장애물 감지 (DangerLevel, Obstacle, ObstacleDetector)
   * u_disparity.py - U/V-Disparity 기반 감지
   * zone_manager.py - 감지 영역 관리 (ZoneManager, DetectionZone)
   * alert_system.py - 경고 시스템 (시각/청각)
   * segmentation.py - 깊이 기반 영역 분할, 점유 그리드
   * object_tracker.py - 객체 추적 (SimpleTracker, TrackedObject)
   * gui.py - GUI (미니맵, 정보 패널, 경고 오버레이)
   * main.py - 메인 애플리케이션
   * detection_config.yaml - 설정 파일

## 🔧 기능 설명

| 구분 | 기능 항목 | 설명 |
|------|-----------|------|
| 위험 분류 | 5단계 위험 분류 | SAFE → CAUTION → WARNING → DANGER → CRITICAL |
| 영역 인식 | 다중 영역 감지 | 중앙 / 좌 / 우 / 하단 / 사용자 정의 영역 |
| 객체 인식 | 실시간 추적 | IOU 기반 객체 추적, 이동 경로 표시 |
| 충돌 판단 | 충돌 예측 | TTC (Time To Collision) 계산 |
| 시각 알림 | 시각적 경고 | 테두리 색상 변경, 깜빡임, 경고 텍스트 표시 |
| 시각화 | 미니맵 | 탑뷰 형태의 장애물 위치 표시 |
| 전처리 | 지면 제거 | 기하학적 / RANSAC 방식 |


---

## 1. 프로젝트 구조

```
Project_02_Obstacle_Detection/
├── README.md
├── requirements.txt
├── config/
│   ├── stereo_params.yaml      # 캘리브레이션
│   ├── detection_config.yaml   # 감지 설정
│   └── alert_config.yaml       # 경고 설정
├── src/
│   ├── __init__.py
│   ├── main.py                 # 메인 실행
│   ├── stereo_camera.py        # 스테레오 카메라
│   ├── depth_processor.py      # 깊이 처리
│   ├── obstacle_detector.py    # 장애물 감지
│   ├── zone_manager.py         # 영역 관리
│   ├── alert_system.py         # 경고 시스템
│   ├── object_tracker.py       # 객체 추적
│   ├── gui.py                  # GUI
│   └── utils.py                # 유틸리티
├── assets/
│   └── sounds/                 # 경고음
│       ├── warning_low.wav
│       ├── warning_medium.wav
│       └── warning_high.wav
└── logs/
    └── detection_log.csv       # 감지 로그
```

---

## 2. 시스템 아키텍처

```
┌─────────────────────────────────────────────────────────────┐
│                 장애물 감지 시스템 아키텍처                    │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│   ┌─────────┐     ┌─────────┐                              │
│   │ Camera  │     │ Camera  │                              │
│   │  Left   │     │  Right  │                              │
│   └────┬────┘     └────┬────┘                              │
│        └───────┬───────┘                                    │
│                ▼                                            │
│        ┌──────────────┐                                    │
│        │ Stereo       │                                    │
│        │ Processing   │                                    │
│        └──────┬───────┘                                    │
│                ▼                                            │
│        ┌──────────────┐                                    │
│        │  Depth Map   │                                    │
│        └──────┬───────┘                                    │
│                │                                            │
│     ┌──────────┼──────────┐                                │
│     ▼          ▼          ▼                                │
│ ┌────────┐ ┌────────┐ ┌────────┐                          │
│ │Ground  │ │Obstacle│ │ Zone   │                          │
│ │Removal │ │Detect  │ │Classify│                          │
│ └───┬────┘ └───┬────┘ └───┬────┘                          │
│     └──────────┼──────────┘                                │
│                ▼                                            │
│        ┌──────────────┐                                    │
│        │   Object     │                                    │
│        │   Tracker    │                                    │
│        └──────┬───────┘                                    │
│                │                                            │
│     ┌──────────┼──────────┐                                │
│     ▼          ▼          ▼                                │
│ ┌────────┐ ┌────────┐ ┌────────┐                          │
│ │ Alert  │ │  GUI   │ │  Log   │                          │
│ │ System │ │Display │ │ Output │                          │
│ └────────┘ └────────┘ └────────┘                          │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

---

## 3. 장애물 감지 알고리즘

### 3.1 깊이 기반 장애물 감지

```python
"""
obstacle_detector.py
장애물 감지 핵심 모듈
"""

import cv2
import numpy as np
from typing import List, Tuple, Optional
from dataclasses import dataclass
from enum import Enum


class DangerLevel(Enum):
    """위험 수준"""
    SAFE = 0        # 안전
    CAUTION = 1     # 주의
    WARNING = 2     # 경고
    DANGER = 3      # 위험
    CRITICAL = 4    # 긴급


@dataclass
class Obstacle:
    """장애물 정보"""
    id: int
    bbox: Tuple[int, int, int, int]  # x, y, w, h
    center: Tuple[int, int]           # 중심 좌표
    distance_mm: float                # 거리
    area: int                         # 픽셀 면적
    danger_level: DangerLevel         # 위험 수준
    velocity: Tuple[float, float] = (0, 0)  # 이동 속도 (선택)


class ObstacleDetector:
    """장애물 감지기"""
    
    def __init__(self, config: dict):
        """
        Parameters:
        - config: 감지 설정 딕셔너리
        """
        
        # 거리 임계값 (mm)
        self.distance_thresholds = config.get('distance_thresholds', {
            'critical': 500,   # 0.5m 이내: 긴급
            'danger': 1000,    # 1m 이내: 위험
            'warning': 2000,   # 2m 이내: 경고
            'caution': 3000,   # 3m 이내: 주의
        })
        
        # 최소/최대 감지 거리
        self.min_distance = config.get('min_distance', 200)   # 20cm
        self.max_distance = config.get('max_distance', 5000)  # 5m
        
        # 최소 장애물 크기 (픽셀)
        self.min_obstacle_area = config.get('min_obstacle_area', 500)
        
        # 지면 제거 설정
        self.ground_removal = config.get('ground_removal', True)
        self.ground_threshold = config.get('ground_threshold', 0.1)  # 하단 10%
        
        # 형태학적 연산 커널
        self.morph_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (5, 5)
        )
        
        # 장애물 ID 카운터
        self.next_id = 0
    
    def detect(self, depth: np.ndarray, 
               color_image: Optional[np.ndarray] = None) -> List[Obstacle]:
        """
        장애물 감지
        
        Parameters:
        - depth: 깊이 맵 (mm)
        - color_image: 컬러 이미지 (선택, 시각화용)
        
        Returns:
        - obstacles: 감지된 장애물 리스트
        """
        
        h, w = depth.shape
        
        # 1. 유효 깊이 마스크
        valid_mask = (depth > self.min_distance) & (depth < self.max_distance)
        
        # 2. 지면 제거 (선택)
        if self.ground_removal:
            ground_y = int(h * (1 - self.ground_threshold))
            valid_mask[ground_y:, :] = False
        
        # 3. 거리별 마스크 생성
        obstacle_mask = np.zeros((h, w), dtype=np.uint8)
        
        # 가까운 물체일수록 장애물로 판단
        close_mask = valid_mask & (depth < self.distance_thresholds['caution'])
        obstacle_mask[close_mask] = 255
        
        # 4. 노이즈 제거 (형태학적 연산)
        obstacle_mask = cv2.morphologyEx(obstacle_mask, cv2.MORPH_OPEN, 
                                         self.morph_kernel)
        obstacle_mask = cv2.morphologyEx(obstacle_mask, cv2.MORPH_CLOSE, 
                                         self.morph_kernel)
        
        # 5. 연결 요소 분석
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
            obstacle_mask, connectivity=8
        )
        
        obstacles = []
        
        for i in range(1, num_labels):  # 0은 배경
            area = stats[i, cv2.CC_STAT_AREA]
            
            # 최소 크기 필터
            if area < self.min_obstacle_area:
                continue
            
            # 바운딩 박스
            x = stats[i, cv2.CC_STAT_LEFT]
            y = stats[i, cv2.CC_STAT_TOP]
            w_box = stats[i, cv2.CC_STAT_WIDTH]
            h_box = stats[i, cv2.CC_STAT_HEIGHT]
            
            # 중심 좌표
            cx, cy = int(centroids[i][0]), int(centroids[i][1])
            
            # 해당 영역의 평균 거리
            region_mask = labels == i
            region_depths = depth[region_mask]
            valid_depths = region_depths[region_depths > 0]
            
            if len(valid_depths) == 0:
                continue
            
            mean_distance = np.mean(valid_depths)
            
            # 위험 수준 결정
            danger_level = self._classify_danger(mean_distance)
            
            obstacle = Obstacle(
                id=self.next_id,
                bbox=(x, y, w_box, h_box),
                center=(cx, cy),
                distance_mm=mean_distance,
                area=area,
                danger_level=danger_level
            )
            
            obstacles.append(obstacle)
            self.next_id += 1
        
        # 거리순 정렬 (가까운 것 먼저)
        obstacles.sort(key=lambda o: o.distance_mm)
        
        return obstacles
    
    def _classify_danger(self, distance_mm: float) -> DangerLevel:
        """거리에 따른 위험 수준 분류"""
        
        if distance_mm < self.distance_thresholds['critical']:
            return DangerLevel.CRITICAL
        elif distance_mm < self.distance_thresholds['danger']:
            return DangerLevel.DANGER
        elif distance_mm < self.distance_thresholds['warning']:
            return DangerLevel.WARNING
        elif distance_mm < self.distance_thresholds['caution']:
            return DangerLevel.CAUTION
        else:
            return DangerLevel.SAFE
    
    def detect_in_zone(self, depth: np.ndarray,
                       zone_mask: np.ndarray) -> List[Obstacle]:
        """
        특정 영역 내 장애물 감지
        
        Parameters:
        - depth: 깊이 맵
        - zone_mask: 관심 영역 마스크
        
        Returns:
        - obstacles: 영역 내 장애물
        """
        
        # 영역 외부 제거
        masked_depth = depth.copy()
        masked_depth[~zone_mask.astype(bool)] = 0
        
        return self.detect(masked_depth)
    
    def get_closest_obstacle(self, obstacles: List[Obstacle]) -> Optional[Obstacle]:
        """가장 가까운 장애물 반환"""
        
        if not obstacles:
            return None
        
        return min(obstacles, key=lambda o: o.distance_mm)
    
    def get_obstacles_by_danger(self, obstacles: List[Obstacle],
                                 min_level: DangerLevel) -> List[Obstacle]:
        """특정 위험 수준 이상의 장애물 필터"""
        
        return [o for o in obstacles if o.danger_level.value >= min_level.value]


class GroundPlaneRemover:
    """지면 평면 제거"""
    
    def __init__(self, camera_height_mm: float = 500,
                 camera_tilt_deg: float = 0):
        """
        Parameters:
        - camera_height_mm: 카메라 높이 (mm)
        - camera_tilt_deg: 카메라 기울기 (도)
        """
        
        self.camera_height = camera_height_mm
        self.camera_tilt = np.radians(camera_tilt_deg)
    
    def remove_ground(self, depth: np.ndarray, 
                      focal_length: float,
                      cy: float) -> np.ndarray:
        """
        지면 제거 (기하학적 방법)
        
        카메라 높이와 기울기를 기반으로 지면 추정
        
        Returns:
        - filtered_depth: 지면이 제거된 깊이 맵
        """
        
        h, w = depth.shape
        
        # 각 행의 예상 지면 거리 계산
        v = np.arange(h)
        
        # 카메라 기하학
        # y = cy + f * tan(theta)
        # Z = H / sin(theta)
        
        theta = np.arctan2(v - cy, focal_length) + self.camera_tilt
        
        # 지면으로 예상되는 거리
        expected_ground_distance = np.where(
            theta > 0,
            self.camera_height / np.sin(theta + 1e-6),
            np.inf
        )
        
        # 마스크 생성
        filtered_depth = depth.copy()
        
        for row in range(h):
            ground_dist = expected_ground_distance[row]
            tolerance = 0.2 * ground_dist  # 20% 허용 오차
            
            ground_mask = np.abs(depth[row] - ground_dist) < tolerance
            filtered_depth[row, ground_mask] = 0
        
        return filtered_depth
    
    def remove_ground_ransac(self, points_3d: np.ndarray,
                             threshold: float = 50) -> np.ndarray:
        """
        RANSAC 기반 지면 제거
        
        Parameters:
        - points_3d: [N, 3] 3D 포인트
        - threshold: 인라이어 거리 임계값 (mm)
        
        Returns:
        - mask: 지면이 아닌 포인트 마스크
        """
        
        if len(points_3d) < 100:
            return np.ones(len(points_3d), dtype=bool)
        
        # RANSAC 파라미터
        n_iterations = 100
        best_inliers = 0
        best_plane = None
        
        for _ in range(n_iterations):
            # 3개 랜덤 포인트 선택
            idx = np.random.choice(len(points_3d), 3, replace=False)
            p1, p2, p3 = points_3d[idx]
            
            # 평면 방정식 계산 (ax + by + cz + d = 0)
            v1 = p2 - p1
            v2 = p3 - p1
            normal = np.cross(v1, v2)
            
            if np.linalg.norm(normal) < 1e-6:
                continue
            
            normal = normal / np.linalg.norm(normal)
            d = -np.dot(normal, p1)
            
            # 지면은 대략 수평 (normal의 y 성분이 큼)
            if abs(normal[1]) < 0.8:  # y가 위/아래 방향
                continue
            
            # 인라이어 계산
            distances = np.abs(np.dot(points_3d, normal) + d)
            inliers = np.sum(distances < threshold)
            
            if inliers > best_inliers:
                best_inliers = inliers
                best_plane = (normal, d)
        
        if best_plane is None:
            return np.ones(len(points_3d), dtype=bool)
        
        normal, d = best_plane
        distances = np.abs(np.dot(points_3d, normal) + d)
        
        # 지면이 아닌 포인트 (거리가 임계값 이상)
        mask = distances >= threshold
        
        return mask
```

### 3.2 U-Disparity 기반 장애물 감지

```python
"""
u_disparity.py
U-Disparity를 이용한 장애물 감지
"""

import cv2
import numpy as np
from typing import List, Tuple


class UDisparityDetector:
    """
    U-Disparity 기반 장애물 감지
    
    U-Disparity: 시차 맵의 열(column) 히스토그램
    - 수직선상의 시차 분포 분석
    - 지면과 장애물 분리에 효과적
    """
    
    def __init__(self, max_disparity: int = 128):
        self.max_disparity = max_disparity
    
    def compute_u_disparity(self, disparity: np.ndarray) -> np.ndarray:
        """
        U-Disparity 이미지 계산
        
        Parameters:
        - disparity: 시차 맵
        
        Returns:
        - u_disp: U-Disparity 이미지 [width, max_disparity]
        """
        
        h, w = disparity.shape
        u_disp = np.zeros((self.max_disparity, w), dtype=np.float32)
        
        for col in range(w):
            column = disparity[:, col]
            valid = (column > 0) & (column < self.max_disparity)
            
            if np.any(valid):
                # 히스토그램
                hist, _ = np.histogram(
                    column[valid].astype(int),
                    bins=self.max_disparity,
                    range=(0, self.max_disparity)
                )
                u_disp[:, col] = hist
        
        return u_disp
    
    def compute_v_disparity(self, disparity: np.ndarray) -> np.ndarray:
        """
        V-Disparity 이미지 계산 (지면 검출용)
        
        Returns:
        - v_disp: V-Disparity 이미지 [height, max_disparity]
        """
        
        h, w = disparity.shape
        v_disp = np.zeros((h, self.max_disparity), dtype=np.float32)
        
        for row in range(h):
            row_data = disparity[row, :]
            valid = (row_data > 0) & (row_data < self.max_disparity)
            
            if np.any(valid):
                hist, _ = np.histogram(
                    row_data[valid].astype(int),
                    bins=self.max_disparity,
                    range=(0, self.max_disparity)
                )
                v_disp[row, :] = hist
        
        return v_disp
    
    def detect_ground_line(self, v_disp: np.ndarray) -> Tuple[float, float]:
        """
        V-Disparity에서 지면 직선 검출 (Hough 변환)
        
        Returns:
        - slope, intercept: 직선 파라미터 (y = slope * d + intercept)
        """
        
        # 이진화
        _, binary = cv2.threshold(
            v_disp.astype(np.uint8), 
            5, 255, cv2.THRESH_BINARY
        )
        
        # Hough 직선 검출
        lines = cv2.HoughLines(binary, 1, np.pi / 180, threshold=50)
        
        if lines is None or len(lines) == 0:
            return 0, 0
        
        # 가장 강한 직선 선택
        rho, theta = lines[0][0]
        
        # y = slope * x + intercept 형태로 변환
        if abs(np.sin(theta)) > 0.01:
            slope = -np.cos(theta) / np.sin(theta)
            intercept = rho / np.sin(theta)
        else:
            slope = 0
            intercept = rho
        
        return slope, intercept
    
    def detect_obstacles_from_u_disparity(self, 
                                           u_disp: np.ndarray,
                                           disparity: np.ndarray,
                                           threshold: int = 10) -> List[dict]:
        """
        U-Disparity에서 장애물 검출
        
        Returns:
        - obstacles: [{'x_range': (x1, x2), 'disparity': d}, ...]
        """
        
        # U-Disparity 이진화
        u_disp_norm = cv2.normalize(u_disp, None, 0, 255, cv2.NORM_MINMAX)
        _, binary = cv2.threshold(
            u_disp_norm.astype(np.uint8),
            threshold, 255, cv2.THRESH_BINARY
        )
        
        # 연결 요소 분석
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
            binary, connectivity=8
        )
        
        obstacles = []
        
        for i in range(1, num_labels):
            x = stats[i, cv2.CC_STAT_LEFT]
            w = stats[i, cv2.CC_STAT_WIDTH]
            y = stats[i, cv2.CC_STAT_TOP]  # disparity 값
            h = stats[i, cv2.CC_STAT_HEIGHT]
            
            # 평균 시차
            avg_disp = y + h // 2
            
            obstacles.append({
                'x_range': (x, x + w),
                'disparity_range': (y, y + h),
                'avg_disparity': avg_disp,
                'width': w
            })
        
        return obstacles
```

---

## 4. 위험 영역 분류

### 4.1 영역 관리자

```python
"""
zone_manager.py
위험 영역 관리
"""

import cv2
import numpy as np
from typing import List, Dict, Optional, Tuple
from dataclasses import dataclass
from enum import Enum


class ZoneType(Enum):
    """영역 타입"""
    FULL_SCREEN = 0     # 전체 화면
    CENTER = 1          # 중앙 영역
    LEFT = 2            # 왼쪽
    RIGHT = 3           # 오른쪽
    BOTTOM = 4          # 하단
    CUSTOM = 5          # 사용자 정의


@dataclass
class DetectionZone:
    """감지 영역"""
    name: str
    zone_type: ZoneType
    mask: np.ndarray
    priority: int = 1
    enabled: bool = True
    color: Tuple[int, int, int] = (0, 255, 0)


class ZoneManager:
    """감지 영역 관리자"""
    
    def __init__(self, image_size: Tuple[int, int]):
        """
        Parameters:
        - image_size: (width, height)
        """
        
        self.width, self.height = image_size
        self.zones: Dict[str, DetectionZone] = {}
        
        # 기본 영역 생성
        self._create_default_zones()
    
    def _create_default_zones(self):
        """기본 영역 생성"""
        
        h, w = self.height, self.width
        
        # 1. 중앙 영역 (60%)
        center_mask = np.zeros((h, w), dtype=np.uint8)
        x1 = int(w * 0.2)
        x2 = int(w * 0.8)
        y1 = int(h * 0.2)
        y2 = int(h * 0.9)
        center_mask[y1:y2, x1:x2] = 255
        
        self.zones['center'] = DetectionZone(
            name='Center',
            zone_type=ZoneType.CENTER,
            mask=center_mask,
            priority=3,
            color=(0, 255, 255)
        )
        
        # 2. 왼쪽 영역
        left_mask = np.zeros((h, w), dtype=np.uint8)
        left_mask[:, :int(w * 0.3)] = 255
        
        self.zones['left'] = DetectionZone(
            name='Left',
            zone_type=ZoneType.LEFT,
            mask=left_mask,
            priority=2,
            color=(255, 0, 0)
        )
        
        # 3. 오른쪽 영역
        right_mask = np.zeros((h, w), dtype=np.uint8)
        right_mask[:, int(w * 0.7):] = 255
        
        self.zones['right'] = DetectionZone(
            name='Right',
            zone_type=ZoneType.RIGHT,
            mask=right_mask,
            priority=2,
            color=(0, 0, 255)
        )
        
        # 4. 하단 영역 (지면 근처)
        bottom_mask = np.zeros((h, w), dtype=np.uint8)
        bottom_mask[int(h * 0.7):, :] = 255
        
        self.zones['bottom'] = DetectionZone(
            name='Bottom',
            zone_type=ZoneType.BOTTOM,
            mask=bottom_mask,
            priority=1,
            color=(0, 255, 0)
        )
    
    def add_custom_zone(self, name: str, 
                        points: List[Tuple[int, int]],
                        priority: int = 1,
                        color: Tuple[int, int, int] = (255, 255, 0)):
        """
        사용자 정의 다각형 영역 추가
        
        Parameters:
        - name: 영역 이름
        - points: 다각형 꼭지점 [(x1,y1), (x2,y2), ...]
        - priority: 우선순위 (높을수록 중요)
        """
        
        mask = np.zeros((self.height, self.width), dtype=np.uint8)
        pts = np.array(points, dtype=np.int32)
        cv2.fillPoly(mask, [pts], 255)
        
        self.zones[name] = DetectionZone(
            name=name,
            zone_type=ZoneType.CUSTOM,
            mask=mask,
            priority=priority,
            color=color
        )
    
    def add_trapezoid_zone(self, name: str,
                           top_width_ratio: float = 0.3,
                           bottom_width_ratio: float = 0.8,
                           height_ratio: float = 0.6,
                           priority: int = 3):
        """
        사다리꼴 영역 추가 (전방 감지용)
        
        원근감을 반영한 형태
        """
        
        h, w = self.height, self.width
        
        top_w = int(w * top_width_ratio)
        bottom_w = int(w * bottom_width_ratio)
        zone_h = int(h * height_ratio)
        
        top_x1 = (w - top_w) // 2
        top_x2 = top_x1 + top_w
        bottom_x1 = (w - bottom_w) // 2
        bottom_x2 = bottom_x1 + bottom_w
        
        top_y = h - zone_h
        bottom_y = h
        
        points = [
            (top_x1, top_y),
            (top_x2, top_y),
            (bottom_x2, bottom_y),
            (bottom_x1, bottom_y)
        ]
        
        self.add_custom_zone(name, points, priority, (255, 255, 0))
    
    def get_zone(self, name: str) -> Optional[DetectionZone]:
        """영역 반환"""
        return self.zones.get(name)
    
    def get_enabled_zones(self) -> List[DetectionZone]:
        """활성화된 영역 반환 (우선순위 순)"""
        enabled = [z for z in self.zones.values() if z.enabled]
        return sorted(enabled, key=lambda z: z.priority, reverse=True)
    
    def enable_zone(self, name: str, enabled: bool = True):
        """영역 활성화/비활성화"""
        if name in self.zones:
            self.zones[name].enabled = enabled
    
    def get_combined_mask(self) -> np.ndarray:
        """모든 활성화 영역의 합집합 마스크"""
        
        combined = np.zeros((self.height, self.width), dtype=np.uint8)
        
        for zone in self.get_enabled_zones():
            combined = cv2.bitwise_or(combined, zone.mask)
        
        return combined
    
    def visualize_zones(self, image: np.ndarray,
                        alpha: float = 0.3) -> np.ndarray:
        """영역 시각화"""
        
        overlay = image.copy()
        
        for zone in self.get_enabled_zones():
            color_mask = np.zeros_like(image)
            color_mask[zone.mask > 0] = zone.color
            
            cv2.addWeighted(color_mask, alpha, overlay, 1, 0, overlay)
            
            # 영역 이름 표시
            moments = cv2.moments(zone.mask)
            if moments['m00'] > 0:
                cx = int(moments['m10'] / moments['m00'])
                cy = int(moments['m01'] / moments['m00'])
                cv2.putText(overlay, zone.name, (cx - 30, cy),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        return overlay
```

---

## 5. 경고 시스템

### 5.1 경고 관리자

```python
"""
alert_system.py
경고 시스템
"""

import time
import threading
from typing import Optional, Callable
from enum import Enum
from dataclasses import dataclass
import numpy as np

# 사운드 재생 (선택)
try:
    import pygame
    pygame.mixer.init()
    SOUND_AVAILABLE = True
except ImportError:
    SOUND_AVAILABLE = False


class AlertType(Enum):
    """경고 타입"""
    VISUAL = 1      # 시각적
    AUDIO = 2       # 청각적
    HAPTIC = 3      # 촉각적 (확장용)
    ALL = 4         # 모든 타입


@dataclass
class AlertConfig:
    """경고 설정"""
    enabled: bool = True
    visual: bool = True
    audio: bool = True
    min_interval_ms: int = 500  # 최소 경고 간격


class AlertSystem:
    """경고 시스템"""
    
    def __init__(self, config: AlertConfig = None):
        self.config = config or AlertConfig()
        
        # 마지막 경고 시간
        self.last_alert_time = {}
        
        # 콜백 함수
        self.alert_callbacks = []
        
        # 사운드 파일 (위험 수준별)
        self.sounds = {}
        if SOUND_AVAILABLE and self.config.audio:
            self._load_sounds()
        
        # 시각적 경고 색상
        self.alert_colors = {
            'SAFE': (0, 255, 0),       # 녹색
            'CAUTION': (0, 255, 255),  # 노란색
            'WARNING': (0, 165, 255),  # 주황색
            'DANGER': (0, 0, 255),     # 빨간색
            'CRITICAL': (0, 0, 255),   # 빨간색 (깜빡임)
        }
    
    def _load_sounds(self):
        """경고음 로드"""
        
        sound_files = {
            'CAUTION': 'assets/sounds/warning_low.wav',
            'WARNING': 'assets/sounds/warning_medium.wav',
            'DANGER': 'assets/sounds/warning_high.wav',
            'CRITICAL': 'assets/sounds/warning_high.wav',
        }
        
        for level, filepath in sound_files.items():
            try:
                self.sounds[level] = pygame.mixer.Sound(filepath)
            except:
                pass
    
    def add_callback(self, callback: Callable):
        """경고 콜백 추가"""
        self.alert_callbacks.append(callback)
    
    def trigger_alert(self, danger_level: str, 
                      distance_mm: float,
                      source: str = "obstacle"):
        """
        경고 발생
        
        Parameters:
        - danger_level: 위험 수준 이름
        - distance_mm: 거리
        - source: 경고 소스
        """
        
        if not self.config.enabled:
            return
        
        # 최소 간격 체크
        current_time = time.time() * 1000
        key = f"{danger_level}_{source}"
        
        if key in self.last_alert_time:
            elapsed = current_time - self.last_alert_time[key]
            if elapsed < self.config.min_interval_ms:
                return
        
        self.last_alert_time[key] = current_time
        
        # 오디오 경고
        if self.config.audio and danger_level in self.sounds:
            self._play_sound(danger_level)
        
        # 콜백 호출
        for callback in self.alert_callbacks:
            callback(danger_level, distance_mm, source)
    
    def _play_sound(self, danger_level: str):
        """경고음 재생"""
        
        if SOUND_AVAILABLE and danger_level in self.sounds:
            try:
                self.sounds[danger_level].play()
            except:
                pass
    
    def get_alert_color(self, danger_level: str) -> tuple:
        """위험 수준에 따른 색상 반환"""
        return self.alert_colors.get(danger_level, (255, 255, 255))
    
    def create_visual_alert(self, image: np.ndarray,
                            danger_level: str,
                            distance_mm: float) -> np.ndarray:
        """
        시각적 경고 오버레이
        
        Returns:
        - image: 경고가 추가된 이미지
        """
        
        h, w = image.shape[:2]
        color = self.get_alert_color(danger_level)
        
        # 테두리 효과
        border_width = 10
        
        if danger_level == 'CRITICAL':
            # 깜빡임 효과
            if int(time.time() * 4) % 2 == 0:
                cv2.rectangle(image, (0, 0), (w-1, h-1), color, border_width)
        elif danger_level in ['DANGER', 'WARNING']:
            cv2.rectangle(image, (0, 0), (w-1, h-1), color, border_width)
        
        # 경고 텍스트
        text = f"WARNING: {danger_level} - {distance_mm/1000:.1f}m"
        
        # 배경
        (text_w, text_h), _ = cv2.getTextSize(
            text, cv2.FONT_HERSHEY_SIMPLEX, 1, 2
        )
        
        cv2.rectangle(image, (w//2 - text_w//2 - 10, 20),
                     (w//2 + text_w//2 + 10, 60), (0, 0, 0), -1)
        
        cv2.putText(image, text, (w//2 - text_w//2, 50),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
        
        return image
    
    def create_distance_bar(self, image: np.ndarray,
                            distance_mm: float,
                            max_distance: float = 5000) -> np.ndarray:
        """
        거리 바 표시
        
        시각적 거리 게이지
        """
        
        h, w = image.shape[:2]
        
        bar_width = 30
        bar_height = h - 100
        bar_x = w - 50
        bar_y = 50
        
        # 배경
        cv2.rectangle(image, (bar_x, bar_y), 
                     (bar_x + bar_width, bar_y + bar_height),
                     (50, 50, 50), -1)
        
        # 거리별 색상 영역
        segments = [
            (0.0, 0.1, (0, 0, 255)),      # CRITICAL
            (0.1, 0.2, (0, 0, 255)),      # DANGER
            (0.2, 0.4, (0, 165, 255)),    # WARNING
            (0.4, 0.6, (0, 255, 255)),    # CAUTION
            (0.6, 1.0, (0, 255, 0)),      # SAFE
        ]
        
        for start_ratio, end_ratio, color in segments:
            y1 = bar_y + int(bar_height * (1 - end_ratio))
            y2 = bar_y + int(bar_height * (1 - start_ratio))
            cv2.rectangle(image, (bar_x, y1), (bar_x + bar_width, y2), color, -1)
        
        # 현재 거리 마커
        ratio = min(distance_mm / max_distance, 1.0)
        marker_y = bar_y + int(bar_height * (1 - ratio))
        
        cv2.line(image, (bar_x - 10, marker_y), 
                (bar_x + bar_width + 10, marker_y), (255, 255, 255), 3)
        
        # 거리 텍스트
        cv2.putText(image, f"{distance_mm/1000:.1f}m", 
                   (bar_x - 50, marker_y + 5),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        return image
```

---

## 6. 영역 분할

### 6.1 시맨틱 분할 (옵션)

```python
"""
segmentation.py
깊이 기반 영역 분할
"""

import cv2
import numpy as np
from typing import Tuple, List


class DepthSegmentation:
    """깊이 기반 영역 분할"""
    
    def __init__(self, num_layers: int = 5):
        """
        Parameters:
        - num_layers: 분할 레이어 수
        """
        self.num_layers = num_layers
    
    def segment_by_distance(self, depth: np.ndarray,
                            distance_ranges: List[Tuple[float, float]]) -> np.ndarray:
        """
        거리 범위별 분할
        
        Parameters:
        - depth: 깊이 맵 (mm)
        - distance_ranges: [(min1, max1), (min2, max2), ...]
        
        Returns:
        - labels: 레이블 맵 (0 = 배경, 1, 2, 3, ...)
        """
        
        labels = np.zeros(depth.shape, dtype=np.uint8)
        
        for i, (d_min, d_max) in enumerate(distance_ranges, 1):
            mask = (depth >= d_min) & (depth < d_max)
            labels[mask] = i
        
        return labels
    
    def segment_equal_depth(self, depth: np.ndarray,
                            min_depth: float = 500,
                            max_depth: float = 5000) -> np.ndarray:
        """
        균등 거리 분할
        
        Returns:
        - labels: 레이블 맵
        """
        
        # 거리 범위 생성
        ranges = []
        step = (max_depth - min_depth) / self.num_layers
        
        for i in range(self.num_layers):
            d_min = min_depth + i * step
            d_max = min_depth + (i + 1) * step
            ranges.append((d_min, d_max))
        
        return self.segment_by_distance(depth, ranges)
    
    def colorize_segments(self, labels: np.ndarray) -> np.ndarray:
        """분할 결과 컬러화"""
        
        # 색상 맵
        colors = [
            (0, 0, 0),        # 배경
            (0, 0, 255),      # 가장 가까움 (빨강)
            (0, 128, 255),    # 주황
            (0, 255, 255),    # 노랑
            (0, 255, 0),      # 초록
            (255, 255, 0),    # 청록
            (255, 0, 0),      # 파랑
        ]
        
        colored = np.zeros((*labels.shape, 3), dtype=np.uint8)
        
        for i, color in enumerate(colors[:self.num_layers + 1]):
            colored[labels == i] = color
        
        return colored
    
    def get_free_space(self, depth: np.ndarray,
                       safe_distance: float = 2000) -> np.ndarray:
        """
        이동 가능 영역 (Free Space) 추출
        
        Returns:
        - free_mask: 안전 영역 마스크
        """
        
        # 안전 거리 이상이거나 유효하지 않은 영역
        free_mask = (depth > safe_distance) | (depth <= 0)
        
        # 노이즈 제거
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        free_mask = cv2.morphologyEx(free_mask.astype(np.uint8), 
                                      cv2.MORPH_OPEN, kernel)
        
        return free_mask.astype(bool)


class OccupancyGrid:
    """점유 그리드 맵"""
    
    def __init__(self, grid_size: Tuple[int, int] = (100, 100),
                 cell_size_mm: float = 50):
        """
        Parameters:
        - grid_size: 그리드 크기 (width, height)
        - cell_size_mm: 셀 크기 (mm)
        """
        
        self.grid_size = grid_size
        self.cell_size = cell_size_mm
        
        # 그리드 초기화 (0.5 = 미지, 0 = 비어있음, 1 = 점유)
        self.grid = np.full(grid_size, 0.5, dtype=np.float32)
    
    def update_from_depth(self, depth: np.ndarray,
                          camera_params: dict):
        """
        깊이 맵에서 점유 그리드 업데이트
        
        Parameters:
        - depth: 깊이 맵
        - camera_params: 카메라 파라미터 (fx, fy, cx, cy)
        """
        
        h, w = depth.shape
        fx = camera_params['fx']
        cx = camera_params['cx']
        
        # 그리드 중심
        grid_cx = self.grid_size[0] // 2
        grid_cy = self.grid_size[1] // 2
        
        # 각 열에 대해 처리
        for col in range(w):
            # 해당 열의 유효한 깊이
            column = depth[:, col]
            valid_idx = np.where(column > 0)[0]
            
            if len(valid_idx) == 0:
                continue
            
            # 최소 깊이 (가장 가까운 장애물)
            min_depth = column[valid_idx].min()
            
            # 카메라 좌표계에서 X 계산
            x_camera = (col - cx) * min_depth / fx
            
            # 그리드 좌표 변환
            grid_x = int(grid_cx + x_camera / self.cell_size)
            grid_y = int(grid_cy - min_depth / self.cell_size)  # Y는 반전
            
            # 범위 체크
            if 0 <= grid_x < self.grid_size[0] and 0 <= grid_y < self.grid_size[1]:
                # 점유 마킹
                self.grid[grid_y, grid_x] = 1.0
                
                # 카메라와 점 사이는 비어있음
                # (간단한 ray casting)
                for gy in range(grid_cy, grid_y):
                    if 0 <= gy < self.grid_size[1]:
                        self.grid[gy, grid_x] = max(0, self.grid[gy, grid_x] - 0.1)
    
    def get_occupancy_image(self) -> np.ndarray:
        """점유 그리드 시각화"""
        
        # 0=흰색(비어있음), 0.5=회색(미지), 1=검정(점유)
        img = ((1 - self.grid) * 255).astype(np.uint8)
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        
        # 카메라 위치 표시
        cx, cy = self.grid_size[0] // 2, self.grid_size[1] // 2
        cv2.circle(img, (cx, cy), 3, (0, 0, 255), -1)
        
        return img
    
    def clear(self):
        """그리드 초기화"""
        self.grid.fill(0.5)
```

---

## 7. 추적 및 예측

### 7.1 간단한 객체 추적기

```python
"""
object_tracker.py
장애물 추적
"""

import numpy as np
from typing import List, Dict, Optional, Tuple
from dataclasses import dataclass, field
from collections import deque


@dataclass
class TrackedObject:
    """추적 중인 객체"""
    id: int
    positions: deque = field(default_factory=lambda: deque(maxlen=30))
    distances: deque = field(default_factory=lambda: deque(maxlen=30))
    last_seen: int = 0
    lost_frames: int = 0
    velocity: Tuple[float, float] = (0, 0)
    approaching: bool = False
    
    def update(self, center: Tuple[int, int], distance: float, frame_id: int):
        """상태 업데이트"""
        self.positions.append(center)
        self.distances.append(distance)
        self.last_seen = frame_id
        self.lost_frames = 0
        
        # 속도 계산
        if len(self.positions) >= 2:
            dx = center[0] - self.positions[-2][0]
            dy = center[1] - self.positions[-2][1]
            self.velocity = (dx, dy)
        
        # 접근 여부
        if len(self.distances) >= 2:
            self.approaching = self.distances[-1] < self.distances[-2]
    
    def predict_position(self, frames_ahead: int = 5) -> Tuple[int, int]:
        """위치 예측"""
        if len(self.positions) == 0:
            return (0, 0)
        
        last_pos = self.positions[-1]
        vx, vy = self.velocity
        
        pred_x = int(last_pos[0] + vx * frames_ahead)
        pred_y = int(last_pos[1] + vy * frames_ahead)
        
        return (pred_x, pred_y)
    
    def get_approach_rate(self) -> float:
        """접근 속도 (mm/frame)"""
        if len(self.distances) < 2:
            return 0
        
        return self.distances[-2] - self.distances[-1]
    
    def estimate_time_to_collision(self) -> Optional[float]:
        """충돌까지 예상 시간 (프레임)"""
        rate = self.get_approach_rate()
        
        if rate <= 0:
            return None  # 멀어지거나 정지
        
        if len(self.distances) == 0:
            return None
        
        current_distance = self.distances[-1]
        
        return current_distance / rate


class SimpleTracker:
    """간단한 장애물 추적기 (IOU 기반)"""
    
    def __init__(self, max_lost_frames: int = 10,
                 iou_threshold: float = 0.3):
        """
        Parameters:
        - max_lost_frames: 추적 유지 최대 손실 프레임
        - iou_threshold: 매칭 IOU 임계값
        """
        
        self.max_lost_frames = max_lost_frames
        self.iou_threshold = iou_threshold
        
        self.tracked_objects: Dict[int, TrackedObject] = {}
        self.next_id = 0
        self.frame_count = 0
    
    def update(self, detections: List[dict]) -> List[TrackedObject]:
        """
        추적 업데이트
        
        Parameters:
        - detections: [{'bbox': (x,y,w,h), 'center': (cx,cy), 'distance': d}, ...]
        
        Returns:
        - tracked_objects: 추적 중인 객체 리스트
        """
        
        self.frame_count += 1
        
        if len(detections) == 0:
            # 모든 추적 객체 손실 카운트 증가
            for obj in self.tracked_objects.values():
                obj.lost_frames += 1
            
            # 오래된 추적 제거
            self._remove_lost_tracks()
            return list(self.tracked_objects.values())
        
        # 현재 추적과 감지 매칭
        matched, unmatched_dets, unmatched_tracks = self._match_detections(detections)
        
        # 매칭된 추적 업데이트
        for det_idx, track_id in matched:
            det = detections[det_idx]
            self.tracked_objects[track_id].update(
                det['center'], det['distance'], self.frame_count
            )
        
        # 새 추적 생성
        for det_idx in unmatched_dets:
            det = detections[det_idx]
            new_track = TrackedObject(id=self.next_id)
            new_track.update(det['center'], det['distance'], self.frame_count)
            self.tracked_objects[self.next_id] = new_track
            self.next_id += 1
        
        # 손실된 추적 카운트 증가
        for track_id in unmatched_tracks:
            self.tracked_objects[track_id].lost_frames += 1
        
        # 오래된 추적 제거
        self._remove_lost_tracks()
        
        return list(self.tracked_objects.values())
    
    def _match_detections(self, detections: List[dict]) -> Tuple[List, List, List]:
        """감지와 추적 매칭"""
        
        if len(self.tracked_objects) == 0:
            return [], list(range(len(detections))), []
        
        track_ids = list(self.tracked_objects.keys())
        
        # 거리 행렬 계산
        cost_matrix = np.zeros((len(detections), len(track_ids)))
        
        for i, det in enumerate(detections):
            for j, track_id in enumerate(track_ids):
                track = self.tracked_objects[track_id]
                if len(track.positions) > 0:
                    last_pos = track.positions[-1]
                    # 유클리드 거리
                    dist = np.sqrt(
                        (det['center'][0] - last_pos[0])**2 +
                        (det['center'][1] - last_pos[1])**2
                    )
                    cost_matrix[i, j] = dist
                else:
                    cost_matrix[i, j] = 1e6
        
        # 그리디 매칭
        matched = []
        unmatched_dets = list(range(len(detections)))
        unmatched_tracks = list(track_ids)
        
        while len(unmatched_dets) > 0 and len(unmatched_tracks) > 0:
            # 최소 비용 찾기
            min_cost = np.inf
            min_i, min_j = -1, -1
            
            for i in unmatched_dets:
                for j_idx, j in enumerate(unmatched_tracks):
                    if cost_matrix[i, track_ids.index(j)] < min_cost:
                        min_cost = cost_matrix[i, track_ids.index(j)]
                        min_i, min_j = i, j
            
            # 임계값 체크
            if min_cost > 100:  # 픽셀 거리 임계값
                break
            
            matched.append((min_i, min_j))
            unmatched_dets.remove(min_i)
            unmatched_tracks.remove(min_j)
        
        return matched, unmatched_dets, unmatched_tracks
    
    def _remove_lost_tracks(self):
        """손실된 추적 제거"""
        to_remove = [
            track_id for track_id, track in self.tracked_objects.items()
            if track.lost_frames > self.max_lost_frames
        ]
        
        for track_id in to_remove:
            del self.tracked_objects[track_id]
    
    def get_approaching_objects(self) -> List[TrackedObject]:
        """접근 중인 객체 반환"""
        return [obj for obj in self.tracked_objects.values() if obj.approaching]
    
    def get_collision_warnings(self, threshold_frames: float = 30) -> List[TrackedObject]:
        """충돌 경고 객체 반환"""
        warnings = []
        
        for obj in self.tracked_objects.values():
            ttc = obj.estimate_time_to_collision()
            if ttc is not None and ttc < threshold_frames:
                warnings.append(obj)
        
        return warnings
```

---

## 8. GUI 구현

### 8.1 장애물 감지 GUI

```python
"""
gui.py
장애물 감지 시스템 GUI
"""

import cv2
import numpy as np
from typing import List, Optional, Tuple
import time


class ObstacleDetectionGUI:
    """장애물 감지 GUI"""
    
    def __init__(self, window_name: str = "Obstacle Detection"):
        self.window_name = window_name
        
        # 표시 설정
        self.show_depth = True
        self.show_zones = True
        self.show_bboxes = True
        self.show_tracks = True
        self.show_grid = False
        
        # 색상
        self.danger_colors = {
            'SAFE': (0, 255, 0),
            'CAUTION': (0, 255, 255),
            'WARNING': (0, 165, 255),
            'DANGER': (0, 0, 255),
            'CRITICAL': (0, 0, 255),
        }
        
        # FPS
        self.fps_history = []
        self.last_time = time.time()
        
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
    
    def update_fps(self) -> float:
        """FPS 업데이트"""
        current = time.time()
        fps = 1.0 / (current - self.last_time + 1e-6)
        self.last_time = current
        
        self.fps_history.append(fps)
        if len(self.fps_history) > 30:
            self.fps_history.pop(0)
        
        return np.mean(self.fps_history)
    
    def draw_obstacle(self, image: np.ndarray, obstacle, 
                      show_info: bool = True):
        """장애물 표시"""
        
        x, y, w, h = obstacle.bbox
        color = self.danger_colors.get(obstacle.danger_level.name, (255, 255, 255))
        
        # 바운딩 박스
        cv2.rectangle(image, (x, y), (x + w, y + h), color, 2)
        
        # 중심점
        cx, cy = obstacle.center
        cv2.circle(image, (cx, cy), 5, color, -1)
        
        # 정보 표시
        if show_info:
            distance_m = obstacle.distance_mm / 1000
            text = f"{obstacle.danger_level.name}: {distance_m:.1f}m"
            
            # 배경
            (text_w, text_h), _ = cv2.getTextSize(
                text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1
            )
            cv2.rectangle(image, (x, y - text_h - 5), 
                         (x + text_w, y), color, -1)
            
            cv2.putText(image, text, (x, y - 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
    
    def draw_track(self, image: np.ndarray, track, 
                   show_prediction: bool = True):
        """추적 경로 표시"""
        
        if len(track.positions) < 2:
            return
        
        # 경로 선
        points = list(track.positions)
        for i in range(1, len(points)):
            alpha = i / len(points)
            color = (
                int(255 * alpha),
                int(255 * (1 - alpha)),
                0
            )
            cv2.line(image, points[i-1], points[i], color, 2)
        
        # 예측 위치
        if show_prediction and track.approaching:
            pred_pos = track.predict_position(10)
            cv2.circle(image, pred_pos, 8, (0, 255, 255), 2)
            cv2.line(image, points[-1], pred_pos, (0, 255, 255), 1)
    
    def draw_info_panel(self, image: np.ndarray,
                        num_obstacles: int,
                        closest_distance: float,
                        danger_level: str,
                        fps: float):
        """정보 패널"""
        
        h, w = image.shape[:2]
        
        # 반투명 배경
        overlay = image.copy()
        cv2.rectangle(overlay, (10, 10), (280, 140), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.6, image, 0.4, 0, image)
        
        # 텍스트
        y = 35
        cv2.putText(image, f"FPS: {fps:.1f}", (20, y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        y += 25
        cv2.putText(image, f"Obstacles: {num_obstacles}", (20, y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        y += 25
        color = self.danger_colors.get(danger_level, (255, 255, 255))
        cv2.putText(image, f"Level: {danger_level}", (20, y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        
        y += 25
        if closest_distance > 0:
            cv2.putText(image, f"Closest: {closest_distance/1000:.2f}m", (20, y),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        else:
            cv2.putText(image, "Closest: --", (20, y),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (128, 128, 128), 2)
    
    def draw_minimap(self, image: np.ndarray,
                     obstacles: list,
                     max_distance: float = 5000):
        """미니맵 (탑뷰)"""
        
        h, w = image.shape[:2]
        
        map_size = 150
        map_x = w - map_size - 20
        map_y = h - map_size - 20
        
        # 배경
        cv2.rectangle(image, (map_x, map_y), 
                     (map_x + map_size, map_y + map_size),
                     (30, 30, 30), -1)
        
        # 거리 원
        center = (map_x + map_size // 2, map_y + map_size)
        for dist_m in [1, 2, 3, 4, 5]:
            radius = int((dist_m / (max_distance / 1000)) * map_size)
            cv2.ellipse(image, center, (radius, radius // 2), 0, 
                       180, 360, (60, 60, 60), 1)
        
        # 장애물 표시
        for obs in obstacles:
            # 간단한 좌표 변환 (X는 가로, Z는 세로)
            rel_x = (obs.center[0] - w // 2) / w  # -0.5 ~ 0.5
            rel_z = obs.distance_mm / max_distance  # 0 ~ 1
            
            px = int(center[0] + rel_x * map_size)
            py = int(center[1] - rel_z * map_size)
            
            color = self.danger_colors.get(obs.danger_level.name, (255, 255, 255))
            cv2.circle(image, (px, py), 5, color, -1)
        
        # 카메라 위치 (삼각형)
        pts = np.array([
            [center[0], center[1] - 5],
            [center[0] - 5, center[1] + 5],
            [center[0] + 5, center[1] + 5]
        ], np.int32)
        cv2.fillPoly(image, [pts], (0, 255, 0))
    
    def colorize_depth(self, depth: np.ndarray,
                       max_depth: float = 5000) -> np.ndarray:
        """깊이 맵 컬러화"""
        
        depth_clipped = np.clip(depth, 0, max_depth)
        depth_norm = (1 - depth_clipped / max_depth) * 255
        depth_color = cv2.applyColorMap(depth_norm.astype(np.uint8), cv2.COLORMAP_TURBO)
        depth_color[depth <= 0] = [0, 0, 0]
        
        return depth_color
    
    def render(self, color_image: np.ndarray,
               depth: np.ndarray,
               obstacles: list,
               tracks: list = None,
               zones=None) -> np.ndarray:
        """전체 렌더링"""
        
        fps = self.update_fps()
        display = color_image.copy()
        h, w = display.shape[:2]
        
        # 영역 표시
        if self.show_zones and zones is not None:
            display = zones.visualize_zones(display, alpha=0.2)
        
        # 장애물 표시
        if self.show_bboxes:
            for obs in obstacles:
                self.draw_obstacle(display, obs)
        
        # 추적 경로 표시
        if self.show_tracks and tracks:
            for track in tracks:
                self.draw_track(display, track)
        
        # 가장 가까운 장애물
        closest_dist = 0
        danger_level = 'SAFE'
        if obstacles:
            closest = min(obstacles, key=lambda o: o.distance_mm)
            closest_dist = closest.distance_mm
            danger_level = closest.danger_level.name
        
        # 정보 패널
        self.draw_info_panel(display, len(obstacles), closest_dist,
                            danger_level, fps)
        
        # 미니맵
        self.draw_minimap(display, obstacles)
        
        # 깊이 맵 (작은 크기)
        if self.show_depth:
            depth_color = self.colorize_depth(depth)
            depth_small = cv2.resize(depth_color, (w // 4, h // 4))
            display[10:10 + h // 4, w - w // 4 - 10:w - 10] = depth_small
        
        return display
    
    def show(self, image: np.ndarray):
        """이미지 표시"""
        cv2.imshow(self.window_name, image)
    
    def wait_key(self, delay: int = 1) -> int:
        """키 입력"""
        return cv2.waitKey(delay) & 0xFF
    
    def close(self):
        """윈도우 닫기"""
        cv2.destroyAllWindows()
```

---

## 9. 전체 코드

### 9.1 메인 실행 파일

```python
"""
main.py
장애물 감지 시스템 메인
"""

import argparse
import yaml
import cv2
import numpy as np
import sys
from datetime import datetime

# 로컬 모듈
sys.path.append('src')
from stereo_camera import StereoCamera
from depth_processor import DepthProcessor
from obstacle_detector import ObstacleDetector, DangerLevel
from zone_manager import ZoneManager
from alert_system import AlertSystem, AlertConfig
from object_tracker import SimpleTracker
from gui import ObstacleDetectionGUI


class ObstacleDetectionSystem:
    """장애물 감지 시스템"""
    
    def __init__(self, config_file: str):
        # 설정 로드
        with open(config_file, 'r') as f:
            self.config = yaml.safe_load(f)
        
        self._init_components()
        
        print("\n" + "="*60)
        print("🚧 장애물 감지 시스템")
        print("="*60)
    
    def _init_components(self):
        """컴포넌트 초기화"""
        
        # 스테레오 카메라
        self.camera = StereoCamera(
            calibration_file=self.config['calibration_file'],
            left_id=self.config['camera']['left_id'],
            right_id=self.config['camera']['right_id']
        )
        
        # 깊이 처리기
        self.depth_processor = DepthProcessor(
            focal_length=self.camera.focal_length,
            baseline=self.camera.baseline,
            **self.config['stereo']
        )
        
        # 장애물 감지기
        self.detector = ObstacleDetector(self.config['detection'])
        
        # 영역 관리자
        img_size = (self.camera.width, self.camera.height)
        self.zone_manager = ZoneManager(img_size)
        self.zone_manager.add_trapezoid_zone('forward', priority=3)
        
        # 경고 시스템
        alert_config = AlertConfig(**self.config.get('alert', {}))
        self.alert_system = AlertSystem(alert_config)
        
        # 추적기
        self.tracker = SimpleTracker(
            max_lost_frames=self.config['tracking']['max_lost_frames'],
            iou_threshold=self.config['tracking']['iou_threshold']
        )
        
        # GUI
        self.gui = ObstacleDetectionGUI()
    
    def run(self):
        """메인 루프"""
        
        print("\n조작 방법:")
        print("  [D] - 깊이맵 표시 토글")
        print("  [Z] - 영역 표시 토글")
        print("  [T] - 추적 표시 토글")
        print("  [S] - 스크린샷")
        print("  [Q] - 종료")
        print("="*60 + "\n")
        
        while True:
            # 프레임 캡처
            rect_left, rect_right = self.camera.capture_rectified()
            
            if rect_left is None:
                continue
            
            # 깊이 계산
            disparity, depth = self.depth_processor.compute(rect_left, rect_right)
            
            # 장애물 감지
            obstacles = self.detector.detect(depth, rect_left)
            
            # 추적 업데이트
            detections = [
                {
                    'bbox': o.bbox,
                    'center': o.center,
                    'distance': o.distance_mm
                }
                for o in obstacles
            ]
            tracks = self.tracker.update(detections)
            
            # 경고 처리
            self._process_alerts(obstacles)
            
            # 렌더링
            display = self.gui.render(
                rect_left, depth, obstacles, tracks, self.zone_manager
            )
            
            # 경고 오버레이
            if obstacles:
                closest = min(obstacles, key=lambda o: o.distance_mm)
                if closest.danger_level.value >= DangerLevel.WARNING.value:
                    display = self.alert_system.create_visual_alert(
                        display,
                        closest.danger_level.name,
                        closest.distance_mm
                    )
            
            self.gui.show(display)
            
            # 키 처리
            key = self.gui.wait_key(1)
            if key == ord('q'):
                break
            elif key == ord('d'):
                self.gui.show_depth = not self.gui.show_depth
            elif key == ord('z'):
                self.gui.show_zones = not self.gui.show_zones
            elif key == ord('t'):
                self.gui.show_tracks = not self.gui.show_tracks
            elif key == ord('s'):
                self._save_screenshot(display)
        
        self._cleanup()
    
    def _process_alerts(self, obstacles):
        """경고 처리"""
        
        for obs in obstacles:
            if obs.danger_level.value >= DangerLevel.WARNING.value:
                self.alert_system.trigger_alert(
                    obs.danger_level.name,
                    obs.distance_mm,
                    f"obstacle_{obs.id}"
                )
    
    def _save_screenshot(self, image):
        """스크린샷 저장"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"screenshots/obstacle_{timestamp}.png"
        cv2.imwrite(filename, image)
        print(f"📸 스크린샷 저장: {filename}")
    
    def _cleanup(self):
        """정리"""
        self.camera.release()
        self.gui.close()
        print("\n시스템 종료")


def main():
    parser = argparse.ArgumentParser(description='장애물 감지 시스템')
    parser.add_argument('--config', default='config/detection_config.yaml')
    args = parser.parse_args()
    
    system = ObstacleDetectionSystem(args.config)
    system.run()


if __name__ == "__main__":
    main()
```

### 9.2 설정 파일

```yaml
# config/detection_config.yaml

calibration_file: "config/stereo_params.yaml"

camera:
  left_id: 0
  right_id: 2
  width: 1280
  height: 720
  fps: 30

stereo:
  num_disparities: 128
  block_size: 5
  use_wls: true

detection:
  distance_thresholds:
    critical: 500    # 0.5m
    danger: 1000     # 1m
    warning: 2000    # 2m
    caution: 3000    # 3m
  min_distance: 200
  max_distance: 5000
  min_obstacle_area: 500
  ground_removal: true

tracking:
  max_lost_frames: 10
  iou_threshold: 0.3

alert:
  enabled: true
  visual: true
  audio: false
  min_interval_ms: 500
```

---

## 10. 성능 최적화

### 10.1 최적화 팁

```
┌─────────────────────────────────────────────────────────────┐
│                    성능 최적화 가이드                         │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  📹 입력 최적화                                              │
│    - 해상도 감소: 1280x720 → 640x480                        │
│    - ROI 처리: 전체 프레임 대신 관심 영역만                   │
│                                                             │
│  🔧 스테레오 최적화                                          │
│    - numDisparities 감소: 128 → 64                         │
│    - blockSize 증가: 5 → 7 (노이즈 감소)                    │
│    - MODE_SGBM_3WAY 사용                                    │
│                                                             │
│  🎯 감지 최적화                                              │
│    - min_obstacle_area 증가: 작은 노이즈 무시               │
│    - 프레임 스킵: 2-3 프레임마다 감지                        │
│                                                             │
│  💾 메모리 최적화                                            │
│    - numpy 배열 재사용                                       │
│    - 불필요한 복사 최소화                                    │
│                                                             │
│  🚀 병렬 처리                                                │
│    - 멀티스레딩: 캡처/처리 분리                              │
│    - GPU: OpenCV CUDA 또는 VPI                             │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

---

## 📝 학습 체크리스트

### 구현 완료

- [ ] 깊이 기반 장애물 감지
- [ ] 위험 영역 분류 (5단계)
- [ ] 감지 영역 관리
- [ ] 경고 시스템 (시각/청각)
- [ ] 객체 추적
- [ ] GUI 구현
- [ ] 미니맵 (탑뷰)

### 테스트 완료

- [ ] 다양한 거리에서 감지 테스트
- [ ] 다중 장애물 감지
- [ ] 추적 정확도 확인
- [ ] 경고 시스템 동작 확인
- [ ] 성능 (FPS) 확인

---

## ➡️ 다음 프로젝트

**[Project 03: 3D 스캐너](../Project_03_3D_Scanner/README.md)**

다음 프로젝트에서는:
- 다중 시점 3D 스캐닝
- 포인트 클라우드 정합
- 메쉬 생성
- STL/OBJ 출력

을 구현합니다.

---

## 📄 라이선스

MIT License - 자유롭게 사용, 수정, 배포 가능
