# Project 04: Visual Odometry (시각적 주행 거리 측정)

[![난이도](https://img.shields.io/badge/난이도-⭐⭐⭐⭐⭐_전문가-purple.svg)]()
[![예상시간](https://img.shields.io/badge/예상시간-15--20시간-blue.svg)]()
[![선수지식](https://img.shields.io/badge/선수지식-Module_01--04,_선형대수-orange.svg)]()

---

## 🎯 프로젝트 개요

| 항목 | 내용 |
|------|------|
| **목표** | 스테레오 카메라로 카메라/로봇의 이동 경로 추정 |
| **기능** | 특징점 추적, 모션 추정, 3D 궤적 시각화, 맵 생성 |
| **응용** | 로봇 내비게이션, 드론 자율비행, AR/VR, SLAM |

---

## 📋 목차

1. [Visual Odometry 개요](#1-visual-odometry-개요)
2. [시스템 아키텍처](#2-시스템-아키텍처)
3. [특징점 검출 및 매칭](#3-특징점-검출-및-매칭)
4. [스테레오 3D 재구성](#4-스테레오-3d-재구성)
5. [모션 추정](#5-모션-추정)
6. [궤적 추적](#6-궤적-추적)
7. [최적화 및 드리프트 보정](#7-최적화-및-드리프트-보정)
8. [시각화](#8-시각화)
9. [전체 코드](#9-전체-코드)
10. [성능 평가](#10-성능-평가)

## 📋 Project 04 주요 내용

| 섹션 | 내용 |
|------|------|
| 1. Visual Odometry 개요 | VO 개념, Mono vs Stereo, 파이프라인 |
| 2. 시스템 아키텍처 | 프로젝트 구조 |
| 3. 특징점 검출 및 매칭 | FeatureDetector, FeatureTracker, FeatureMatcher |
| 4. 스테레오 3D 재구성 | StereoTriangulator, 삼각측량 |
| 5. 모션 추정 | MotionEstimator (PnP, Essential Matrix), RANSAC |
| 6. 궤적 추적 | TrajectoryTracker, 포즈 누적 |
| 7. 최적화 및 드리프트 보정 | Bundle Adjustment, Loop Closure |
| 8. 시각화 | VOVisualizer (2D/3D 궤적) |
| 9. 전체 코드 | StereoVisualOdometry 클래스, main.py |
| 10. 성능 평가 | ATE, RPE 메트릭 |


---

## 1. Visual Odometry 개요

### 1.1 Visual Odometry란?

Visual Odometry(VO)는 연속적인 카메라 이미지를 분석하여 카메라의 위치와 방향 변화를 추정하는 기술입니다.

```
┌─────────────────────────────────────────────────────────────┐
│                Visual Odometry 개념                          │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│   시간 t         시간 t+1       시간 t+2                     │
│                                                             │
│   ┌───┐          ┌───┐          ┌───┐                      │
│   │ 📷 │ ──────→ │ 📷 │ ──────→ │ 📷 │                      │
│   └───┘          └───┘          └───┘                      │
│     P₀             P₁             P₂                        │
│                                                             │
│   이미지 분석을 통해 카메라 이동 (T₀₁, T₁₂) 추정            │
│                                                             │
│   ┌─────────────────────────────────────────┐              │
│   │                                         │              │
│   │    P₀ ──T₀₁──→ P₁ ──T₁₂──→ P₂          │              │
│   │     ↓                                   │              │
│   │   누적하여 전체 궤적 생성               │              │
│   │                                         │              │
│   └─────────────────────────────────────────┘              │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### 1.2 Monocular vs Stereo VO

| 특성 | Monocular VO | Stereo VO |
|------|-------------|-----------|
| **카메라** | 1대 | 2대 |
| **스케일** | 알 수 없음 (up-to-scale) | 실제 스케일 복원 가능 |
| **초기화** | 필요 (5-point 등) | 불필요 |
| **정확도** | 상대적 낮음 | 높음 |
| **비용** | 저렴 | 상대적 높음 |

### 1.3 VO 파이프라인

```
┌─────────────────────────────────────────────────────────────┐
│              Stereo Visual Odometry 파이프라인               │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  [Frame t-1]         [Frame t]                              │
│  Left  Right         Left  Right                            │
│    │     │             │     │                              │
│    └──┬──┘             └──┬──┘                              │
│       │                   │                                 │
│       ▼                   ▼                                 │
│  ┌─────────┐         ┌─────────┐                           │
│  │Feature  │         │Feature  │                           │
│  │Detection│         │Detection│                           │
│  └────┬────┘         └────┬────┘                           │
│       │                   │                                 │
│       ▼                   ▼                                 │
│  ┌─────────┐         ┌─────────┐                           │
│  │ Stereo  │         │ Stereo  │                           │
│  │Matching │         │Matching │                           │
│  └────┬────┘         └────┬────┘                           │
│       │                   │                                 │
│       ▼                   ▼                                 │
│  ┌─────────┐         ┌─────────┐                           │
│  │3D Points│         │3D Points│                           │
│  │  (t-1)  │         │   (t)   │                           │
│  └────┬────┘         └────┬────┘                           │
│       │                   │                                 │
│       └─────────┬─────────┘                                │
│                 ▼                                           │
│         ┌─────────────┐                                    │
│         │  Temporal   │                                    │
│         │  Matching   │                                    │
│         └──────┬──────┘                                    │
│                ▼                                            │
│         ┌─────────────┐                                    │
│         │   Motion    │                                    │
│         │ Estimation  │                                    │
│         │  (PnP/ICP)  │                                    │
│         └──────┬──────┘                                    │
│                ▼                                            │
│         ┌─────────────┐                                    │
│         │    Pose     │                                    │
│         │   Update    │                                    │
│         └─────────────┘                                    │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

---

## 2. 시스템 아키텍처

### 2.1 프로젝트 구조

```
Project_04_Visual_Odometry/
├── README.md
├── requirements.txt
├── config/
│   ├── stereo_params.yaml      # 캘리브레이션
│   └── vo_config.yaml          # VO 설정
├── src/
│   ├── __init__.py
│   ├── main.py                 # 메인 실행
│   ├── stereo_camera.py        # 스테레오 카메라
│   ├── feature_detector.py     # 특징점 검출
│   ├── feature_matcher.py      # 특징점 매칭
│   ├── stereo_matcher.py       # 스테레오 매칭
│   ├── motion_estimator.py     # 모션 추정
│   ├── trajectory_tracker.py   # 궤적 추적
│   ├── local_map.py            # 로컬 맵
│   ├── visualizer.py           # 시각화
│   └── utils.py                # 유틸리티
├── data/
│   └── sequences/              # 테스트 시퀀스
└── output/
    ├── trajectories/           # 궤적 결과
    └── maps/                   # 맵 출력
```

---

## 3. 특징점 검출 및 매칭

### 3.1 특징점 검출기

```python
"""
feature_detector.py
특징점 검출
"""

import cv2
import numpy as np
from typing import Tuple, List, Optional
from enum import Enum


class DetectorType(Enum):
    """검출기 타입"""
    ORB = 1
    SIFT = 2
    SURF = 3
    FAST = 4
    GFTT = 5  # Good Features To Track


class FeatureDetector:
    """특징점 검출기"""
    
    def __init__(self, detector_type: DetectorType = DetectorType.ORB,
                 max_features: int = 2000):
        """
        Parameters:
        - detector_type: 검출기 타입
        - max_features: 최대 특징점 수
        """
        
        self.detector_type = detector_type
        self.max_features = max_features
        
        self._create_detector()
    
    def _create_detector(self):
        """검출기 생성"""
        
        if self.detector_type == DetectorType.ORB:
            self.detector = cv2.ORB_create(
                nfeatures=self.max_features,
                scaleFactor=1.2,
                nlevels=8,
                edgeThreshold=31,
                firstLevel=0,
                WTA_K=2,
                patchSize=31,
                fastThreshold=20
            )
            self.descriptor = self.detector
            
        elif self.detector_type == DetectorType.SIFT:
            self.detector = cv2.SIFT_create(
                nfeatures=self.max_features,
                nOctaveLayers=3,
                contrastThreshold=0.04,
                edgeThreshold=10,
                sigma=1.6
            )
            self.descriptor = self.detector
            
        elif self.detector_type == DetectorType.FAST:
            self.detector = cv2.FastFeatureDetector_create(
                threshold=20,
                nonmaxSuppression=True
            )
            # FAST는 디스크립터가 없으므로 ORB 사용
            self.descriptor = cv2.ORB_create(nfeatures=self.max_features)
            
        elif self.detector_type == DetectorType.GFTT:
            self.detector = None  # cv2.goodFeaturesToTrack 사용
            self.descriptor = cv2.ORB_create(nfeatures=self.max_features)
    
    def detect(self, image: np.ndarray, 
               mask: Optional[np.ndarray] = None) -> Tuple[List, np.ndarray]:
        """
        특징점 검출 및 디스크립터 계산
        
        Parameters:
        - image: 그레이스케일 이미지
        - mask: 관심 영역 마스크
        
        Returns:
        - keypoints: 키포인트 리스트
        - descriptors: 디스크립터 배열
        """
        
        if len(image.shape) == 3:
            image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        if self.detector_type == DetectorType.GFTT:
            # Good Features To Track
            corners = cv2.goodFeaturesToTrack(
                image,
                maxCorners=self.max_features,
                qualityLevel=0.01,
                minDistance=10,
                mask=mask
            )
            
            if corners is None:
                return [], None
            
            # KeyPoint로 변환
            keypoints = [cv2.KeyPoint(x=c[0][0], y=c[0][1], size=20) 
                        for c in corners]
            
            # 디스크립터 계산
            keypoints, descriptors = self.descriptor.compute(image, keypoints)
        else:
            # 일반 검출기
            keypoints, descriptors = self.detector.detectAndCompute(image, mask)
        
        return keypoints, descriptors
    
    def detect_and_compute_stereo(self, left: np.ndarray, 
                                   right: np.ndarray) -> dict:
        """
        스테레오 이미지 쌍에서 특징점 검출
        
        Returns:
        - dict: {'left_kp', 'left_desc', 'right_kp', 'right_desc'}
        """
        
        left_kp, left_desc = self.detect(left)
        right_kp, right_desc = self.detect(right)
        
        return {
            'left_kp': left_kp,
            'left_desc': left_desc,
            'right_kp': right_kp,
            'right_desc': right_desc
        }


class FeatureTracker:
    """광학 흐름 기반 특징점 추적"""
    
    def __init__(self, max_features: int = 2000):
        self.max_features = max_features
        
        # Lucas-Kanade 파라미터
        self.lk_params = dict(
            winSize=(21, 21),
            maxLevel=3,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01)
        )
        
        # 특징점 검출 파라미터
        self.feature_params = dict(
            maxCorners=max_features,
            qualityLevel=0.01,
            minDistance=10,
            blockSize=7
        )
        
        self.prev_image = None
        self.prev_points = None
        self.track_ids = None
        self.next_id = 0
    
    def initialize(self, image: np.ndarray) -> np.ndarray:
        """첫 프레임 초기화"""
        
        if len(image.shape) == 3:
            image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        self.prev_image = image
        
        # 특징점 검출
        points = cv2.goodFeaturesToTrack(image, **self.feature_params)
        
        if points is not None:
            self.prev_points = points.reshape(-1, 2)
            self.track_ids = np.arange(len(self.prev_points))
            self.next_id = len(self.prev_points)
        else:
            self.prev_points = np.array([])
            self.track_ids = np.array([])
        
        return self.prev_points
    
    def track(self, image: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        프레임 간 특징점 추적
        
        Returns:
        - prev_points: 이전 프레임 점들
        - curr_points: 현재 프레임 점들
        - track_ids: 추적 ID
        """
        
        if len(image.shape) == 3:
            image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        if self.prev_points is None or len(self.prev_points) == 0:
            self.initialize(image)
            return None, None, None
        
        # 광학 흐름 추적
        curr_points, status, _ = cv2.calcOpticalFlowPyrLK(
            self.prev_image, image,
            self.prev_points.astype(np.float32),
            None, **self.lk_params
        )
        
        # 역방향 검증
        if curr_points is not None:
            prev_points_back, status_back, _ = cv2.calcOpticalFlowPyrLK(
                image, self.prev_image,
                curr_points,
                None, **self.lk_params
            )
            
            # 양방향 오차
            fb_error = np.linalg.norm(
                self.prev_points - prev_points_back.reshape(-1, 2), axis=1
            )
            
            # 유효한 추적만 선택
            valid = (status.flatten() == 1) & (fb_error < 1.0)
        else:
            valid = np.zeros(len(self.prev_points), dtype=bool)
        
        # 유효한 점 필터링
        prev_valid = self.prev_points[valid]
        curr_valid = curr_points.reshape(-1, 2)[valid]
        ids_valid = self.track_ids[valid]
        
        # 특징점 보충
        if len(curr_valid) < self.max_features * 0.5:
            mask = np.ones(image.shape[:2], dtype=np.uint8) * 255
            
            for pt in curr_valid:
                cv2.circle(mask, (int(pt[0]), int(pt[1])), 10, 0, -1)
            
            new_points = cv2.goodFeaturesToTrack(
                image, 
                maxCorners=self.max_features - len(curr_valid),
                qualityLevel=0.01,
                minDistance=10,
                mask=mask
            )
            
            if new_points is not None:
                new_points = new_points.reshape(-1, 2)
                new_ids = np.arange(self.next_id, self.next_id + len(new_points))
                self.next_id += len(new_points)
                
                curr_valid = np.vstack([curr_valid, new_points])
                ids_valid = np.concatenate([ids_valid, new_ids])
        
        # 상태 업데이트
        result_prev = self.prev_points[valid] if len(self.prev_points) > 0 else None
        
        self.prev_image = image
        self.prev_points = curr_valid
        self.track_ids = ids_valid
        
        return result_prev, curr_valid[:len(result_prev)] if result_prev is not None else None, ids_valid[:len(result_prev)] if result_prev is not None else None
```

### 3.2 특징점 매칭

```python
"""
feature_matcher.py
특징점 매칭
"""

import cv2
import numpy as np
from typing import List, Tuple, Optional


class FeatureMatcher:
    """특징점 매칭"""
    
    def __init__(self, matcher_type: str = 'BF', 
                 cross_check: bool = True,
                 ratio_threshold: float = 0.75):
        """
        Parameters:
        - matcher_type: 'BF' (Brute Force) 또는 'FLANN'
        - cross_check: 교차 검증 사용
        - ratio_threshold: Lowe's ratio test 임계값
        """
        
        self.ratio_threshold = ratio_threshold
        
        if matcher_type == 'BF':
            self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=cross_check)
        else:
            # FLANN
            index_params = dict(algorithm=6,  # FLANN_INDEX_LSH
                               table_number=6,
                               key_size=12,
                               multi_probe_level=1)
            search_params = dict(checks=50)
            self.matcher = cv2.FlannBasedMatcher(index_params, search_params)
        
        self.cross_check = cross_check
    
    def match(self, desc1: np.ndarray, desc2: np.ndarray) -> List:
        """
        디스크립터 매칭
        
        Returns:
        - matches: 매칭 리스트
        """
        
        if desc1 is None or desc2 is None:
            return []
        
        if len(desc1) == 0 or len(desc2) == 0:
            return []
        
        if self.cross_check:
            matches = self.matcher.match(desc1, desc2)
            # 거리순 정렬
            matches = sorted(matches, key=lambda x: x.distance)
        else:
            # KNN 매칭 + Lowe's ratio test
            matches_knn = self.matcher.knnMatch(desc1, desc2, k=2)
            
            matches = []
            for m_n in matches_knn:
                if len(m_n) == 2:
                    m, n = m_n
                    if m.distance < self.ratio_threshold * n.distance:
                        matches.append(m)
        
        return matches
    
    def match_stereo(self, left_kp: List, left_desc: np.ndarray,
                     right_kp: List, right_desc: np.ndarray,
                     max_y_diff: float = 2.0) -> Tuple[np.ndarray, np.ndarray, List]:
        """
        스테레오 매칭 (에피폴라 제약)
        
        정류된 이미지에서는 매칭점이 같은 행에 있어야 함
        
        Returns:
        - left_pts: 왼쪽 이미지 점들 [N, 2]
        - right_pts: 오른쪽 이미지 점들 [N, 2]
        - matches: 유효한 매칭
        """
        
        matches = self.match(left_desc, right_desc)
        
        valid_matches = []
        left_pts = []
        right_pts = []
        
        for m in matches:
            pt_l = left_kp[m.queryIdx].pt
            pt_r = right_kp[m.trainIdx].pt
            
            # 에피폴라 제약: y 좌표 차이가 작아야 함
            if abs(pt_l[1] - pt_r[1]) < max_y_diff:
                # 시차 양수 확인 (왼쪽이 오른쪽보다 x 좌표가 큼)
                if pt_l[0] > pt_r[0]:
                    valid_matches.append(m)
                    left_pts.append(pt_l)
                    right_pts.append(pt_r)
        
        return np.array(left_pts), np.array(right_pts), valid_matches
    
    def match_temporal(self, kp_prev: List, desc_prev: np.ndarray,
                       kp_curr: List, desc_curr: np.ndarray,
                       max_distance: float = 100) -> Tuple[np.ndarray, np.ndarray, List]:
        """
        시간적 매칭 (프레임 간)
        
        Returns:
        - prev_pts: 이전 프레임 점들
        - curr_pts: 현재 프레임 점들
        - matches: 매칭
        """
        
        matches = self.match(desc_prev, desc_curr)
        
        valid_matches = []
        prev_pts = []
        curr_pts = []
        
        for m in matches:
            pt_prev = kp_prev[m.queryIdx].pt
            pt_curr = kp_curr[m.trainIdx].pt
            
            # 거리 제한
            dist = np.sqrt((pt_prev[0] - pt_curr[0])**2 + 
                          (pt_prev[1] - pt_curr[1])**2)
            
            if dist < max_distance:
                valid_matches.append(m)
                prev_pts.append(pt_prev)
                curr_pts.append(pt_curr)
        
        return np.array(prev_pts), np.array(curr_pts), valid_matches
```

---

## 4. 스테레오 3D 재구성

### 4.1 삼각측량

```python
"""
stereo_triangulation.py
스테레오 삼각측량
"""

import cv2
import numpy as np
from typing import Tuple, Optional
import yaml


class StereoTriangulator:
    """스테레오 삼각측량"""
    
    def __init__(self, calibration_file: str):
        """캘리브레이션 로드"""
        
        with open(calibration_file, 'r') as f:
            params = yaml.safe_load(f)
        
        self.P1 = np.array(params['P1'])  # 왼쪽 투영 행렬
        self.P2 = np.array(params['P2'])  # 오른쪽 투영 행렬
        self.Q = np.array(params['Q'])    # 시차→3D 변환 행렬
        
        self.baseline = params['baseline_mm']
        self.fx = self.P1[0, 0]
        self.fy = self.P1[1, 1]
        self.cx = self.P1[0, 2]
        self.cy = self.P1[1, 2]
    
    def triangulate(self, left_pts: np.ndarray, 
                    right_pts: np.ndarray) -> np.ndarray:
        """
        2D 점들을 3D로 삼각측량
        
        Parameters:
        - left_pts: 왼쪽 이미지 점들 [N, 2]
        - right_pts: 오른쪽 이미지 점들 [N, 2]
        
        Returns:
        - points_3d: 3D 점들 [N, 3] (mm 단위)
        """
        
        if len(left_pts) == 0:
            return np.array([])
        
        # OpenCV triangulatePoints는 [2, N] 형태 필요
        left_pts_T = left_pts.T.astype(np.float64)
        right_pts_T = right_pts.T.astype(np.float64)
        
        # 동차 좌표로 삼각측량
        points_4d = cv2.triangulatePoints(
            self.P1, self.P2,
            left_pts_T, right_pts_T
        )
        
        # 동차 좌표 → 유클리드 좌표
        points_3d = points_4d[:3] / points_4d[3]
        points_3d = points_3d.T  # [N, 3]
        
        return points_3d
    
    def triangulate_from_disparity(self, left_pts: np.ndarray,
                                    disparities: np.ndarray) -> np.ndarray:
        """
        시차를 이용한 삼각측량
        
        Z = f * B / d
        X = (u - cx) * Z / fx
        Y = (v - cy) * Z / fy
        """
        
        if len(left_pts) == 0:
            return np.array([])
        
        # 유효한 시차만
        valid = disparities > 0
        
        u = left_pts[valid, 0]
        v = left_pts[valid, 1]
        d = disparities[valid]
        
        Z = self.fx * self.baseline / d
        X = (u - self.cx) * Z / self.fx
        Y = (v - self.cy) * Z / self.fy
        
        points_3d = np.column_stack([X, Y, Z])
        
        return points_3d
    
    def reproject_to_image(self, points_3d: np.ndarray,
                           camera: str = 'left') -> np.ndarray:
        """
        3D 점을 이미지로 재투영
        
        Parameters:
        - points_3d: 3D 점들 [N, 3]
        - camera: 'left' 또는 'right'
        
        Returns:
        - points_2d: 2D 점들 [N, 2]
        """
        
        P = self.P1 if camera == 'left' else self.P2
        
        # 동차 좌표
        points_h = np.hstack([points_3d, np.ones((len(points_3d), 1))])
        
        # 투영
        projected = (P @ points_h.T).T
        
        # 정규화
        points_2d = projected[:, :2] / projected[:, 2:3]
        
        return points_2d
```

---

## 5. 모션 추정

### 5.1 PnP 기반 모션 추정

```python
"""
motion_estimator.py
카메라 모션 추정
"""

import cv2
import numpy as np
from typing import Tuple, Optional
import yaml


class MotionEstimator:
    """카메라 모션 추정"""
    
    def __init__(self, calibration_file: str):
        """캘리브레이션 로드"""
        
        with open(calibration_file, 'r') as f:
            params = yaml.safe_load(f)
        
        self.K = np.array(params['K1'])  # 내부 파라미터
        self.dist = np.array(params['D1'])  # 왜곡 계수
        
        self.fx = self.K[0, 0]
        self.fy = self.K[1, 1]
        self.cx = self.K[0, 2]
        self.cy = self.K[1, 2]
    
    def estimate_motion_pnp(self, points_3d: np.ndarray,
                            points_2d: np.ndarray,
                            method: str = 'RANSAC') -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        PnP로 모션 추정
        
        이전 프레임의 3D 점들과 현재 프레임의 2D 투영을 이용
        
        Parameters:
        - points_3d: 이전 프레임 3D 점들 [N, 3]
        - points_2d: 현재 프레임 2D 점들 [N, 2]
        
        Returns:
        - R: 회전 행렬 [3, 3]
        - t: 이동 벡터 [3, 1]
        - inliers: 인라이어 인덱스
        """
        
        if len(points_3d) < 4:
            return None, None, None
        
        # PnP 방법 선택
        if method == 'RANSAC':
            success, rvec, tvec, inliers = cv2.solvePnPRansac(
                points_3d.astype(np.float64),
                points_2d.astype(np.float64),
                self.K,
                self.dist,
                iterationsCount=1000,
                reprojectionError=2.0,
                confidence=0.99,
                flags=cv2.SOLVEPNP_ITERATIVE
            )
        else:
            success, rvec, tvec = cv2.solvePnP(
                points_3d.astype(np.float64),
                points_2d.astype(np.float64),
                self.K,
                self.dist,
                flags=cv2.SOLVEPNP_ITERATIVE
            )
            inliers = np.arange(len(points_3d))
        
        if not success:
            return None, None, None
        
        # 회전 벡터 → 회전 행렬
        R, _ = cv2.Rodrigues(rvec)
        t = tvec
        
        return R, t, inliers.flatten() if inliers is not None else None
    
    def estimate_motion_essential(self, pts_prev: np.ndarray,
                                   pts_curr: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Essential Matrix로 모션 추정
        
        모노큘러 방식 (스케일 모름)
        
        Returns:
        - R: 회전 행렬
        - t: 이동 벡터 (단위 벡터)
        - mask: 인라이어 마스크
        """
        
        if len(pts_prev) < 5:
            return None, None, None
        
        # Essential Matrix 추정
        E, mask = cv2.findEssentialMat(
            pts_prev, pts_curr,
            self.K,
            method=cv2.RANSAC,
            prob=0.999,
            threshold=1.0
        )
        
        if E is None:
            return None, None, None
        
        # 모션 복원
        _, R, t, mask_pose = cv2.recoverPose(
            E, pts_prev, pts_curr, self.K, mask=mask
        )
        
        return R, t, mask.flatten()
    
    def estimate_motion_3d_3d(self, points_3d_prev: np.ndarray,
                              points_3d_curr: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        3D-3D 매칭으로 모션 추정 (ICP 스타일)
        
        Returns:
        - R: 회전 행렬
        - t: 이동 벡터
        """
        
        if len(points_3d_prev) < 3:
            return None, None
        
        # 중심점
        centroid_prev = np.mean(points_3d_prev, axis=0)
        centroid_curr = np.mean(points_3d_curr, axis=0)
        
        # 중심 이동
        prev_centered = points_3d_prev - centroid_prev
        curr_centered = points_3d_curr - centroid_curr
        
        # SVD로 회전 계산
        H = prev_centered.T @ curr_centered
        U, S, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        
        # 반사 체크
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = Vt.T @ U.T
        
        # 이동 계산
        t = centroid_curr - R @ centroid_prev
        
        return R, t.reshape(3, 1)
    
    def compose_transformation(self, R: np.ndarray, 
                               t: np.ndarray) -> np.ndarray:
        """
        회전/이동을 4x4 변환 행렬로 결합
        
        Returns:
        - T: [4, 4] 변환 행렬
        """
        
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = t.flatten()
        
        return T
    
    def decompose_transformation(self, T: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        4x4 변환 행렬을 회전/이동으로 분해
        """
        
        R = T[:3, :3]
        t = T[:3, 3].reshape(3, 1)
        
        return R, t
```

### 5.2 RANSAC 기반 강건 추정

```python
"""
robust_estimator.py
강건한 모션 추정
"""

import numpy as np
from typing import Tuple, Optional, Callable


class RANSACEstimator:
    """RANSAC 기반 강건 추정"""
    
    def __init__(self, min_samples: int = 4,
                 residual_threshold: float = 2.0,
                 max_trials: int = 1000,
                 confidence: float = 0.99):
        """
        Parameters:
        - min_samples: 모델 추정에 필요한 최소 샘플 수
        - residual_threshold: 인라이어 판정 임계값
        - max_trials: 최대 반복 횟수
        - confidence: 신뢰도
        """
        
        self.min_samples = min_samples
        self.residual_threshold = residual_threshold
        self.max_trials = max_trials
        self.confidence = confidence
    
    def estimate(self, data: np.ndarray,
                 model_func: Callable,
                 residual_func: Callable) -> Tuple[any, np.ndarray]:
        """
        RANSAC 추정
        
        Parameters:
        - data: 데이터 [N, ...]
        - model_func: 모델 추정 함수 (samples -> model)
        - residual_func: 잔차 계산 함수 (model, data -> residuals)
        
        Returns:
        - best_model: 최적 모델
        - inliers: 인라이어 마스크
        """
        
        n_samples = len(data)
        best_model = None
        best_inliers = np.zeros(n_samples, dtype=bool)
        best_n_inliers = 0
        
        # 동적 반복 횟수 계산
        n_trials = self.max_trials
        trial = 0
        
        while trial < n_trials:
            # 랜덤 샘플 선택
            indices = np.random.choice(n_samples, self.min_samples, replace=False)
            samples = data[indices]
            
            # 모델 추정
            try:
                model = model_func(samples)
            except:
                trial += 1
                continue
            
            if model is None:
                trial += 1
                continue
            
            # 잔차 계산
            residuals = residual_func(model, data)
            
            # 인라이어 판정
            inliers = np.abs(residuals) < self.residual_threshold
            n_inliers = np.sum(inliers)
            
            # 최적 모델 업데이트
            if n_inliers > best_n_inliers:
                best_n_inliers = n_inliers
                best_model = model
                best_inliers = inliers
                
                # 반복 횟수 재계산
                inlier_ratio = n_inliers / n_samples
                if inlier_ratio > 0:
                    n_trials = int(np.log(1 - self.confidence) / 
                                  np.log(1 - inlier_ratio ** self.min_samples))
                    n_trials = min(n_trials, self.max_trials)
            
            trial += 1
        
        # 인라이어로 모델 재추정 (선택)
        if best_n_inliers >= self.min_samples:
            try:
                best_model = model_func(data[best_inliers])
            except:
                pass
        
        return best_model, best_inliers
```

---

## 6. 궤적 추적

### 6.1 포즈 누적 및 궤적

```python
"""
trajectory_tracker.py
카메라 궤적 추적
"""

import numpy as np
from typing import List, Tuple, Optional
from dataclasses import dataclass, field
import json


@dataclass
class Pose:
    """카메라 포즈"""
    timestamp: float
    position: np.ndarray      # [x, y, z]
    rotation: np.ndarray      # [3, 3] 회전 행렬
    transformation: np.ndarray = None  # [4, 4] 전체 변환
    
    def __post_init__(self):
        if self.transformation is None:
            self.transformation = np.eye(4)
            self.transformation[:3, :3] = self.rotation
            self.transformation[:3, 3] = self.position


class TrajectoryTracker:
    """궤적 추적기"""
    
    def __init__(self):
        self.poses: List[Pose] = []
        
        # 현재 글로벌 포즈
        self.current_pose = np.eye(4)
        
        # 프레임 카운터
        self.frame_count = 0
    
    def update(self, R: np.ndarray, t: np.ndarray, 
               timestamp: float = None) -> Pose:
        """
        새로운 상대 모션으로 포즈 업데이트
        
        Parameters:
        - R: 상대 회전 [3, 3]
        - t: 상대 이동 [3, 1]
        - timestamp: 타임스탬프
        
        Returns:
        - pose: 새로운 글로벌 포즈
        """
        
        if timestamp is None:
            timestamp = self.frame_count
        
        # 상대 변환 행렬
        T_rel = np.eye(4)
        T_rel[:3, :3] = R
        T_rel[:3, 3] = t.flatten()
        
        # 글로벌 포즈 누적
        # T_global = T_global * T_rel
        self.current_pose = self.current_pose @ T_rel
        
        # 포즈 저장
        pose = Pose(
            timestamp=timestamp,
            position=self.current_pose[:3, 3].copy(),
            rotation=self.current_pose[:3, :3].copy(),
            transformation=self.current_pose.copy()
        )
        
        self.poses.append(pose)
        self.frame_count += 1
        
        return pose
    
    def get_trajectory(self) -> np.ndarray:
        """
        전체 궤적 반환
        
        Returns:
        - trajectory: [N, 3] 위치 배열
        """
        
        if len(self.poses) == 0:
            return np.array([])
        
        return np.array([p.position for p in self.poses])
    
    def get_poses(self) -> List[np.ndarray]:
        """모든 포즈 변환 행렬 반환"""
        return [p.transformation for p in self.poses]
    
    def get_current_position(self) -> np.ndarray:
        """현재 위치"""
        return self.current_pose[:3, 3]
    
    def get_current_rotation(self) -> np.ndarray:
        """현재 회전"""
        return self.current_pose[:3, :3]
    
    def get_total_distance(self) -> float:
        """총 이동 거리"""
        
        trajectory = self.get_trajectory()
        
        if len(trajectory) < 2:
            return 0.0
        
        distances = np.linalg.norm(np.diff(trajectory, axis=0), axis=1)
        
        return np.sum(distances)
    
    def reset(self):
        """리셋"""
        self.poses.clear()
        self.current_pose = np.eye(4)
        self.frame_count = 0
    
    def save(self, filename: str):
        """궤적 저장"""
        
        data = {
            'num_poses': len(self.poses),
            'trajectory': self.get_trajectory().tolist(),
            'poses': [
                {
                    'timestamp': p.timestamp,
                    'position': p.position.tolist(),
                    'rotation': p.rotation.tolist()
                }
                for p in self.poses
            ]
        }
        
        with open(filename, 'w') as f:
            json.dump(data, f, indent=2)
        
        print(f"✅ 궤적 저장: {filename}")
    
    def load(self, filename: str):
        """궤적 로드"""
        
        with open(filename, 'r') as f:
            data = json.load(f)
        
        self.reset()
        
        for pose_data in data['poses']:
            pose = Pose(
                timestamp=pose_data['timestamp'],
                position=np.array(pose_data['position']),
                rotation=np.array(pose_data['rotation'])
            )
            self.poses.append(pose)
        
        if len(self.poses) > 0:
            self.current_pose = self.poses[-1].transformation
            self.frame_count = len(self.poses)
        
        print(f"✅ 궤적 로드: {len(self.poses)} poses")
```

---

## 7. 최적화 및 드리프트 보정

### 7.1 로컬 번들 조정

```python
"""
bundle_adjustment.py
번들 조정 (최적화)
"""

import numpy as np
from scipy.optimize import least_squares
from typing import List, Tuple, Optional


class LocalBundleAdjustment:
    """로컬 번들 조정"""
    
    def __init__(self, window_size: int = 5):
        """
        Parameters:
        - window_size: 최적화할 최근 프레임 수
        """
        
        self.window_size = window_size
    
    def optimize(self, poses: List[np.ndarray],
                 points_3d: List[np.ndarray],
                 observations: List[List[np.ndarray]],
                 K: np.ndarray) -> Tuple[List[np.ndarray], np.ndarray]:
        """
        로컬 번들 조정
        
        Parameters:
        - poses: 카메라 포즈 리스트 [4x4]
        - points_3d: 3D 점들
        - observations: 각 프레임에서의 2D 관측 [[frame_idx, pt_idx, u, v], ...]
        - K: 내부 파라미터
        
        Returns:
        - optimized_poses: 최적화된 포즈
        - optimized_points: 최적화된 3D 점
        """
        
        # 간소화된 구현
        # 실제로는 g2o, GTSAM 등 사용 권장
        
        n_poses = len(poses)
        n_points = len(points_3d)
        
        if n_poses < 2 or n_points < 4:
            return poses, np.array(points_3d)
        
        # 파라미터 벡터 구성
        # [pose_0, pose_1, ..., point_0, point_1, ...]
        
        def residual_func(params):
            """잔차 계산"""
            residuals = []
            
            # 파라미터 분해
            # ... (생략)
            
            return np.array(residuals)
        
        # 최적화
        # ... (생략)
        
        return poses, np.array(points_3d)


class LoopClosureDetector:
    """루프 클로저 검출"""
    
    def __init__(self, min_interval: int = 50,
                 similarity_threshold: float = 0.8):
        """
        Parameters:
        - min_interval: 루프 검출 최소 프레임 간격
        - similarity_threshold: 유사도 임계값
        """
        
        self.min_interval = min_interval
        self.similarity_threshold = similarity_threshold
        
        # 키프레임 디스크립터
        self.keyframe_descriptors = []
        self.keyframe_indices = []
    
    def add_keyframe(self, frame_idx: int, descriptors: np.ndarray):
        """키프레임 추가"""
        
        # BoW (Bag of Words) 벡터 계산
        # 간단히 평균 디스크립터 사용
        if descriptors is not None and len(descriptors) > 0:
            bow = np.mean(descriptors.astype(np.float32), axis=0)
            self.keyframe_descriptors.append(bow)
            self.keyframe_indices.append(frame_idx)
    
    def detect_loop(self, frame_idx: int, 
                    descriptors: np.ndarray) -> Optional[int]:
        """
        루프 클로저 검출
        
        Returns:
        - matched_idx: 매칭된 키프레임 인덱스 또는 None
        """
        
        if len(self.keyframe_descriptors) == 0:
            return None
        
        if descriptors is None or len(descriptors) == 0:
            return None
        
        # 현재 BoW
        current_bow = np.mean(descriptors.astype(np.float32), axis=0)
        
        best_similarity = 0
        best_idx = None
        
        for i, (kf_bow, kf_idx) in enumerate(
            zip(self.keyframe_descriptors, self.keyframe_indices)
        ):
            # 최소 간격 체크
            if frame_idx - kf_idx < self.min_interval:
                continue
            
            # 코사인 유사도
            similarity = np.dot(current_bow, kf_bow) / (
                np.linalg.norm(current_bow) * np.linalg.norm(kf_bow) + 1e-6
            )
            
            if similarity > best_similarity:
                best_similarity = similarity
                best_idx = kf_idx
        
        if best_similarity > self.similarity_threshold:
            print(f"🔄 루프 클로저 검출: frame {frame_idx} → {best_idx}")
            return best_idx
        
        return None
```

---

## 8. 시각화

### 8.1 궤적 시각화

```python
"""
visualizer.py
Visual Odometry 시각화
"""

import cv2
import numpy as np
from typing import List, Optional, Tuple
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class VOVisualizer:
    """Visual Odometry 시각화"""
    
    def __init__(self, window_name: str = "Visual Odometry"):
        self.window_name = window_name
        
        # 2D 궤적 캔버스
        self.trajectory_canvas = None
        self.canvas_size = (800, 800)
        self.scale = 1.0  # mm → pixel
        self.offset = np.array([400, 700])  # 캔버스 중심
        
        # 3D 궤적 (matplotlib)
        self.fig_3d = None
        self.ax_3d = None
        
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    
    def initialize_canvas(self):
        """2D 캔버스 초기화"""
        self.trajectory_canvas = np.zeros(
            (self.canvas_size[1], self.canvas_size[0], 3), dtype=np.uint8
        )
        
        # 그리드 그리기
        for i in range(0, self.canvas_size[0], 100):
            cv2.line(self.trajectory_canvas, (i, 0), (i, self.canvas_size[1]),
                    (30, 30, 30), 1)
        for i in range(0, self.canvas_size[1], 100):
            cv2.line(self.trajectory_canvas, (0, i), (self.canvas_size[0], i),
                    (30, 30, 30), 1)
        
        # 원점
        cv2.circle(self.trajectory_canvas, 
                  tuple(self.offset.astype(int)), 5, (0, 0, 255), -1)
    
    def world_to_canvas(self, position: np.ndarray) -> Tuple[int, int]:
        """월드 좌표 → 캔버스 좌표"""
        
        # X-Z 평면 투영 (Y는 위쪽)
        x = position[0] * self.scale + self.offset[0]
        y = -position[2] * self.scale + self.offset[1]  # Z가 앞쪽
        
        return int(x), int(y)
    
    def draw_trajectory_2d(self, trajectory: np.ndarray,
                           current_pose: np.ndarray = None) -> np.ndarray:
        """2D 궤적 그리기 (탑뷰)"""
        
        if self.trajectory_canvas is None:
            self.initialize_canvas()
        
        canvas = self.trajectory_canvas.copy()
        
        if len(trajectory) < 2:
            return canvas
        
        # 스케일 자동 조정
        max_range = np.max(np.abs(trajectory[:, [0, 2]]))
        if max_range > 0:
            self.scale = min(300 / max_range, 2.0)
        
        # 궤적 그리기
        for i in range(1, len(trajectory)):
            pt1 = self.world_to_canvas(trajectory[i-1])
            pt2 = self.world_to_canvas(trajectory[i])
            
            cv2.line(canvas, pt1, pt2, (0, 255, 0), 2)
        
        # 현재 위치
        if len(trajectory) > 0:
            current_pt = self.world_to_canvas(trajectory[-1])
            cv2.circle(canvas, current_pt, 8, (0, 255, 255), -1)
            
            # 방향 표시
            if current_pose is not None:
                direction = current_pose[:3, :3] @ np.array([0, 0, 50])
                end_pt = self.world_to_canvas(trajectory[-1] + direction)
                cv2.arrowedLine(canvas, current_pt, end_pt, (0, 255, 255), 2)
        
        # 정보 표시
        total_dist = np.sum(np.linalg.norm(np.diff(trajectory, axis=0), axis=1))
        cv2.putText(canvas, f"Distance: {total_dist/1000:.2f}m", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(canvas, f"Frames: {len(trajectory)}", (10, 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        if len(trajectory) > 0:
            pos = trajectory[-1]
            cv2.putText(canvas, f"X: {pos[0]/1000:.2f}m", (10, 90),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
            cv2.putText(canvas, f"Y: {pos[1]/1000:.2f}m", (10, 115),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
            cv2.putText(canvas, f"Z: {pos[2]/1000:.2f}m", (10, 140),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        
        return canvas
    
    def draw_feature_matches(self, image: np.ndarray,
                             prev_pts: np.ndarray,
                             curr_pts: np.ndarray,
                             inliers: np.ndarray = None) -> np.ndarray:
        """특징점 매칭 시각화"""
        
        display = image.copy()
        
        if prev_pts is None or curr_pts is None:
            return display
        
        for i in range(len(prev_pts)):
            pt1 = tuple(prev_pts[i].astype(int))
            pt2 = tuple(curr_pts[i].astype(int))
            
            color = (0, 255, 0) if (inliers is None or inliers[i]) else (0, 0, 255)
            
            cv2.circle(display, pt2, 3, color, -1)
            cv2.line(display, pt1, pt2, color, 1)
        
        return display
    
    def draw_combined(self, image: np.ndarray,
                      trajectory: np.ndarray,
                      current_pose: np.ndarray,
                      prev_pts: np.ndarray = None,
                      curr_pts: np.ndarray = None) -> np.ndarray:
        """통합 시각화"""
        
        h, w = image.shape[:2]
        
        # 특징점 매칭 그리기
        image_display = self.draw_feature_matches(image, prev_pts, curr_pts)
        
        # 2D 궤적
        traj_display = self.draw_trajectory_2d(trajectory, current_pose)
        traj_display = cv2.resize(traj_display, (w // 2, h // 2))
        
        # 결합
        combined = np.zeros((h, w + w // 2, 3), dtype=np.uint8)
        combined[:, :w] = image_display
        combined[:h//2, w:] = traj_display
        
        return combined
    
    def show(self, image: np.ndarray):
        """이미지 표시"""
        cv2.imshow(self.window_name, image)
    
    def wait_key(self, delay: int = 1) -> int:
        """키 입력"""
        return cv2.waitKey(delay) & 0xFF
    
    def plot_3d_trajectory(self, trajectory: np.ndarray,
                           show: bool = True):
        """3D 궤적 플롯 (matplotlib)"""
        
        if len(trajectory) == 0:
            return
        
        if self.fig_3d is None:
            self.fig_3d = plt.figure(figsize=(10, 8))
            self.ax_3d = self.fig_3d.add_subplot(111, projection='3d')
        
        self.ax_3d.clear()
        
        # 궤적
        self.ax_3d.plot(trajectory[:, 0], trajectory[:, 2], trajectory[:, 1],
                       'b-', linewidth=2, label='Trajectory')
        
        # 시작점
        self.ax_3d.scatter(trajectory[0, 0], trajectory[0, 2], trajectory[0, 1],
                          c='g', s=100, marker='o', label='Start')
        
        # 끝점
        self.ax_3d.scatter(trajectory[-1, 0], trajectory[-1, 2], trajectory[-1, 1],
                          c='r', s=100, marker='x', label='End')
        
        self.ax_3d.set_xlabel('X (mm)')
        self.ax_3d.set_ylabel('Z (mm)')
        self.ax_3d.set_zlabel('Y (mm)')
        self.ax_3d.legend()
        self.ax_3d.set_title('Camera Trajectory')
        
        # 축 비율 동일하게
        max_range = np.max(np.abs(trajectory))
        self.ax_3d.set_xlim([-max_range, max_range])
        self.ax_3d.set_ylim([0, max_range * 2])
        self.ax_3d.set_zlim([-max_range, max_range])
        
        if show:
            plt.show()
        else:
            plt.savefig('trajectory_3d.png')
    
    def close(self):
        """종료"""
        cv2.destroyAllWindows()
        if self.fig_3d:
            plt.close(self.fig_3d)
```

---

## 9. 전체 코드

### 9.1 Visual Odometry 메인 클래스

```python
"""
visual_odometry.py
Stereo Visual Odometry 메인 클래스
"""

import cv2
import numpy as np
from typing import Optional, Tuple
import yaml
import time

from feature_detector import FeatureDetector, FeatureTracker, DetectorType
from feature_matcher import FeatureMatcher
from stereo_triangulation import StereoTriangulator
from motion_estimator import MotionEstimator
from trajectory_tracker import TrajectoryTracker


class StereoVisualOdometry:
    """Stereo Visual Odometry"""
    
    def __init__(self, calibration_file: str, config: dict = None):
        """
        Parameters:
        - calibration_file: 캘리브레이션 파일
        - config: 설정 딕셔너리
        """
        
        self.config = config or {}
        
        # 컴포넌트 초기화
        self.detector = FeatureDetector(
            detector_type=DetectorType.ORB,
            max_features=self.config.get('max_features', 2000)
        )
        
        self.tracker = FeatureTracker(
            max_features=self.config.get('max_features', 2000)
        )
        
        self.matcher = FeatureMatcher(
            matcher_type='BF',
            cross_check=False,
            ratio_threshold=0.75
        )
        
        self.triangulator = StereoTriangulator(calibration_file)
        self.motion_estimator = MotionEstimator(calibration_file)
        self.trajectory = TrajectoryTracker()
        
        # 이전 프레임 데이터
        self.prev_left = None
        self.prev_kp = None
        self.prev_desc = None
        self.prev_points_3d = None
        
        # 상태
        self.frame_count = 0
        self.is_initialized = False
        
        # 통계
        self.processing_times = []
    
    def process_frame(self, left: np.ndarray, 
                      right: np.ndarray) -> Tuple[Optional[np.ndarray], dict]:
        """
        프레임 처리
        
        Parameters:
        - left: 정류된 왼쪽 이미지
        - right: 정류된 오른쪽 이미지
        
        Returns:
        - pose: 현재 포즈 [4, 4] 또는 None
        - info: 처리 정보 딕셔너리
        """
        
        start_time = time.time()
        
        info = {
            'frame_id': self.frame_count,
            'num_features': 0,
            'num_matches': 0,
            'num_inliers': 0,
            'tracking_quality': 'unknown'
        }
        
        # 그레이스케일
        gray_left = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY) if len(left.shape) == 3 else left
        gray_right = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY) if len(right.shape) == 3 else right
        
        # 1. 특징점 검출
        curr_kp, curr_desc = self.detector.detect(gray_left)
        info['num_features'] = len(curr_kp)
        
        if len(curr_kp) < 10:
            info['tracking_quality'] = 'poor'
            return None, info
        
        # 2. 스테레오 매칭 → 3D 점
        stereo_data = self.detector.detect_and_compute_stereo(gray_left, gray_right)
        
        left_pts, right_pts, stereo_matches = self.matcher.match_stereo(
            stereo_data['left_kp'], stereo_data['left_desc'],
            stereo_data['right_kp'], stereo_data['right_desc']
        )
        
        if len(left_pts) < 10:
            info['tracking_quality'] = 'poor'
            return None, info
        
        # 3D 삼각측량
        curr_points_3d = self.triangulator.triangulate(left_pts, right_pts)
        
        # 첫 프레임
        if not self.is_initialized:
            self.prev_left = gray_left
            self.prev_kp = curr_kp
            self.prev_desc = curr_desc
            self.prev_points_3d = curr_points_3d
            self.prev_left_pts = left_pts
            self.is_initialized = True
            
            # 초기 포즈 저장
            self.trajectory.update(np.eye(3), np.zeros((3, 1)), self.frame_count)
            
            self.frame_count += 1
            info['tracking_quality'] = 'initialized'
            
            return self.trajectory.current_pose, info
        
        # 3. 시간적 매칭
        prev_matched, curr_matched, temporal_matches = self.matcher.match_temporal(
            self.prev_kp, self.prev_desc,
            curr_kp, curr_desc
        )
        
        info['num_matches'] = len(temporal_matches)
        
        if len(temporal_matches) < 10:
            info['tracking_quality'] = 'lost'
            # 재초기화
            self.prev_left = gray_left
            self.prev_kp = curr_kp
            self.prev_desc = curr_desc
            self.prev_points_3d = curr_points_3d
            return None, info
        
        # 4. 매칭된 3D 점 찾기
        # 이전 프레임의 매칭된 키포인트 → 3D 점 찾기
        matched_3d = []
        matched_2d = []
        
        for match in temporal_matches:
            prev_pt = self.prev_kp[match.queryIdx].pt
            
            # 이전 프레임의 스테레오 매칭된 점에서 가장 가까운 점 찾기
            min_dist = float('inf')
            closest_3d = None
            
            for i, pt_2d in enumerate(self.prev_left_pts):
                dist = np.linalg.norm(np.array(prev_pt) - pt_2d)
                if dist < min_dist and dist < 5:  # 5픽셀 이내
                    min_dist = dist
                    closest_3d = self.prev_points_3d[i]
            
            if closest_3d is not None:
                matched_3d.append(closest_3d)
                matched_2d.append(curr_kp[match.trainIdx].pt)
        
        matched_3d = np.array(matched_3d)
        matched_2d = np.array(matched_2d)
        
        if len(matched_3d) < 6:
            info['tracking_quality'] = 'insufficient'
            return None, info
        
        # 5. PnP로 모션 추정
        R, t, inliers = self.motion_estimator.estimate_motion_pnp(
            matched_3d, matched_2d
        )
        
        if R is None:
            info['tracking_quality'] = 'failed'
            return None, info
        
        info['num_inliers'] = len(inliers) if inliers is not None else 0
        
        # 인라이어 비율로 품질 판단
        inlier_ratio = info['num_inliers'] / len(matched_3d)
        if inlier_ratio > 0.7:
            info['tracking_quality'] = 'good'
        elif inlier_ratio > 0.5:
            info['tracking_quality'] = 'fair'
        else:
            info['tracking_quality'] = 'poor'
        
        # 6. 포즈 업데이트
        # PnP 결과는 카메라→월드 변환이므로 역변환 필요
        R_inv = R.T
        t_inv = -R.T @ t
        
        pose = self.trajectory.update(R_inv, t_inv, self.frame_count)
        
        # 상태 업데이트
        self.prev_left = gray_left
        self.prev_kp = curr_kp
        self.prev_desc = curr_desc
        self.prev_points_3d = curr_points_3d
        self.prev_left_pts = left_pts
        
        self.frame_count += 1
        
        # 처리 시간
        elapsed = time.time() - start_time
        self.processing_times.append(elapsed)
        info['processing_time_ms'] = elapsed * 1000
        
        return self.trajectory.current_pose, info
    
    def get_trajectory(self) -> np.ndarray:
        """궤적 반환"""
        return self.trajectory.get_trajectory()
    
    def get_current_pose(self) -> np.ndarray:
        """현재 포즈 반환"""
        return self.trajectory.current_pose
    
    def get_statistics(self) -> dict:
        """통계 반환"""
        return {
            'total_frames': self.frame_count,
            'total_distance': self.trajectory.get_total_distance(),
            'avg_processing_time_ms': np.mean(self.processing_times) * 1000 if self.processing_times else 0
        }
    
    def reset(self):
        """리셋"""
        self.prev_left = None
        self.prev_kp = None
        self.prev_desc = None
        self.prev_points_3d = None
        self.frame_count = 0
        self.is_initialized = False
        self.trajectory.reset()
        self.processing_times.clear()
```

### 9.2 메인 실행 파일

```python
"""
main.py
Visual Odometry 메인
"""

import argparse
import yaml
import cv2
import numpy as np
import sys

from stereo_camera import StereoCamera
from visual_odometry import StereoVisualOdometry
from visualizer import VOVisualizer


def main():
    parser = argparse.ArgumentParser(description='Stereo Visual Odometry')
    parser.add_argument('--config', default='config/vo_config.yaml')
    parser.add_argument('--save', action='store_true', help='궤적 저장')
    args = parser.parse_args()
    
    # 설정 로드
    with open(args.config, 'r') as f:
        config = yaml.safe_load(f)
    
    print("\n" + "="*60)
    print("📍 Stereo Visual Odometry")
    print("="*60)
    
    # 컴포넌트 초기화
    camera = StereoCamera(
        config['calibration_file'],
        config['camera']['left_id'],
        config['camera']['right_id']
    )
    
    vo = StereoVisualOdometry(
        config['calibration_file'],
        config.get('vo', {})
    )
    
    visualizer = VOVisualizer()
    
    print("\n조작 방법:")
    print("  [R] - 리셋")
    print("  [S] - 궤적 저장")
    print("  [3] - 3D 뷰")
    print("  [Q] - 종료")
    print("="*60 + "\n")
    
    prev_pts = None
    curr_pts = None
    
    while True:
        # 프레임 캡처
        rect_left, rect_right = camera.capture_rectified()
        
        if rect_left is None:
            continue
        
        # VO 처리
        pose, info = vo.process_frame(rect_left, rect_right)
        
        # 궤적
        trajectory = vo.get_trajectory()
        
        # 시각화
        display = visualizer.draw_combined(
            rect_left, trajectory, pose, prev_pts, curr_pts
        )
        
        # 정보 표시
        quality = info.get('tracking_quality', 'unknown')
        color = (0, 255, 0) if quality == 'good' else (0, 255, 255) if quality == 'fair' else (0, 0, 255)
        
        cv2.putText(display, f"Quality: {quality}", (10, display.shape[0] - 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        cv2.putText(display, f"Features: {info.get('num_features', 0)}", (10, display.shape[0] - 35),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        cv2.putText(display, f"Inliers: {info.get('num_inliers', 0)}", (10, display.shape[0] - 15),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        
        visualizer.show(display)
        
        # 키 처리
        key = visualizer.wait_key(1)
        
        if key == ord('q'):
            break
        elif key == ord('r'):
            vo.reset()
            visualizer.initialize_canvas()
            print("🔄 리셋")
        elif key == ord('s'):
            vo.trajectory.save('output/trajectory.json')
        elif key == ord('3'):
            visualizer.plot_3d_trajectory(trajectory)
    
    # 종료
    stats = vo.get_statistics()
    print("\n" + "="*60)
    print("📊 결과 요약")
    print(f"  총 프레임: {stats['total_frames']}")
    print(f"  총 이동거리: {stats['total_distance']/1000:.2f} m")
    print(f"  평균 처리시간: {stats['avg_processing_time_ms']:.1f} ms")
    print("="*60)
    
    if args.save:
        vo.trajectory.save('output/trajectory.json')
    
    camera.release()
    visualizer.close()


if __name__ == "__main__":
    main()
```

### 9.3 설정 파일

```yaml
# config/vo_config.yaml

calibration_file: "config/stereo_params.yaml"

camera:
  left_id: 0
  right_id: 2
  width: 1280
  height: 720

vo:
  max_features: 2000
  detector: "ORB"         # ORB, SIFT, FAST
  matcher: "BF"           # BF, FLANN
  min_inliers: 10
  ransac_threshold: 2.0

output:
  save_trajectory: true
  output_dir: "output/trajectories"
```

---

## 10. 성능 평가

### 10.1 평가 메트릭

```python
"""
evaluation.py
VO 성능 평가
"""

import numpy as np
from typing import List, Tuple


def compute_ate(estimated: np.ndarray, 
                ground_truth: np.ndarray) -> Tuple[float, float]:
    """
    ATE (Absolute Trajectory Error) 계산
    
    Returns:
    - rmse: RMSE
    - mean: 평균 오차
    """
    
    errors = np.linalg.norm(estimated - ground_truth, axis=1)
    
    rmse = np.sqrt(np.mean(errors ** 2))
    mean_error = np.mean(errors)
    
    return rmse, mean_error


def compute_rpe(estimated: np.ndarray,
                ground_truth: np.ndarray,
                delta: int = 1) -> Tuple[float, float]:
    """
    RPE (Relative Pose Error) 계산
    
    Parameters:
    - delta: 프레임 간격
    
    Returns:
    - trans_error: 이동 오차
    - rot_error: 회전 오차 (도)
    """
    
    trans_errors = []
    
    for i in range(len(estimated) - delta):
        est_rel = estimated[i + delta] - estimated[i]
        gt_rel = ground_truth[i + delta] - ground_truth[i]
        
        error = np.linalg.norm(est_rel - gt_rel)
        trans_errors.append(error)
    
    return np.mean(trans_errors), np.std(trans_errors)
```

---

## 📝 학습 체크리스트

### 이론 이해

- [ ] Visual Odometry의 원리를 이해했다
- [ ] 스테레오 삼각측량을 설명할 수 있다
- [ ] PnP 문제와 해법을 알고 있다
- [ ] RANSAC의 역할을 이해했다

### 구현 완료

- [ ] 특징점 검출/매칭
- [ ] 스테레오 3D 재구성
- [ ] PnP 기반 모션 추정
- [ ] 궤적 누적 및 추적
- [ ] 시각화 (2D/3D)

---

## 📄 라이선스

MIT License - 자유롭게 사용, 수정, 배포 가능

---

## 🎉 스테레오 비전 교육 커리큘럼 완료!

축하합니다! 11개의 모듈과 4개의 프로젝트로 구성된 스테레오 비전 교육 커리큘럼을 완료했습니다.

### 커리큘럼 요약

| 구분 | 항목 | 상태 |
|------|------|------|
| **이론** | Module 01-07 | ✅ 완료 |
| **프로젝트** | Project 01-04 | ✅ 완료 |

이 커리큘럼을 통해 스테레오 비전의 기초부터 고급 응용까지 학습할 수 있습니다!
