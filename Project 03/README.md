# Project 03: 3D 스캐너

[![난이도](https://img.shields.io/badge/난이도-⭐⭐⭐⭐⭐_전문가-purple.svg)]()
[![예상시간](https://img.shields.io/badge/예상시간-15--20시간-blue.svg)]()
[![선수지식](https://img.shields.io/badge/선수지식-Module_04,_Open3D-orange.svg)]()

---

## 🎯 프로젝트 개요

| 항목 | 내용 |
|------|------|
| **목표** | 스테레오 카메라로 물체를 다양한 각도에서 스캔하여 3D 모델 생성 |
| **기능** | 다중 시점 캡처, 포인트 클라우드 정합, 메쉬 생성, 파일 출력 |
| **출력** | PLY, OBJ, STL 파일 (3D 프린팅 가능) |

---

## 📋 목차

1. [프로젝트 구조](#1-프로젝트-구조)
2. [시스템 아키텍처](#2-시스템-아키텍처)
3. [다중 시점 캡처](#3-다중-시점-캡처)
4. [포인트 클라우드 생성](#4-포인트-클라우드-생성)
5. [포인트 클라우드 정합](#5-포인트-클라우드-정합)
6. [후처리 및 필터링](#6-후처리-및-필터링)
7. [메쉬 생성](#7-메쉬-생성)
8. [파일 출력](#8-파일-출력)
9. [GUI 구현](#9-gui-구현)
10. [전체 코드](#10-전체-코드)

---

## 1. 프로젝트 구조

```
Project_03_3D_Scanner/
├── README.md
├── requirements.txt
├── config/
│   ├── stereo_params.yaml      # 캘리브레이션
│   └── scanner_config.yaml     # 스캐너 설정
├── src/
│   ├── __init__.py
│   ├── main.py                 # 메인 실행
│   ├── stereo_camera.py        # 스테레오 카메라
│   ├── pointcloud_generator.py # 포인트 클라우드 생성
│   ├── cloud_registration.py   # 포인트 클라우드 정합
│   ├── cloud_processor.py      # 후처리
│   ├── mesh_generator.py       # 메쉬 생성
│   ├── file_exporter.py        # 파일 출력
│   ├── turntable_controller.py # 턴테이블 제어 (옵션)
│   ├── gui.py                  # GUI
│   └── utils.py                # 유틸리티
├── output/
│   ├── pointclouds/            # 포인트 클라우드
│   ├── meshes/                 # 메쉬 파일
│   └── projects/               # 프로젝트 저장
└── assets/
    └── icons/                  # GUI 아이콘
```

---

## 2. 시스템 아키텍처

```
┌─────────────────────────────────────────────────────────────┐
│                    3D 스캐너 워크플로우                       │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ┌──────────────────────────────────────────────────────┐  │
│  │                  1. 다중 시점 캡처                     │  │
│  │  ┌─────┐   ┌─────┐   ┌─────┐   ┌─────┐   ┌─────┐   │  │
│  │  │ 0°  │   │ 45° │   │ 90° │   │135° │   │180° │...│  │
│  │  └──┬──┘   └──┬──┘   └──┬──┘   └──┬──┘   └──┬──┘   │  │
│  └─────┼────────┼────────┼────────┼────────┼────────┘  │
│        ▼        ▼        ▼        ▼        ▼            │
│  ┌──────────────────────────────────────────────────────┐  │
│  │              2. 개별 포인트 클라우드 생성               │  │
│  │    ┌───┐    ┌───┐    ┌───┐    ┌───┐    ┌───┐       │  │
│  │    │PC1│    │PC2│    │PC3│    │PC4│    │PC5│ ...   │  │
│  │    └─┬─┘    └─┬─┘    └─┬─┘    └─┬─┘    └─┬─┘       │  │
│  └──────┼────────┼────────┼────────┼────────┼────────┘  │
│         └────────┴────────┼────────┴────────┘            │
│                           ▼                              │
│  ┌──────────────────────────────────────────────────────┐  │
│  │              3. 포인트 클라우드 정합 (ICP)             │  │
│  │                                                      │  │
│  │         PC1 ──┐                                      │  │
│  │         PC2 ──┼──→ 정합된 포인트 클라우드              │  │
│  │         PC3 ──┤                                      │  │
│  │         ...   │                                      │  │
│  └───────────────┼──────────────────────────────────────┘  │
│                  ▼                                        │
│  ┌──────────────────────────────────────────────────────┐  │
│  │              4. 후처리                                │  │
│  │    노이즈 제거 → 다운샘플링 → 법선 추정 → 정리         │  │
│  └───────────────┼──────────────────────────────────────┘  │
│                  ▼                                        │
│  ┌──────────────────────────────────────────────────────┐  │
│  │              5. 메쉬 생성                             │  │
│  │    Poisson Reconstruction / Ball Pivoting            │  │
│  └───────────────┼──────────────────────────────────────┘  │
│                  ▼                                        │
│  ┌──────────────────────────────────────────────────────┐  │
│  │              6. 파일 출력                             │  │
│  │         PLY / OBJ / STL / GLTF                       │  │
│  └──────────────────────────────────────────────────────┘  │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

---

## 3. 다중 시점 캡처

### 3.1 스캔 세션 관리

```python
"""
scan_session.py
스캔 세션 관리
"""

import os
import json
import numpy as np
from datetime import datetime
from typing import List, Dict, Optional, Tuple
from dataclasses import dataclass, field, asdict
import cv2


@dataclass
class CaptureFrame:
    """캡처된 프레임"""
    frame_id: int
    angle: float                    # 회전 각도 (도)
    timestamp: str                  # 캡처 시간
    left_image_path: str           # 왼쪽 이미지 경로
    right_image_path: str          # 오른쪽 이미지 경로
    depth_path: Optional[str] = None        # 깊이 맵 경로
    pointcloud_path: Optional[str] = None   # 포인트 클라우드 경로
    transform: Optional[List] = None        # 변환 행렬


@dataclass
class ScanSession:
    """스캔 세션"""
    session_id: str
    name: str
    created_at: str
    num_captures: int = 0
    angle_step: float = 30.0        # 각도 간격
    total_angles: int = 12          # 총 캡처 수 (360/30)
    captures: List[CaptureFrame] = field(default_factory=list)
    merged_cloud_path: Optional[str] = None
    mesh_path: Optional[str] = None
    status: str = "created"         # created, capturing, processing, completed


class ScanSessionManager:
    """스캔 세션 관리자"""
    
    def __init__(self, output_dir: str = "output/projects"):
        self.output_dir = output_dir
        self.current_session: Optional[ScanSession] = None
        
        os.makedirs(output_dir, exist_ok=True)
    
    def create_session(self, name: str, 
                       angle_step: float = 30.0) -> ScanSession:
        """새 스캔 세션 생성"""
        
        session_id = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        session = ScanSession(
            session_id=session_id,
            name=name,
            created_at=datetime.now().isoformat(),
            angle_step=angle_step,
            total_angles=int(360 / angle_step)
        )
        
        # 세션 디렉토리 생성
        session_dir = os.path.join(self.output_dir, session_id)
        os.makedirs(session_dir, exist_ok=True)
        os.makedirs(os.path.join(session_dir, "images"), exist_ok=True)
        os.makedirs(os.path.join(session_dir, "depth"), exist_ok=True)
        os.makedirs(os.path.join(session_dir, "pointclouds"), exist_ok=True)
        
        self.current_session = session
        self._save_session()
        
        print(f"✅ 새 스캔 세션 생성: {session_id}")
        print(f"   이름: {name}")
        print(f"   각도 간격: {angle_step}°")
        print(f"   총 캡처 수: {session.total_angles}")
        
        return session
    
    def load_session(self, session_id: str) -> Optional[ScanSession]:
        """세션 로드"""
        
        session_file = os.path.join(self.output_dir, session_id, "session.json")
        
        if not os.path.exists(session_file):
            print(f"❌ 세션을 찾을 수 없음: {session_id}")
            return None
        
        with open(session_file, 'r') as f:
            data = json.load(f)
        
        # CaptureFrame 복원
        captures = [CaptureFrame(**c) for c in data.get('captures', [])]
        data['captures'] = captures
        
        session = ScanSession(**data)
        self.current_session = session
        
        print(f"✅ 세션 로드: {session_id}")
        
        return session
    
    def _save_session(self):
        """현재 세션 저장"""
        
        if self.current_session is None:
            return
        
        session_dir = os.path.join(self.output_dir, self.current_session.session_id)
        session_file = os.path.join(session_dir, "session.json")
        
        # dataclass를 dict로 변환
        data = asdict(self.current_session)
        
        with open(session_file, 'w') as f:
            json.dump(data, f, indent=2)
    
    def add_capture(self, left_image: np.ndarray, 
                    right_image: np.ndarray,
                    angle: float) -> CaptureFrame:
        """캡처 추가"""
        
        if self.current_session is None:
            raise ValueError("활성 세션이 없습니다.")
        
        session = self.current_session
        session_dir = os.path.join(self.output_dir, session.session_id)
        
        frame_id = session.num_captures
        timestamp = datetime.now().isoformat()
        
        # 이미지 저장
        left_path = os.path.join(session_dir, "images", f"left_{frame_id:03d}.png")
        right_path = os.path.join(session_dir, "images", f"right_{frame_id:03d}.png")
        
        cv2.imwrite(left_path, left_image)
        cv2.imwrite(right_path, right_image)
        
        # 캡처 프레임 생성
        capture = CaptureFrame(
            frame_id=frame_id,
            angle=angle,
            timestamp=timestamp,
            left_image_path=left_path,
            right_image_path=right_path
        )
        
        session.captures.append(capture)
        session.num_captures += 1
        session.status = "capturing"
        
        self._save_session()
        
        print(f"📸 캡처 #{frame_id}: 각도 {angle}°")
        
        return capture
    
    def get_session_dir(self) -> str:
        """현재 세션 디렉토리 반환"""
        if self.current_session:
            return os.path.join(self.output_dir, self.current_session.session_id)
        return ""
    
    def list_sessions(self) -> List[str]:
        """모든 세션 목록"""
        
        sessions = []
        
        for name in os.listdir(self.output_dir):
            session_file = os.path.join(self.output_dir, name, "session.json")
            if os.path.exists(session_file):
                sessions.append(name)
        
        return sorted(sessions, reverse=True)
```

### 3.2 수동/자동 캡처

```python
"""
capture_controller.py
캡처 제어
"""

import cv2
import numpy as np
import time
from typing import Optional, Callable
from enum import Enum


class CaptureMode(Enum):
    """캡처 모드"""
    MANUAL = 1          # 수동 (키 입력)
    TIMED = 2           # 시간 기반 자동
    TURNTABLE = 3       # 턴테이블 연동


class CaptureController:
    """캡처 제어기"""
    
    def __init__(self, stereo_camera, session_manager,
                 mode: CaptureMode = CaptureMode.MANUAL):
        """
        Parameters:
        - stereo_camera: StereoCamera 인스턴스
        - session_manager: ScanSessionManager 인스턴스
        - mode: 캡처 모드
        """
        
        self.camera = stereo_camera
        self.session_mgr = session_manager
        self.mode = mode
        
        # 캡처 설정
        self.angle_step = 30.0
        self.current_angle = 0.0
        
        # 타이머 설정 (TIMED 모드)
        self.capture_interval = 3.0  # 초
        self.last_capture_time = 0
        
        # 콜백
        self.on_capture: Optional[Callable] = None
        self.on_complete: Optional[Callable] = None
        
        # 상태
        self.is_capturing = False
        self.captures_remaining = 0
    
    def start_capture_sequence(self, angle_step: float = 30.0):
        """캡처 시퀀스 시작"""
        
        self.angle_step = angle_step
        self.current_angle = 0.0
        self.captures_remaining = int(360 / angle_step)
        self.is_capturing = True
        
        print(f"\n🎬 캡처 시퀀스 시작")
        print(f"   각도 간격: {angle_step}°")
        print(f"   총 캡처: {self.captures_remaining}")
        
        if self.mode == CaptureMode.MANUAL:
            print("   [SPACE] 캡처 | [ESC] 취소")
        elif self.mode == CaptureMode.TIMED:
            print(f"   {self.capture_interval}초마다 자동 캡처")
    
    def capture_single(self) -> bool:
        """단일 캡처 수행"""
        
        if self.captures_remaining <= 0:
            self._complete_sequence()
            return False
        
        # 이미지 캡처
        rect_left, rect_right = self.camera.capture_rectified()
        
        if rect_left is None or rect_right is None:
            print("❌ 캡처 실패")
            return False
        
        # 세션에 추가
        capture = self.session_mgr.add_capture(
            rect_left, rect_right, self.current_angle
        )
        
        # 콜백 호출
        if self.on_capture:
            self.on_capture(capture)
        
        # 다음 각도
        self.current_angle += self.angle_step
        self.captures_remaining -= 1
        
        if self.captures_remaining <= 0:
            self._complete_sequence()
        
        return True
    
    def update(self) -> bool:
        """업데이트 (메인 루프에서 호출)"""
        
        if not self.is_capturing:
            return False
        
        if self.mode == CaptureMode.TIMED:
            current_time = time.time()
            if current_time - self.last_capture_time >= self.capture_interval:
                self.last_capture_time = current_time
                return self.capture_single()
        
        return False
    
    def handle_key(self, key: int) -> bool:
        """키 입력 처리"""
        
        if not self.is_capturing:
            return False
        
        if self.mode == CaptureMode.MANUAL:
            if key == ord(' '):  # 스페이스바
                return self.capture_single()
            elif key == 27:  # ESC
                self.cancel()
                return False
        
        return False
    
    def cancel(self):
        """캡처 시퀀스 취소"""
        self.is_capturing = False
        print("❌ 캡처 시퀀스 취소됨")
    
    def _complete_sequence(self):
        """시퀀스 완료"""
        
        self.is_capturing = False
        
        if self.session_mgr.current_session:
            self.session_mgr.current_session.status = "captured"
            self.session_mgr._save_session()
        
        print("✅ 캡처 시퀀스 완료!")
        
        if self.on_complete:
            self.on_complete()
    
    def get_progress(self) -> Tuple[int, int, float]:
        """진행 상황 반환"""
        
        if self.session_mgr.current_session:
            total = self.session_mgr.current_session.total_angles
            current = self.session_mgr.current_session.num_captures
            return current, total, self.current_angle
        
        return 0, 0, 0.0
```

---

## 4. 포인트 클라우드 생성

### 4.1 깊이 맵에서 포인트 클라우드 생성

```python
"""
pointcloud_generator.py
포인트 클라우드 생성
"""

import cv2
import numpy as np
import open3d as o3d
from typing import Tuple, Optional
import yaml


class PointCloudGenerator:
    """포인트 클라우드 생성기"""
    
    def __init__(self, calibration_file: str):
        """
        Parameters:
        - calibration_file: 캘리브레이션 파일 경로
        """
        
        self._load_calibration(calibration_file)
        self._setup_stereo_matcher()
    
    def _load_calibration(self, filename: str):
        """캘리브레이션 로드"""
        
        with open(filename, 'r') as f:
            params = yaml.safe_load(f)
        
        self.img_size = tuple(params['image_size'])
        self.Q = np.array(params['Q'])
        self.P1 = np.array(params['P1'])
        self.baseline = params['baseline_mm']
        
        self.fx = self.P1[0, 0]
        self.fy = self.P1[1, 1]
        self.cx = self.P1[0, 2]
        self.cy = self.P1[1, 2]
        
        # 정류 맵
        K1, D1 = np.array(params['K1']), np.array(params['D1'])
        K2, D2 = np.array(params['K2']), np.array(params['D2'])
        R1, R2 = np.array(params['R1']), np.array(params['R2'])
        P1, P2 = np.array(params['P1']), np.array(params['P2'])
        
        self.map1_left, self.map2_left = cv2.initUndistortRectifyMap(
            K1, D1, R1, P1, self.img_size, cv2.CV_32FC1)
        self.map1_right, self.map2_right = cv2.initUndistortRectifyMap(
            K2, D2, R2, P2, self.img_size, cv2.CV_32FC1)
    
    def _setup_stereo_matcher(self):
        """스테레오 매처 설정"""
        
        self.stereo = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=128,
            blockSize=5,
            P1=8 * 3 * 5 ** 2,
            P2=32 * 3 * 5 ** 2,
            disp12MaxDiff=1,
            uniquenessRatio=10,
            speckleWindowSize=100,
            speckleRange=2,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
        )
        
        # WLS 필터
        self.right_matcher = cv2.ximgproc.createRightMatcher(self.stereo)
        self.wls_filter = cv2.ximgproc.createDisparityWLSFilter(self.stereo)
        self.wls_filter.setLambda(8000)
        self.wls_filter.setSigmaColor(1.5)
    
    def rectify_images(self, left: np.ndarray, 
                       right: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """이미지 정류"""
        
        rect_left = cv2.remap(left, self.map1_left, self.map2_left, 
                              cv2.INTER_LINEAR)
        rect_right = cv2.remap(right, self.map1_right, self.map2_right,
                               cv2.INTER_LINEAR)
        
        return rect_left, rect_right
    
    def compute_disparity(self, rect_left: np.ndarray,
                          rect_right: np.ndarray) -> np.ndarray:
        """시차 맵 계산"""
        
        # 왼쪽/오른쪽 시차
        disp_left = self.stereo.compute(rect_left, rect_right)
        disp_right = self.right_matcher.compute(rect_right, rect_left)
        
        # WLS 필터
        disparity = self.wls_filter.filter(
            disp_left, rect_left, 
            disparity_map_right=disp_right
        )
        
        disparity = disparity.astype(np.float32) / 16.0
        
        return disparity
    
    def disparity_to_pointcloud(self, disparity: np.ndarray,
                                 color_image: np.ndarray,
                                 min_depth: float = 100,
                                 max_depth: float = 2000) -> o3d.geometry.PointCloud:
        """
        시차 맵을 포인트 클라우드로 변환
        
        Parameters:
        - disparity: 시차 맵
        - color_image: 컬러 이미지 (BGR)
        - min_depth, max_depth: 깊이 범위 (mm)
        
        Returns:
        - pcd: Open3D PointCloud
        """
        
        # 3D 재투영
        points_3d = cv2.reprojectImageTo3D(disparity, self.Q)
        
        h, w = disparity.shape
        
        # 유효한 점 마스크
        z = points_3d[:, :, 2]
        mask = (disparity > 0) & (z > min_depth) & (z < max_depth)
        
        # 포인트 추출
        points = points_3d[mask]
        
        # 색상 추출 (BGR → RGB, 정규화)
        colors = color_image[mask][:, ::-1] / 255.0
        
        # Open3D 포인트 클라우드 생성
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.colors = o3d.utility.Vector3dVector(colors)
        
        return pcd
    
    def generate_from_stereo_pair(self, left_image: np.ndarray,
                                   right_image: np.ndarray,
                                   min_depth: float = 100,
                                   max_depth: float = 2000) -> Tuple[o3d.geometry.PointCloud, np.ndarray]:
        """
        스테레오 이미지 쌍에서 포인트 클라우드 생성
        
        Returns:
        - pcd: 포인트 클라우드
        - disparity: 시차 맵
        """
        
        # 정류
        rect_left, rect_right = self.rectify_images(left_image, right_image)
        
        # 시차 계산
        disparity = self.compute_disparity(rect_left, rect_right)
        
        # 포인트 클라우드 생성
        pcd = self.disparity_to_pointcloud(disparity, rect_left, 
                                           min_depth, max_depth)
        
        return pcd, disparity
    
    def generate_from_files(self, left_path: str, right_path: str,
                            min_depth: float = 100,
                            max_depth: float = 2000) -> o3d.geometry.PointCloud:
        """파일에서 포인트 클라우드 생성"""
        
        left = cv2.imread(left_path)
        right = cv2.imread(right_path)
        
        if left is None or right is None:
            raise ValueError(f"이미지 로드 실패: {left_path}, {right_path}")
        
        pcd, _ = self.generate_from_stereo_pair(left, right, min_depth, max_depth)
        
        return pcd
```

### 4.2 배경 제거

```python
"""
background_removal.py
배경 제거 유틸리티
"""

import cv2
import numpy as np
import open3d as o3d
from typing import Tuple, Optional


class BackgroundRemover:
    """배경 제거"""
    
    def __init__(self):
        # 배경 빼기 알고리즘
        self.bg_subtractor = cv2.createBackgroundSubtractorMOG2(
            history=100,
            varThreshold=50,
            detectShadows=False
        )
        
        # 학습된 배경 (참조용)
        self.background_depth: Optional[np.ndarray] = None
    
    def learn_background(self, depth: np.ndarray, num_frames: int = 30):
        """배경 학습 (물체 없이)"""
        
        # 여러 프레임 평균
        # 실제 구현에서는 반복 캡처 필요
        self.background_depth = depth.copy()
        print("✅ 배경 학습 완료")
    
    def remove_by_depth_difference(self, depth: np.ndarray,
                                    threshold: float = 50) -> np.ndarray:
        """
        깊이 차이로 배경 제거
        
        Returns:
        - mask: 전경 마스크
        """
        
        if self.background_depth is None:
            return np.ones(depth.shape, dtype=bool)
        
        diff = np.abs(depth - self.background_depth)
        
        # 물체 영역: 배경보다 가까움 (시차가 큼, 깊이가 작음)
        foreground = (self.background_depth - depth) > threshold
        
        return foreground
    
    def remove_by_distance(self, depth: np.ndarray,
                           min_dist: float = 200,
                           max_dist: float = 1000) -> np.ndarray:
        """
        거리 범위로 배경 제거
        
        Returns:
        - mask: 거리 범위 내 마스크
        """
        
        mask = (depth > min_dist) & (depth < max_dist)
        
        return mask
    
    def remove_by_color(self, image: np.ndarray,
                        bg_color: Tuple[int, int, int],
                        threshold: int = 30) -> np.ndarray:
        """
        색상으로 배경 제거 (그린스크린 등)
        
        Parameters:
        - image: BGR 이미지
        - bg_color: 배경 색상 (B, G, R)
        - threshold: 색상 임계값
        
        Returns:
        - mask: 전경 마스크
        """
        
        diff = np.abs(image.astype(np.int16) - np.array(bg_color))
        distance = np.sqrt(np.sum(diff ** 2, axis=2))
        
        mask = distance > threshold
        
        return mask
    
    def apply_mask_to_pointcloud(self, pcd: o3d.geometry.PointCloud,
                                  mask: np.ndarray,
                                  image_shape: Tuple[int, int]) -> o3d.geometry.PointCloud:
        """
        2D 마스크를 3D 포인트 클라우드에 적용
        
        참고: 실제 구현에서는 픽셀-포인트 매핑 필요
        """
        
        # 간단한 구현: 포인트 개수 기반 필터링
        # 실제로는 역투영 필요
        
        return pcd
    
    def remove_plane(self, pcd: o3d.geometry.PointCloud,
                     distance_threshold: float = 10) -> o3d.geometry.PointCloud:
        """
        평면 제거 (테이블 등)
        
        RANSAC으로 평면 검출 후 제거
        """
        
        # RANSAC 평면 검출
        plane_model, inliers = pcd.segment_plane(
            distance_threshold=distance_threshold,
            ransac_n=3,
            num_iterations=1000
        )
        
        # 평면이 아닌 포인트만 추출
        outlier_cloud = pcd.select_by_index(inliers, invert=True)
        
        print(f"✅ 평면 제거: {len(inliers)} points removed")
        
        return outlier_cloud
```

---

## 5. 포인트 클라우드 정합

### 5.1 ICP 기반 정합

```python
"""
cloud_registration.py
포인트 클라우드 정합
"""

import numpy as np
import open3d as o3d
from typing import List, Tuple, Optional
import copy


class PointCloudRegistration:
    """포인트 클라우드 정합"""
    
    def __init__(self, voxel_size: float = 5.0):
        """
        Parameters:
        - voxel_size: 다운샘플링 복셀 크기 (mm)
        """
        
        self.voxel_size = voxel_size
    
    def preprocess(self, pcd: o3d.geometry.PointCloud) -> Tuple[o3d.geometry.PointCloud, o3d.pipelines.registration.Feature]:
        """
        정합을 위한 전처리
        
        Returns:
        - pcd_down: 다운샘플링된 포인트 클라우드
        - fpfh: FPFH 특징
        """
        
        # 다운샘플링
        pcd_down = pcd.voxel_down_sample(self.voxel_size)
        
        # 법선 추정
        pcd_down.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=self.voxel_size * 2,
                max_nn=30
            )
        )
        
        # FPFH 특징 계산
        fpfh = o3d.pipelines.registration.compute_fpfh_feature(
            pcd_down,
            o3d.geometry.KDTreeSearchParamHybrid(
                radius=self.voxel_size * 5,
                max_nn=100
            )
        )
        
        return pcd_down, fpfh
    
    def global_registration(self, source: o3d.geometry.PointCloud,
                            target: o3d.geometry.PointCloud) -> np.ndarray:
        """
        글로벌 정합 (RANSAC 기반)
        
        초기 정렬을 위한 거친 정합
        
        Returns:
        - transformation: 4x4 변환 행렬
        """
        
        # 전처리
        source_down, source_fpfh = self.preprocess(source)
        target_down, target_fpfh = self.preprocess(target)
        
        # RANSAC 정합
        distance_threshold = self.voxel_size * 1.5
        
        result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
            source_down, target_down,
            source_fpfh, target_fpfh,
            mutual_filter=True,
            max_correspondence_distance=distance_threshold,
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
            ransac_n=4,
            checkers=[
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)
            ],
            criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(4000000, 500)
        )
        
        print(f"글로벌 정합 fitness: {result.fitness:.4f}")
        
        return result.transformation
    
    def icp_registration(self, source: o3d.geometry.PointCloud,
                         target: o3d.geometry.PointCloud,
                         init_transform: np.ndarray = np.eye(4),
                         max_iteration: int = 100) -> Tuple[np.ndarray, float]:
        """
        ICP (Iterative Closest Point) 정합
        
        Returns:
        - transformation: 4x4 변환 행렬
        - fitness: 정합 품질 (0-1)
        """
        
        threshold = self.voxel_size * 0.5
        
        # Point-to-Plane ICP
        source.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=self.voxel_size * 2, max_nn=30
            )
        )
        target.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=self.voxel_size * 2, max_nn=30
            )
        )
        
        result = o3d.pipelines.registration.registration_icp(
            source, target,
            threshold,
            init_transform,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(
                max_iteration=max_iteration
            )
        )
        
        print(f"ICP 정합 fitness: {result.fitness:.4f}, RMSE: {result.inlier_rmse:.4f}")
        
        return result.transformation, result.fitness
    
    def colored_icp(self, source: o3d.geometry.PointCloud,
                    target: o3d.geometry.PointCloud,
                    init_transform: np.ndarray = np.eye(4)) -> Tuple[np.ndarray, float]:
        """
        색상 정보를 활용한 ICP
        
        텍스처가 있는 물체에 효과적
        """
        
        # 법선 추정
        for pcd in [source, target]:
            pcd.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(
                    radius=self.voxel_size * 2, max_nn=30
                )
            )
        
        result = o3d.pipelines.registration.registration_colored_icp(
            source, target,
            self.voxel_size,
            init_transform,
            o3d.pipelines.registration.TransformationEstimationForColoredICP(),
            o3d.pipelines.registration.ICPConvergenceCriteria(
                relative_fitness=1e-6,
                relative_rmse=1e-6,
                max_iteration=50
            )
        )
        
        print(f"Colored ICP fitness: {result.fitness:.4f}")
        
        return result.transformation, result.fitness
    
    def register_with_known_rotation(self, source: o3d.geometry.PointCloud,
                                      target: o3d.geometry.PointCloud,
                                      angle_deg: float) -> np.ndarray:
        """
        알려진 회전 각도로 초기화 후 ICP
        
        턴테이블 사용 시 활용
        
        Parameters:
        - angle_deg: Y축 기준 회전 각도 (도)
        """
        
        # 회전 행렬 생성 (Y축 기준)
        angle_rad = np.radians(angle_deg)
        R = np.array([
            [np.cos(angle_rad), 0, np.sin(angle_rad)],
            [0, 1, 0],
            [-np.sin(angle_rad), 0, np.cos(angle_rad)]
        ])
        
        init_transform = np.eye(4)
        init_transform[:3, :3] = R
        
        # ICP 정밀 정합
        transform, fitness = self.icp_registration(
            source, target, init_transform
        )
        
        return transform


class MultiViewRegistration:
    """다중 시점 정합"""
    
    def __init__(self, voxel_size: float = 5.0):
        self.registration = PointCloudRegistration(voxel_size)
        self.voxel_size = voxel_size
    
    def register_sequential(self, pointclouds: List[o3d.geometry.PointCloud],
                            angles: List[float] = None) -> o3d.geometry.PointCloud:
        """
        순차적 정합
        
        Parameters:
        - pointclouds: 포인트 클라우드 리스트
        - angles: 각 포인트 클라우드의 회전 각도 (옵션)
        
        Returns:
        - merged: 병합된 포인트 클라우드
        """
        
        if len(pointclouds) == 0:
            return o3d.geometry.PointCloud()
        
        if len(pointclouds) == 1:
            return pointclouds[0]
        
        print(f"\n🔄 {len(pointclouds)}개 포인트 클라우드 정합 시작")
        
        # 첫 번째를 기준으로
        merged = copy.deepcopy(pointclouds[0])
        transforms = [np.eye(4)]
        
        for i in range(1, len(pointclouds)):
            print(f"\n[{i}/{len(pointclouds)-1}] 정합 중...")
            
            source = pointclouds[i]
            
            # 초기 변환 (알려진 각도 사용)
            if angles and len(angles) > i:
                angle_diff = angles[i] - angles[0]
                init_transform = self._rotation_matrix_y(angle_diff)
            else:
                # 글로벌 정합으로 초기화
                init_transform = self.registration.global_registration(source, merged)
            
            # ICP 정밀 정합
            transform, fitness = self.registration.colored_icp(
                source, merged, init_transform
            )
            
            transforms.append(transform)
            
            # 변환 적용 및 병합
            source_transformed = copy.deepcopy(source)
            source_transformed.transform(transform)
            merged += source_transformed
            
            # 중간 다운샘플링 (메모리 관리)
            if len(merged.points) > 1000000:
                merged = merged.voxel_down_sample(self.voxel_size)
        
        print(f"\n✅ 정합 완료: {len(merged.points):,} points")
        
        return merged
    
    def register_global_optimization(self, pointclouds: List[o3d.geometry.PointCloud]) -> o3d.geometry.PointCloud:
        """
        글로벌 최적화 기반 정합
        
        모든 쌍 간의 관계를 고려한 최적화
        """
        
        n = len(pointclouds)
        
        if n == 0:
            return o3d.geometry.PointCloud()
        
        print(f"\n🔄 글로벌 최적화 정합 ({n}개)")
        
        # 포즈 그래프 생성
        pose_graph = o3d.pipelines.registration.PoseGraph()
        
        # 노드 추가
        for i in range(n):
            pose_graph.nodes.append(
                o3d.pipelines.registration.PoseGraphNode(np.eye(4))
            )
        
        # 엣지 추가 (인접 쌍)
        for i in range(n):
            j = (i + 1) % n
            
            source = pointclouds[i]
            target = pointclouds[j]
            
            # 정합
            init_transform = self.registration.global_registration(source, target)
            transform, _ = self.registration.icp_registration(
                source, target, init_transform
            )
            
            # 엣지 추가
            pose_graph.edges.append(
                o3d.pipelines.registration.PoseGraphEdge(
                    i, j, transform,
                    np.eye(6),  # 정보 행렬
                    uncertain=False
                )
            )
        
        # 글로벌 최적화
        option = o3d.pipelines.registration.GlobalOptimizationOption(
            max_correspondence_distance=self.voxel_size * 1.5,
            edge_prune_threshold=0.25,
            preference_loop_closure=1.0
        )
        
        o3d.pipelines.registration.global_optimization(
            pose_graph,
            o3d.pipelines.registration.GlobalOptimizationLevenbergMarquardt(),
            o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria(),
            option
        )
        
        # 변환 적용 및 병합
        merged = o3d.geometry.PointCloud()
        
        for i, pcd in enumerate(pointclouds):
            pcd_transformed = copy.deepcopy(pcd)
            pcd_transformed.transform(pose_graph.nodes[i].pose)
            merged += pcd_transformed
        
        print(f"✅ 글로벌 최적화 완료: {len(merged.points):,} points")
        
        return merged
    
    def _rotation_matrix_y(self, angle_deg: float) -> np.ndarray:
        """Y축 회전 행렬"""
        angle_rad = np.radians(angle_deg)
        R = np.array([
            [np.cos(angle_rad), 0, np.sin(angle_rad), 0],
            [0, 1, 0, 0],
            [-np.sin(angle_rad), 0, np.cos(angle_rad), 0],
            [0, 0, 0, 1]
        ])
        return R
```

---

## 6. 후처리 및 필터링

### 6.1 포인트 클라우드 처리

```python
"""
cloud_processor.py
포인트 클라우드 후처리
"""

import numpy as np
import open3d as o3d
from typing import Tuple, Optional


class PointCloudProcessor:
    """포인트 클라우드 처리기"""
    
    def __init__(self):
        pass
    
    def filter_statistical_outlier(self, pcd: o3d.geometry.PointCloud,
                                    nb_neighbors: int = 20,
                                    std_ratio: float = 2.0) -> o3d.geometry.PointCloud:
        """
        통계적 이상치 제거
        """
        
        original_count = len(pcd.points)
        
        filtered, _ = pcd.remove_statistical_outlier(
            nb_neighbors=nb_neighbors,
            std_ratio=std_ratio
        )
        
        removed = original_count - len(filtered.points)
        print(f"통계적 이상치 제거: {removed:,} points ({removed/original_count*100:.1f}%)")
        
        return filtered
    
    def filter_radius_outlier(self, pcd: o3d.geometry.PointCloud,
                              nb_points: int = 16,
                              radius: float = 10.0) -> o3d.geometry.PointCloud:
        """
        반경 기반 이상치 제거
        """
        
        original_count = len(pcd.points)
        
        filtered, _ = pcd.remove_radius_outlier(
            nb_points=nb_points,
            radius=radius
        )
        
        removed = original_count - len(filtered.points)
        print(f"반경 이상치 제거: {removed:,} points")
        
        return filtered
    
    def downsample(self, pcd: o3d.geometry.PointCloud,
                   voxel_size: float = 2.0) -> o3d.geometry.PointCloud:
        """
        복셀 다운샘플링
        """
        
        original_count = len(pcd.points)
        
        downsampled = pcd.voxel_down_sample(voxel_size)
        
        new_count = len(downsampled.points)
        print(f"다운샘플링: {original_count:,} → {new_count:,} points")
        
        return downsampled
    
    def estimate_normals(self, pcd: o3d.geometry.PointCloud,
                         radius: float = 10.0,
                         max_nn: int = 30) -> o3d.geometry.PointCloud:
        """
        법선 추정
        """
        
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=radius,
                max_nn=max_nn
            )
        )
        
        # 일관된 방향으로 조정
        pcd.orient_normals_consistent_tangent_plane(k=15)
        
        print(f"법선 추정 완료: {len(pcd.normals)} normals")
        
        return pcd
    
    def smooth(self, pcd: o3d.geometry.PointCloud,
               iterations: int = 1) -> o3d.geometry.PointCloud:
        """
        포인트 클라우드 스무딩 (평균 필터)
        
        주의: Open3D 기본 제공 아님, 간단한 구현
        """
        
        points = np.asarray(pcd.points)
        colors = np.asarray(pcd.colors) if pcd.has_colors() else None
        
        # KD-Tree 생성
        kdtree = o3d.geometry.KDTreeFlann(pcd)
        
        new_points = np.zeros_like(points)
        
        for i in range(len(points)):
            # 이웃 찾기
            [k, idx, _] = kdtree.search_knn_vector_3d(points[i], 10)
            
            # 평균 위치
            new_points[i] = np.mean(points[idx], axis=0)
        
        smoothed = o3d.geometry.PointCloud()
        smoothed.points = o3d.utility.Vector3dVector(new_points)
        
        if colors is not None:
            smoothed.colors = o3d.utility.Vector3dVector(colors)
        
        print(f"스무딩 완료 ({iterations} iterations)")
        
        return smoothed
    
    def crop_by_bbox(self, pcd: o3d.geometry.PointCloud,
                     min_bound: np.ndarray,
                     max_bound: np.ndarray) -> o3d.geometry.PointCloud:
        """
        바운딩 박스로 크롭
        """
        
        bbox = o3d.geometry.AxisAlignedBoundingBox(
            min_bound=min_bound,
            max_bound=max_bound
        )
        
        cropped = pcd.crop(bbox)
        
        print(f"크롭: {len(pcd.points):,} → {len(cropped.points):,} points")
        
        return cropped
    
    def process_pipeline(self, pcd: o3d.geometry.PointCloud,
                         voxel_size: float = 2.0,
                         remove_outliers: bool = True) -> o3d.geometry.PointCloud:
        """
        표준 처리 파이프라인
        """
        
        print("\n🔧 포인트 클라우드 처리 파이프라인")
        print("="*40)
        
        # 1. 다운샘플링
        pcd = self.downsample(pcd, voxel_size)
        
        # 2. 이상치 제거
        if remove_outliers:
            pcd = self.filter_statistical_outlier(pcd)
        
        # 3. 법선 추정
        pcd = self.estimate_normals(pcd, radius=voxel_size * 3)
        
        print("="*40)
        print(f"✅ 처리 완료: {len(pcd.points):,} points")
        
        return pcd
```

---

## 7. 메쉬 생성

### 7.1 표면 재구성

```python
"""
mesh_generator.py
메쉬 생성
"""

import numpy as np
import open3d as o3d
from typing import Optional, Tuple


class MeshGenerator:
    """메쉬 생성기"""
    
    def __init__(self):
        pass
    
    def poisson_reconstruction(self, pcd: o3d.geometry.PointCloud,
                                depth: int = 9,
                                scale: float = 1.1) -> Tuple[o3d.geometry.TriangleMesh, np.ndarray]:
        """
        Poisson Surface Reconstruction
        
        매끄러운 표면 생성에 적합
        
        Parameters:
        - depth: 옥트리 깊이 (높을수록 세밀)
        - scale: 바운딩 박스 스케일
        
        Returns:
        - mesh: 생성된 메쉬
        - densities: 밀도 값 (후처리용)
        """
        
        print("\n🔨 Poisson 표면 재구성")
        print(f"   옥트리 깊이: {depth}")
        
        # 법선 필요
        if not pcd.has_normals():
            pcd.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(
                    radius=10, max_nn=30
                )
            )
            pcd.orient_normals_consistent_tangent_plane(k=15)
        
        # Poisson 재구성
        mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
            pcd,
            depth=depth,
            scale=scale,
            linear_fit=False
        )
        
        densities = np.asarray(densities)
        
        print(f"✅ 메쉬 생성: {len(mesh.vertices):,} vertices, {len(mesh.triangles):,} triangles")
        
        return mesh, densities
    
    def ball_pivoting(self, pcd: o3d.geometry.PointCloud,
                      radii: list = None) -> o3d.geometry.TriangleMesh:
        """
        Ball Pivoting Algorithm (BPA)
        
        원본 포인트를 유지하는 표면 생성
        
        Parameters:
        - radii: 구의 반지름 리스트
        """
        
        print("\n🔨 Ball Pivoting 표면 재구성")
        
        if radii is None:
            # 평균 포인트 거리 기반 자동 계산
            distances = pcd.compute_nearest_neighbor_distance()
            avg_dist = np.mean(distances)
            radii = [avg_dist * 1.5, avg_dist * 3, avg_dist * 6]
        
        print(f"   반지름: {radii}")
        
        # 법선 필요
        if not pcd.has_normals():
            pcd.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(
                    radius=10, max_nn=30
                )
            )
        
        # Ball Pivoting
        radii_o3d = o3d.utility.DoubleVector(radii)
        mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
            pcd, radii_o3d
        )
        
        print(f"✅ 메쉬 생성: {len(mesh.vertices):,} vertices, {len(mesh.triangles):,} triangles")
        
        return mesh
    
    def alpha_shape(self, pcd: o3d.geometry.PointCloud,
                    alpha: float = 30.0) -> o3d.geometry.TriangleMesh:
        """
        Alpha Shape
        
        Parameters:
        - alpha: 알파 값 (작을수록 세밀, 구멍 많음)
        """
        
        print(f"\n🔨 Alpha Shape (alpha={alpha})")
        
        mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(
            pcd, alpha
        )
        
        print(f"✅ 메쉬 생성: {len(mesh.vertices):,} vertices, {len(mesh.triangles):,} triangles")
        
        return mesh
    
    def clean_mesh(self, mesh: o3d.geometry.TriangleMesh,
                   densities: np.ndarray = None,
                   density_threshold: float = 0.1) -> o3d.geometry.TriangleMesh:
        """
        메쉬 정리
        
        Parameters:
        - densities: Poisson 밀도 값
        - density_threshold: 밀도 임계값 (하위 % 제거)
        """
        
        print("\n🧹 메쉬 정리")
        
        # 저밀도 영역 제거
        if densities is not None:
            threshold = np.quantile(densities, density_threshold)
            vertices_to_remove = densities < threshold
            mesh.remove_vertices_by_mask(vertices_to_remove)
            print(f"   저밀도 영역 제거: {np.sum(vertices_to_remove):,} vertices")
        
        # 중복 제거
        mesh.remove_duplicated_vertices()
        mesh.remove_duplicated_triangles()
        
        # 퇴화 삼각형 제거
        mesh.remove_degenerate_triangles()
        
        # 비다양체 엣지 제거
        mesh.remove_non_manifold_edges()
        
        # 연결되지 않은 작은 컴포넌트 제거
        triangle_clusters, cluster_n_triangles, _ = mesh.cluster_connected_triangles()
        triangle_clusters = np.asarray(triangle_clusters)
        cluster_n_triangles = np.asarray(cluster_n_triangles)
        
        # 가장 큰 클러스터만 유지
        largest_cluster_idx = np.argmax(cluster_n_triangles)
        triangles_to_remove = triangle_clusters != largest_cluster_idx
        mesh.remove_triangles_by_mask(triangles_to_remove)
        
        # 법선 재계산
        mesh.compute_vertex_normals()
        
        print(f"✅ 정리 완료: {len(mesh.vertices):,} vertices, {len(mesh.triangles):,} triangles")
        
        return mesh
    
    def smooth_mesh(self, mesh: o3d.geometry.TriangleMesh,
                    iterations: int = 5,
                    method: str = 'laplacian') -> o3d.geometry.TriangleMesh:
        """
        메쉬 스무딩
        
        Parameters:
        - iterations: 반복 횟수
        - method: 'laplacian' 또는 'taubin'
        """
        
        print(f"\n🧴 메쉬 스무딩 ({method}, {iterations} iterations)")
        
        if method == 'laplacian':
            smoothed = mesh.filter_smooth_laplacian(
                number_of_iterations=iterations
            )
        elif method == 'taubin':
            smoothed = mesh.filter_smooth_taubin(
                number_of_iterations=iterations
            )
        else:
            smoothed = mesh
        
        smoothed.compute_vertex_normals()
        
        print("✅ 스무딩 완료")
        
        return smoothed
    
    def simplify_mesh(self, mesh: o3d.geometry.TriangleMesh,
                      target_triangles: int = 50000) -> o3d.geometry.TriangleMesh:
        """
        메쉬 단순화 (삼각형 수 감소)
        """
        
        original = len(mesh.triangles)
        
        simplified = mesh.simplify_quadric_decimation(
            target_number_of_triangles=target_triangles
        )
        
        simplified.compute_vertex_normals()
        
        print(f"메쉬 단순화: {original:,} → {len(simplified.triangles):,} triangles")
        
        return simplified
    
    def add_texture(self, mesh: o3d.geometry.TriangleMesh,
                    pcd: o3d.geometry.PointCloud) -> o3d.geometry.TriangleMesh:
        """
        포인트 클라우드 색상을 메쉬에 적용
        """
        
        if not pcd.has_colors():
            print("⚠️ 포인트 클라우드에 색상 없음")
            return mesh
        
        # 가장 가까운 포인트의 색상 할당
        pcd_tree = o3d.geometry.KDTreeFlann(pcd)
        mesh_vertices = np.asarray(mesh.vertices)
        pcd_colors = np.asarray(pcd.colors)
        
        vertex_colors = np.zeros((len(mesh_vertices), 3))
        
        for i, vertex in enumerate(mesh_vertices):
            [k, idx, _] = pcd_tree.search_knn_vector_3d(vertex, 1)
            vertex_colors[i] = pcd_colors[idx[0]]
        
        mesh.vertex_colors = o3d.utility.Vector3dVector(vertex_colors)
        
        print("✅ 텍스처 적용 완료")
        
        return mesh
```

---

## 8. 파일 출력

### 8.1 파일 내보내기

```python
"""
file_exporter.py
3D 모델 파일 출력
"""

import os
import numpy as np
import open3d as o3d
from typing import Optional


class FileExporter:
    """파일 출력 관리"""
    
    def __init__(self, output_dir: str = "output"):
        self.output_dir = output_dir
        
        os.makedirs(os.path.join(output_dir, "pointclouds"), exist_ok=True)
        os.makedirs(os.path.join(output_dir, "meshes"), exist_ok=True)
    
    def export_pointcloud(self, pcd: o3d.geometry.PointCloud,
                          filename: str,
                          format: str = 'ply') -> str:
        """
        포인트 클라우드 내보내기
        
        Parameters:
        - pcd: 포인트 클라우드
        - filename: 파일명 (확장자 제외)
        - format: 'ply', 'pcd', 'xyz', 'pts'
        
        Returns:
        - filepath: 저장된 파일 경로
        """
        
        filepath = os.path.join(
            self.output_dir, "pointclouds", f"{filename}.{format}"
        )
        
        success = o3d.io.write_point_cloud(filepath, pcd)
        
        if success:
            size_mb = os.path.getsize(filepath) / (1024 * 1024)
            print(f"✅ 포인트 클라우드 저장: {filepath} ({size_mb:.1f} MB)")
        else:
            print(f"❌ 저장 실패: {filepath}")
        
        return filepath
    
    def export_mesh(self, mesh: o3d.geometry.TriangleMesh,
                    filename: str,
                    format: str = 'obj') -> str:
        """
        메쉬 내보내기
        
        Parameters:
        - mesh: 메쉬
        - filename: 파일명 (확장자 제외)
        - format: 'obj', 'ply', 'stl', 'gltf', 'glb'
        
        Returns:
        - filepath: 저장된 파일 경로
        """
        
        filepath = os.path.join(
            self.output_dir, "meshes", f"{filename}.{format}"
        )
        
        success = o3d.io.write_triangle_mesh(filepath, mesh)
        
        if success:
            size_mb = os.path.getsize(filepath) / (1024 * 1024)
            print(f"✅ 메쉬 저장: {filepath} ({size_mb:.1f} MB)")
            print(f"   정점: {len(mesh.vertices):,}, 삼각형: {len(mesh.triangles):,}")
        else:
            print(f"❌ 저장 실패: {filepath}")
        
        return filepath
    
    def export_for_3d_printing(self, mesh: o3d.geometry.TriangleMesh,
                               filename: str,
                               scale: float = 1.0) -> str:
        """
        3D 프린팅용 STL 내보내기
        
        Parameters:
        - scale: 스케일 (1.0 = 원본, mm 단위)
        """
        
        # 복사
        mesh_copy = o3d.geometry.TriangleMesh(mesh)
        
        # 스케일 적용
        if scale != 1.0:
            mesh_copy.scale(scale, center=mesh_copy.get_center())
        
        # 워터타이트 확인
        if not mesh_copy.is_watertight():
            print("⚠️ 메쉬가 워터타이트하지 않음 (구멍 있음)")
            print("   3D 프린팅 시 문제가 발생할 수 있습니다.")
        
        # STL 저장 (바이너리)
        filepath = os.path.join(
            self.output_dir, "meshes", f"{filename}_print.stl"
        )
        
        o3d.io.write_triangle_mesh(filepath, mesh_copy, write_ascii=False)
        
        size_mb = os.path.getsize(filepath) / (1024 * 1024)
        print(f"✅ 3D 프린팅용 STL 저장: {filepath} ({size_mb:.1f} MB)")
        
        # 크기 정보
        bbox = mesh_copy.get_axis_aligned_bounding_box()
        extent = bbox.get_extent()
        print(f"   크기: {extent[0]:.1f} x {extent[1]:.1f} x {extent[2]:.1f} mm")
        
        return filepath
    
    def export_all_formats(self, mesh: o3d.geometry.TriangleMesh,
                           filename: str) -> dict:
        """
        모든 형식으로 내보내기
        """
        
        formats = ['obj', 'ply', 'stl', 'gltf']
        paths = {}
        
        for fmt in formats:
            try:
                path = self.export_mesh(mesh, filename, fmt)
                paths[fmt] = path
            except Exception as e:
                print(f"⚠️ {fmt} 형식 저장 실패: {e}")
        
        return paths
```

---

## 9. GUI 구현

### 9.1 스캐너 GUI

```python
"""
gui.py
3D 스캐너 GUI
"""

import cv2
import numpy as np
import open3d as o3d
from typing import Optional, List, Callable
import time
import threading


class Scanner3DGUI:
    """3D 스캐너 GUI"""
    
    def __init__(self, window_name: str = "3D Scanner"):
        self.window_name = window_name
        
        # 상태
        self.current_view = 'camera'  # 'camera', 'preview', 'result'
        
        # FPS
        self.fps_history = []
        self.last_time = time.time()
        
        # 3D 뷰어
        self.vis: Optional[o3d.visualization.Visualizer] = None
        
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
    
    def update_fps(self) -> float:
        """FPS 계산"""
        current = time.time()
        fps = 1.0 / (current - self.last_time + 1e-6)
        self.last_time = current
        
        self.fps_history.append(fps)
        if len(self.fps_history) > 30:
            self.fps_history.pop(0)
        
        return np.mean(self.fps_history)
    
    def draw_capture_overlay(self, image: np.ndarray,
                             current_angle: float,
                             num_captured: int,
                             total: int,
                             is_capturing: bool) -> np.ndarray:
        """캡처 오버레이"""
        
        h, w = image.shape[:2]
        
        # 상태 패널
        overlay = image.copy()
        cv2.rectangle(overlay, (10, 10), (350, 150), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.7, image, 0.3, 0, image)
        
        # 텍스트
        y = 40
        fps = self.update_fps()
        cv2.putText(image, f"FPS: {fps:.1f}", (20, y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        y += 30
        cv2.putText(image, f"Angle: {current_angle:.0f}°", (20, y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        y += 30
        cv2.putText(image, f"Captures: {num_captured}/{total}", (20, y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        y += 30
        if is_capturing:
            cv2.putText(image, "● CAPTURING", (20, y),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        else:
            cv2.putText(image, "○ READY", (20, y),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # 진행률 바
        progress = num_captured / total if total > 0 else 0
        bar_x = 20
        bar_y = 160
        bar_w = 310
        bar_h = 20
        
        cv2.rectangle(image, (bar_x, bar_y), (bar_x + bar_w, bar_y + bar_h),
                     (100, 100, 100), -1)
        cv2.rectangle(image, (bar_x, bar_y), 
                     (bar_x + int(bar_w * progress), bar_y + bar_h),
                     (0, 255, 0), -1)
        cv2.rectangle(image, (bar_x, bar_y), (bar_x + bar_w, bar_y + bar_h),
                     (255, 255, 255), 1)
        
        # 도움말
        help_text = "[SPACE] Capture | [S] Start | [P] Process | [Q] Quit"
        cv2.putText(image, help_text, (20, h - 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        
        return image
    
    def draw_processing_overlay(self, image: np.ndarray,
                                 status: str,
                                 progress: float) -> np.ndarray:
        """처리 중 오버레이"""
        
        h, w = image.shape[:2]
        
        # 어둡게
        overlay = np.zeros_like(image)
        cv2.addWeighted(image, 0.3, overlay, 0.7, 0, image)
        
        # 상태 텍스트
        cv2.putText(image, status, (w//2 - 100, h//2),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
        # 진행률 바
        bar_w = 400
        bar_h = 30
        bar_x = (w - bar_w) // 2
        bar_y = h // 2 + 40
        
        cv2.rectangle(image, (bar_x, bar_y), (bar_x + bar_w, bar_y + bar_h),
                     (100, 100, 100), -1)
        cv2.rectangle(image, (bar_x, bar_y),
                     (bar_x + int(bar_w * progress), bar_y + bar_h),
                     (0, 200, 255), -1)
        
        cv2.putText(image, f"{progress*100:.0f}%", 
                   (w//2 - 20, bar_y + bar_h + 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        return image
    
    def show_3d_preview(self, pcd: o3d.geometry.PointCloud):
        """3D 미리보기 (별도 창)"""
        
        if self.vis is None:
            self.vis = o3d.visualization.Visualizer()
            self.vis.create_window("3D Preview", width=800, height=600)
            
            opt = self.vis.get_render_option()
            opt.point_size = 2.0
            opt.background_color = np.array([0.1, 0.1, 0.1])
        
        self.vis.clear_geometries()
        self.vis.add_geometry(pcd)
        
        # 좌표축
        coord = o3d.geometry.TriangleMesh.create_coordinate_frame(size=50)
        self.vis.add_geometry(coord)
        
        self.vis.poll_events()
        self.vis.update_renderer()
    
    def show_mesh_result(self, mesh: o3d.geometry.TriangleMesh):
        """최종 메쉬 결과 표시"""
        
        # 별도 창에서 상호작용 가능한 뷰어
        o3d.visualization.draw_geometries(
            [mesh],
            window_name="3D Scan Result",
            width=1024,
            height=768
        )
    
    def show(self, image: np.ndarray):
        """이미지 표시"""
        cv2.imshow(self.window_name, image)
    
    def wait_key(self, delay: int = 1) -> int:
        """키 입력"""
        return cv2.waitKey(delay) & 0xFF
    
    def close(self):
        """종료"""
        cv2.destroyAllWindows()
        if self.vis:
            self.vis.destroy_window()
```

---

## 10. 전체 코드

### 10.1 메인 실행 파일

```python
"""
main.py
3D 스캐너 메인
"""

import argparse
import yaml
import cv2
import numpy as np
import open3d as o3d
import sys
from datetime import datetime

sys.path.append('src')

from stereo_camera import StereoCamera
from scan_session import ScanSessionManager
from capture_controller import CaptureController, CaptureMode
from pointcloud_generator import PointCloudGenerator
from cloud_registration import MultiViewRegistration
from cloud_processor import PointCloudProcessor
from mesh_generator import MeshGenerator
from file_exporter import FileExporter
from gui import Scanner3DGUI


class Scanner3DApp:
    """3D 스캐너 애플리케이션"""
    
    def __init__(self, config_file: str):
        with open(config_file, 'r') as f:
            self.config = yaml.safe_load(f)
        
        self._init_components()
        
        print("\n" + "="*60)
        print("📦 3D 스캐너")
        print("="*60)
    
    def _init_components(self):
        """컴포넌트 초기화"""
        
        # 스테레오 카메라
        self.camera = StereoCamera(
            self.config['calibration_file'],
            self.config['camera']['left_id'],
            self.config['camera']['right_id']
        )
        
        # 세션 관리
        self.session_mgr = ScanSessionManager(self.config['output_dir'])
        
        # 캡처 컨트롤러
        self.capture_ctrl = CaptureController(
            self.camera, self.session_mgr,
            CaptureMode.MANUAL
        )
        
        # 포인트 클라우드 생성
        self.pc_generator = PointCloudGenerator(self.config['calibration_file'])
        
        # 정합
        self.registration = MultiViewRegistration(
            voxel_size=self.config['processing']['voxel_size']
        )
        
        # 처리
        self.processor = PointCloudProcessor()
        
        # 메쉬 생성
        self.mesh_generator = MeshGenerator()
        
        # 파일 출력
        self.exporter = FileExporter(self.config['output_dir'])
        
        # GUI
        self.gui = Scanner3DGUI()
        
        # 상태
        self.pointclouds = []
        self.merged_cloud = None
        self.final_mesh = None
    
    def run(self):
        """메인 루프"""
        
        print("\n조작 방법:")
        print("  [N]     - 새 스캔 세션")
        print("  [S]     - 캡처 시작")
        print("  [SPACE] - 단일 캡처")
        print("  [P]     - 처리 (정합 + 메쉬)")
        print("  [V]     - 3D 미리보기")
        print("  [E]     - 내보내기")
        print("  [Q]     - 종료")
        print("="*60 + "\n")
        
        while True:
            # 캡처
            rect_left, rect_right = self.camera.capture_rectified()
            
            if rect_left is None:
                continue
            
            # 자동 캡처 업데이트
            self.capture_ctrl.update()
            
            # 진행 상황
            current, total, angle = self.capture_ctrl.get_progress()
            
            # 렌더링
            display = self.gui.draw_capture_overlay(
                rect_left.copy(),
                angle,
                current,
                total,
                self.capture_ctrl.is_capturing
            )
            
            self.gui.show(display)
            
            # 키 처리
            key = self.gui.wait_key(1)
            
            if key == ord('q'):
                break
            
            elif key == ord('n'):
                self._new_session()
            
            elif key == ord('s'):
                self._start_capture()
            
            elif key == ord(' '):
                self.capture_ctrl.handle_key(key)
            
            elif key == ord('p'):
                self._process()
            
            elif key == ord('v'):
                self._preview_3d()
            
            elif key == ord('e'):
                self._export()
        
        self._cleanup()
    
    def _new_session(self):
        """새 세션"""
        
        name = f"scan_{datetime.now().strftime('%H%M%S')}"
        angle_step = self.config['scan']['angle_step']
        
        self.session_mgr.create_session(name, angle_step)
        self.pointclouds = []
        self.merged_cloud = None
        self.final_mesh = None
        
        print(f"✅ 새 세션: {name}")
    
    def _start_capture(self):
        """캡처 시작"""
        
        if self.session_mgr.current_session is None:
            self._new_session()
        
        angle_step = self.config['scan']['angle_step']
        self.capture_ctrl.start_capture_sequence(angle_step)
    
    def _process(self):
        """처리 (정합 + 메쉬 생성)"""
        
        session = self.session_mgr.current_session
        if session is None or len(session.captures) == 0:
            print("⚠️ 캡처된 이미지 없음")
            return
        
        print(f"\n🔄 {len(session.captures)}개 캡처 처리 중...")
        
        # 1. 포인트 클라우드 생성
        self.pointclouds = []
        angles = []
        
        for capture in session.captures:
            pcd = self.pc_generator.generate_from_files(
                capture.left_image_path,
                capture.right_image_path,
                min_depth=self.config['processing']['min_depth'],
                max_depth=self.config['processing']['max_depth']
            )
            
            # 배경 제거 (평면)
            from background_removal import BackgroundRemover
            bg_remover = BackgroundRemover()
            pcd = bg_remover.remove_plane(pcd, distance_threshold=20)
            
            self.pointclouds.append(pcd)
            angles.append(capture.angle)
            
            print(f"  캡처 #{capture.frame_id}: {len(pcd.points):,} points")
        
        # 2. 정합
        self.merged_cloud = self.registration.register_sequential(
            self.pointclouds, angles
        )
        
        # 3. 후처리
        self.merged_cloud = self.processor.process_pipeline(
            self.merged_cloud,
            voxel_size=self.config['processing']['voxel_size']
        )
        
        # 4. 메쉬 생성
        self.final_mesh, densities = self.mesh_generator.poisson_reconstruction(
            self.merged_cloud,
            depth=self.config['mesh']['poisson_depth']
        )
        
        # 메쉬 정리
        self.final_mesh = self.mesh_generator.clean_mesh(
            self.final_mesh, densities,
            density_threshold=0.1
        )
        
        # 색상 적용
        self.final_mesh = self.mesh_generator.add_texture(
            self.final_mesh, self.merged_cloud
        )
        
        # 스무딩
        self.final_mesh = self.mesh_generator.smooth_mesh(
            self.final_mesh, iterations=3
        )
        
        print("\n✅ 처리 완료!")
    
    def _preview_3d(self):
        """3D 미리보기"""
        
        if self.merged_cloud is not None:
            self.gui.show_3d_preview(self.merged_cloud)
        elif len(self.pointclouds) > 0:
            self.gui.show_3d_preview(self.pointclouds[-1])
        else:
            print("⚠️ 미리보기할 데이터 없음")
    
    def _export(self):
        """내보내기"""
        
        session = self.session_mgr.current_session
        if session is None:
            print("⚠️ 세션 없음")
            return
        
        name = session.name
        
        # 포인트 클라우드
        if self.merged_cloud is not None:
            self.exporter.export_pointcloud(self.merged_cloud, f"{name}_cloud", 'ply')
        
        # 메쉬
        if self.final_mesh is not None:
            self.exporter.export_mesh(self.final_mesh, f"{name}_mesh", 'obj')
            self.exporter.export_for_3d_printing(self.final_mesh, name)
        
        print("✅ 내보내기 완료")
    
    def _cleanup(self):
        """정리"""
        self.camera.release()
        self.gui.close()
        print("\n프로그램 종료")


def main():
    parser = argparse.ArgumentParser(description='3D 스캐너')
    parser.add_argument('--config', default='config/scanner_config.yaml')
    args = parser.parse_args()
    
    app = Scanner3DApp(args.config)
    app.run()


if __name__ == "__main__":
    main()
```

### 10.2 설정 파일

```yaml
# config/scanner_config.yaml

calibration_file: "config/stereo_params.yaml"
output_dir: "output/projects"

camera:
  left_id: 0
  right_id: 2
  width: 1280
  height: 720

scan:
  angle_step: 30          # 30° 간격 = 12 캡처
  capture_mode: "manual"  # manual, timed

processing:
  min_depth: 200          # mm
  max_depth: 1000         # mm
  voxel_size: 2.0         # mm

mesh:
  method: "poisson"       # poisson, ball_pivoting
  poisson_depth: 9
  smooth_iterations: 3

export:
  formats: ["obj", "stl", "ply"]
  scale_for_print: 1.0
```

---

## 📝 학습 체크리스트

### 구현 완료

- [ ] 다중 시점 캡처 시스템
- [ ] 포인트 클라우드 생성
- [ ] 배경/평면 제거
- [ ] ICP 기반 정합
- [ ] 포인트 클라우드 후처리
- [ ] Poisson 메쉬 생성
- [ ] 파일 출력 (OBJ, STL, PLY)
- [ ] GUI 구현

### 테스트 완료

- [ ] 단일 캡처 포인트 클라우드 품질
- [ ] 다중 시점 정합 정확도
- [ ] 메쉬 품질 (워터타이트, 구멍)
- [ ] 3D 프린팅 호환성

---

## ➡️ 다음 프로젝트

**[Project 04: Visual Odometry](../Project_04_Visual_Odometry/README.md)**

다음 프로젝트에서는:
- 스테레오 Visual Odometry
- 특징점 추적
- 모션 추정
- 궤적 시각화

를 구현합니다.

---

## 📄 라이선스

MIT License - 자유롭게 사용, 수정, 배포 가능
