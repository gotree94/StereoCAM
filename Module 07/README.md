# Module 07: 임베디드 구현 (Jetson/STM32H7)

[![난이도](https://img.shields.io/badge/난이도-⭐⭐⭐⭐⭐_전문가-purple.svg)]()
[![예상시간](https://img.shields.io/badge/예상시간-15--20시간-blue.svg)]()
[![선수지식](https://img.shields.io/badge/선수지식-Module_03,_C/C++,_임베디드_기초-orange.svg)]()

---

## 📋 모듈 개요

| 항목 | 내용 |
|------|------|
| **학습 목표** | 스테레오 비전 시스템을 임베디드 플랫폼에 최적화하여 구현 |
| **핵심 키워드** | Jetson, STM32H7, CUDA, DMA, DCMI, 실시간 처리 |
| **산출물** | Jetson 최적화 코드, STM32H7 펌웨어, 성능 벤치마크 |

---

## 📚 목차

1. [임베디드 스테레오 비전 개요](#1-임베디드-스테레오-비전-개요)
2. [플랫폼 비교 및 선택](#2-플랫폼-비교-및-선택)
3. [NVIDIA Jetson 구현](#3-nvidia-jetson-구현)
4. [Jetson CUDA 최적화](#4-jetson-cuda-최적화)
5. [Jetson VPI 활용](#5-jetson-vpi-활용)
6. [STM32H7 구현](#6-stm32h7-구현)
7. [STM32H7 카메라 인터페이스](#7-stm32h7-카메라-인터페이스)
8. [STM32H7 스테레오 매칭](#8-stm32h7-스테레오-매칭)
9. [성능 최적화 기법](#9-성능-최적화-기법)
10. [실습 프로젝트](#10-실습-프로젝트)

---

## 1. 임베디드 스테레오 비전 개요

### 1.1 임베디드 시스템의 도전과제

```
┌─────────────────────────────────────────────────────────────┐
│              임베디드 스테레오 비전 도전과제                   │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  🔋 전력 제약                                                │
│     PC: 200W+ → Jetson: 10-30W → STM32: 0.5W              │
│                                                             │
│  💾 메모리 제약                                              │
│     PC: 16GB+ → Jetson: 4-8GB → STM32: 1MB               │
│                                                             │
│  ⚡ 처리 성능                                                │
│     실시간 요구사항 충족 필요 (10-30 FPS)                    │
│                                                             │
│  🌡️ 열 관리                                                 │
│     제한된 방열 환경에서 동작                                 │
│                                                             │
│  📦 크기/무게 제약                                           │
│     드론, 소형 로봇 등 물리적 제약                           │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### 1.2 임베디드 플랫폼 계층

```
┌─────────────────────────────────────────────────────────────┐
│                   성능/전력 트레이드오프                      │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  고성능                                                      │
│    ▲                                                        │
│    │  ┌──────────────┐                                     │
│    │  │ Jetson AGX   │ 딥러닝 스테레오, 고해상도            │
│    │  │ Orin (60W)   │ RAFT-Stereo 실시간 가능             │
│    │  └──────────────┘                                     │
│    │  ┌──────────────┐                                     │
│    │  │ Jetson Xavier│ 딥러닝 스테레오 (저해상도)           │
│    │  │ NX (15W)     │ SGBM 실시간                         │
│    │  └──────────────┘                                     │
│    │  ┌──────────────┐                                     │
│    │  │ Jetson Nano  │ SGBM 실시간                         │
│    │  │ (10W)        │ 제한된 딥러닝                        │
│    │  └──────────────┘                                     │
│    │  ┌──────────────┐                                     │
│    │  │ STM32H7      │ 단순 Block Matching                 │
│    │  │ (0.5W)       │ 저해상도, 낮은 FPS                   │
│    │  └──────────────┘                                     │
│    │                                                        │
│    └──────────────────────────────────────────────→ 저전력  │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

---

## 2. 플랫폼 비교 및 선택

### 2.1 상세 비교표

| 특성 | Jetson Orin | Jetson Xavier NX | Jetson Nano | STM32H7 |
|------|-------------|------------------|-------------|---------|
| **CPU** | 12x Cortex-A78 | 6x Carmel | 4x Cortex-A57 | 1x Cortex-M7 |
| **GPU** | 2048 CUDA | 384 CUDA | 128 CUDA | 없음 |
| **메모리** | 32GB | 8GB | 4GB | 1MB SRAM |
| **전력** | 15-60W | 10-20W | 5-10W | 0.3-0.5W |
| **가격** | ~$1000 | ~$400 | ~$150 | ~$20 |
| **OS** | Linux | Linux | Linux | Bare Metal/RTOS |
| **스테레오 성능** | 1080p@60fps | 720p@30fps | 480p@15fps | 160x120@5fps |

### 2.2 용도별 추천

```
┌─────────────────────────────────────────────────────────────┐
│                    용도별 플랫폼 추천                         │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  🚗 자율주행 차량                                            │
│     → Jetson AGX Orin                                      │
│     → 고해상도, 딥러닝, 다중 센서 융합                        │
│                                                             │
│  🤖 서비스 로봇                                              │
│     → Jetson Xavier NX                                     │
│     → 실내 네비게이션, 물체 인식                             │
│                                                             │
│  🚁 소형 드론                                                │
│     → Jetson Nano                                          │
│     → 무게/전력 제약, 장애물 회피                            │
│                                                             │
│  🏭 산업용 센서                                              │
│     → STM32H7                                              │
│     → 초저전력, 단순 거리 측정                               │
│                                                             │
│  📚 교육/프로토타이핑                                        │
│     → Jetson Nano                                          │
│     → 가격 대비 성능, 풍부한 자료                            │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

---

## 3. NVIDIA Jetson 구현

### 3.1 Jetson 환경 설정

```bash
# JetPack 설치 확인
cat /etc/nv_tegra_release

# CUDA 버전 확인
nvcc --version

# OpenCV CUDA 지원 확인
python3 -c "import cv2; print(cv2.getBuildInformation())" | grep CUDA

# 필수 패키지 설치
sudo apt update
sudo apt install -y \
    python3-pip \
    python3-opencv \
    libopencv-dev \
    cmake \
    build-essential

# Python 패키지
pip3 install numpy pyyaml

# VPI (Vision Programming Interface) 설치 확인
python3 -c "import vpi; print(vpi.__version__)"
```

### 3.2 기본 스테레오 구현 (Jetson)

```python
"""
jetson_stereo_basic.py
Jetson 기본 스테레오 비전 구현
"""

import cv2
import numpy as np
import time
import yaml


class JetsonStereoBasic:
    def __init__(self, calibration_file, left_id=0, right_id=1):
        """
        Jetson 기본 스테레오 시스템
        
        Parameters:
        - calibration_file: 캘리브레이션 파일
        - left_id, right_id: 카메라 인덱스
        """
        
        # 캘리브레이션 로드
        self.load_calibration(calibration_file)
        
        # GStreamer 파이프라인 (Jetson 최적화)
        self.cap_left = self.create_camera(left_id)
        self.cap_right = self.create_camera(right_id)
        
        # SGBM 매처 (OpenCV CUDA 사용)
        self.setup_stereo_matcher()
        
        # FPS 계산
        self.fps_history = []
        
    def create_camera(self, camera_id, width=1280, height=720, fps=30):
        """GStreamer 파이프라인으로 카메라 생성"""
        
        # USB 카메라용 GStreamer 파이프라인
        gst_pipeline = (
            f"v4l2src device=/dev/video{camera_id} ! "
            f"video/x-raw, width={width}, height={height}, framerate={fps}/1 ! "
            f"videoconvert ! "
            f"video/x-raw, format=BGR ! "
            f"appsink drop=1"
        )
        
        cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
        
        if not cap.isOpened():
            # GStreamer 실패 시 기본 방식
            print(f"GStreamer 실패, 기본 방식 사용 (camera {camera_id})")
            cap = cv2.VideoCapture(camera_id)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            cap.set(cv2.CAP_PROP_FPS, fps)
        
        return cap
    
    def load_calibration(self, filename):
        """캘리브레이션 로드"""
        
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
        self.focal = self.P1[0, 0]
        
        # 정류 맵 생성
        self.map1_left, self.map2_left = cv2.initUndistortRectifyMap(
            self.K1, self.D1, self.R1, self.P1, self.img_size, cv2.CV_32FC1
        )
        self.map1_right, self.map2_right = cv2.initUndistortRectifyMap(
            self.K2, self.D2, self.R2, self.P2, self.img_size, cv2.CV_32FC1
        )
        
        print(f"캘리브레이션 로드: {self.img_size}")
    
    def setup_stereo_matcher(self):
        """스테레오 매처 설정"""
        
        # CUDA 사용 가능 여부 확인
        self.use_cuda = cv2.cuda.getCudaEnabledDeviceCount() > 0
        
        if self.use_cuda:
            print("CUDA 스테레오 매처 사용")
            # CUDA StereoBM
            self.stereo = cv2.cuda.createStereoBM(
                numDisparities=128,
                blockSize=19
            )
        else:
            print("CPU 스테레오 매처 사용")
            self.stereo = cv2.StereoSGBM_create(
                minDisparity=0,
                numDisparities=128,
                blockSize=5,
                P1=8 * 3 * 5 ** 2,
                P2=32 * 3 * 5 ** 2,
                mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
            )
    
    def process_frame(self, frame_left, frame_right):
        """프레임 처리"""
        
        # 정류
        rect_left = cv2.remap(frame_left, self.map1_left, self.map2_left, 
                              cv2.INTER_LINEAR)
        rect_right = cv2.remap(frame_right, self.map1_right, self.map2_right,
                               cv2.INTER_LINEAR)
        
        # 그레이스케일 변환
        gray_left = cv2.cvtColor(rect_left, cv2.COLOR_BGR2GRAY)
        gray_right = cv2.cvtColor(rect_right, cv2.COLOR_BGR2GRAY)
        
        if self.use_cuda:
            # CUDA 처리
            gpu_left = cv2.cuda_GpuMat(gray_left)
            gpu_right = cv2.cuda_GpuMat(gray_right)
            
            gpu_disparity = self.stereo.compute(gpu_left, gpu_right)
            disparity = gpu_disparity.download().astype(np.float32) / 16.0
        else:
            # CPU 처리
            disparity = self.stereo.compute(rect_left, rect_right)
            disparity = disparity.astype(np.float32) / 16.0
        
        # 깊이 계산
        depth = np.zeros_like(disparity)
        valid = disparity > 0
        depth[valid] = (self.focal * self.baseline) / disparity[valid]
        
        return rect_left, disparity, depth
    
    def run(self):
        """메인 루프"""
        
        print("\n" + "="*50)
        print("Jetson 스테레오 비전 시작")
        print("="*50)
        print("Q: 종료")
        
        while True:
            t_start = time.time()
            
            # 캡처
            ret1, frame_left = self.cap_left.read()
            ret2, frame_right = self.cap_right.read()
            
            if not ret1 or not ret2:
                print("프레임 캡처 실패")
                continue
            
            # 처리
            rect_left, disparity, depth = self.process_frame(frame_left, frame_right)
            
            # FPS 계산
            elapsed = time.time() - t_start
            fps = 1.0 / elapsed
            self.fps_history.append(fps)
            if len(self.fps_history) > 30:
                self.fps_history.pop(0)
            avg_fps = np.mean(self.fps_history)
            
            # 시각화
            disp_vis = cv2.normalize(disparity, None, 0, 255, cv2.NORM_MINMAX)
            disp_color = cv2.applyColorMap(disp_vis.astype(np.uint8), cv2.COLORMAP_JET)
            
            # FPS 표시
            cv2.putText(rect_left, f"FPS: {avg_fps:.1f}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            # 중앙 깊이 표시
            h, w = depth.shape
            center_depth = depth[h//2, w//2]
            cv2.putText(rect_left, f"Depth: {center_depth/1000:.2f}m", (10, 70),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
            
            # 디스플레이
            display = cv2.hconcat([
                cv2.resize(rect_left, (640, 360)),
                cv2.resize(disp_color, (640, 360))
            ])
            
            cv2.imshow("Jetson Stereo", display)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        self.cleanup()
    
    def cleanup(self):
        """정리"""
        self.cap_left.release()
        self.cap_right.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    stereo = JetsonStereoBasic("stereo_params.yaml", left_id=0, right_id=1)
    stereo.run()
```

---

## 4. Jetson CUDA 최적화

### 4.1 CUDA 커널 기반 스테레오

```cpp
/**
 * cuda_stereo.cu
 * CUDA 커널을 이용한 스테레오 매칭
 */

#include <cuda_runtime.h>
#include <stdio.h>

// SAD (Sum of Absolute Differences) 계산 커널
__global__ void compute_sad_kernel(
    const unsigned char* left,
    const unsigned char* right,
    float* disparity,
    int width,
    int height,
    int max_disp,
    int block_size
) {
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    
    int half_block = block_size / 2;
    
    // 경계 체크
    if (x < half_block || x >= width - half_block ||
        y < half_block || y >= height - half_block) {
        disparity[y * width + x] = 0;
        return;
    }
    
    int min_sad = INT_MAX;
    int best_d = 0;
    
    // 시차 범위 탐색
    for (int d = 0; d < max_disp && x - d >= half_block; d++) {
        int sad = 0;
        
        // 블록 SAD 계산
        for (int dy = -half_block; dy <= half_block; dy++) {
            for (int dx = -half_block; dx <= half_block; dx++) {
                int ly = y + dy;
                int lx = x + dx;
                int rx = x + dx - d;
                
                int left_val = left[ly * width + lx];
                int right_val = right[ly * width + rx];
                
                sad += abs(left_val - right_val);
            }
        }
        
        if (sad < min_sad) {
            min_sad = sad;
            best_d = d;
        }
    }
    
    disparity[y * width + x] = (float)best_d;
}

// 시차 맵 계산 래퍼
extern "C" void compute_disparity_cuda(
    const unsigned char* d_left,
    const unsigned char* d_right,
    float* d_disparity,
    int width,
    int height,
    int max_disp,
    int block_size
) {
    dim3 block(16, 16);
    dim3 grid((width + block.x - 1) / block.x,
              (height + block.y - 1) / block.y);
    
    compute_sad_kernel<<<grid, block>>>(
        d_left, d_right, d_disparity,
        width, height, max_disp, block_size
    );
    
    cudaDeviceSynchronize();
}

// Census Transform 커널 (더 강건한 매칭)
__global__ void census_transform_kernel(
    const unsigned char* input,
    unsigned long long* census,
    int width,
    int height,
    int window_size
) {
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    
    int half_win = window_size / 2;
    
    if (x < half_win || x >= width - half_win ||
        y < half_win || y >= height - half_win) {
        census[y * width + x] = 0;
        return;
    }
    
    unsigned char center = input[y * width + x];
    unsigned long long result = 0;
    int bit = 0;
    
    for (int dy = -half_win; dy <= half_win; dy++) {
        for (int dx = -half_win; dx <= half_win; dx++) {
            if (dx == 0 && dy == 0) continue;
            
            unsigned char neighbor = input[(y + dy) * width + (x + dx)];
            if (neighbor < center) {
                result |= (1ULL << bit);
            }
            bit++;
        }
    }
    
    census[y * width + x] = result;
}

// Hamming Distance 기반 매칭 커널
__global__ void match_census_kernel(
    const unsigned long long* census_left,
    const unsigned long long* census_right,
    float* disparity,
    int width,
    int height,
    int max_disp
) {
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    
    if (x >= width || y >= height) return;
    
    unsigned long long left_census = census_left[y * width + x];
    
    int min_hamming = 64;  // 최대 비트 수
    int best_d = 0;
    
    for (int d = 0; d < max_disp && x - d >= 0; d++) {
        unsigned long long right_census = census_right[y * width + (x - d)];
        
        // Hamming distance (XOR 후 1 비트 개수)
        unsigned long long xor_val = left_census ^ right_census;
        int hamming = __popcll(xor_val);
        
        if (hamming < min_hamming) {
            min_hamming = hamming;
            best_d = d;
        }
    }
    
    disparity[y * width + x] = (float)best_d;
}
```

### 4.2 Python 바인딩

```python
"""
cuda_stereo_wrapper.py
CUDA 스테레오 Python 래퍼
"""

import numpy as np
import ctypes
from ctypes import c_int, c_float, POINTER, c_void_p
import cv2


class CUDAStereo:
    def __init__(self, max_disp=128, block_size=9):
        """
        CUDA 스테레오 매처
        
        Parameters:
        - max_disp: 최대 시차
        - block_size: 블록 크기
        """
        
        self.max_disp = max_disp
        self.block_size = block_size
        
        # CUDA 라이브러리 로드
        try:
            self.cuda_lib = ctypes.CDLL('./libcuda_stereo.so')
            self.setup_functions()
            self.cuda_available = True
            print("CUDA 스테레오 라이브러리 로드 성공")
        except OSError:
            print("CUDA 라이브러리 로드 실패, CPU 폴백 사용")
            self.cuda_available = False
    
    def setup_functions(self):
        """C 함수 설정"""
        
        # compute_disparity_cuda
        self.cuda_lib.compute_disparity_cuda.argtypes = [
            c_void_p,   # d_left
            c_void_p,   # d_right
            c_void_p,   # d_disparity
            c_int,      # width
            c_int,      # height
            c_int,      # max_disp
            c_int       # block_size
        ]
    
    def compute(self, left, right):
        """
        시차 맵 계산
        
        Parameters:
        - left: 왼쪽 이미지 (그레이스케일)
        - right: 오른쪽 이미지 (그레이스케일)
        
        Returns:
        - disparity: 시차 맵
        """
        
        if not self.cuda_available:
            return self.compute_cpu(left, right)
        
        h, w = left.shape
        
        # GPU 메모리 할당 및 복사 (실제 구현에서는 pycuda 사용)
        # 여기서는 개념적 코드
        
        # ... CUDA 처리 ...
        
        return disparity
    
    def compute_cpu(self, left, right):
        """CPU 폴백"""
        
        stereo = cv2.StereoBM_create(
            numDisparities=self.max_disp,
            blockSize=self.block_size
        )
        
        disparity = stereo.compute(left, right).astype(np.float32) / 16.0
        
        return disparity
```

---

## 5. Jetson VPI 활용

### 5.1 VPI (Vision Programming Interface)

VPI는 NVIDIA에서 제공하는 하드웨어 가속 비전 라이브러리입니다.

```python
"""
jetson_vpi_stereo.py
Jetson VPI를 이용한 스테레오 비전
"""

import vpi
import numpy as np
import cv2
import time


class VPIStereo:
    def __init__(self, width=1280, height=720, max_disp=128):
        """
        VPI 스테레오 시스템
        
        Parameters:
        - width, height: 이미지 크기
        - max_disp: 최대 시차
        """
        
        self.width = width
        self.height = height
        self.max_disp = max_disp
        
        # VPI 백엔드 선택
        # VPI_BACKEND_CUDA: GPU
        # VPI_BACKEND_PVA: 프로그래머블 비전 가속기 (저전력)
        # VPI_BACKEND_OFA: 광학 흐름 가속기
        
        self.backend = vpi.Backend.CUDA
        
        # VPI 이미지 생성
        self.vpi_left = vpi.Image((width, height), vpi.Format.U8)
        self.vpi_right = vpi.Image((width, height), vpi.Format.U8)
        self.vpi_disparity = vpi.Image((width, height), vpi.Format.U16)
        
        # 스테레오 매처 파라미터
        self.stereo_params = vpi.StereoDisparityEstimatorParams()
        self.stereo_params.window_size = 5
        self.stereo_params.max_disparity = max_disp
        
        # 컨피던스 맵 (선택)
        self.vpi_confidence = vpi.Image((width, height), vpi.Format.U16)
        
        print(f"VPI 스테레오 초기화: {width}x{height}, max_disp={max_disp}")
        print(f"백엔드: {self.backend}")
    
    def compute(self, left_gray, right_gray):
        """
        VPI로 시차 맵 계산
        
        Parameters:
        - left_gray: 왼쪽 그레이스케일 이미지
        - right_gray: 오른쪽 그레이스케일 이미지
        
        Returns:
        - disparity: 시차 맵 (float32)
        - confidence: 신뢰도 맵
        """
        
        # numpy → VPI 변환
        with self.vpi_left.lock_cpu() as data:
            data[:] = left_gray
        
        with self.vpi_right.lock_cpu() as data:
            data[:] = right_gray
        
        # 스테레오 매칭 실행
        with vpi.Backend.CUDA:
            vpi.stereo_disparity(
                self.vpi_left,
                self.vpi_right,
                self.vpi_disparity,
                confidence=self.vpi_confidence,
                params=self.stereo_params
            )
        
        # VPI → numpy 변환
        with self.vpi_disparity.lock_cpu() as data:
            # Q10.5 형식 → float
            disparity = data.astype(np.float32) / 32.0
        
        with self.vpi_confidence.lock_cpu() as data:
            confidence = data.astype(np.float32) / 65535.0
        
        return disparity, confidence


class VPIStereoSystem:
    def __init__(self, calibration_file, left_id=0, right_id=1):
        """전체 VPI 스테레오 시스템"""
        
        import yaml
        
        # 캘리브레이션 로드
        with open(calibration_file, 'r') as f:
            params = yaml.safe_load(f)
        
        self.img_size = tuple(params['image_size'])
        self.Q = np.array(params['Q'])
        self.baseline = params['baseline_mm']
        self.focal = params['P1'][0][0]
        
        # 정류 맵
        K1, D1 = np.array(params['K1']), np.array(params['D1'])
        K2, D2 = np.array(params['K2']), np.array(params['D2'])
        R1, R2 = np.array(params['R1']), np.array(params['R2'])
        P1, P2 = np.array(params['P1']), np.array(params['P2'])
        
        self.map1_left, self.map2_left = cv2.initUndistortRectifyMap(
            K1, D1, R1, P1, self.img_size, cv2.CV_32FC1)
        self.map1_right, self.map2_right = cv2.initUndistortRectifyMap(
            K2, D2, R2, P2, self.img_size, cv2.CV_32FC1)
        
        # 카메라
        self.cap_left = cv2.VideoCapture(left_id)
        self.cap_right = cv2.VideoCapture(right_id)
        
        for cap in [self.cap_left, self.cap_right]:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.img_size[0])
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.img_size[1])
        
        # VPI 스테레오
        self.vpi_stereo = VPIStereo(self.img_size[0], self.img_size[1])
        
        self.fps_history = []
    
    def run(self):
        """메인 루프"""
        
        print("\n" + "="*50)
        print("VPI 스테레오 시스템 시작")
        print("="*50)
        
        while True:
            t_start = time.time()
            
            ret1, frame_left = self.cap_left.read()
            ret2, frame_right = self.cap_right.read()
            
            if not ret1 or not ret2:
                continue
            
            # 정류
            rect_left = cv2.remap(frame_left, self.map1_left, self.map2_left,
                                  cv2.INTER_LINEAR)
            rect_right = cv2.remap(frame_right, self.map1_right, self.map2_right,
                                   cv2.INTER_LINEAR)
            
            # 그레이스케일
            gray_left = cv2.cvtColor(rect_left, cv2.COLOR_BGR2GRAY)
            gray_right = cv2.cvtColor(rect_right, cv2.COLOR_BGR2GRAY)
            
            # VPI 스테레오 매칭
            disparity, confidence = self.vpi_stereo.compute(gray_left, gray_right)
            
            # 깊이 계산
            depth = np.zeros_like(disparity)
            valid = disparity > 0
            depth[valid] = (self.focal * self.baseline) / disparity[valid]
            
            # FPS
            elapsed = time.time() - t_start
            fps = 1.0 / elapsed
            self.fps_history.append(fps)
            if len(self.fps_history) > 30:
                self.fps_history.pop(0)
            avg_fps = np.mean(self.fps_history)
            
            # 시각화
            disp_vis = cv2.normalize(disparity, None, 0, 255, cv2.NORM_MINMAX)
            disp_color = cv2.applyColorMap(disp_vis.astype(np.uint8), cv2.COLORMAP_TURBO)
            
            conf_vis = (confidence * 255).astype(np.uint8)
            conf_color = cv2.applyColorMap(conf_vis, cv2.COLORMAP_BONE)
            
            cv2.putText(rect_left, f"FPS: {avg_fps:.1f}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            # 디스플레이
            top = cv2.hconcat([cv2.resize(rect_left, (480, 270)),
                              cv2.resize(disp_color, (480, 270))])
            bottom = cv2.hconcat([cv2.resize(conf_color, (480, 270)),
                                 np.zeros((270, 480, 3), dtype=np.uint8)])
            display = cv2.vconcat([top, bottom])
            
            cv2.imshow("VPI Stereo", display)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        self.cap_left.release()
        self.cap_right.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    system = VPIStereoSystem("stereo_params.yaml", left_id=0, right_id=1)
    system.run()
```

---

## 6. STM32H7 구현

### 6.1 STM32H7 개요

```
┌─────────────────────────────────────────────────────────────┐
│                   STM32H7 스테레오 시스템                     │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ┌─────────────────────────────────────────────────────┐   │
│  │                    STM32H743/H750                    │   │
│  │                                                      │   │
│  │  ┌─────────┐  ┌─────────┐  ┌─────────┐             │   │
│  │  │ Cortex  │  │  DMA2D  │  │ JPEG    │             │   │
│  │  │   M7    │  │ (그래픽)│  │ Codec   │             │   │
│  │  │ 480MHz  │  │         │  │         │             │   │
│  │  └─────────┘  └─────────┘  └─────────┘             │   │
│  │       │                                             │   │
│  │  ┌─────────┐  ┌─────────┐  ┌─────────┐             │   │
│  │  │  DCMI   │  │   RAM   │  │   FMC   │             │   │
│  │  │(카메라) │  │  1MB    │  │(외부RAM)│             │   │
│  │  └─────────┘  └─────────┘  └─────────┘             │   │
│  │                                                      │   │
│  └─────────────────────────────────────────────────────┘   │
│                          │                                  │
│            ┌─────────────┼─────────────┐                   │
│            │             │             │                   │
│            ▼             ▼             ▼                   │
│       ┌────────┐   ┌────────┐   ┌────────┐                │
│       │ Camera │   │ Camera │   │  LCD   │                │
│       │  Left  │   │ Right  │   │Display │                │
│       │(OV7670)│   │(OV7670)│   │        │                │
│       └────────┘   └────────┘   └────────┘                │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### 6.2 CubeMX 설정

```
프로젝트 설정 (STM32H743ZI):

1. System Core
   - RCC: HSE=25MHz, SYSCLK=480MHz
   - DMA: DMA2 Stream0, Stream1 (DCMI용)

2. Connectivity
   - DCMI: 8-bit parallel mode
   - I2C1: 카메라 제어용

3. Middleware
   - 없음 (Bare metal)

4. 클럭 설정
   - HCLK: 240MHz
   - DCMI Clock: 48MHz
```

### 6.3 메인 펌웨어 구조

```c
/**
 * main.c
 * STM32H7 스테레오 비전 메인 펌웨어
 */

#include "main.h"
#include "dcmi.h"
#include "dma.h"
#include "i2c.h"
#include "stereo_matching.h"
#include "ov7670.h"

/* 이미지 버퍼 (외부 SDRAM 또는 내부 RAM) */
#define IMG_WIDTH   160
#define IMG_HEIGHT  120

/* DTCM RAM (빠른 접근) */
__attribute__((section(".dtcmram")))
uint8_t left_image[IMG_HEIGHT][IMG_WIDTH];

__attribute__((section(".dtcmram")))
uint8_t right_image[IMG_HEIGHT][IMG_WIDTH];

/* AXI SRAM */
__attribute__((section(".axisram")))
int16_t disparity_map[IMG_HEIGHT][IMG_WIDTH];

/* 상태 플래그 */
volatile uint8_t left_frame_ready = 0;
volatile uint8_t right_frame_ready = 0;

/* 스테레오 파라미터 */
stereo_params_t stereo_params = {
    .max_disparity = 32,
    .block_size = 5,
    .threshold = 10
};

int main(void)
{
    /* 초기화 */
    HAL_Init();
    SystemClock_Config();
    
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_DCMI_Init();
    MX_I2C1_Init();
    
    /* 카메라 초기화 */
    if (OV7670_Init(&hi2c1, OV7670_ADDR_LEFT) != HAL_OK) {
        Error_Handler();
    }
    
    if (OV7670_Init(&hi2c1, OV7670_ADDR_RIGHT) != HAL_OK) {
        Error_Handler();
    }
    
    /* 해상도 설정 */
    OV7670_SetResolution(IMG_WIDTH, IMG_HEIGHT);
    
    printf("STM32H7 Stereo Vision Started\n");
    printf("Resolution: %dx%d\n", IMG_WIDTH, IMG_HEIGHT);
    printf("Max Disparity: %d\n", stereo_params.max_disparity);
    
    /* 메인 루프 */
    while (1)
    {
        /* 왼쪽 카메라 캡처 */
        Camera_Select(CAMERA_LEFT);
        HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT,
                          (uint32_t)left_image, IMG_WIDTH * IMG_HEIGHT / 4);
        
        while (!left_frame_ready);
        left_frame_ready = 0;
        
        /* 오른쪽 카메라 캡처 */
        Camera_Select(CAMERA_RIGHT);
        HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT,
                          (uint32_t)right_image, IMG_WIDTH * IMG_HEIGHT / 4);
        
        while (!right_frame_ready);
        right_frame_ready = 0;
        
        /* 스테레오 매칭 */
        uint32_t tick_start = HAL_GetTick();
        
        Stereo_BlockMatching(
            (uint8_t*)left_image,
            (uint8_t*)right_image,
            (int16_t*)disparity_map,
            IMG_WIDTH, IMG_HEIGHT,
            &stereo_params
        );
        
        uint32_t elapsed = HAL_GetTick() - tick_start;
        
        /* 결과 출력 */
        printf("Processing time: %lu ms, FPS: %.1f\n", 
               elapsed, 1000.0f / elapsed);
        
        /* 중앙 깊이 계산 */
        int16_t center_disp = disparity_map[IMG_HEIGHT/2][IMG_WIDTH/2];
        if (center_disp > 0) {
            float depth = (FOCAL_LENGTH * BASELINE) / center_disp;
            printf("Center depth: %.0f mm\n", depth);
        }
    }
}

/* DCMI 프레임 완료 콜백 */
void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *hdcmi)
{
    if (current_camera == CAMERA_LEFT) {
        left_frame_ready = 1;
    } else {
        right_frame_ready = 1;
    }
}
```

---

## 7. STM32H7 카메라 인터페이스

### 7.1 OV7670 드라이버

```c
/**
 * ov7670.h
 * OV7670 카메라 드라이버 헤더
 */

#ifndef OV7670_H
#define OV7670_H

#include "stm32h7xx_hal.h"

/* I2C 주소 */
#define OV7670_ADDR_LEFT    0x42
#define OV7670_ADDR_RIGHT   0x43

/* 레지스터 */
#define OV7670_REG_GAIN     0x00
#define OV7670_REG_BLUE     0x01
#define OV7670_REG_RED      0x02
#define OV7670_REG_VREF     0x03
#define OV7670_REG_COM1     0x04
#define OV7670_REG_BAVE     0x05
#define OV7670_REG_COM7     0x12
#define OV7670_REG_COM8     0x13
#define OV7670_REG_COM10    0x15
#define OV7670_REG_HSTART   0x17
#define OV7670_REG_HSTOP    0x18
#define OV7670_REG_VSTART   0x19
#define OV7670_REG_VSTOP    0x1A

/* 함수 프로토타입 */
HAL_StatusTypeDef OV7670_Init(I2C_HandleTypeDef *hi2c, uint8_t addr);
HAL_StatusTypeDef OV7670_SetResolution(uint16_t width, uint16_t height);
HAL_StatusTypeDef OV7670_WriteReg(uint8_t reg, uint8_t value);
HAL_StatusTypeDef OV7670_ReadReg(uint8_t reg, uint8_t *value);

#endif
```

```c
/**
 * ov7670.c
 * OV7670 카메라 드라이버
 */

#include "ov7670.h"

static I2C_HandleTypeDef *hi2c_cam;
static uint8_t current_addr;

/* QVGA 초기화 시퀀스 */
static const uint8_t ov7670_init_regs[][2] = {
    {OV7670_REG_COM7, 0x80},   // 리셋
    {0xFF, 0x64},               // 딜레이
    {OV7670_REG_COM7, 0x00},   // VGA, YUV
    {OV7670_REG_CLKRC, 0x01},  // 클럭 프리스케일러
    {OV7670_REG_COM10, 0x02},  // VSYNC 네거티브
    // ... 추가 설정
    {0xFF, 0xFF}                // 종료
};

HAL_StatusTypeDef OV7670_Init(I2C_HandleTypeDef *hi2c, uint8_t addr)
{
    hi2c_cam = hi2c;
    current_addr = addr;
    
    HAL_StatusTypeDef status;
    
    /* 소프트 리셋 */
    status = OV7670_WriteReg(OV7670_REG_COM7, 0x80);
    if (status != HAL_OK) return status;
    
    HAL_Delay(100);
    
    /* 초기화 시퀀스 적용 */
    for (int i = 0; ov7670_init_regs[i][0] != 0xFF || 
                    ov7670_init_regs[i][1] != 0xFF; i++) {
        
        if (ov7670_init_regs[i][0] == 0xFF) {
            HAL_Delay(ov7670_init_regs[i][1]);
        } else {
            status = OV7670_WriteReg(ov7670_init_regs[i][0],
                                     ov7670_init_regs[i][1]);
            if (status != HAL_OK) return status;
        }
    }
    
    return HAL_OK;
}

HAL_StatusTypeDef OV7670_WriteReg(uint8_t reg, uint8_t value)
{
    uint8_t data[2] = {reg, value};
    return HAL_I2C_Master_Transmit(hi2c_cam, current_addr, data, 2, 100);
}

HAL_StatusTypeDef OV7670_ReadReg(uint8_t reg, uint8_t *value)
{
    HAL_StatusTypeDef status;
    
    status = HAL_I2C_Master_Transmit(hi2c_cam, current_addr, &reg, 1, 100);
    if (status != HAL_OK) return status;
    
    return HAL_I2C_Master_Receive(hi2c_cam, current_addr, value, 1, 100);
}
```

### 7.2 DCMI 설정

```c
/**
 * dcmi.c
 * DCMI 설정 (CubeMX 생성 + 수정)
 */

#include "dcmi.h"

DCMI_HandleTypeDef hdcmi;
DMA_HandleTypeDef hdma_dcmi;

void MX_DCMI_Init(void)
{
    hdcmi.Instance = DCMI;
    hdcmi.Init.SynchroMode = DCMI_SYNCHRO_HARDWARE;
    hdcmi.Init.PCKPolarity = DCMI_PCKPOLARITY_RISING;
    hdcmi.Init.VSPolarity = DCMI_VSPOLARITY_HIGH;
    hdcmi.Init.HSPolarity = DCMI_HSPOLARITY_LOW;
    hdcmi.Init.CaptureRate = DCMI_CR_ALL_FRAME;
    hdcmi.Init.ExtendedDataMode = DCMI_EXTEND_DATA_8B;
    hdcmi.Init.JPEGMode = DCMI_JPEG_DISABLE;
    hdcmi.Init.ByteSelectMode = DCMI_BSM_ALL;
    hdcmi.Init.ByteSelectStart = DCMI_OEBS_ODD;
    hdcmi.Init.LineSelectMode = DCMI_LSM_ALL;
    hdcmi.Init.LineSelectStart = DCMI_OELS_ODD;
    
    if (HAL_DCMI_Init(&hdcmi) != HAL_OK) {
        Error_Handler();
    }
}

/* DMA 설정 */
void HAL_DCMI_MspInit(DCMI_HandleTypeDef* dcmiHandle)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    __HAL_RCC_DCMI_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    
    /* DCMI GPIO 설정 */
    /* D0-D7, HSYNC, VSYNC, PIXCLK */
    
    /* DMA 설정 */
    __HAL_RCC_DMA2_CLK_ENABLE();
    
    hdma_dcmi.Instance = DMA2_Stream1;
    hdma_dcmi.Init.Request = DMA_REQUEST_DCMI;
    hdma_dcmi.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_dcmi.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_dcmi.Init.MemInc = DMA_MINC_ENABLE;
    hdma_dcmi.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_dcmi.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_dcmi.Init.Mode = DMA_NORMAL;
    hdma_dcmi.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_dcmi.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    hdma_dcmi.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    hdma_dcmi.Init.MemBurst = DMA_MBURST_INC4;
    hdma_dcmi.Init.PeriphBurst = DMA_PBURST_SINGLE;
    
    if (HAL_DMA_Init(&hdma_dcmi) != HAL_OK) {
        Error_Handler();
    }
    
    __HAL_LINKDMA(dcmiHandle, DMA_Handle, hdma_dcmi);
    
    /* 인터럽트 */
    HAL_NVIC_SetPriority(DCMI_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DCMI_IRQn);
    
    HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
}
```

---

## 8. STM32H7 스테레오 매칭

### 8.1 최적화된 블록 매칭

```c
/**
 * stereo_matching.h
 * STM32H7 최적화 스테레오 매칭
 */

#ifndef STEREO_MATCHING_H
#define STEREO_MATCHING_H

#include <stdint.h>

typedef struct {
    uint8_t max_disparity;
    uint8_t block_size;
    uint8_t threshold;
} stereo_params_t;

void Stereo_BlockMatching(
    const uint8_t *left,
    const uint8_t *right,
    int16_t *disparity,
    uint16_t width,
    uint16_t height,
    const stereo_params_t *params
);

void Stereo_BlockMatching_SIMD(
    const uint8_t *left,
    const uint8_t *right,
    int16_t *disparity,
    uint16_t width,
    uint16_t height,
    const stereo_params_t *params
);

#endif
```

```c
/**
 * stereo_matching.c
 * STM32H7 최적화 스테레오 매칭 구현
 */

#include "stereo_matching.h"
#include "arm_math.h"
#include <string.h>

/* SAD (Sum of Absolute Differences) 계산 - 기본 */
static inline uint32_t compute_sad(
    const uint8_t *left,
    const uint8_t *right,
    int x, int y, int d,
    int width, int height,
    int half_block
) {
    uint32_t sad = 0;
    
    for (int dy = -half_block; dy <= half_block; dy++) {
        for (int dx = -half_block; dx <= half_block; dx++) {
            int ly = y + dy;
            int lx = x + dx;
            int rx = x + dx - d;
            
            if (rx >= 0) {
                int diff = (int)left[ly * width + lx] - (int)right[ly * width + rx];
                sad += (diff >= 0) ? diff : -diff;
            }
        }
    }
    
    return sad;
}

/* 기본 블록 매칭 */
void Stereo_BlockMatching(
    const uint8_t *left,
    const uint8_t *right,
    int16_t *disparity,
    uint16_t width,
    uint16_t height,
    const stereo_params_t *params
) {
    int half_block = params->block_size / 2;
    
    for (int y = half_block; y < height - half_block; y++) {
        for (int x = half_block; x < width - half_block; x++) {
            
            uint32_t min_sad = UINT32_MAX;
            int best_d = 0;
            
            for (int d = 0; d < params->max_disparity && x - d >= half_block; d++) {
                uint32_t sad = compute_sad(left, right, x, y, d,
                                          width, height, half_block);
                
                if (sad < min_sad) {
                    min_sad = sad;
                    best_d = d;
                }
            }
            
            /* 임계값 체크 */
            if (min_sad < params->threshold * params->block_size * params->block_size) {
                disparity[y * width + x] = best_d;
            } else {
                disparity[y * width + x] = 0;
            }
        }
    }
}

/* SIMD 최적화 버전 (CMSIS-DSP 활용) */
void Stereo_BlockMatching_SIMD(
    const uint8_t *left,
    const uint8_t *right,
    int16_t *disparity,
    uint16_t width,
    uint16_t height,
    const stereo_params_t *params
) {
    int half_block = params->block_size / 2;
    int block_size = params->block_size;
    
    /* 임시 버퍼 (스택 또는 DTCM) */
    int16_t left_block[11 * 11];   // 최대 11x11
    int16_t right_block[11 * 11];
    int16_t diff_block[11 * 11];
    
    for (int y = half_block; y < height - half_block; y++) {
        for (int x = half_block; x < width - half_block; x++) {
            
            /* 왼쪽 블록 추출 */
            int idx = 0;
            for (int dy = -half_block; dy <= half_block; dy++) {
                for (int dx = -half_block; dx <= half_block; dx++) {
                    left_block[idx++] = left[(y + dy) * width + (x + dx)];
                }
            }
            
            uint32_t min_sad = UINT32_MAX;
            int best_d = 0;
            
            for (int d = 0; d < params->max_disparity && x - d >= half_block; d++) {
                
                /* 오른쪽 블록 추출 */
                idx = 0;
                for (int dy = -half_block; dy <= half_block; dy++) {
                    for (int dx = -half_block; dx <= half_block; dx++) {
                        right_block[idx++] = right[(y + dy) * width + (x + dx - d)];
                    }
                }
                
                /* SIMD 뺄셈 */
                arm_sub_q15(left_block, right_block, diff_block, block_size * block_size);
                
                /* SIMD 절대값 합 */
                uint32_t sad = 0;
                for (int i = 0; i < block_size * block_size; i++) {
                    sad += (diff_block[i] >= 0) ? diff_block[i] : -diff_block[i];
                }
                
                if (sad < min_sad) {
                    min_sad = sad;
                    best_d = d;
                }
            }
            
            disparity[y * width + x] = best_d;
        }
    }
}

/* Census Transform 기반 매칭 (더 강건) */
static inline uint32_t census_transform(
    const uint8_t *img,
    int x, int y,
    int width,
    int half_win
) {
    uint8_t center = img[y * width + x];
    uint32_t census = 0;
    int bit = 0;
    
    for (int dy = -half_win; dy <= half_win; dy++) {
        for (int dx = -half_win; dx <= half_win; dx++) {
            if (dx == 0 && dy == 0) continue;
            
            if (img[(y + dy) * width + (x + dx)] < center) {
                census |= (1U << bit);
            }
            bit++;
        }
    }
    
    return census;
}

/* Hamming Distance */
static inline uint8_t hamming_distance(uint32_t a, uint32_t b)
{
    uint32_t xor_val = a ^ b;
    
    /* Population count (비트 수 세기) */
    xor_val = xor_val - ((xor_val >> 1) & 0x55555555);
    xor_val = (xor_val & 0x33333333) + ((xor_val >> 2) & 0x33333333);
    xor_val = (xor_val + (xor_val >> 4)) & 0x0F0F0F0F;
    xor_val = xor_val + (xor_val >> 8);
    xor_val = xor_val + (xor_val >> 16);
    
    return xor_val & 0x3F;
}

void Stereo_CensusMatching(
    const uint8_t *left,
    const uint8_t *right,
    int16_t *disparity,
    uint16_t width,
    uint16_t height,
    const stereo_params_t *params
) {
    int half_win = 2;  // 5x5 윈도우 (24비트 Census)
    int margin = half_win + params->max_disparity;
    
    for (int y = margin; y < height - margin; y++) {
        for (int x = margin; x < width - margin; x++) {
            
            uint32_t left_census = census_transform(left, x, y, width, half_win);
            
            uint8_t min_hamming = 255;
            int best_d = 0;
            
            for (int d = 0; d < params->max_disparity; d++) {
                uint32_t right_census = census_transform(right, x - d, y, width, half_win);
                
                uint8_t ham = hamming_distance(left_census, right_census);
                
                if (ham < min_hamming) {
                    min_hamming = ham;
                    best_d = d;
                }
            }
            
            disparity[y * width + x] = best_d;
        }
    }
}
```

---

## 9. 성능 최적화 기법

### 9.1 Jetson 최적화 체크리스트

```
┌─────────────────────────────────────────────────────────────┐
│                  Jetson 최적화 체크리스트                    │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ☐ 전력 모드 설정                                           │
│    sudo nvpmodel -m 0  # 최대 성능                          │
│    sudo jetson_clocks  # 클럭 고정                          │
│                                                             │
│  ☐ CUDA 메모리 관리                                         │
│    - Unified Memory 사용 최소화                             │
│    - 비동기 메모리 전송                                      │
│    - 메모리 풀 사용                                         │
│                                                             │
│  ☐ GStreamer 파이프라인                                     │
│    - nvvidconv 사용 (하드웨어 색공간 변환)                   │
│    - nvjpegdec 사용 (하드웨어 JPEG 디코딩)                  │
│                                                             │
│  ☐ VPI 활용                                                 │
│    - PVA 백엔드 (저전력)                                    │
│    - CUDA 백엔드 (고성능)                                   │
│                                                             │
│  ☐ 해상도 최적화                                            │
│    - 필요 최소 해상도 사용                                   │
│    - 관심 영역(ROI)만 처리                                  │
│                                                             │
│  ☐ 배치 처리                                                │
│    - 여러 프레임 동시 처리                                   │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### 9.2 STM32 최적화 체크리스트

```
┌─────────────────────────────────────────────────────────────┐
│                  STM32 최적화 체크리스트                     │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ☐ 메모리 배치                                              │
│    - DTCM: 자주 접근하는 데이터                             │
│    - ITCM: 중요한 함수 코드                                 │
│    - AXI SRAM: 대용량 버퍼                                  │
│                                                             │
│  ☐ DMA 활용                                                 │
│    - DCMI → 메모리 DMA 전송                                │
│    - 더블 버퍼링                                            │
│                                                             │
│  ☐ 캐시 최적화                                              │
│    - D-Cache 활성화                                         │
│    - I-Cache 활성화                                         │
│    - 캐시 라인 정렬                                         │
│                                                             │
│  ☐ SIMD 명령어                                              │
│    - CMSIS-DSP 활용                                         │
│    - __SADD8, __USAD8 등                                   │
│                                                             │
│  ☐ 루프 최적화                                              │
│    - 루프 언롤링                                            │
│    - 분기 최소화                                            │
│                                                             │
│  ☐ 컴파일러 최적화                                          │
│    - -O3 또는 -Ofast                                       │
│    - LTO (Link Time Optimization)                          │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### 9.3 성능 벤치마크

```c
/**
 * benchmark.c
 * 성능 벤치마크
 */

#include "benchmark.h"
#include "stereo_matching.h"

void run_benchmark(void)
{
    printf("\n=== 스테레오 매칭 벤치마크 ===\n");
    
    stereo_params_t params = {
        .max_disparity = 32,
        .block_size = 5,
        .threshold = 10
    };
    
    uint32_t times[3];
    
    /* 기본 블록 매칭 */
    uint32_t start = DWT->CYCCNT;
    Stereo_BlockMatching(left_image, right_image, disparity_map,
                        IMG_WIDTH, IMG_HEIGHT, &params);
    times[0] = DWT->CYCCNT - start;
    
    /* SIMD 블록 매칭 */
    start = DWT->CYCCNT;
    Stereo_BlockMatching_SIMD(left_image, right_image, disparity_map,
                              IMG_WIDTH, IMG_HEIGHT, &params);
    times[1] = DWT->CYCCNT - start;
    
    /* Census 매칭 */
    start = DWT->CYCCNT;
    Stereo_CensusMatching(left_image, right_image, disparity_map,
                          IMG_WIDTH, IMG_HEIGHT, &params);
    times[2] = DWT->CYCCNT - start;
    
    /* 결과 출력 */
    float cpu_freq = SystemCoreClock / 1000000.0f;
    
    printf("해상도: %dx%d\n", IMG_WIDTH, IMG_HEIGHT);
    printf("최대 시차: %d\n", params.max_disparity);
    printf("블록 크기: %d\n\n", params.block_size);
    
    printf("%-20s %12s %12s\n", "알고리즘", "사이클", "시간(ms)");
    printf("%-20s %12lu %12.2f\n", "기본 BM", times[0], times[0] / cpu_freq / 1000);
    printf("%-20s %12lu %12.2f\n", "SIMD BM", times[1], times[1] / cpu_freq / 1000);
    printf("%-20s %12lu %12.2f\n", "Census", times[2], times[2] / cpu_freq / 1000);
    
    printf("\nSIMD 속도 향상: %.1fx\n", (float)times[0] / times[1]);
}
```

---

## 10. 실습 프로젝트

### 10.1 Jetson 완전한 시스템

```python
"""
jetson_complete_system.py
Jetson 완전한 스테레오 비전 시스템
"""

import cv2
import numpy as np
import time
import yaml
import argparse

try:
    import vpi
    VPI_AVAILABLE = True
except ImportError:
    VPI_AVAILABLE = False
    print("VPI 사용 불가, OpenCV 사용")


class JetsonStereoSystem:
    def __init__(self, config_file):
        """완전한 Jetson 스테레오 시스템"""
        
        # 설정 로드
        with open(config_file, 'r') as f:
            self.config = yaml.safe_load(f)
        
        # 캘리브레이션
        self.load_calibration(self.config['calibration_file'])
        
        # 카메라 초기화
        self.init_cameras()
        
        # 스테레오 매처 초기화
        self.init_stereo_matcher()
        
        # 성능 모니터링
        self.fps_history = []
        self.latency_history = []
    
    def load_calibration(self, filename):
        """캘리브레이션 로드"""
        
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
        self.focal = self.P1[0, 0]
        
        # 정류 맵
        self.map1_left, self.map2_left = cv2.initUndistortRectifyMap(
            self.K1, self.D1, self.R1, self.P1, self.img_size, cv2.CV_32FC1)
        self.map1_right, self.map2_right = cv2.initUndistortRectifyMap(
            self.K2, self.D2, self.R2, self.P2, self.img_size, cv2.CV_32FC1)
    
    def init_cameras(self):
        """카메라 초기화"""
        
        cam_config = self.config['camera']
        
        self.cap_left = cv2.VideoCapture(cam_config['left_id'])
        self.cap_right = cv2.VideoCapture(cam_config['right_id'])
        
        for cap in [self.cap_left, self.cap_right]:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, cam_config['width'])
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cam_config['height'])
            cap.set(cv2.CAP_PROP_FPS, cam_config['fps'])
    
    def init_stereo_matcher(self):
        """스테레오 매처 초기화"""
        
        stereo_config = self.config['stereo']
        self.use_vpi = stereo_config.get('use_vpi', False) and VPI_AVAILABLE
        
        if self.use_vpi:
            self.stereo = vpi.StereoDisparityEstimator(
                self.img_size[0], self.img_size[1],
                max_disparity=stereo_config['max_disparity']
            )
            print("VPI 스테레오 매처 사용")
        else:
            self.stereo = cv2.StereoSGBM_create(
                minDisparity=0,
                numDisparities=stereo_config['max_disparity'],
                blockSize=stereo_config['block_size'],
                P1=stereo_config['p1'],
                P2=stereo_config['p2'],
                mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
            )
            print("OpenCV SGBM 사용")
    
    def process(self, frame_left, frame_right):
        """프레임 처리"""
        
        # 정류
        rect_left = cv2.remap(frame_left, self.map1_left, self.map2_left,
                              cv2.INTER_LINEAR)
        rect_right = cv2.remap(frame_right, self.map1_right, self.map2_right,
                               cv2.INTER_LINEAR)
        
        # 그레이스케일
        gray_left = cv2.cvtColor(rect_left, cv2.COLOR_BGR2GRAY)
        gray_right = cv2.cvtColor(rect_right, cv2.COLOR_BGR2GRAY)
        
        # 시차 계산
        if self.use_vpi:
            # VPI
            disparity = self.stereo.compute(gray_left, gray_right)
        else:
            # OpenCV
            disparity = self.stereo.compute(rect_left, rect_right)
            disparity = disparity.astype(np.float32) / 16.0
        
        # 깊이 계산
        depth = np.zeros_like(disparity)
        valid = disparity > 0
        depth[valid] = (self.focal * self.baseline) / disparity[valid]
        
        return rect_left, disparity, depth
    
    def run(self):
        """메인 루프"""
        
        print("\n" + "="*60)
        print("Jetson 스테레오 비전 시스템")
        print("="*60)
        print("Q: 종료 | S: 스크린샷 | P: 성능 정보")
        print("="*60)
        
        while True:
            t_start = time.time()
            
            # 캡처
            ret1, frame_left = self.cap_left.read()
            ret2, frame_right = self.cap_right.read()
            
            if not ret1 or not ret2:
                continue
            
            # 처리
            rect_left, disparity, depth = self.process(frame_left, frame_right)
            
            # 성능 계산
            latency = (time.time() - t_start) * 1000
            fps = 1000.0 / latency
            
            self.fps_history.append(fps)
            self.latency_history.append(latency)
            if len(self.fps_history) > 100:
                self.fps_history.pop(0)
                self.latency_history.pop(0)
            
            # 시각화
            display = self.visualize(rect_left, disparity, depth)
            
            cv2.imshow("Jetson Stereo", display)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                self.save_screenshot(rect_left, disparity, depth)
            elif key == ord('p'):
                self.print_performance()
        
        self.cleanup()
    
    def visualize(self, rect_left, disparity, depth):
        """시각화"""
        
        h, w = rect_left.shape[:2]
        
        # 시차 컬러맵
        disp_vis = cv2.normalize(disparity, None, 0, 255, cv2.NORM_MINMAX)
        disp_color = cv2.applyColorMap(disp_vis.astype(np.uint8), cv2.COLORMAP_TURBO)
        
        # 깊이 컬러맵
        depth_clipped = np.clip(depth, 0, 5000)
        depth_vis = (255 - depth_clipped / 5000 * 255).astype(np.uint8)
        depth_color = cv2.applyColorMap(depth_vis, cv2.COLORMAP_MAGMA)
        
        # 정보 오버레이
        avg_fps = np.mean(self.fps_history) if self.fps_history else 0
        avg_latency = np.mean(self.latency_history) if self.latency_history else 0
        
        cv2.putText(rect_left, f"FPS: {avg_fps:.1f}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(rect_left, f"Latency: {avg_latency:.1f}ms", (10, 70),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        # 중앙 깊이
        center_depth = depth[h//2, w//2]
        cv2.putText(rect_left, f"Depth: {center_depth/1000:.2f}m", (10, 110),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
        
        # 십자선
        cv2.drawMarker(rect_left, (w//2, h//2), (0, 255, 0),
                      cv2.MARKER_CROSS, 30, 2)
        
        # 조합
        top = cv2.hconcat([cv2.resize(rect_left, (480, 270)),
                          cv2.resize(disp_color, (480, 270))])
        bottom = cv2.hconcat([cv2.resize(depth_color, (480, 270)),
                             np.zeros((270, 480, 3), dtype=np.uint8)])
        
        return cv2.vconcat([top, bottom])
    
    def save_screenshot(self, rect_left, disparity, depth):
        """스크린샷 저장"""
        
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        cv2.imwrite(f"screenshot_{timestamp}_left.png", rect_left)
        
        disp_vis = cv2.normalize(disparity, None, 0, 255, cv2.NORM_MINMAX)
        cv2.imwrite(f"screenshot_{timestamp}_disparity.png", disp_vis.astype(np.uint8))
        
        np.save(f"screenshot_{timestamp}_depth.npy", depth)
        
        print(f"스크린샷 저장됨: screenshot_{timestamp}_*")
    
    def print_performance(self):
        """성능 정보 출력"""
        
        print("\n" + "="*40)
        print("성능 정보")
        print("="*40)
        print(f"평균 FPS: {np.mean(self.fps_history):.1f}")
        print(f"평균 지연: {np.mean(self.latency_history):.1f} ms")
        print(f"최소 FPS: {np.min(self.fps_history):.1f}")
        print(f"최대 FPS: {np.max(self.fps_history):.1f}")
        print("="*40)
    
    def cleanup(self):
        """정리"""
        self.cap_left.release()
        self.cap_right.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', default='jetson_config.yaml')
    args = parser.parse_args()
    
    system = JetsonStereoSystem(args.config)
    system.run()
```

---

## 📝 학습 체크리스트

### 이론 이해

- [ ] 임베디드 스테레오의 제약사항을 이해했다
- [ ] Jetson과 STM32의 차이점을 설명할 수 있다
- [ ] CUDA/VPI 가속의 원리를 이해했다
- [ ] 메모리 최적화 기법을 알고 있다

### 실습 완료 (Jetson)

- [ ] Jetson 환경 설정
- [ ] 기본 스테레오 구현
- [ ] CUDA 가속 적용
- [ ] VPI 활용
- [ ] 성능 최적화

### 실습 완료 (STM32)

- [ ] CubeMX 프로젝트 설정
- [ ] DCMI 카메라 인터페이스
- [ ] 기본 블록 매칭 구현
- [ ] SIMD 최적화
- [ ] 성능 벤치마크

---

## 📄 라이선스

MIT License - 자유롭게 사용, 수정, 배포 가능
