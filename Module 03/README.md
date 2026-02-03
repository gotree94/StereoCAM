# Module 03: 스테레오 매칭 & 깊이 추정

[![난이도](https://img.shields.io/badge/난이도-⭐⭐⭐_중급-yellow.svg)]()
[![예상시간](https://img.shields.io/badge/예상시간-12--16시간-blue.svg)]()
[![선수지식](https://img.shields.io/badge/선수지식-Module_02_완료-orange.svg)]()

---

## 📋 모듈 개요

| 항목 | 내용 |
|------|------|
| **학습 목표** | 스테레오 매칭 알고리즘 이해 및 깊이 맵 생성 |
| **핵심 키워드** | Block Matching, SGBM, Disparity Map, Depth Map |
| **산출물** | 실시간 깊이 추정 프로그램, 최적화된 파라미터 설정 |

---

## 📚 목차

1. [스테레오 매칭 개요](#1-스테레오-매칭-개요) : 파이프라인 흐름도, 알고리즘 비교, 비용 함수
2. [Block Matching (BM)](#2-block-matching-bm) : OpenCV StereoBM 사용법, 파라미터 설명
3. [Semi-Global Block Matching (SGBM)](#3-semi-global-block-matching-sgbm) : 에너지 함수, OpenCV StereoSGBM, BM vs SGBM 비교
4. [Disparity Map 후처리](#4-disparity-map-후처리) : WLS 필터, Bilateral 필터, Median 필터, 홀 채우기
5. [Depth Map 변환](#5-depth-map-변환) : 시차→깊이 공식, Q 행렬 사용, 시각화
6. [파라미터 튜닝 가이드](#6-파라미터-튜닝-가이드) : GUI 튜너, 튜닝 가이드라인
7. [실시간 깊이 추정](#7-실시간-깊이-추정) : 완전한 실시간 시스템 코드
8. [성능 최적화](#8-성능-최적화) : 해상도 트레이드오프, 최적화 기법
9. [실습 코드](#9-실습-코드) : 전체 파이프라인 데모
10. [트러블슈팅](#10-트러블슈팅) : 일반적인 문제와 해결책


📁 포함된 코드
   * stereo_bm.py - Block Matching 매처 클래스
   * stereo_sgbm.py - SGBM 매처 클래스
   * disparity_filter.py - 시차 맵 후처리 필터들
   * disparity_to_depth.py - 깊이 변환 및 시각화
   * parameter_tuning_gui.py - 파라미터 튜닝 GUI
   * realtime_depth.py - 실시간 깊이 추정 시스템
   * optimization_tips.py - 성능 최적화 기법
   * stereo_matching_demo.py - 전체 데모

---

## 1. 스테레오 매칭 개요

### 1.1 스테레오 매칭이란?

정류된 스테레오 이미지에서 **대응점(corresponding points)**을 찾아 **시차(disparity)**를 계산하는 과정입니다.

```
┌─────────────────────────────────────────────────────────────┐
│                    스테레오 매칭 파이프라인                    │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│   정류된 왼쪽 이미지        정류된 오른쪽 이미지               │
│   ┌─────────────┐          ┌─────────────┐                 │
│   │ ████████    │          │   ████████  │                 │
│   │ ████████    │          │   ████████  │                 │
│   └─────────────┘          └─────────────┘                 │
│          │                        │                        │
│          └──────────┬─────────────┘                        │
│                     ▼                                      │
│              ┌─────────────┐                               │
│              │ 스테레오     │                               │
│              │ 매칭 알고리즘 │                               │
│              └─────────────┘                               │
│                     │                                      │
│                     ▼                                      │
│              ┌─────────────┐                               │
│              │ Disparity   │  시차 = 왼쪽x - 오른쪽x        │
│              │ Map         │  (픽셀 단위)                   │
│              └─────────────┘                               │
│                     │                                      │
│                     ▼                                      │
│              ┌─────────────┐                               │
│              │ Depth Map   │  깊이 = f × B / disparity     │
│              │             │  (미터 단위)                   │
│              └─────────────┘                               │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### 1.2 매칭 알고리즘 비교

| 알고리즘 | 속도 | 품질 | 특징 | 용도 |
|----------|------|------|------|------|
| **BM** | ⭐⭐⭐⭐⭐ | ⭐⭐ | 단순, 빠름 | 실시간, 임베디드 |
| **SGBM** | ⭐⭐⭐ | ⭐⭐⭐⭐ | 균형 잡힌 성능 | 범용 |
| **Graph Cut** | ⭐ | ⭐⭐⭐⭐⭐ | 최고 품질 | 오프라인 처리 |
| **Deep Learning** | ⭐⭐ (GPU) | ⭐⭐⭐⭐⭐ | 최신 기술 | 고정밀 응용 |

### 1.3 매칭 비용 함수

두 픽셀이 얼마나 유사한지 측정하는 함수:

```
1. SAD (Sum of Absolute Differences)
   SAD = Σ|IL(x,y) - IR(x-d,y)|
   
2. SSD (Sum of Squared Differences)  
   SSD = Σ(IL(x,y) - IR(x-d,y))²
   
3. NCC (Normalized Cross Correlation)
   NCC = Σ(IL - μL)(IR - μR) / (σL × σR)

여기서:
- IL, IR: 왼쪽/오른쪽 이미지 픽셀값
- d: 시차 (disparity)
- μ, σ: 평균, 표준편차
```

---

## 2. Block Matching (BM)

### 2.1 알고리즘 원리

```
왼쪽 이미지                     오른쪽 이미지
┌───────────────────┐          ┌───────────────────┐
│                   │          │                   │
│    ┌─────┐        │          │ ┌─────┐           │
│    │Block│        │   검색    │ │ ? ? │← 검색 범위 │
│    │ 15  │────────┼────→     │ └─────┘           │
│    └─────┘        │          │                   │
│                   │          │                   │
└───────────────────┘          └───────────────────┘

1. 왼쪽 이미지에서 블록(윈도우) 선택
2. 오른쪽 이미지의 같은 행에서 가장 유사한 블록 검색
3. 위치 차이 = 시차 (disparity)
```

### 2.2 OpenCV StereoBM

```python
"""
stereo_bm.py
Block Matching 알고리즘을 이용한 시차 맵 생성
"""

import cv2
import numpy as np


class StereoBM_Matcher:
    def __init__(self):
        """Block Matching 매처 초기화"""
        
        # StereoBM 생성
        self.stereo = cv2.StereoBM_create()
        
        # 기본 파라미터 설정
        self.set_default_params()
        
    def set_default_params(self):
        """기본 파라미터 설정"""
        
        # numDisparities: 최대 시차 (16의 배수)
        # - 클수록 더 가까운 물체 감지 가능
        # - 계산량 증가
        self.stereo.setNumDisparities(128)
        
        # blockSize: 매칭 블록 크기 (홀수, 5~255)
        # - 클수록 노이즈 감소, 디테일 손실
        # - 작을수록 디테일 보존, 노이즈 증가
        self.stereo.setBlockSize(15)
        
        # preFilterType: 전처리 필터 타입
        # - PREFILTER_NORMALIZED_RESPONSE (0): 정규화 응답
        # - PREFILTER_XSOBEL (1): X-Sobel 필터
        self.stereo.setPreFilterType(cv2.STEREO_BM_PREFILTER_NORMALIZED_RESPONSE)
        
        # preFilterSize: 전처리 필터 크기 (홀수, 5~255)
        self.stereo.setPreFilterSize(9)
        
        # preFilterCap: 전처리 필터 캡 (1~63)
        self.stereo.setPreFilterCap(31)
        
        # textureThreshold: 텍스처 임계값
        # - 텍스처가 부족한 영역 필터링
        self.stereo.setTextureThreshold(10)
        
        # uniquenessRatio: 유일성 비율 (0~100)
        # - 최적 매칭이 차선보다 얼마나 좋아야 하는지
        # - 높을수록 신뢰도 높은 매칭만 남음
        self.stereo.setUniquenessRatio(15)
        
        # speckleWindowSize: 스페클 필터 윈도우 크기
        # - 작은 노이즈 영역 제거
        self.stereo.setSpeckleWindowSize(100)
        
        # speckleRange: 스페클 필터 범위 (1~2 권장)
        self.stereo.setSpeckleRange(2)
        
        # minDisparity: 최소 시차 (보통 0)
        self.stereo.setMinDisparity(0)
        
    def compute(self, img_left, img_right):
        """
        시차 맵 계산
        
        Parameters:
        - img_left: 정류된 왼쪽 이미지 (그레이스케일)
        - img_right: 정류된 오른쪽 이미지 (그레이스케일)
        
        Returns:
        - disparity: 시차 맵 (float32, 실제 시차값)
        """
        
        # 그레이스케일 변환
        if len(img_left.shape) == 3:
            img_left = cv2.cvtColor(img_left, cv2.COLOR_BGR2GRAY)
        if len(img_right.shape) == 3:
            img_right = cv2.cvtColor(img_right, cv2.COLOR_BGR2GRAY)
        
        # 시차 계산 (16배 스케일된 정수로 반환)
        disparity_raw = self.stereo.compute(img_left, img_right)
        
        # 실제 시차값으로 변환 (16으로 나눔)
        disparity = disparity_raw.astype(np.float32) / 16.0
        
        return disparity
    
    def get_params(self):
        """현재 파라미터 반환"""
        return {
            'numDisparities': self.stereo.getNumDisparities(),
            'blockSize': self.stereo.getBlockSize(),
            'preFilterType': self.stereo.getPreFilterType(),
            'preFilterSize': self.stereo.getPreFilterSize(),
            'preFilterCap': self.stereo.getPreFilterCap(),
            'textureThreshold': self.stereo.getTextureThreshold(),
            'uniquenessRatio': self.stereo.getUniquenessRatio(),
            'speckleWindowSize': self.stereo.getSpeckleWindowSize(),
            'speckleRange': self.stereo.getSpeckleRange(),
            'minDisparity': self.stereo.getMinDisparity()
        }


def visualize_disparity(disparity, colormap=cv2.COLORMAP_JET):
    """
    시차 맵 시각화
    
    Parameters:
    - disparity: 시차 맵 (float32)
    - colormap: 컬러맵
    
    Returns:
    - disparity_color: 컬러 시차 맵 (BGR)
    """
    
    # 유효한 시차만 사용
    valid_mask = disparity > 0
    
    if not np.any(valid_mask):
        return np.zeros((*disparity.shape, 3), dtype=np.uint8)
    
    # 정규화 (0-255)
    disp_min = disparity[valid_mask].min()
    disp_max = disparity[valid_mask].max()
    
    disparity_norm = np.zeros_like(disparity)
    disparity_norm[valid_mask] = (disparity[valid_mask] - disp_min) / (disp_max - disp_min) * 255
    disparity_norm = disparity_norm.astype(np.uint8)
    
    # 컬러맵 적용
    disparity_color = cv2.applyColorMap(disparity_norm, colormap)
    
    # 유효하지 않은 영역은 검정색
    disparity_color[~valid_mask] = [0, 0, 0]
    
    return disparity_color


if __name__ == "__main__":
    # 테스트
    matcher = StereoBM_Matcher()
    print("StereoBM 파라미터:")
    for k, v in matcher.get_params().items():
        print(f"  {k}: {v}")
```

### 2.3 BM 파라미터 설명

| 파라미터 | 범위 | 기본값 | 설명 |
|----------|------|--------|------|
| `numDisparities` | 16~256 (16배수) | 128 | 검색할 최대 시차 |
| `blockSize` | 5~255 (홀수) | 15 | 매칭 블록 크기 |
| `preFilterCap` | 1~63 | 31 | 전처리 클리핑 값 |
| `uniquenessRatio` | 0~100 | 15 | 매칭 유일성 비율 |
| `speckleWindowSize` | 0~200 | 100 | 노이즈 제거 윈도우 |
| `speckleRange` | 1~2 | 2 | 노이즈 제거 범위 |

---

## 3. Semi-Global Block Matching (SGBM)

### 3.1 알고리즘 원리

SGBM은 BM의 로컬 매칭에 **전역 최적화**를 추가합니다.

```
                    경로 1 (↓)
                        │
                        ▼
        경로 8 (↘)  ┌─────────┐  경로 2 (↙)
              ───→  │ 현재    │ ←───
                    │ 픽셀    │
              ←───  │         │ ───→
        경로 7 (↗)  └─────────┘  경로 3 (↖)
                        ▲
                        │
                    경로 6 (↑)
                    
5개 또는 8개 방향에서 에너지 함수 최적화
→ 일관된 시차 맵 생성
```

### 3.2 에너지 함수

```
E(D) = Σ[ C(p, Dp) + Σ P1·T(|Dp - Dq| = 1) + Σ P2·T(|Dp - Dq| > 1) ]

여기서:
- C(p, Dp): 픽셀 p에서 시차 Dp의 매칭 비용
- P1: 시차가 1 다를 때 페널티 (부드러운 표면)
- P2: 시차가 1 이상 다를 때 페널티 (경계)
- T(): 조건이 참이면 1, 거짓이면 0
```

### 3.3 OpenCV StereoSGBM

```python
"""
stereo_sgbm.py
Semi-Global Block Matching 알고리즘
"""

import cv2
import numpy as np


class StereoSGBM_Matcher:
    def __init__(self):
        """SGBM 매처 초기화"""
        
        # StereoSGBM 생성
        self.stereo = cv2.StereoSGBM_create()
        
        # 기본 파라미터 설정
        self.set_default_params()
        
    def set_default_params(self):
        """기본 파라미터 설정"""
        
        # minDisparity: 최소 시차
        self.stereo.setMinDisparity(0)
        
        # numDisparities: 최대 시차 (16의 배수)
        self.stereo.setNumDisparities(128)
        
        # blockSize: 매칭 블록 크기 (홀수, 1~11 권장)
        # SGBM은 BM보다 작은 블록 사용 가능
        self.stereo.setBlockSize(5)
        
        # P1: 시차가 1 다를 때 페널티
        # 권장: 8 * channels * blockSize²
        channels = 3
        block_size = 5
        self.stereo.setP1(8 * channels * block_size ** 2)
        
        # P2: 시차가 1 이상 다를 때 페널티
        # 권장: 32 * channels * blockSize², P1보다 커야 함
        self.stereo.setP2(32 * channels * block_size ** 2)
        
        # disp12MaxDiff: 좌우 일관성 검사 최대 차이 (-1: 비활성화)
        self.stereo.setDisp12MaxDiff(1)
        
        # preFilterCap: 전처리 필터 캡 (1~63)
        self.stereo.setPreFilterCap(63)
        
        # uniquenessRatio: 유일성 비율 (5~15 권장)
        self.stereo.setUniquenessRatio(10)
        
        # speckleWindowSize: 스페클 필터 윈도우
        self.stereo.setSpeckleWindowSize(100)
        
        # speckleRange: 스페클 필터 범위
        self.stereo.setSpeckleRange(2)
        
        # mode: SGBM 모드
        # - STEREO_SGBM_MODE_SGBM (0): 8방향 (품질↑, 속도↓)
        # - STEREO_SGBM_MODE_HH (1): 8방향 + 전체 DP (최고 품질)
        # - STEREO_SGBM_MODE_SGBM_3WAY (2): 5방향 (속도↑)
        # - STEREO_SGBM_MODE_HH4 (3): 4방향 (속도↑↑)
        self.stereo.setMode(cv2.STEREO_SGBM_MODE_SGBM)
        
    def set_params_for_quality(self):
        """품질 우선 파라미터"""
        self.stereo.setBlockSize(5)
        self.stereo.setP1(8 * 3 * 5 ** 2)
        self.stereo.setP2(32 * 3 * 5 ** 2)
        self.stereo.setMode(cv2.STEREO_SGBM_MODE_HH)
        self.stereo.setUniquenessRatio(5)
        
    def set_params_for_speed(self):
        """속도 우선 파라미터"""
        self.stereo.setBlockSize(7)
        self.stereo.setP1(8 * 3 * 7 ** 2)
        self.stereo.setP2(32 * 3 * 7 ** 2)
        self.stereo.setMode(cv2.STEREO_SGBM_MODE_SGBM_3WAY)
        self.stereo.setUniquenessRatio(15)
        
    def compute(self, img_left, img_right):
        """
        시차 맵 계산
        
        Parameters:
        - img_left: 정류된 왼쪽 이미지 (컬러 또는 그레이)
        - img_right: 정류된 오른쪽 이미지
        
        Returns:
        - disparity: 시차 맵 (float32)
        """
        
        # 시차 계산
        disparity_raw = self.stereo.compute(img_left, img_right)
        
        # 실제 시차값으로 변환
        disparity = disparity_raw.astype(np.float32) / 16.0
        
        return disparity
    
    def compute_both(self, img_left, img_right):
        """
        양방향 시차 맵 계산 (좌우 일관성 검사용)
        
        Returns:
        - disparity_left: 왼쪽 기준 시차 맵
        - disparity_right: 오른쪽 기준 시차 맵
        """
        
        # 왼쪽 기준
        disparity_left = self.compute(img_left, img_right)
        
        # 오른쪽 기준 (이미지 순서 바꿈)
        # 별도의 오른쪽 매처 필요
        right_matcher = cv2.ximgproc.createRightMatcher(self.stereo)
        disparity_right_raw = right_matcher.compute(img_right, img_left)
        disparity_right = disparity_right_raw.astype(np.float32) / 16.0
        
        return disparity_left, disparity_right
    
    def get_params(self):
        """현재 파라미터 반환"""
        return {
            'minDisparity': self.stereo.getMinDisparity(),
            'numDisparities': self.stereo.getNumDisparities(),
            'blockSize': self.stereo.getBlockSize(),
            'P1': self.stereo.getP1(),
            'P2': self.stereo.getP2(),
            'disp12MaxDiff': self.stereo.getDisp12MaxDiff(),
            'preFilterCap': self.stereo.getPreFilterCap(),
            'uniquenessRatio': self.stereo.getUniquenessRatio(),
            'speckleWindowSize': self.stereo.getSpeckleWindowSize(),
            'speckleRange': self.stereo.getSpeckleRange(),
            'mode': self.stereo.getMode()
        }


if __name__ == "__main__":
    # 테스트
    matcher = StereoSGBM_Matcher()
    print("StereoSGBM 파라미터:")
    for k, v in matcher.get_params().items():
        print(f"  {k}: {v}")
```

### 3.4 SGBM 파라미터 설명

| 파라미터 | 범위 | 권장값 | 설명 |
|----------|------|--------|------|
| `blockSize` | 1~11 (홀수) | 5 | 매칭 블록 크기 (BM보다 작게) |
| `P1` | > 0 | 8×ch×bs² | 부드러운 표면 페널티 |
| `P2` | > P1 | 32×ch×bs² | 경계 페널티 |
| `disp12MaxDiff` | -1~∞ | 1 | 좌우 일관성 허용 차이 |
| `uniquenessRatio` | 0~100 | 10 | 매칭 유일성 비율 |
| `mode` | 0~3 | 0 | 알고리즘 모드 |

### 3.5 BM vs SGBM 비교

```
                BM                          SGBM
        ┌─────────────────┐          ┌─────────────────┐
        │ 로컬 매칭만     │          │ 로컬 + 전역    │
        │                 │          │                 │
        │ ░░██░░░░██░░   │          │ ████████████   │
        │ ░░██░░░░██░░   │          │ ████████████   │
        │ ░░░░░░░░░░░░   │          │ ████████████   │
        │ (노이즈 많음)   │          │ (연속적)       │
        └─────────────────┘          └─────────────────┘
        
        속도: 빠름 ⭐⭐⭐⭐⭐          속도: 보통 ⭐⭐⭐
        품질: 보통 ⭐⭐                품질: 좋음 ⭐⭐⭐⭐
```

---

## 4. Disparity Map 후처리

### 4.1 WLS 필터 (Weighted Least Squares)

```python
"""
disparity_filter.py
Disparity Map 후처리 필터
"""

import cv2
import numpy as np


class DisparityFilter:
    def __init__(self):
        """Disparity 필터 초기화"""
        pass
    
    def apply_wls_filter(self, disparity_left, disparity_right, img_left, 
                         lambda_val=8000, sigma_val=1.5):
        """
        WLS (Weighted Least Squares) 필터 적용
        
        Parameters:
        - disparity_left: 왼쪽 기준 시차 맵
        - disparity_right: 오른쪽 기준 시차 맵
        - img_left: 왼쪽 이미지 (가이드)
        - lambda_val: 평활화 강도 (8000~80000)
        - sigma_val: 엣지 민감도 (1.0~2.0)
        
        Returns:
        - filtered_disparity: 필터링된 시차 맵
        """
        
        # WLS 필터 생성
        wls_filter = cv2.ximgproc.createDisparityWLSFilterGeneric(False)
        wls_filter.setLambda(lambda_val)
        wls_filter.setSigmaColor(sigma_val)
        
        # 16비트 정수로 변환 (필터 입력 요구사항)
        disp_left_int = (disparity_left * 16).astype(np.int16)
        disp_right_int = (disparity_right * 16).astype(np.int16)
        
        # 필터 적용
        filtered = wls_filter.filter(disp_left_int, img_left, 
                                      disparity_map_right=disp_right_int)
        
        # float32로 변환
        filtered_disparity = filtered.astype(np.float32) / 16.0
        
        return filtered_disparity
    
    def apply_bilateral_filter(self, disparity, d=9, sigma_color=75, sigma_space=75):
        """
        Bilateral 필터 적용 (엣지 보존 평활화)
        
        Parameters:
        - disparity: 시차 맵
        - d: 필터 직경
        - sigma_color: 색상 공간 시그마
        - sigma_space: 좌표 공간 시그마
        
        Returns:
        - filtered_disparity: 필터링된 시차 맵
        """
        
        # 유효한 영역만 처리
        valid_mask = disparity > 0
        
        # 0-255로 정규화
        disp_norm = cv2.normalize(disparity, None, 0, 255, cv2.NORM_MINMAX)
        disp_norm = disp_norm.astype(np.uint8)
        
        # Bilateral 필터
        filtered_norm = cv2.bilateralFilter(disp_norm, d, sigma_color, sigma_space)
        
        # 원래 스케일로 복원
        if valid_mask.any():
            scale = disparity[valid_mask].max() / 255.0
            filtered_disparity = filtered_norm.astype(np.float32) * scale
            filtered_disparity[~valid_mask] = 0
        else:
            filtered_disparity = np.zeros_like(disparity)
        
        return filtered_disparity
    
    def apply_median_filter(self, disparity, ksize=5):
        """
        Median 필터 적용 (노이즈 제거)
        
        Parameters:
        - disparity: 시차 맵
        - ksize: 커널 크기 (홀수)
        
        Returns:
        - filtered_disparity: 필터링된 시차 맵
        """
        
        # 정규화
        disp_norm = cv2.normalize(disparity, None, 0, 255, cv2.NORM_MINMAX)
        disp_norm = disp_norm.astype(np.uint8)
        
        # Median 필터
        filtered_norm = cv2.medianBlur(disp_norm, ksize)
        
        # 스케일 복원
        valid_mask = disparity > 0
        if valid_mask.any():
            scale = disparity[valid_mask].max() / 255.0
            filtered_disparity = filtered_norm.astype(np.float32) * scale
        else:
            filtered_disparity = np.zeros_like(disparity)
        
        return filtered_disparity
    
    def fill_holes(self, disparity, max_hole_size=100):
        """
        작은 구멍 채우기 (inpainting)
        
        Parameters:
        - disparity: 시차 맵
        - max_hole_size: 채울 최대 구멍 크기
        
        Returns:
        - filled_disparity: 구멍이 채워진 시차 맵
        """
        
        # 유효하지 않은 영역 마스크
        invalid_mask = (disparity <= 0).astype(np.uint8)
        
        # 작은 구멍만 선택
        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(invalid_mask)
        
        small_holes_mask = np.zeros_like(invalid_mask)
        for i in range(1, num_labels):
            if stats[i, cv2.CC_STAT_AREA] < max_hole_size:
                small_holes_mask[labels == i] = 255
        
        if not np.any(small_holes_mask):
            return disparity
        
        # Inpainting
        disp_norm = cv2.normalize(disparity, None, 0, 255, cv2.NORM_MINMAX)
        disp_norm = disp_norm.astype(np.uint8)
        
        filled_norm = cv2.inpaint(disp_norm, small_holes_mask, 3, cv2.INPAINT_NS)
        
        # 스케일 복원
        valid_mask = disparity > 0
        if valid_mask.any():
            scale = disparity[valid_mask].max() / 255.0
            filled_disparity = filled_norm.astype(np.float32) * scale
        else:
            filled_disparity = np.zeros_like(disparity)
        
        return filled_disparity


def demo_filters():
    """필터 데모"""
    
    # 더미 시차 맵 생성 (테스트용)
    h, w = 480, 640
    disparity = np.zeros((h, w), dtype=np.float32)
    
    # 경사면 생성
    for i in range(h):
        for j in range(w):
            disparity[i, j] = 50 + 50 * np.sin(j / 50)
    
    # 노이즈 추가
    noise = np.random.normal(0, 5, (h, w))
    disparity_noisy = disparity + noise
    disparity_noisy[disparity_noisy < 0] = 0
    
    # 필터 적용
    df = DisparityFilter()
    
    filtered_bilateral = df.apply_bilateral_filter(disparity_noisy)
    filtered_median = df.apply_median_filter(disparity_noisy)
    
    print("✅ 필터 테스트 완료")
    
    return disparity_noisy, filtered_bilateral, filtered_median


if __name__ == "__main__":
    demo_filters()
```

### 4.2 후처리 비교

```
원본 Disparity          Bilateral 필터        WLS 필터
┌─────────────┐        ┌─────────────┐      ┌─────────────┐
│ ░░█░░█░░█░  │        │ ██████████  │      │ ██████████  │
│ ░█░░█░░░█░  │   →    │ ██████████  │      │ ██████████  │
│ █░░░█░█░░█  │        │ ██████████  │      │ ██████████  │
│ (노이즈)    │        │ (평활화)    │      │ (엣지보존)  │
└─────────────┘        └─────────────┘      └─────────────┘
```

---

## 5. Depth Map 변환

### 5.1 시차에서 깊이로

```python
"""
disparity_to_depth.py
시차 맵을 깊이 맵으로 변환
"""

import cv2
import numpy as np
import yaml


class DepthEstimator:
    def __init__(self, calibration_file=None):
        """
        깊이 추정기 초기화
        
        Parameters:
        - calibration_file: 캘리브레이션 파라미터 파일 (.yaml)
        """
        
        self.Q = None
        self.baseline = None
        self.focal_length = None
        
        if calibration_file:
            self.load_calibration(calibration_file)
    
    def load_calibration(self, filename):
        """캘리브레이션 파라미터 로드"""
        
        with open(filename, 'r') as f:
            params = yaml.safe_load(f)
        
        self.Q = np.array(params['Q'])
        self.baseline = params['baseline_mm']
        
        # P1에서 초점거리 추출 (P1[0,0] = fx)
        P1 = np.array(params['P1'])
        self.focal_length = P1[0, 0]
        
        print(f"✅ 캘리브레이션 로드 완료")
        print(f"   베이스라인: {self.baseline:.2f} mm")
        print(f"   초점거리: {self.focal_length:.2f} px")
    
    def set_params_manual(self, focal_length, baseline):
        """수동 파라미터 설정"""
        self.focal_length = focal_length
        self.baseline = baseline
    
    def disparity_to_depth(self, disparity, method='formula'):
        """
        시차 맵을 깊이 맵으로 변환
        
        Parameters:
        - disparity: 시차 맵 (float32, 픽셀 단위)
        - method: 변환 방법
            - 'formula': Z = f * B / d
            - 'Q_matrix': cv2.reprojectImageTo3D 사용
        
        Returns:
        - depth: 깊이 맵 (float32, mm 단위)
        """
        
        if method == 'formula':
            return self._depth_from_formula(disparity)
        elif method == 'Q_matrix':
            return self._depth_from_Q(disparity)
        else:
            raise ValueError(f"Unknown method: {method}")
    
    def _depth_from_formula(self, disparity):
        """공식을 이용한 깊이 계산: Z = f * B / d"""
        
        if self.focal_length is None or self.baseline is None:
            raise ValueError("focal_length와 baseline이 설정되지 않았습니다.")
        
        # 0으로 나누기 방지
        depth = np.zeros_like(disparity)
        valid_mask = disparity > 0
        
        depth[valid_mask] = (self.focal_length * self.baseline) / disparity[valid_mask]
        
        return depth
    
    def _depth_from_Q(self, disparity):
        """Q 행렬을 이용한 깊이 계산"""
        
        if self.Q is None:
            raise ValueError("Q 행렬이 설정되지 않았습니다.")
        
        # 3D 복원
        points_3d = cv2.reprojectImageTo3D(disparity, self.Q)
        
        # Z 채널만 추출
        depth = points_3d[:, :, 2]
        
        # 유효하지 않은 값 처리
        depth[disparity <= 0] = 0
        depth[depth < 0] = 0
        depth[depth > 100000] = 0  # 100m 이상은 무효
        
        return depth
    
    def depth_to_meters(self, depth_mm):
        """mm를 m로 변환"""
        return depth_mm / 1000.0
    
    def get_depth_at_point(self, depth, x, y):
        """특정 좌표의 깊이 반환"""
        
        h, w = depth.shape
        if 0 <= x < w and 0 <= y < h:
            return depth[y, x]
        return 0
    
    def get_depth_stats(self, depth, roi=None):
        """
        깊이 통계 계산
        
        Parameters:
        - depth: 깊이 맵
        - roi: 관심 영역 (x, y, w, h) 또는 None (전체)
        
        Returns:
        - stats: 통계 딕셔너리
        """
        
        if roi is not None:
            x, y, w, h = roi
            depth_roi = depth[y:y+h, x:x+w]
        else:
            depth_roi = depth
        
        valid_mask = depth_roi > 0
        
        if not np.any(valid_mask):
            return {'valid': False}
        
        valid_depths = depth_roi[valid_mask]
        
        return {
            'valid': True,
            'min_mm': float(np.min(valid_depths)),
            'max_mm': float(np.max(valid_depths)),
            'mean_mm': float(np.mean(valid_depths)),
            'median_mm': float(np.median(valid_depths)),
            'std_mm': float(np.std(valid_depths)),
            'valid_ratio': float(np.sum(valid_mask) / depth_roi.size)
        }


def visualize_depth(depth, max_depth_mm=5000, colormap=cv2.COLORMAP_TURBO):
    """
    깊이 맵 시각화
    
    Parameters:
    - depth: 깊이 맵 (mm)
    - max_depth_mm: 표시할 최대 깊이
    - colormap: 컬러맵
    
    Returns:
    - depth_color: 컬러 깊이 맵 (BGR)
    """
    
    # 클리핑
    depth_clipped = np.clip(depth, 0, max_depth_mm)
    
    # 정규화 (가까울수록 밝게)
    depth_norm = (1 - depth_clipped / max_depth_mm) * 255
    depth_norm = depth_norm.astype(np.uint8)
    
    # 컬러맵 적용
    depth_color = cv2.applyColorMap(depth_norm, colormap)
    
    # 유효하지 않은 영역은 검정색
    depth_color[depth <= 0] = [0, 0, 0]
    
    return depth_color


def add_depth_colorbar(img, max_depth_mm=5000, width=30):
    """깊이 맵에 컬러바 추가"""
    
    h = img.shape[0]
    
    # 컬러바 생성
    colorbar = np.zeros((h, width, 3), dtype=np.uint8)
    for i in range(h):
        val = int(255 * (1 - i / h))
        colorbar[i, :] = cv2.applyColorMap(np.array([[val]], dtype=np.uint8), 
                                            cv2.COLORMAP_TURBO)[0, 0]
    
    # 눈금 추가
    for i in range(0, 6):
        y = int(h * i / 5)
        depth_val = max_depth_mm * i / 5
        cv2.line(colorbar, (0, y), (5, y), (255, 255, 255), 1)
        cv2.putText(colorbar, f"{depth_val/1000:.1f}m", (8, y+5),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)
    
    # 합치기
    result = cv2.hconcat([img, colorbar])
    
    return result


if __name__ == "__main__":
    # 테스트
    estimator = DepthEstimator()
    estimator.set_params_manual(focal_length=1317, baseline=85)
    
    # 더미 시차 맵
    disparity = np.random.uniform(20, 150, (480, 640)).astype(np.float32)
    
    # 깊이 변환
    depth = estimator.disparity_to_depth(disparity, method='formula')
    
    # 통계
    stats = estimator.get_depth_stats(depth)
    print("\n깊이 통계:")
    for k, v in stats.items():
        print(f"  {k}: {v}")
    
    # 시각화
    depth_color = visualize_depth(depth)
    depth_with_bar = add_depth_colorbar(depth_color)
    
    cv2.imwrite("depth_test.png", depth_with_bar)
    print("\n✅ 저장됨: depth_test.png")
```

---

## 6. 파라미터 튜닝 가이드

### 6.1 튜닝 GUI

```python
"""
parameter_tuning_gui.py
스테레오 매칭 파라미터 튜닝 GUI
"""

import cv2
import numpy as np


class StereoTunerGUI:
    def __init__(self, img_left, img_right, algorithm='SGBM'):
        """
        파라미터 튜닝 GUI
        
        Parameters:
        - img_left: 정류된 왼쪽 이미지
        - img_right: 정류된 오른쪽 이미지
        - algorithm: 'BM' 또는 'SGBM'
        """
        
        self.img_left = img_left
        self.img_right = img_right
        self.algorithm = algorithm
        
        # 그레이스케일 변환
        if len(img_left.shape) == 3:
            self.gray_left = cv2.cvtColor(img_left, cv2.COLOR_BGR2GRAY)
            self.gray_right = cv2.cvtColor(img_right, cv2.COLOR_BGR2GRAY)
        else:
            self.gray_left = img_left
            self.gray_right = img_right
        
        # 매처 생성
        if algorithm == 'BM':
            self.stereo = cv2.StereoBM_create()
            self.setup_bm_trackbars()
        else:
            self.stereo = cv2.StereoSGBM_create()
            self.setup_sgbm_trackbars()
        
        self.window_name = f"Stereo {algorithm} Tuner"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        
    def setup_bm_trackbars(self):
        """BM 트랙바 설정"""
        
        cv2.createTrackbar('numDisparities', self.window_name, 8, 16, self.on_change)
        cv2.createTrackbar('blockSize', self.window_name, 7, 25, self.on_change)
        cv2.createTrackbar('preFilterCap', self.window_name, 31, 63, self.on_change)
        cv2.createTrackbar('uniquenessRatio', self.window_name, 15, 50, self.on_change)
        cv2.createTrackbar('speckleWindowSize', self.window_name, 100, 200, self.on_change)
        cv2.createTrackbar('speckleRange', self.window_name, 2, 10, self.on_change)
        cv2.createTrackbar('textureThreshold', self.window_name, 10, 100, self.on_change)
        
    def setup_sgbm_trackbars(self):
        """SGBM 트랙바 설정"""
        
        cv2.createTrackbar('numDisparities', self.window_name, 8, 16, self.on_change)
        cv2.createTrackbar('blockSize', self.window_name, 2, 10, self.on_change)
        cv2.createTrackbar('P1_mult', self.window_name, 8, 32, self.on_change)
        cv2.createTrackbar('P2_mult', self.window_name, 32, 128, self.on_change)
        cv2.createTrackbar('disp12MaxDiff', self.window_name, 1, 10, self.on_change)
        cv2.createTrackbar('preFilterCap', self.window_name, 63, 127, self.on_change)
        cv2.createTrackbar('uniquenessRatio', self.window_name, 10, 50, self.on_change)
        cv2.createTrackbar('speckleWindowSize', self.window_name, 100, 200, self.on_change)
        cv2.createTrackbar('speckleRange', self.window_name, 2, 10, self.on_change)
        cv2.createTrackbar('mode', self.window_name, 0, 3, self.on_change)
        
    def on_change(self, val):
        """트랙바 변경 콜백 (dummy)"""
        pass
    
    def get_bm_params(self):
        """BM 파라미터 읽기"""
        
        num_disp = max(16, cv2.getTrackbarPos('numDisparities', self.window_name) * 16)
        block_size = cv2.getTrackbarPos('blockSize', self.window_name) * 2 + 5
        
        self.stereo.setNumDisparities(num_disp)
        self.stereo.setBlockSize(block_size)
        self.stereo.setPreFilterCap(cv2.getTrackbarPos('preFilterCap', self.window_name))
        self.stereo.setUniquenessRatio(cv2.getTrackbarPos('uniquenessRatio', self.window_name))
        self.stereo.setSpeckleWindowSize(cv2.getTrackbarPos('speckleWindowSize', self.window_name))
        self.stereo.setSpeckleRange(cv2.getTrackbarPos('speckleRange', self.window_name))
        self.stereo.setTextureThreshold(cv2.getTrackbarPos('textureThreshold', self.window_name))
        
    def get_sgbm_params(self):
        """SGBM 파라미터 읽기"""
        
        num_disp = max(16, cv2.getTrackbarPos('numDisparities', self.window_name) * 16)
        block_size = cv2.getTrackbarPos('blockSize', self.window_name) * 2 + 1
        if block_size < 1:
            block_size = 1
        
        p1_mult = max(1, cv2.getTrackbarPos('P1_mult', self.window_name))
        p2_mult = max(1, cv2.getTrackbarPos('P2_mult', self.window_name))
        
        channels = 3 if len(self.img_left.shape) == 3 else 1
        P1 = p1_mult * channels * block_size ** 2
        P2 = p2_mult * channels * block_size ** 2
        
        self.stereo.setNumDisparities(num_disp)
        self.stereo.setBlockSize(block_size)
        self.stereo.setP1(P1)
        self.stereo.setP2(max(P1 + 1, P2))  # P2 > P1 보장
        self.stereo.setDisp12MaxDiff(cv2.getTrackbarPos('disp12MaxDiff', self.window_name))
        self.stereo.setPreFilterCap(cv2.getTrackbarPos('preFilterCap', self.window_name))
        self.stereo.setUniquenessRatio(cv2.getTrackbarPos('uniquenessRatio', self.window_name))
        self.stereo.setSpeckleWindowSize(cv2.getTrackbarPos('speckleWindowSize', self.window_name))
        self.stereo.setSpeckleRange(cv2.getTrackbarPos('speckleRange', self.window_name))
        self.stereo.setMode(cv2.getTrackbarPos('mode', self.window_name))
        
    def compute_disparity(self):
        """시차 맵 계산"""
        
        if self.algorithm == 'BM':
            self.get_bm_params()
            disparity = self.stereo.compute(self.gray_left, self.gray_right)
        else:
            self.get_sgbm_params()
            disparity = self.stereo.compute(self.img_left, self.img_right)
        
        return disparity.astype(np.float32) / 16.0
    
    def run(self):
        """튜닝 GUI 실행"""
        
        print("="*60)
        print(f"스테레오 {self.algorithm} 파라미터 튜닝")
        print("="*60)
        print("조작:")
        print("  트랙바: 파라미터 조정")
        print("  S: 현재 파라미터 저장")
        print("  Q: 종료")
        print("="*60)
        
        while True:
            # 시차 계산
            disparity = self.compute_disparity()
            
            # 시각화
            valid_mask = disparity > 0
            if np.any(valid_mask):
                disp_min = disparity[valid_mask].min()
                disp_max = disparity[valid_mask].max()
                disp_norm = np.zeros_like(disparity)
                disp_norm[valid_mask] = (disparity[valid_mask] - disp_min) / (disp_max - disp_min + 1e-6) * 255
            else:
                disp_norm = np.zeros_like(disparity)
            
            disp_color = cv2.applyColorMap(disp_norm.astype(np.uint8), cv2.COLORMAP_JET)
            disp_color[~valid_mask] = [0, 0, 0]
            
            # 원본과 함께 표시
            h, w = self.img_left.shape[:2]
            if len(self.img_left.shape) == 2:
                img_display = cv2.cvtColor(self.img_left, cv2.COLOR_GRAY2BGR)
            else:
                img_display = self.img_left.copy()
            
            img_display = cv2.resize(img_display, (w//2, h//2))
            disp_color = cv2.resize(disp_color, (w//2, h//2))
            
            combined = cv2.hconcat([img_display, disp_color])
            
            # 정보 표시
            cv2.putText(combined, f"Disparity range: {disp_min:.1f} - {disp_max:.1f} px", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            
            cv2.imshow(self.window_name, combined)
            
            key = cv2.waitKey(30) & 0xFF
            
            if key == ord('q'):
                break
            elif key == ord('s'):
                self.save_params()
        
        cv2.destroyAllWindows()
        
    def save_params(self):
        """현재 파라미터 저장"""
        
        if self.algorithm == 'BM':
            params = {
                'algorithm': 'BM',
                'numDisparities': self.stereo.getNumDisparities(),
                'blockSize': self.stereo.getBlockSize(),
                'preFilterCap': self.stereo.getPreFilterCap(),
                'uniquenessRatio': self.stereo.getUniquenessRatio(),
                'speckleWindowSize': self.stereo.getSpeckleWindowSize(),
                'speckleRange': self.stereo.getSpeckleRange(),
                'textureThreshold': self.stereo.getTextureThreshold()
            }
        else:
            params = {
                'algorithm': 'SGBM',
                'numDisparities': self.stereo.getNumDisparities(),
                'blockSize': self.stereo.getBlockSize(),
                'P1': self.stereo.getP1(),
                'P2': self.stereo.getP2(),
                'disp12MaxDiff': self.stereo.getDisp12MaxDiff(),
                'preFilterCap': self.stereo.getPreFilterCap(),
                'uniquenessRatio': self.stereo.getUniquenessRatio(),
                'speckleWindowSize': self.stereo.getSpeckleWindowSize(),
                'speckleRange': self.stereo.getSpeckleRange(),
                'mode': self.stereo.getMode()
            }
        
        import yaml
        filename = f"stereo_{self.algorithm.lower()}_params.yaml"
        with open(filename, 'w') as f:
            yaml.dump(params, f, default_flow_style=False)
        
        print(f"\n✅ 파라미터 저장됨: {filename}")
        for k, v in params.items():
            print(f"   {k}: {v}")


if __name__ == "__main__":
    # 테스트용 더미 이미지
    img_left = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
    img_right = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
    
    tuner = StereoTunerGUI(img_left, img_right, algorithm='SGBM')
    tuner.run()
```

### 6.2 튜닝 가이드라인

| 문제 | 원인 | 해결책 |
|------|------|--------|
| 노이즈가 많음 | blockSize 작음 | blockSize 증가 |
| 디테일 손실 | blockSize 큼 | blockSize 감소 |
| 근거리 측정 불가 | numDisparities 작음 | numDisparities 증가 |
| 계산 느림 | numDisparities 큼 | numDisparities 감소 |
| 작은 점 노이즈 | 스페클 필터 약함 | speckleWindowSize 증가 |
| 경계가 뭉개짐 | P1, P2 너무 큼 | P1, P2 감소 |
| 표면이 불연속 | P1, P2 너무 작음 | P1, P2 증가 |

---

## 7. 실시간 깊이 추정

### 7.1 완전한 실시간 시스템

```python
"""
realtime_depth.py
실시간 깊이 추정 시스템
"""

import cv2
import numpy as np
import yaml
import time


class RealtimeDepthSystem:
    def __init__(self, calibration_file, left_idx=0, right_idx=2):
        """
        실시간 깊이 추정 시스템
        
        Parameters:
        - calibration_file: 캘리브레이션 파라미터 파일
        - left_idx, right_idx: 카메라 인덱스
        """
        
        # 캘리브레이션 로드
        self.load_calibration(calibration_file)
        
        # 카메라 초기화
        self.cap_left = cv2.VideoCapture(left_idx)
        self.cap_right = cv2.VideoCapture(right_idx)
        
        for cap in [self.cap_left, self.cap_right]:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.img_size[0])
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.img_size[1])
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            cap.set(cv2.CAP_PROP_FPS, 30)
        
        # SGBM 매처 초기화
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
        
        # FPS 계산용
        self.fps_history = []
        
    def load_calibration(self, filename):
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
        
        print(f"✅ 캘리브레이션 로드: {self.img_size}")
        
    def process_frame(self, frame_left, frame_right):
        """프레임 처리"""
        
        # 정류
        rect_left = cv2.remap(frame_left, self.map1_left, self.map2_left, cv2.INTER_LINEAR)
        rect_right = cv2.remap(frame_right, self.map1_right, self.map2_right, cv2.INTER_LINEAR)
        
        # 시차 계산
        disparity_raw = self.stereo.compute(rect_left, rect_right)
        disparity = disparity_raw.astype(np.float32) / 16.0
        
        # 깊이 계산
        depth = np.zeros_like(disparity)
        valid_mask = disparity > 0
        depth[valid_mask] = (self.focal_length * self.baseline) / disparity[valid_mask]
        
        return rect_left, rect_right, disparity, depth
    
    def visualize(self, rect_left, disparity, depth, fps):
        """결과 시각화"""
        
        h, w = rect_left.shape[:2]
        
        # 시차 맵 컬러화
        valid_mask = disparity > 0
        disp_color = np.zeros((h, w, 3), dtype=np.uint8)
        
        if np.any(valid_mask):
            disp_norm = np.zeros_like(disparity)
            d_min, d_max = disparity[valid_mask].min(), disparity[valid_mask].max()
            disp_norm[valid_mask] = (disparity[valid_mask] - d_min) / (d_max - d_min + 1e-6) * 255
            disp_color = cv2.applyColorMap(disp_norm.astype(np.uint8), cv2.COLORMAP_JET)
            disp_color[~valid_mask] = [0, 0, 0]
        
        # 깊이 맵 컬러화 (0-5m)
        depth_clipped = np.clip(depth, 0, 5000)
        depth_norm = (1 - depth_clipped / 5000) * 255
        depth_color = cv2.applyColorMap(depth_norm.astype(np.uint8), cv2.COLORMAP_TURBO)
        depth_color[depth <= 0] = [0, 0, 0]
        
        # 리사이즈
        scale = 0.5
        rect_small = cv2.resize(rect_left, None, fx=scale, fy=scale)
        disp_small = cv2.resize(disp_color, None, fx=scale, fy=scale)
        depth_small = cv2.resize(depth_color, None, fx=scale, fy=scale)
        
        # 중앙 깊이 표시
        cx, cy = w // 2, h // 2
        center_depth = depth[cy, cx]
        
        # 합치기
        top_row = cv2.hconcat([rect_small, disp_small])
        
        # 깊이 정보 패널
        info_panel = np.zeros_like(depth_small)
        cv2.putText(info_panel, f"FPS: {fps:.1f}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(info_panel, f"Center: {center_depth/1000:.2f}m", (10, 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.putText(info_panel, "Depth Map (0-5m)", (10, 90),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        bottom_row = cv2.hconcat([depth_small, info_panel])
        
        result = cv2.vconcat([top_row, bottom_row])
        
        # 십자선
        rh, rw = rect_small.shape[:2]
        cv2.drawMarker(result, (rw//2, rh//2), (0, 255, 0), 
                      cv2.MARKER_CROSS, 20, 2)
        
        return result
    
    def run(self):
        """메인 루프"""
        
        print("\n" + "="*60)
        print("실시간 깊이 추정 시스템")
        print("="*60)
        print("조작:")
        print("  Q: 종료")
        print("  S: 스크린샷 저장")
        print("="*60)
        
        while True:
            t_start = time.time()
            
            # 프레임 캡처
            ret1, frame_left = self.cap_left.read()
            ret2, frame_right = self.cap_right.read()
            
            if not ret1 or not ret2:
                print("❌ 카메라 읽기 실패")
                break
            
            # 처리
            rect_left, rect_right, disparity, depth = self.process_frame(
                frame_left, frame_right
            )
            
            # FPS 계산
            elapsed = time.time() - t_start
            fps = 1.0 / elapsed if elapsed > 0 else 0
            self.fps_history.append(fps)
            if len(self.fps_history) > 30:
                self.fps_history.pop(0)
            avg_fps = np.mean(self.fps_history)
            
            # 시각화
            display = self.visualize(rect_left, disparity, depth, avg_fps)
            
            cv2.imshow("Realtime Depth", display)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                cv2.imwrite("screenshot_depth.png", display)
                print("✅ 스크린샷 저장됨")
        
        self.cleanup()
        
    def cleanup(self):
        """리소스 정리"""
        self.cap_left.release()
        self.cap_right.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    system = RealtimeDepthSystem("stereo_params.yaml", left_idx=0, right_idx=2)
    system.run()
```

---

## 8. 성능 최적화

### 8.1 해상도 vs 속도 트레이드오프

| 해상도 | 처리 시간 (SGBM) | 용도 |
|--------|-----------------|------|
| 1920×1080 | ~150ms | 고정밀 측정 |
| 1280×720 | ~70ms | 범용 |
| 640×480 | ~25ms | 실시간 |
| 320×240 | ~8ms | 고속 실시간 |

### 8.2 최적화 기법

```python
"""
optimization_tips.py
성능 최적화 기법
"""

import cv2
import numpy as np


def optimize_for_realtime(stereo_sgbm, target_fps=30):
    """실시간 처리를 위한 최적화"""
    
    # 1. 5방향 모드 사용 (8방향 대비 약 40% 빠름)
    stereo_sgbm.setMode(cv2.STEREO_SGBM_MODE_SGBM_3WAY)
    
    # 2. numDisparities 최소화
    stereo_sgbm.setNumDisparities(64)  # 128 → 64
    
    # 3. blockSize 약간 증가 (노이즈 감소)
    stereo_sgbm.setBlockSize(7)
    
    # 4. 스페클 필터 비활성화 (후처리로 대체)
    stereo_sgbm.setSpeckleWindowSize(0)
    
    return stereo_sgbm


def downsample_process_upsample(img_left, img_right, stereo, scale=0.5):
    """다운샘플링 후 처리, 업샘플링"""
    
    h, w = img_left.shape[:2]
    
    # 다운샘플링
    img_left_small = cv2.resize(img_left, None, fx=scale, fy=scale)
    img_right_small = cv2.resize(img_right, None, fx=scale, fy=scale)
    
    # 시차 계산
    disparity_small = stereo.compute(img_left_small, img_right_small)
    disparity_small = disparity_small.astype(np.float32) / 16.0
    
    # 업샘플링 (시차도 스케일 조정)
    disparity = cv2.resize(disparity_small, (w, h)) / scale
    
    return disparity


def use_roi(img_left, img_right, stereo, roi):
    """관심 영역만 처리"""
    
    x, y, w, h = roi
    
    # ROI 추출
    roi_left = img_left[y:y+h, x:x+w]
    roi_right = img_right[y:y+h, x:x+w]
    
    # 시차 계산
    disparity_roi = stereo.compute(roi_left, roi_right)
    disparity_roi = disparity_roi.astype(np.float32) / 16.0
    
    # 전체 크기로 확장
    disparity = np.zeros(img_left.shape[:2], dtype=np.float32)
    disparity[y:y+h, x:x+w] = disparity_roi
    
    return disparity
```

---

## 9. 실습 코드

### 9.1 전체 파이프라인 예제

```python
"""
stereo_matching_demo.py
스테레오 매칭 전체 데모
"""

import cv2
import numpy as np
import yaml
from stereo_sgbm import StereoSGBM_Matcher, visualize_disparity
from disparity_to_depth import DepthEstimator, visualize_depth, add_depth_colorbar


def run_stereo_matching_demo(left_image, right_image, calibration_file):
    """
    스테레오 매칭 데모
    
    Parameters:
    - left_image: 왼쪽 이미지 경로
    - right_image: 오른쪽 이미지 경로
    - calibration_file: 캘리브레이션 파일 경로
    """
    
    print("="*60)
    print("스테레오 매칭 데모")
    print("="*60)
    
    # 1. 이미지 로드
    print("\n[1/5] 이미지 로드...")
    img_left = cv2.imread(left_image)
    img_right = cv2.imread(right_image)
    
    if img_left is None or img_right is None:
        print("❌ 이미지 로드 실패")
        return
    
    print(f"  이미지 크기: {img_left.shape}")
    
    # 2. 캘리브레이션 로드 및 정류
    print("\n[2/5] 정류 적용...")
    with open(calibration_file, 'r') as f:
        params = yaml.safe_load(f)
    
    img_size = tuple(params['image_size'])
    K1 = np.array(params['K1'])
    D1 = np.array(params['D1'])
    K2 = np.array(params['K2'])
    D2 = np.array(params['D2'])
    R1 = np.array(params['R1'])
    R2 = np.array(params['R2'])
    P1 = np.array(params['P1'])
    P2 = np.array(params['P2'])
    
    map1_left, map2_left = cv2.initUndistortRectifyMap(K1, D1, R1, P1, img_size, cv2.CV_32FC1)
    map1_right, map2_right = cv2.initUndistortRectifyMap(K2, D2, R2, P2, img_size, cv2.CV_32FC1)
    
    rect_left = cv2.remap(img_left, map1_left, map2_left, cv2.INTER_LINEAR)
    rect_right = cv2.remap(img_right, map1_right, map2_right, cv2.INTER_LINEAR)
    
    print("  ✅ 정류 완료")
    
    # 3. 스테레오 매칭
    print("\n[3/5] 스테레오 매칭 (SGBM)...")
    matcher = StereoSGBM_Matcher()
    disparity = matcher.compute(rect_left, rect_right)
    
    valid_pixels = np.sum(disparity > 0)
    total_pixels = disparity.size
    print(f"  유효 픽셀: {valid_pixels}/{total_pixels} ({100*valid_pixels/total_pixels:.1f}%)")
    
    # 4. 깊이 변환
    print("\n[4/5] 깊이 맵 생성...")
    depth_estimator = DepthEstimator(calibration_file)
    depth = depth_estimator.disparity_to_depth(disparity, method='formula')
    
    stats = depth_estimator.get_depth_stats(depth)
    print(f"  깊이 범위: {stats['min_mm']/1000:.2f}m ~ {stats['max_mm']/1000:.2f}m")
    print(f"  평균 깊이: {stats['mean_mm']/1000:.2f}m")
    
    # 5. 시각화 및 저장
    print("\n[5/5] 결과 저장...")
    
    # 시차 맵
    disp_color = visualize_disparity(disparity)
    cv2.imwrite("result_disparity.png", disp_color)
    
    # 깊이 맵
    depth_color = visualize_depth(depth)
    depth_with_bar = add_depth_colorbar(depth_color)
    cv2.imwrite("result_depth.png", depth_with_bar)
    
    # 정류 이미지 (에피폴라 라인 포함)
    rect_combined = cv2.hconcat([rect_left, rect_right])
    for y in range(0, rect_combined.shape[0], 50):
        cv2.line(rect_combined, (0, y), (rect_combined.shape[1], y), (0, 255, 0), 1)
    cv2.imwrite("result_rectified.png", rect_combined)
    
    print("\n✅ 저장 완료:")
    print("  - result_disparity.png")
    print("  - result_depth.png")
    print("  - result_rectified.png")
    
    # 디스플레이
    cv2.imshow("Disparity", cv2.resize(disp_color, (640, 360)))
    cv2.imshow("Depth", cv2.resize(depth_with_bar, (640, 360)))
    print("\n아무 키나 누르면 종료...")
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    # 사용 예시
    run_stereo_matching_demo(
        "test_left.png",
        "test_right.png", 
        "stereo_params.yaml"
    )
```

---

## 10. 트러블슈팅

### 10.1 일반적인 문제

| 문제 | 원인 | 해결책 |
|------|------|--------|
| 시차가 모두 0 | 이미지 순서 반대 | left/right 교환 |
| 줄무늬 패턴 | blockSize 너무 작음 | blockSize 증가 |
| 검은 영역 많음 | numDisparities 부족 | numDisparities 증가 |
| 노이즈 많음 | 텍스처 부족 | 조명 개선, 필터 적용 |
| 경계 흐림 | P1, P2 너무 큼 | P1, P2 감소 |
| 처리 느림 | 해상도/설정 과다 | 해상도 감소, 모드 변경 |

---

## 📝 학습 체크리스트

### 이론 이해

- [ ] BM과 SGBM의 차이점을 설명할 수 있다
- [ ] P1, P2 파라미터의 역할을 이해했다
- [ ] 시차에서 깊이로 변환하는 공식을 알고 있다
- [ ] WLS 필터의 목적을 설명할 수 있다

### 실습 완료

- [ ] StereoBM으로 시차 맵 생성
- [ ] StereoSGBM으로 시차 맵 생성
- [ ] 파라미터 튜닝 GUI 사용
- [ ] 깊이 맵 생성 및 시각화
- [ ] 실시간 깊이 추정 시스템 실행

---

## ➡️ 다음 모듈

**[Module 04: 3D 포인트 클라우드 생성](../Module_04_PointCloud/README.md)**

다음 모듈에서는:
- 깊이 맵을 3D 포인트 클라우드로 변환
- Open3D를 이용한 시각화
- 포인트 클라우드 필터링 및 처리
- 메쉬 생성

을 학습합니다.

---

## 📄 라이선스

MIT License - 자유롭게 사용, 수정, 배포 가능
