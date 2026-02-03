# Module 05: 딥러닝 기반 스테레오 (RAFT-Stereo, AANet)

[![난이도](https://img.shields.io/badge/난이도-⭐⭐⭐⭐_고급-red.svg)]()
[![예상시간](https://img.shields.io/badge/예상시간-15--20시간-blue.svg)]()
[![선수지식](https://img.shields.io/badge/선수지식-Module_03,_PyTorch_기초-orange.svg)]()

---

## 📋 모듈 개요

| 항목 | 내용 |
|------|------|
| **학습 목표** | 최신 딥러닝 스테레오 매칭 모델 이해 및 활용 |
| **핵심 키워드** | RAFT-Stereo, AANet, Cost Volume, Disparity Regression |
| **산출물** | 딥러닝 기반 깊이 추정 시스템, 성능 비교 분석 |

---

## 📚 목차

1. [딥러닝 스테레오 매칭 개요](#1-딥러닝-스테레오-매칭-개요)
2. [핵심 개념](#2-핵심-개념)
3. [환경 설정](#3-환경-설정)
4. [RAFT-Stereo](#4-raft-stereo)
5. [AANet](#5-aanet)
6. [기타 주요 모델](#6-기타-주요-모델)
7. [전통적 방법 vs 딥러닝 비교](#7-전통적-방법-vs-딥러닝-비교)
8. [커스텀 데이터 적용](#8-커스텀-데이터-적용)
9. [성능 최적화](#9-성능-최적화)
10. [실습 프로젝트](#10-실습-프로젝트)

---

## 1. 딥러닝 스테레오 매칭 개요

### 1.1 전통적 방법의 한계

```
┌─────────────────────────────────────────────────────────────┐
│              전통적 스테레오 매칭의 한계                       │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ❌ 텍스처 없는 영역 (벽, 하늘)                              │
│     → 매칭 실패, 구멍 발생                                   │
│                                                             │
│  ❌ 반복 패턴 (타일, 울타리)                                 │
│     → 잘못된 매칭, 노이즈                                    │
│                                                             │
│  ❌ 반사/투명 표면 (유리, 물)                                │
│     → 불안정한 결과                                         │
│                                                             │
│  ❌ 가려진 영역 (Occlusion)                                 │
│     → 정보 부족                                             │
│                                                             │
│  ❌ 수동 파라미터 튜닝 필요                                   │
│     → 환경마다 재조정                                       │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### 1.2 딥러닝 접근법의 장점

```
┌─────────────────────────────────────────────────────────────┐
│              딥러닝 스테레오 매칭의 장점                       │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ✅ 학습된 특징 추출                                         │
│     → 텍스처 없는 영역에서도 문맥 파악                        │
│                                                             │
│  ✅ End-to-End 학습                                         │
│     → 자동 최적화, 파라미터 튜닝 불필요                       │
│                                                             │
│  ✅ 대규모 데이터 학습                                       │
│     → 다양한 상황에 일반화                                   │
│                                                             │
│  ✅ 서브픽셀 정확도                                          │
│     → 회귀 기반으로 연속적인 시차 출력                        │
│                                                             │
│  ✅ 가려진 영역 처리                                         │
│     → 문맥 정보로 추론                                       │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### 1.3 주요 딥러닝 모델 타임라인

```
2016    2017    2018    2019    2020    2021    2022    2023
  │       │       │       │       │       │       │       │
  ▼       ▼       ▼       ▼       ▼       ▼       ▼       ▼
DispNet  GC-Net  PSMNet  GA-Net  AANet  RAFT-   IGEV   선행
         │               │       │     Stereo  Stereo  연구
         │               │       │       │       │
    3D Conv         Attention  적응형  반복적   기하학
    Cost Volume     메커니즘   집계   업데이트  볼륨
```

### 1.4 모델 성능 비교 (KITTI 2015 벤치마크)

| 모델 | D1-all (%) | 추론 시간 | 특징 |
|------|-----------|----------|------|
| SGM (전통) | 10.86 | ~1s | 베이스라인 |
| PSMNet | 2.32 | ~0.4s | 피라미드 풀링 |
| GA-Net | 1.81 | ~1.8s | 가이드 집계 |
| **AANet** | 2.03 | **0.07s** | 빠른 속도 |
| **RAFT-Stereo** | **1.27** | ~0.3s | 최고 정확도 |
| IGEV-Stereo | 1.12 | ~0.2s | 최신 SOTA |

---

## 2. 핵심 개념

### 2.1 Cost Volume

스테레오 매칭의 핵심 데이터 구조:

```
┌─────────────────────────────────────────────────────────────┐
│                    Cost Volume 구조                         │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│   왼쪽 특징 맵         오른쪽 특징 맵                         │
│   [H × W × C]          [H × W × C]                         │
│       │                    │                               │
│       └────────┬───────────┘                               │
│                ▼                                            │
│        ┌───────────────┐                                   │
│        │  Cost Volume  │                                   │
│        │ [D × H × W × C]│  D: 시차 범위                     │
│        │               │  H: 높이                          │
│        │   d=0 ────────│  W: 너비                          │
│        │   d=1 ────────│  C: 채널 (특징 차이/상관)           │
│        │   ...         │                                   │
│        │   d=D ────────│                                   │
│        └───────────────┘                                   │
│                │                                            │
│                ▼                                            │
│        3D CNN / GRU로 처리                                  │
│                │                                            │
│                ▼                                            │
│        Disparity Map [H × W]                               │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### 2.2 Cost Volume 구성 방법

```python
"""
cost_volume_example.py
Cost Volume 구성 예시
"""

import torch
import torch.nn.functional as F


def build_concat_volume(left_feat, right_feat, max_disp):
    """
    Concatenation 기반 Cost Volume
    
    Parameters:
    - left_feat: [B, C, H, W] 왼쪽 특징 맵
    - right_feat: [B, C, H, W] 오른쪽 특징 맵
    - max_disp: 최대 시차
    
    Returns:
    - cost_volume: [B, 2C, D, H, W]
    """
    
    B, C, H, W = left_feat.shape
    cost_volume = torch.zeros(B, 2*C, max_disp, H, W, device=left_feat.device)
    
    for d in range(max_disp):
        if d == 0:
            cost_volume[:, :C, d, :, :] = left_feat
            cost_volume[:, C:, d, :, :] = right_feat
        else:
            cost_volume[:, :C, d, :, d:] = left_feat[:, :, :, d:]
            cost_volume[:, C:, d, :, d:] = right_feat[:, :, :, :-d]
    
    return cost_volume


def build_correlation_volume(left_feat, right_feat, max_disp):
    """
    Correlation 기반 Cost Volume
    
    Returns:
    - cost_volume: [B, D, H, W]
    """
    
    B, C, H, W = left_feat.shape
    cost_volume = torch.zeros(B, max_disp, H, W, device=left_feat.device)
    
    for d in range(max_disp):
        if d == 0:
            cost_volume[:, d, :, :] = (left_feat * right_feat).sum(dim=1)
        else:
            cost_volume[:, d, :, d:] = (left_feat[:, :, :, d:] * right_feat[:, :, :, :-d]).sum(dim=1)
    
    # 정규화
    cost_volume = cost_volume / C
    
    return cost_volume


def build_group_correlation_volume(left_feat, right_feat, max_disp, num_groups=8):
    """
    Group-wise Correlation (GwcNet 스타일)
    
    채널을 그룹으로 나누어 상관 계산
    → 더 풍부한 매칭 정보
    
    Returns:
    - cost_volume: [B, G, D, H, W]
    """
    
    B, C, H, W = left_feat.shape
    assert C % num_groups == 0
    
    channels_per_group = C // num_groups
    cost_volume = torch.zeros(B, num_groups, max_disp, H, W, device=left_feat.device)
    
    # 그룹으로 분할
    left_groups = left_feat.view(B, num_groups, channels_per_group, H, W)
    right_groups = right_feat.view(B, num_groups, channels_per_group, H, W)
    
    for d in range(max_disp):
        if d == 0:
            cost_volume[:, :, d, :, :] = (left_groups * right_groups).sum(dim=2)
        else:
            cost_volume[:, :, d, :, d:] = (left_groups[:, :, :, :, d:] * 
                                           right_groups[:, :, :, :, :-d]).sum(dim=2)
    
    cost_volume = cost_volume / channels_per_group
    
    return cost_volume
```

### 2.3 Disparity Regression

```python
"""
disparity_regression.py
시차 회귀 방법
"""

import torch
import torch.nn.functional as F


def soft_argmin(cost_volume, temperature=1.0):
    """
    Soft Argmin (Differentiable Argmin)
    
    Cost를 확률로 변환 후 기댓값 계산
    → 서브픽셀 정확도의 시차 출력
    
    Parameters:
    - cost_volume: [B, D, H, W] 비용 볼륨
    - temperature: softmax 온도 (낮을수록 sharp)
    
    Returns:
    - disparity: [B, H, W] 시차 맵
    """
    
    B, D, H, W = cost_volume.shape
    
    # Softmax로 확률 변환 (비용이 낮을수록 높은 확률)
    prob = F.softmax(-cost_volume / temperature, dim=1)  # [B, D, H, W]
    
    # 시차 인덱스
    disp_candidates = torch.arange(D, device=cost_volume.device, dtype=torch.float32)
    disp_candidates = disp_candidates.view(1, D, 1, 1).expand(B, D, H, W)
    
    # 기댓값 계산
    disparity = (prob * disp_candidates).sum(dim=1)  # [B, H, W]
    
    return disparity


def disparity_regression_uncertainty(cost_volume):
    """
    불확실성과 함께 시차 회귀
    
    Returns:
    - disparity: 시차 맵
    - uncertainty: 불확실성 맵 (분산)
    """
    
    B, D, H, W = cost_volume.shape
    
    prob = F.softmax(-cost_volume, dim=1)
    
    disp_candidates = torch.arange(D, device=cost_volume.device, dtype=torch.float32)
    disp_candidates = disp_candidates.view(1, D, 1, 1).expand(B, D, H, W)
    
    # 평균 (시차)
    disparity = (prob * disp_candidates).sum(dim=1)
    
    # 분산 (불확실성)
    disparity_expanded = disparity.unsqueeze(1).expand(B, D, H, W)
    variance = (prob * (disp_candidates - disparity_expanded) ** 2).sum(dim=1)
    uncertainty = torch.sqrt(variance)
    
    return disparity, uncertainty
```

---

## 3. 환경 설정

### 3.1 요구사항

```bash
# 하드웨어 요구사항
- GPU: NVIDIA GPU (최소 6GB VRAM, 권장 8GB+)
- CUDA: 11.0 이상

# 소프트웨어 요구사항
Python >= 3.8
PyTorch >= 1.10
torchvision >= 0.11
CUDA Toolkit >= 11.0
```

### 3.2 환경 설정

```bash
# 가상환경 생성
conda create -n stereo_dl python=3.9
conda activate stereo_dl

# PyTorch 설치 (CUDA 11.8 기준)
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118

# 기본 라이브러리
pip install numpy opencv-python matplotlib tqdm pyyaml

# Open3D (포인트 클라우드용)
pip install open3d

# RAFT-Stereo 추가 의존성
pip install scipy tensorboard

# AANet 추가 의존성
pip install scikit-image
```

### 3.3 GPU 확인

```python
"""
check_gpu.py
GPU 환경 확인
"""

import torch

def check_gpu():
    print("="*50)
    print("GPU 환경 확인")
    print("="*50)
    
    print(f"PyTorch 버전: {torch.__version__}")
    print(f"CUDA 사용 가능: {torch.cuda.is_available()}")
    
    if torch.cuda.is_available():
        print(f"CUDA 버전: {torch.version.cuda}")
        print(f"GPU 개수: {torch.cuda.device_count()}")
        
        for i in range(torch.cuda.device_count()):
            props = torch.cuda.get_device_properties(i)
            print(f"\nGPU {i}: {props.name}")
            print(f"  메모리: {props.total_memory / 1024**3:.1f} GB")
            print(f"  Compute Capability: {props.major}.{props.minor}")
    else:
        print("❌ CUDA를 사용할 수 없습니다.")
        print("CPU 모드로 실행됩니다 (매우 느림).")

if __name__ == "__main__":
    check_gpu()
```

---

## 4. RAFT-Stereo

### 4.1 모델 개요

RAFT-Stereo는 RAFT (Recurrent All-Pairs Field Transforms)를 스테레오 매칭에 적용한 모델입니다.

```
┌─────────────────────────────────────────────────────────────┐
│                   RAFT-Stereo 아키텍처                       │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│   왼쪽 이미지 ──→ Feature     ──┐                           │
│                 Encoder         │                           │
│                                 ├──→ Correlation Volume     │
│   오른쪽 이미지 ──→ Feature    ──┘    (All-Pairs)           │
│                 Encoder                   │                 │
│                                           │                 │
│   왼쪽 이미지 ──→ Context     ──────────┐ │                 │
│                 Encoder                 │ │                 │
│                                         ▼ ▼                 │
│                              ┌──────────────────┐           │
│                              │   GRU Update     │ ←─┐       │
│                              │   (반복적 정제)   │   │       │
│                              └────────┬─────────┘   │       │
│                                       │             │       │
│                                       ▼             │       │
│                              Disparity Update ──────┘       │
│                                       │                     │
│                                       ▼ (N번 반복)          │
│                              Final Disparity Map            │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### 4.2 핵심 구성요소

1. **Feature Encoder**: 이미지에서 특징 추출
2. **Context Encoder**: 문맥 정보 추출
3. **Correlation Volume**: 모든 픽셀 쌍의 상관도 계산
4. **GRU Update**: 반복적으로 시차 추정 정제

### 4.3 RAFT-Stereo 사용법

```python
"""
raft_stereo_inference.py
RAFT-Stereo 추론 코드
"""

import sys
import torch
import numpy as np
import cv2
from pathlib import Path

# RAFT-Stereo 경로 추가 (클론 후)
# git clone https://github.com/princeton-vl/RAFT-Stereo.git
sys.path.append('RAFT-Stereo')

from core.raft_stereo import RAFTStereo
from core.utils.utils import InputPadder


class RAFTStereoInference:
    def __init__(self, model_path, device='cuda'):
        """
        RAFT-Stereo 추론 클래스
        
        Parameters:
        - model_path: 사전 훈련된 모델 경로
        - device: 'cuda' 또는 'cpu'
        """
        
        self.device = torch.device(device if torch.cuda.is_available() else 'cpu')
        
        # 모델 설정 (기본값)
        class Args:
            corr_implementation = "reg"  # 또는 "alt" (메모리 효율적)
            shared_backbone = False
            corr_levels = 4
            corr_radius = 4
            n_downsample = 2
            context_norm = "batch"
            slow_fast_gru = False
            n_gru_layers = 3
            hidden_dims = [128] * 3
            mixed_precision = False
        
        self.args = Args()
        
        # 모델 로드
        self.model = RAFTStereo(self.args)
        
        checkpoint = torch.load(model_path, map_location=self.device)
        self.model.load_state_dict(checkpoint, strict=False)
        
        self.model = self.model.to(self.device)
        self.model.eval()
        
        print(f"✅ RAFT-Stereo 모델 로드 완료: {model_path}")
        print(f"   Device: {self.device}")
    
    def preprocess(self, img_left, img_right):
        """이미지 전처리"""
        
        # BGR → RGB
        if len(img_left.shape) == 3 and img_left.shape[2] == 3:
            img_left = cv2.cvtColor(img_left, cv2.COLOR_BGR2RGB)
            img_right = cv2.cvtColor(img_right, cv2.COLOR_BGR2RGB)
        
        # numpy → tensor
        img_left = torch.from_numpy(img_left).permute(2, 0, 1).float()
        img_right = torch.from_numpy(img_right).permute(2, 0, 1).float()
        
        # 배치 차원 추가
        img_left = img_left.unsqueeze(0).to(self.device)
        img_right = img_right.unsqueeze(0).to(self.device)
        
        # 패딩 (8의 배수로)
        padder = InputPadder(img_left.shape, divis_by=32)
        img_left, img_right = padder.pad(img_left, img_right)
        
        return img_left, img_right, padder
    
    @torch.no_grad()
    def inference(self, img_left, img_right, iters=32):
        """
        시차 맵 추론
        
        Parameters:
        - img_left: 왼쪽 이미지 (BGR, numpy)
        - img_right: 오른쪽 이미지 (BGR, numpy)
        - iters: GRU 반복 횟수 (많을수록 정확, 느림)
        
        Returns:
        - disparity: 시차 맵 (numpy, float32)
        """
        
        # 전처리
        left_tensor, right_tensor, padder = self.preprocess(img_left, img_right)
        
        # 추론
        _, flow_up = self.model(left_tensor, right_tensor, iters=iters, test_mode=True)
        
        # 후처리
        disparity = -flow_up.squeeze().cpu().numpy()
        
        # 패딩 제거
        h, w = img_left.shape[:2]
        disparity = disparity[:h, :w]
        
        return disparity
    
    def inference_with_uncertainty(self, img_left, img_right, iters=32, num_samples=5):
        """
        불확실성 추정과 함께 추론 (MC Dropout 스타일)
        
        Parameters:
        - num_samples: 샘플 수
        
        Returns:
        - disparity_mean: 평균 시차 맵
        - disparity_std: 표준편차 (불확실성)
        """
        
        disparities = []
        
        for _ in range(num_samples):
            disp = self.inference(img_left, img_right, iters=iters)
            disparities.append(disp)
        
        disparities = np.stack(disparities, axis=0)
        disparity_mean = np.mean(disparities, axis=0)
        disparity_std = np.std(disparities, axis=0)
        
        return disparity_mean, disparity_std


def demo_raft_stereo():
    """RAFT-Stereo 데모"""
    
    print("="*60)
    print("RAFT-Stereo 데모")
    print("="*60)
    
    # 모델 경로 (다운로드 필요)
    # https://github.com/princeton-vl/RAFT-Stereo/releases
    model_path = "models/raftstereo-realtime.pth"
    
    # 테스트 이미지
    img_left = cv2.imread("test_left.png")
    img_right = cv2.imread("test_right.png")
    
    if img_left is None or img_right is None:
        print("테스트 이미지를 찾을 수 없습니다.")
        return
    
    # 추론
    raft = RAFTStereoInference(model_path)
    disparity = raft.inference(img_left, img_right, iters=32)
    
    print(f"\n시차 범위: {disparity.min():.1f} ~ {disparity.max():.1f}")
    
    # 시각화
    disp_vis = cv2.normalize(disparity, None, 0, 255, cv2.NORM_MINMAX)
    disp_color = cv2.applyColorMap(disp_vis.astype(np.uint8), cv2.COLORMAP_MAGMA)
    
    cv2.imwrite("raft_stereo_result.png", disp_color)
    print("✅ 저장됨: raft_stereo_result.png")


if __name__ == "__main__":
    demo_raft_stereo()
```

### 4.4 사전 훈련 모델 다운로드

```bash
# RAFT-Stereo 저장소 클론
git clone https://github.com/princeton-vl/RAFT-Stereo.git
cd RAFT-Stereo

# 사전 훈련 모델 다운로드
mkdir -p models
cd models

# ETH3D 훈련 모델 (고정밀)
wget https://www.dropbox.com/s/xxx/raftstereo-eth3d.pth

# Middlebury 훈련 모델
wget https://www.dropbox.com/s/xxx/raftstereo-middlebury.pth

# 실시간 버전 (빠름, 정확도 약간 낮음)
wget https://www.dropbox.com/s/xxx/raftstereo-realtime.pth
```

---

## 5. AANet

### 5.1 모델 개요

AANet (Adaptive Aggregation Network)은 빠른 속도에 초점을 맞춘 모델입니다.

```
┌─────────────────────────────────────────────────────────────┐
│                    AANet 아키텍처                            │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│   왼쪽/오른쪽 이미지                                          │
│         │                                                   │
│         ▼                                                   │
│   ┌─────────────┐                                          │
│   │ Feature     │  다중 스케일 특징 추출                     │
│   │ Extraction  │                                          │
│   └─────────────┘                                          │
│         │                                                   │
│         ▼                                                   │
│   ┌─────────────┐                                          │
│   │ Cost Volume │  상관 기반 (3D Conv 없음!)                 │
│   │ (Sparse)    │  → 메모리 효율적                          │
│   └─────────────┘                                          │
│         │                                                   │
│         ▼                                                   │
│   ┌─────────────┐                                          │
│   │ Adaptive    │  ISA: Intra-Scale Aggregation            │
│   │ Aggregation │  CSA: Cross-Scale Aggregation            │
│   │ Module      │  → 변형 가능한 합성곱 사용                  │
│   └─────────────┘                                          │
│         │                                                   │
│         ▼                                                   │
│   ┌─────────────┐                                          │
│   │ Disparity   │  Soft Argmin                             │
│   │ Regression  │                                          │
│   └─────────────┘                                          │
│         │                                                   │
│         ▼                                                   │
│   Final Disparity Map                                       │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### 5.2 AANet의 핵심 특징

1. **No 3D Convolution**: 메모리와 속도 효율
2. **Adaptive Aggregation**: 변형 가능한 합성곱으로 유연한 집계
3. **Multi-scale**: 다양한 스케일의 정보 활용

### 5.3 AANet 사용법

```python
"""
aanet_inference.py
AANet 추론 코드
"""

import sys
import torch
import numpy as np
import cv2

# AANet 경로 추가
# git clone https://github.com/haofeixu/aanet.git
sys.path.append('aanet')

from nets import AANet


class AANetInference:
    def __init__(self, model_path, max_disp=192, device='cuda'):
        """
        AANet 추론 클래스
        
        Parameters:
        - model_path: 사전 훈련된 모델 경로
        - max_disp: 최대 시차
        - device: 'cuda' 또는 'cpu'
        """
        
        self.device = torch.device(device if torch.cuda.is_available() else 'cpu')
        self.max_disp = max_disp
        
        # 모델 생성
        self.model = AANet(
            max_disp=max_disp,
            num_downsample=2,
            feature_type='aanet',
            no_feature_mdconv=False,
            feature_pyramid=False,
            feature_pyramid_network=True,
            feature_similarity='correlation',
            aggregation_type='adaptive',
            num_scales=3,
            num_fusions=6,
            num_stage_blocks=1,
            num_deform_blocks=3,
            no_intermediate_supervision=True,
            refinement_type='stereodrnet',
            mdconv_dilation=2,
            deformable_groups=2
        )
        
        # 가중치 로드
        checkpoint = torch.load(model_path, map_location=self.device)
        self.model.load_state_dict(checkpoint['state_dict'], strict=False)
        
        self.model = self.model.to(self.device)
        self.model.eval()
        
        print(f"✅ AANet 모델 로드 완료: {model_path}")
        print(f"   최대 시차: {max_disp}")
        print(f"   Device: {self.device}")
    
    def preprocess(self, img_left, img_right):
        """이미지 전처리"""
        
        # BGR → RGB
        img_left = cv2.cvtColor(img_left, cv2.COLOR_BGR2RGB)
        img_right = cv2.cvtColor(img_right, cv2.COLOR_BGR2RGB)
        
        # 정규화
        mean = [0.485, 0.456, 0.406]
        std = [0.229, 0.224, 0.225]
        
        img_left = img_left.astype(np.float32) / 255.0
        img_right = img_right.astype(np.float32) / 255.0
        
        img_left = (img_left - mean) / std
        img_right = (img_right - mean) / std
        
        # numpy → tensor
        img_left = torch.from_numpy(img_left).permute(2, 0, 1).float()
        img_right = torch.from_numpy(img_right).permute(2, 0, 1).float()
        
        # 배치 차원 추가
        img_left = img_left.unsqueeze(0).to(self.device)
        img_right = img_right.unsqueeze(0).to(self.device)
        
        return img_left, img_right
    
    def pad_to_multiple(self, img, multiple=64):
        """이미지를 multiple의 배수로 패딩"""
        
        _, _, h, w = img.shape
        
        new_h = ((h - 1) // multiple + 1) * multiple
        new_w = ((w - 1) // multiple + 1) * multiple
        
        pad_h = new_h - h
        pad_w = new_w - w
        
        img_padded = torch.nn.functional.pad(img, (0, pad_w, 0, pad_h))
        
        return img_padded, (h, w)
    
    @torch.no_grad()
    def inference(self, img_left, img_right):
        """
        시차 맵 추론
        
        Parameters:
        - img_left: 왼쪽 이미지 (BGR, numpy)
        - img_right: 오른쪽 이미지 (BGR, numpy)
        
        Returns:
        - disparity: 시차 맵 (numpy, float32)
        """
        
        original_h, original_w = img_left.shape[:2]
        
        # 전처리
        left_tensor, right_tensor = self.preprocess(img_left, img_right)
        
        # 패딩
        left_tensor, _ = self.pad_to_multiple(left_tensor, 64)
        right_tensor, _ = self.pad_to_multiple(right_tensor, 64)
        
        # 추론
        pred_disp = self.model(left_tensor, right_tensor)[-1]  # 마지막 스케일
        
        # 후처리
        disparity = pred_disp.squeeze().cpu().numpy()
        
        # 원본 크기로 크롭
        disparity = disparity[:original_h, :original_w]
        
        return disparity


def demo_aanet():
    """AANet 데모"""
    
    print("="*60)
    print("AANet 데모")
    print("="*60)
    
    # 모델 경로
    model_path = "models/aanet_kitti15.pth"
    
    # 테스트 이미지
    img_left = cv2.imread("test_left.png")
    img_right = cv2.imread("test_right.png")
    
    if img_left is None or img_right is None:
        print("테스트 이미지를 찾을 수 없습니다.")
        return
    
    # 추론
    aanet = AANetInference(model_path, max_disp=192)
    
    import time
    start = time.time()
    disparity = aanet.inference(img_left, img_right)
    elapsed = time.time() - start
    
    print(f"\n추론 시간: {elapsed*1000:.1f} ms")
    print(f"시차 범위: {disparity.min():.1f} ~ {disparity.max():.1f}")
    
    # 시각화
    disp_vis = cv2.normalize(disparity, None, 0, 255, cv2.NORM_MINMAX)
    disp_color = cv2.applyColorMap(disp_vis.astype(np.uint8), cv2.COLORMAP_MAGMA)
    
    cv2.imwrite("aanet_result.png", disp_color)
    print("✅ 저장됨: aanet_result.png")


if __name__ == "__main__":
    demo_aanet()
```

---

## 6. 기타 주요 모델

### 6.1 모델 요약

| 모델 | 연도 | 특징 | GitHub |
|------|------|------|--------|
| **PSMNet** | 2018 | 피라미드 풀링, 3D Conv | [JiaRenChang/PSMNet](https://github.com/JiaRenChang/PSMNet) |
| **GwcNet** | 2019 | 그룹 상관, 개선된 집계 | [xy-guo/GwcNet](https://github.com/xy-guo/GwcNet) |
| **GA-Net** | 2019 | 가이드 집계, SGA | [feihuzhang/GANet](https://github.com/feihuzhang/GANet) |
| **IGEV-Stereo** | 2023 | 기하학 인코딩, SOTA | [gangweiX/IGEV](https://github.com/gangweiX/IGEV) |
| **Unimatch** | 2023 | 통합 매칭, 범용 | [autonomousvision/unimatch](https://github.com/autonomousvision/unimatch) |

### 6.2 간단한 모델 선택 가이드

```
┌─────────────────────────────────────────────────────────────┐
│                    모델 선택 가이드                          │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  "최고 정확도가 필요해"                                       │
│      → RAFT-Stereo 또는 IGEV-Stereo                        │
│      → 추론 시간: 200-400ms                                 │
│                                                             │
│  "실시간 처리가 필요해" (>10 FPS)                            │
│      → AANet 또는 RAFT-Stereo (realtime)                   │
│      → 추론 시간: 50-100ms                                  │
│                                                             │
│  "메모리가 제한적이야" (<6GB VRAM)                           │
│      → AANet 또는 PSMNet (작은 해상도)                      │
│                                                             │
│  "다양한 도메인에 적용할 거야"                                │
│      → Unimatch (zero-shot 일반화 우수)                    │
│                                                             │
│  "입문/학습 목적이야"                                        │
│      → PSMNet (구조 이해 쉬움, 자료 많음)                    │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

---

## 7. 전통적 방법 vs 딥러닝 비교

### 7.1 비교 코드

```python
"""
compare_methods.py
전통적 방법 vs 딥러닝 비교
"""

import cv2
import numpy as np
import time
import torch


class MethodComparison:
    """스테레오 매칭 방법 비교"""
    
    def __init__(self):
        # SGBM
        self.sgbm = cv2.StereoSGBM_create(
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
        
        # 딥러닝 모델은 별도 로드 필요
        self.raft_stereo = None
        self.aanet = None
    
    def run_sgbm(self, img_left, img_right):
        """SGBM 실행"""
        
        start = time.time()
        disparity = self.sgbm.compute(img_left, img_right).astype(np.float32) / 16.0
        elapsed = time.time() - start
        
        return disparity, elapsed
    
    def run_raft_stereo(self, img_left, img_right, iters=32):
        """RAFT-Stereo 실행"""
        
        if self.raft_stereo is None:
            print("RAFT-Stereo 모델이 로드되지 않았습니다.")
            return None, 0
        
        start = time.time()
        disparity = self.raft_stereo.inference(img_left, img_right, iters=iters)
        elapsed = time.time() - start
        
        return disparity, elapsed
    
    def run_aanet(self, img_left, img_right):
        """AANet 실행"""
        
        if self.aanet is None:
            print("AANet 모델이 로드되지 않았습니다.")
            return None, 0
        
        start = time.time()
        disparity = self.aanet.inference(img_left, img_right)
        elapsed = time.time() - start
        
        return disparity, elapsed
    
    def compute_metrics(self, pred, gt, max_disp=192):
        """
        평가 메트릭 계산
        
        Parameters:
        - pred: 예측 시차 맵
        - gt: Ground Truth 시차 맵
        
        Returns:
        - metrics: 딕셔너리
        """
        
        # 유효한 영역만
        mask = (gt > 0) & (gt < max_disp)
        
        if not np.any(mask):
            return {'valid': False}
        
        pred_valid = pred[mask]
        gt_valid = gt[mask]
        
        # 오차 계산
        error = np.abs(pred_valid - gt_valid)
        
        # EPE (End-Point Error)
        epe = np.mean(error)
        
        # D1 (> 3px 또는 > 5% 오차 비율)
        bad3 = np.mean((error > 3) & (error / gt_valid > 0.05)) * 100
        
        # RMSE
        rmse = np.sqrt(np.mean(error ** 2))
        
        return {
            'valid': True,
            'EPE': epe,
            'D1': bad3,
            'RMSE': rmse,
            'valid_ratio': np.sum(mask) / mask.size * 100
        }
    
    def compare_visual(self, img_left, results, save_path="comparison.png"):
        """
        시각적 비교
        
        Parameters:
        - img_left: 왼쪽 이미지
        - results: {'method_name': (disparity, time), ...}
        """
        
        num_results = len(results)
        fig_height = 300
        fig_width = img_left.shape[1]
        
        # 전체 이미지
        canvas = np.zeros((fig_height * (num_results + 1), fig_width, 3), dtype=np.uint8)
        
        # 원본 이미지
        canvas[:fig_height, :, :] = cv2.resize(img_left, (fig_width, fig_height))
        cv2.putText(canvas, "Input", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                   1, (255, 255, 255), 2)
        
        # 각 방법의 결과
        row = 1
        for name, (disparity, elapsed) in results.items():
            if disparity is None:
                continue
            
            # 시각화
            disp_vis = cv2.normalize(disparity, None, 0, 255, cv2.NORM_MINMAX)
            disp_color = cv2.applyColorMap(disp_vis.astype(np.uint8), cv2.COLORMAP_MAGMA)
            disp_color = cv2.resize(disp_color, (fig_width, fig_height))
            
            y_start = row * fig_height
            canvas[y_start:y_start+fig_height, :, :] = disp_color
            
            # 텍스트
            cv2.putText(canvas, f"{name}: {elapsed*1000:.1f}ms", 
                       (10, y_start + 30), cv2.FONT_HERSHEY_SIMPLEX, 
                       1, (255, 255, 255), 2)
            
            row += 1
        
        cv2.imwrite(save_path, canvas)
        print(f"✅ 비교 이미지 저장됨: {save_path}")
        
        return canvas


def demo_comparison():
    """비교 데모"""
    
    print("="*60)
    print("스테레오 매칭 방법 비교")
    print("="*60)
    
    # 테스트 이미지
    img_left = cv2.imread("test_left.png")
    img_right = cv2.imread("test_right.png")
    
    if img_left is None:
        print("테스트 이미지 없음. 더미 이미지 생성...")
        img_left = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        img_right = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
    
    comp = MethodComparison()
    
    results = {}
    
    # SGBM
    print("\n[1] SGBM 실행...")
    disp_sgbm, time_sgbm = comp.run_sgbm(img_left, img_right)
    results['SGBM'] = (disp_sgbm, time_sgbm)
    print(f"    시간: {time_sgbm*1000:.1f} ms")
    
    # 결과 요약
    print("\n" + "="*60)
    print("결과 요약")
    print("="*60)
    print(f"{'방법':<15} {'시간 (ms)':<15} {'FPS':<10}")
    print("-"*40)
    
    for name, (disp, t) in results.items():
        if disp is not None:
            fps = 1.0 / t if t > 0 else 0
            print(f"{name:<15} {t*1000:<15.1f} {fps:<10.1f}")
    
    # 시각적 비교
    comp.compare_visual(img_left, results)


if __name__ == "__main__":
    demo_comparison()
```

### 7.2 비교 표

| 항목 | SGBM | RAFT-Stereo | AANet |
|------|------|-------------|-------|
| **정확도 (D1)** | ~10% | ~1.3% | ~2% |
| **속도 (640x480)** | ~30ms | ~300ms | ~70ms |
| **메모리** | ~100MB | ~4GB | ~2GB |
| **텍스처 없는 영역** | ❌ 취약 | ✅ 강함 | ✅ 강함 |
| **GPU 필요** | ❌ | ✅ | ✅ |
| **파라미터 튜닝** | 필요 | 불필요 | 불필요 |
| **일반화** | 도메인 의존 | 우수 | 양호 |

---

## 8. 커스텀 데이터 적용

### 8.1 정류 후 딥러닝 적용

```python
"""
custom_data_pipeline.py
커스텀 데이터에 딥러닝 모델 적용
"""

import cv2
import numpy as np
import yaml


class CustomStereoPipeline:
    """커스텀 스테레오 데이터 파이프라인"""
    
    def __init__(self, calibration_file, model_type='sgbm'):
        """
        Parameters:
        - calibration_file: 캘리브레이션 파일 경로
        - model_type: 'sgbm', 'raft', 'aanet'
        """
        
        self.load_calibration(calibration_file)
        self.model_type = model_type
        self.model = None
        
        # SGBM은 기본 생성
        if model_type == 'sgbm':
            self.model = cv2.StereoSGBM_create(
                minDisparity=0,
                numDisparities=128,
                blockSize=5,
                P1=8 * 3 * 5 ** 2,
                P2=32 * 3 * 5 ** 2,
                mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
            )
    
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
        
        # 정류 맵 생성
        self.map1_left, self.map2_left = cv2.initUndistortRectifyMap(
            self.K1, self.D1, self.R1, self.P1, self.img_size, cv2.CV_32FC1
        )
        self.map1_right, self.map2_right = cv2.initUndistortRectifyMap(
            self.K2, self.D2, self.R2, self.P2, self.img_size, cv2.CV_32FC1
        )
        
        print(f"✅ 캘리브레이션 로드: {self.img_size}")
    
    def load_dl_model(self, model_path):
        """딥러닝 모델 로드"""
        
        if self.model_type == 'raft':
            from raft_stereo_inference import RAFTStereoInference
            self.model = RAFTStereoInference(model_path)
        elif self.model_type == 'aanet':
            from aanet_inference import AANetInference
            self.model = AANetInference(model_path)
    
    def rectify(self, img_left, img_right):
        """이미지 정류"""
        
        rect_left = cv2.remap(img_left, self.map1_left, self.map2_left, cv2.INTER_LINEAR)
        rect_right = cv2.remap(img_right, self.map1_right, self.map2_right, cv2.INTER_LINEAR)
        
        return rect_left, rect_right
    
    def compute_disparity(self, rect_left, rect_right):
        """시차 계산"""
        
        if self.model_type == 'sgbm':
            disparity = self.model.compute(rect_left, rect_right).astype(np.float32) / 16.0
        elif self.model_type in ['raft', 'aanet']:
            disparity = self.model.inference(rect_left, rect_right)
        else:
            raise ValueError(f"Unknown model type: {self.model_type}")
        
        return disparity
    
    def disparity_to_depth(self, disparity):
        """시차를 깊이로 변환"""
        
        focal = self.P1[0, 0]
        depth = np.zeros_like(disparity)
        valid = disparity > 0
        depth[valid] = (focal * self.baseline) / disparity[valid]
        
        return depth
    
    def process(self, img_left, img_right):
        """
        전체 파이프라인 실행
        
        Returns:
        - rect_left: 정류된 왼쪽 이미지
        - disparity: 시차 맵
        - depth: 깊이 맵 (mm)
        """
        
        # 1. 정류
        rect_left, rect_right = self.rectify(img_left, img_right)
        
        # 2. 시차 계산
        disparity = self.compute_disparity(rect_left, rect_right)
        
        # 3. 깊이 변환
        depth = self.disparity_to_depth(disparity)
        
        return rect_left, disparity, depth


def demo_custom_pipeline():
    """커스텀 파이프라인 데모"""
    
    print("="*60)
    print("커스텀 데이터 파이프라인")
    print("="*60)
    
    # 파이프라인 생성 (SGBM)
    pipeline = CustomStereoPipeline("stereo_params.yaml", model_type='sgbm')
    
    # 이미지 로드
    img_left = cv2.imread("test_left.png")
    img_right = cv2.imread("test_right.png")
    
    if img_left is None:
        print("테스트 이미지가 없습니다.")
        return
    
    # 처리
    rect_left, disparity, depth = pipeline.process(img_left, img_right)
    
    print(f"\n결과:")
    print(f"  시차 범위: {disparity[disparity > 0].min():.1f} ~ {disparity.max():.1f} px")
    print(f"  깊이 범위: {depth[depth > 0].min():.1f} ~ {depth[depth > 0].max():.1f} mm")
    
    # 시각화
    disp_vis = cv2.normalize(disparity, None, 0, 255, cv2.NORM_MINMAX)
    disp_color = cv2.applyColorMap(disp_vis.astype(np.uint8), cv2.COLORMAP_MAGMA)
    
    cv2.imwrite("custom_disparity.png", disp_color)
    print("✅ 저장됨: custom_disparity.png")


if __name__ == "__main__":
    demo_custom_pipeline()
```

---

## 9. 성능 최적화

### 9.1 추론 속도 최적화

```python
"""
optimization.py
딥러닝 모델 최적화
"""

import torch
import numpy as np
import time


def optimize_with_half_precision(model):
    """
    FP16 (Half Precision) 변환
    
    - 메모리 ~50% 감소
    - 속도 ~2x 향상 (최신 GPU)
    """
    
    model = model.half()
    print("✅ FP16 모드 활성화")
    
    return model


def optimize_with_torch_compile(model):
    """
    PyTorch 2.0 torch.compile 사용
    
    - 자동 최적화
    - 속도 ~1.5-2x 향상
    """
    
    if hasattr(torch, 'compile'):
        model = torch.compile(model, mode='reduce-overhead')
        print("✅ torch.compile 활성화")
    else:
        print("⚠️ torch.compile 미지원 (PyTorch 2.0+ 필요)")
    
    return model


def optimize_with_tensorrt(model, input_shape, save_path="model.engine"):
    """
    TensorRT 변환 (NVIDIA GPU 전용)
    
    - 속도 2-5x 향상
    - 추론 전용
    """
    
    try:
        import torch_tensorrt
        
        # TensorRT 컴파일
        inputs = [torch_tensorrt.Input(shape=input_shape, dtype=torch.float32)]
        
        trt_model = torch_tensorrt.compile(
            model,
            inputs=inputs,
            enabled_precisions={torch.float32, torch.float16},
            workspace_size=1 << 30  # 1GB
        )
        
        torch.jit.save(trt_model, save_path)
        print(f"✅ TensorRT 모델 저장: {save_path}")
        
        return trt_model
    
    except ImportError:
        print("⚠️ torch_tensorrt 미설치")
        return model


def benchmark_inference(model, input_left, input_right, num_runs=50, warmup=10):
    """
    추론 속도 벤치마크
    
    Parameters:
    - model: 모델
    - input_left, input_right: 입력 텐서
    - num_runs: 측정 횟수
    - warmup: 워밍업 횟수
    """
    
    device = next(model.parameters()).device
    
    # GPU 동기화 함수
    if device.type == 'cuda':
        sync = torch.cuda.synchronize
    else:
        sync = lambda: None
    
    # 워밍업
    print("워밍업 중...")
    with torch.no_grad():
        for _ in range(warmup):
            _ = model(input_left, input_right)
            sync()
    
    # 벤치마크
    print(f"벤치마크 실행 ({num_runs}회)...")
    times = []
    
    with torch.no_grad():
        for _ in range(num_runs):
            sync()
            start = time.perf_counter()
            
            _ = model(input_left, input_right)
            
            sync()
            elapsed = time.perf_counter() - start
            times.append(elapsed)
    
    times = np.array(times)
    
    print("\n" + "="*50)
    print("벤치마크 결과")
    print("="*50)
    print(f"평균 시간: {times.mean()*1000:.2f} ms")
    print(f"표준편차: {times.std()*1000:.2f} ms")
    print(f"최소 시간: {times.min()*1000:.2f} ms")
    print(f"최대 시간: {times.max()*1000:.2f} ms")
    print(f"FPS: {1/times.mean():.1f}")
    
    return times
```

### 9.2 해상도별 속도 가이드

```
┌─────────────────────────────────────────────────────────────┐
│              해상도별 추론 속도 가이드 (RTX 3080 기준)        │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  해상도        RAFT-Stereo    AANet      권장 용도          │
│  ─────────────────────────────────────────────────────────  │
│  320×240       ~50ms          ~20ms      실시간 임베디드     │
│  640×480       ~150ms         ~50ms      실시간 PC          │
│  1280×720      ~400ms         ~120ms     준실시간           │
│  1920×1080     ~800ms         ~250ms     오프라인 처리      │
│                                                             │
│  * iters=12 (RAFT-Stereo 실시간 모드)                       │
│  * FP16 모드 사용 시 ~30% 추가 향상                         │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

---

## 10. 실습 프로젝트

### 10.1 종합 비교 시스템

```python
"""
stereo_dl_demo.py
딥러닝 스테레오 종합 데모
"""

import cv2
import numpy as np
import time
import argparse


def main():
    parser = argparse.ArgumentParser(description='딥러닝 스테레오 데모')
    parser.add_argument('--left', type=str, required=True, help='왼쪽 이미지')
    parser.add_argument('--right', type=str, required=True, help='오른쪽 이미지')
    parser.add_argument('--calibration', type=str, default=None, help='캘리브레이션 파일')
    parser.add_argument('--method', type=str, default='sgbm', 
                       choices=['sgbm', 'raft', 'aanet', 'all'])
    parser.add_argument('--output', type=str, default='result.png')
    args = parser.parse_args()
    
    print("="*60)
    print("딥러닝 스테레오 매칭 데모")
    print("="*60)
    
    # 이미지 로드
    img_left = cv2.imread(args.left)
    img_right = cv2.imread(args.right)
    
    if img_left is None or img_right is None:
        print("❌ 이미지 로드 실패")
        return
    
    print(f"이미지 크기: {img_left.shape}")
    
    # 정류 (캘리브레이션 있는 경우)
    if args.calibration:
        from custom_data_pipeline import CustomStereoPipeline
        pipeline = CustomStereoPipeline(args.calibration, model_type='sgbm')
        img_left, img_right = pipeline.rectify(img_left, img_right)
        print("✅ 정류 완료")
    
    results = {}
    
    # SGBM
    if args.method in ['sgbm', 'all']:
        print("\n[SGBM] 실행 중...")
        sgbm = cv2.StereoSGBM_create(
            minDisparity=0, numDisparities=128, blockSize=5,
            P1=8*3*5**2, P2=32*3*5**2,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
        )
        
        start = time.time()
        disp_sgbm = sgbm.compute(img_left, img_right).astype(np.float32) / 16.0
        time_sgbm = time.time() - start
        
        results['SGBM'] = (disp_sgbm, time_sgbm)
        print(f"  시간: {time_sgbm*1000:.1f}ms, FPS: {1/time_sgbm:.1f}")
    
    # 결과 시각화
    num_results = len(results)
    h, w = img_left.shape[:2]
    
    canvas = np.zeros((h * 2, w * max(1, num_results), 3), dtype=np.uint8)
    
    # 원본 이미지
    canvas[:h, :w] = img_left
    cv2.putText(canvas, "Input (Rectified)", (10, 30),
               cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    
    # 결과
    col = 0
    for name, (disp, t) in results.items():
        disp_vis = cv2.normalize(disp, None, 0, 255, cv2.NORM_MINMAX)
        disp_vis[disp <= 0] = 0
        disp_color = cv2.applyColorMap(disp_vis.astype(np.uint8), cv2.COLORMAP_MAGMA)
        
        x_start = col * w
        canvas[h:, x_start:x_start+w] = disp_color
        
        cv2.putText(canvas, f"{name}: {t*1000:.1f}ms", (x_start+10, h+30),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
        col += 1
    
    cv2.imwrite(args.output, canvas)
    print(f"\n✅ 결과 저장: {args.output}")
    
    # 디스플레이
    cv2.imshow("Result", cv2.resize(canvas, (canvas.shape[1]//2, canvas.shape[0]//2)))
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
```

---

## 📝 학습 체크리스트

### 이론 이해

- [ ] Cost Volume의 개념과 구성 방법을 이해했다
- [ ] Soft Argmin의 원리를 설명할 수 있다
- [ ] RAFT-Stereo의 반복적 업데이트 구조를 이해했다
- [ ] AANet의 적응형 집계 개념을 알고 있다
- [ ] 전통적 방법과 딥러닝의 장단점을 비교할 수 있다

### 실습 완료

- [ ] 딥러닝 환경 설정 (PyTorch, CUDA)
- [ ] RAFT-Stereo 또는 AANet 추론 실행
- [ ] 전통적 방법 (SGBM)과 비교
- [ ] 커스텀 데이터에 적용
- [ ] 추론 속도 벤치마크

---

## ➡️ 다음 모듈

**[Module 06: ROS2 연동 개발](../Module_06_ROS2/README.md)**

다음 모듈에서는:
- ROS2 기초 및 설치
- 스테레오 카메라 ROS2 노드 개발
- 깊이 이미지 퍼블리시
- 포인트 클라우드 퍼블리시
- Rviz2 시각화

를 학습합니다.

---

## 📄 라이선스

MIT License - 자유롭게 사용, 수정, 배포 가능
