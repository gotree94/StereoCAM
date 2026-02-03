# Module 02: 카메라 캘리브레이션 실습

[![난이도](https://img.shields.io/badge/난이도-⭐⭐⭐_중급-yellow.svg)]()
[![예상시간](https://img.shields.io/badge/예상시간-10--15시간-blue.svg)]()
[![선수지식](https://img.shields.io/badge/선수지식-Module_01_완료-orange.svg)]()

---

## 📋 모듈 개요

| 항목 | 내용 |
|------|------|
| **학습 목표** | USB 웹캠 2대의 스테레오 캘리브레이션 완전 마스터 |
| **핵심 키워드** | 체스보드, 내부 파라미터, 외부 파라미터, 정류 맵 |
| **산출물** | 캘리브레이션 파라미터 파일 (.yaml), 정류된 이미지 |

---

## 📚 목차

1. [캘리브레이션 개요](#1-캘리브레이션-개요) : 전체 파이프라인 흐름도, 필요한 파라미터 설명
2. [하드웨어 준비](#2-하드웨어-준비) : 마운트 구성, 카메라 인덱스 확인 방법
3. [체스보드 패턴 준비](#3-체스보드-패턴-준비) : 패턴 사양, 생성 코드, 준비 팁
4. [캘리브레이션 이미지 캡처](#4-캘리브레이션-이미지-캡처) : 캡처 가이드라인, 캡처 프로그램 코드
5. [단일 카메라 캘리브레이션](#5-단일-카메라-캘리브레이션) : K, D 추출 이론 및 구현
6. [스테레오 캘리브레이션](#6-스테레오-캘리브레이션) : R, T 추출, 전체 구현 클래스
7. [스테레오 정류](#7-스테레오-정류) : 정류 맵 생성, 시각화
8. [캘리브레이션 검증](#8-캘리브레이션-검증) : 품질 지표, 검증 코드
9. [실습 코드](#9-실습-코드) : 전체 파이프라인 통합
10. [트러블슈팅](#10-트러블슈팅) : 일반적인 문제와 해결책


📁 포함된 코드
  * generate_checkerboard.py - 체스보드 패턴 생성
  * capture_calibration_images.py - 캘리브레이션 이미지 캡처
  * single_camera_calibration.py - 단일 카메라 캘리브레이션
  * stereo_calibration.py - 스테레오 캘리브레이션 클래스
  * visualize_rectification.py - 정류 결과 시각화
  * validate_calibration.py - 캘리브레이션 검증
  * full_calibration_pipeline.py - 전체 파이프라인

---

## 1. 캘리브레이션 개요

### 1.1 캘리브레이션이 필요한 이유

```
실제 카메라                      이상적인 카메라
┌──────────────┐                ┌──────────────┐
│  ╭──────╮    │                │  ┌──────┐    │
│  │ 왜곡  │    │      →        │  │ 직선  │    │
│  │ 발생  │    │   캘리브레이션  │  │ 유지  │    │
│  ╰──────╯    │                │  └──────┘    │
└──────────────┘                └──────────────┘
```

캘리브레이션으로 얻는 것:

| 파라미터 | 내용 | 용도 |
|----------|------|------|
| **내부 파라미터 (K)** | 초점거리, 주점 | 투영 계산 |
| **왜곡 계수 (D)** | 방사/접선 왜곡 | 이미지 보정 |
| **외부 파라미터 (R, T)** | 카메라 간 회전/이동 | 스테레오 기하 |

### 1.2 캘리브레이션 파이프라인

```
┌─────────────────────────────────────────────────────────────┐
│                  캘리브레이션 전체 흐름                        │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  [1] 체스보드 준비        [2] 이미지 캡처 (30장+)             │
│         │                        │                         │
│         ▼                        ▼                         │
│  ┌─────────────┐          ┌─────────────┐                  │
│  │ 9x6 패턴    │          │ 다양한 각도  │                  │
│  │ 25mm 정사각 │          │ 다양한 거리  │                  │
│  └─────────────┘          └─────────────┘                  │
│                                  │                         │
│         ┌────────────────────────┼────────────────────┐    │
│         ▼                        ▼                    ▼    │
│  [3] 단일 캘리브레이션     [4] 스테레오 캘리브레이션           │
│  ┌─────────────┐          ┌─────────────┐                  │
│  │ K1, D1      │          │ R, T        │                  │
│  │ K2, D2      │          │ E, F        │                  │
│  └─────────────┘          └─────────────┘                  │
│         │                        │                         │
│         └────────────┬───────────┘                         │
│                      ▼                                     │
│              [5] 스테레오 정류                               │
│              ┌─────────────┐                               │
│              │ R1, R2      │                               │
│              │ P1, P2      │                               │
│              │ Q           │                               │
│              └─────────────┘                               │
│                      │                                     │
│                      ▼                                     │
│              [6] 검증 및 저장                                │
│              ┌─────────────┐                               │
│              │ .yaml 파일  │                               │
│              └─────────────┘                               │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

---

## 2. 하드웨어 준비

### 2.1 스테레오 카메라 마운트

```
                    ┌─────────────────────────────────────┐
                    │          마운트 플레이트             │
                    │                                     │
     ┌──────┐       │       ← 85mm+ →                    │       ┌──────┐
     │ 📷   │───────┼───────────────────────────────────┼───────│  📷  │
     │ LEFT │       │                                     │       │RIGHT │
     └──────┘       │                                     │       └──────┘
                    └─────────────────────────────────────┘
                                    │
                                    │ 삼각대
                                   ╱╲
```

### 2.2 중요 체크리스트

- [ ] 두 카메라가 **동일 모델**인가?
- [ ] 베이스라인(카메라 간격)이 **85mm 이상**인가?
- [ ] 두 카메라 렌즈가 **같은 높이**에 있는가?
- [ ] 두 카메라가 **평행하게** 정렬되었는가?
- [ ] 마운트가 **단단히 고정**되어 흔들림이 없는가?
- [ ] USB 케이블이 **각각 다른 USB 컨트롤러**에 연결되었는가?

### 2.3 카메라 인덱스 확인

```bash
# Linux에서 카메라 장치 확인
ls /dev/video*

# 일반적인 결과:
# /dev/video0  ← 왼쪽 카메라 (또는 내장 웹캠)
# /dev/video1  ← 메타데이터
# /dev/video2  ← 오른쪽 카메라
# /dev/video3  ← 메타데이터
```

```python
# Python으로 카메라 인덱스 찾기
import cv2

for i in range(10):
    cap = cv2.VideoCapture(i)
    if cap.isOpened():
        ret, frame = cap.read()
        if ret:
            print(f"Camera index {i}: Available ({frame.shape})")
        cap.release()
```

---

## 3. 체스보드 패턴 준비

### 3.1 패턴 사양

| 항목 | 권장값 | 설명 |
|------|--------|------|
| 내부 코너 수 | 9×6 또는 10×7 | 가로×세로 코너 개수 |
| 정사각형 크기 | 25mm | 측정 정확도를 위해 정확해야 함 |
| 재질 | 단단한 판 | 평평해야 함 (종이 휘어짐 주의) |
| 색상 | 흑백, 고대비 | 코너 검출 정확도 |

### 3.2 체스보드 생성

```python
"""
generate_checkerboard.py
캘리브레이션용 체스보드 패턴 생성
"""

import numpy as np
import cv2

def generate_checkerboard(rows=7, cols=10, square_size_mm=25, dpi=300):
    """
    인쇄용 체스보드 패턴 생성
    
    Parameters:
    - rows: 세로 정사각형 개수
    - cols: 가로 정사각형 개수
    - square_size_mm: 정사각형 한 변 길이 (mm)
    - dpi: 인쇄 해상도
    """
    
    # 픽셀 크기 계산
    mm_to_inch = 1 / 25.4
    square_size_px = int(square_size_mm * mm_to_inch * dpi)
    
    # 이미지 생성
    img_height = rows * square_size_px
    img_width = cols * square_size_px
    
    checkerboard = np.zeros((img_height, img_width), dtype=np.uint8)
    
    for i in range(rows):
        for j in range(cols):
            if (i + j) % 2 == 0:
                y1 = i * square_size_px
                y2 = (i + 1) * square_size_px
                x1 = j * square_size_px
                x2 = (j + 1) * square_size_px
                checkerboard[y1:y2, x1:x2] = 255
    
    # 여백 추가
    margin = square_size_px
    bordered = np.ones((img_height + 2*margin, img_width + 2*margin), dtype=np.uint8) * 255
    bordered[margin:margin+img_height, margin:margin+img_width] = checkerboard
    
    # 정보 텍스트 추가
    info_text = f"Checkerboard {cols}x{rows} | Square: {square_size_mm}mm | Inner corners: {cols-1}x{rows-1}"
    cv2.putText(bordered, info_text, (margin, margin//2), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, 0, 1)
    
    return bordered

if __name__ == "__main__":
    # 체스보드 생성 (10x7 정사각형 = 9x6 내부 코너)
    checkerboard = generate_checkerboard(rows=7, cols=10, square_size_mm=25, dpi=300)
    
    # 저장
    cv2.imwrite("checkerboard_10x7_25mm.png", checkerboard)
    print("✅ 체스보드 저장됨: checkerboard_10x7_25mm.png")
    print("   A4 용지에 '실제 크기'로 인쇄하세요.")
    print(f"   내부 코너: 9 x 6")
    print(f"   정사각형 크기: 25mm")
```

### 3.3 체스보드 준비 팁

1. **정확한 크기로 인쇄**: 프린터 설정에서 "실제 크기" 또는 "100%" 선택
2. **크기 검증**: 자로 정사각형 크기 측정 (오차 ±0.5mm 이내)
3. **평평한 판에 부착**: 폼보드, MDF, 아크릴 판 등
4. **휘어짐 방지**: 양면 테이프로 전체 면적 부착

---

## 4. 캘리브레이션 이미지 캡처

### 4.1 캡처 가이드라인

```
┌─────────────────────────────────────────────────────────────┐
│                    이미지 캡처 가이드                        │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ✅ 올바른 예                    ❌ 잘못된 예               │
│                                                             │
│  ┌─────────┐                    ┌─────────┐                │
│  │ ▦▦▦▦▦▦  │ 전체 보임           │  ▦▦▦    │ 잘림          │
│  │ ▦▦▦▦▦▦  │                    │  ▦▦▦    │               │
│  └─────────┘                    └─────────┘                │
│                                                             │
│  ┌─────────┐                    ┌─────────┐                │
│  │   ◇◇◇   │ 기울어진 각도       │ ▦▦▦▦▦▦  │ 항상 정면만   │
│  │  ◇◇◇    │ (다양성 필요)       │ ▦▦▦▦▦▦  │ (다양성 부족) │
│  └─────────┘                    └─────────┘                │
│                                                             │
│  ┌─────────┐                    ┌─────────┐                │
│  │▦        │ 이미지 가장자리      │         │ 중앙에만      │
│  │    ▦    │ 전체 활용           │   ▦▦▦   │ (왜곡 보정↓)  │
│  │       ▦ │                    │         │               │
│  └─────────┘                    └─────────┘                │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### 4.2 캡처 요구사항

| 요구사항 | 최소 | 권장 | 설명 |
|----------|------|------|------|
| 이미지 수 | 15쌍 | 30-50쌍 | 많을수록 정확 |
| 체스보드 각도 | 3가지 | 다양하게 | X, Y, Z축 회전 |
| 체스보드 거리 | 2가지 | 다양하게 | 0.5m ~ 2m |
| 체스보드 위치 | 중앙만 | 전체 영역 | 특히 가장자리 중요 |

### 4.3 캡처 프로그램

```python
"""
capture_calibration_images.py
스테레오 캘리브레이션용 이미지 캡처
"""

import cv2
import os
from datetime import datetime

class StereoCalibrationCapture:
    def __init__(self, left_idx=0, right_idx=2, width=1920, height=1080):
        """
        Parameters:
        - left_idx: 왼쪽 카메라 인덱스
        - right_idx: 오른쪽 카메라 인덱스
        - width, height: 해상도
        """
        self.cap_left = cv2.VideoCapture(left_idx)
        self.cap_right = cv2.VideoCapture(right_idx)
        
        # 해상도 설정
        for cap in [self.cap_left, self.cap_right]:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        
        # 저장 디렉토리
        self.save_dir = f"calibration_images_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        os.makedirs(f"{self.save_dir}/left", exist_ok=True)
        os.makedirs(f"{self.save_dir}/right", exist_ok=True)
        
        self.image_count = 0
        
        # 체스보드 파라미터
        self.checkerboard_size = (9, 6)  # 내부 코너 수
        
    def capture_loop(self):
        """메인 캡처 루프"""
        
        print("="*60)
        print("스테레오 캘리브레이션 이미지 캡처")
        print("="*60)
        print(f"저장 위치: {self.save_dir}")
        print("-"*60)
        print("조작 방법:")
        print("  [SPACE] - 이미지 캡처 (체스보드 검출 시)")
        print("  [Q]     - 종료")
        print("="*60)
        
        while True:
            ret_l, frame_l = self.cap_left.read()
            ret_r, frame_r = self.cap_right.read()
            
            if not ret_l or not ret_r:
                print("❌ 카메라 읽기 실패")
                break
            
            # 체스보드 검출
            gray_l = cv2.cvtColor(frame_l, cv2.COLOR_BGR2GRAY)
            gray_r = cv2.cvtColor(frame_r, cv2.COLOR_BGR2GRAY)
            
            found_l, corners_l = cv2.findChessboardCorners(
                gray_l, self.checkerboard_size,
                cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
            )
            found_r, corners_r = cv2.findChessboardCorners(
                gray_r, self.checkerboard_size,
                cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
            )
            
            # 디스플레이용 복사본
            display_l = frame_l.copy()
            display_r = frame_r.copy()
            
            # 체스보드 표시
            if found_l:
                cv2.drawChessboardCorners(display_l, self.checkerboard_size, corners_l, found_l)
            if found_r:
                cv2.drawChessboardCorners(display_r, self.checkerboard_size, corners_r, found_r)
            
            # 상태 표시
            status_l = "✓ DETECTED" if found_l else "✗ Not found"
            status_r = "✓ DETECTED" if found_r else "✗ Not found"
            color_l = (0, 255, 0) if found_l else (0, 0, 255)
            color_r = (0, 255, 0) if found_r else (0, 0, 255)
            
            cv2.putText(display_l, f"LEFT: {status_l}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, color_l, 2)
            cv2.putText(display_r, f"RIGHT: {status_r}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, color_r, 2)
            cv2.putText(display_l, f"Captured: {self.image_count}", (10, 70),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)
            
            # 두 이미지 합치기
            display_l_resized = cv2.resize(display_l, (640, 360))
            display_r_resized = cv2.resize(display_r, (640, 360))
            combined = cv2.hconcat([display_l_resized, display_r_resized])
            
            cv2.imshow("Stereo Calibration Capture", combined)
            
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord(' '):  # SPACE: 캡처
                if found_l and found_r:
                    # 이미지 저장
                    cv2.imwrite(f"{self.save_dir}/left/img_{self.image_count:03d}.png", frame_l)
                    cv2.imwrite(f"{self.save_dir}/right/img_{self.image_count:03d}.png", frame_r)
                    self.image_count += 1
                    print(f"✅ 캡처 완료: {self.image_count}쌍")
                else:
                    print("❌ 양쪽 모두에서 체스보드가 검출되어야 합니다.")
                    
            elif key == ord('q'):  # Q: 종료
                break
        
        self.cleanup()
        
    def cleanup(self):
        """리소스 정리"""
        self.cap_left.release()
        self.cap_right.release()
        cv2.destroyAllWindows()
        
        print("="*60)
        print(f"✅ 캡처 완료: 총 {self.image_count}쌍")
        print(f"📁 저장 위치: {self.save_dir}")
        print("="*60)


if __name__ == "__main__":
    # 카메라 인덱스는 시스템에 맞게 조정하세요
    capture = StereoCalibrationCapture(left_idx=0, right_idx=2)
    capture.capture_loop()
```

---

## 5. 단일 카메라 캘리브레이션

### 5.1 이론

단일 카메라 캘리브레이션은 다음을 추출합니다:

```
내부 파라미터 행렬 K:
┌               ┐
│ fx   0   cx  │     fx, fy: 초점거리 (픽셀)
│  0  fy   cy  │     cx, cy: 주점 (이미지 중심)
│  0   0    1  │
└               ┘

왜곡 계수 D:
D = [k1, k2, p1, p2, k3]

k1, k2, k3: 방사 왜곡 계수
p1, p2: 접선 왜곡 계수
```

### 5.2 구현

```python
"""
single_camera_calibration.py
단일 카메라 캘리브레이션
"""

import cv2
import numpy as np
import glob
import yaml


def calibrate_single_camera(image_dir, checkerboard_size=(9, 6), square_size=25.0):
    """
    단일 카메라 캘리브레이션
    
    Parameters:
    - image_dir: 이미지 디렉토리 경로
    - checkerboard_size: 내부 코너 수 (가로, 세로)
    - square_size: 정사각형 크기 (mm)
    
    Returns:
    - ret: RMS 재투영 오차
    - K: 내부 파라미터 행렬 (3x3)
    - D: 왜곡 계수 (1x5)
    - rvecs: 회전 벡터 리스트
    - tvecs: 병진 벡터 리스트
    """
    
    # 3D 객체 점 준비 (체스보드 좌표)
    objp = np.zeros((checkerboard_size[0] * checkerboard_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:checkerboard_size[0], 0:checkerboard_size[1]].T.reshape(-1, 2)
    objp *= square_size  # 실제 크기로 스케일링
    
    obj_points = []  # 3D 점
    img_points = []  # 2D 점
    
    # 이미지 파일 로드
    images = sorted(glob.glob(f"{image_dir}/*.png"))
    
    if len(images) == 0:
        print(f"❌ 이미지를 찾을 수 없습니다: {image_dir}")
        return None, None, None, None, None
    
    print(f"📁 이미지 로드: {len(images)}개")
    
    img_shape = None
    successful = 0
    
    for img_path in images:
        img = cv2.imread(img_path)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        if img_shape is None:
            img_shape = gray.shape[::-1]  # (width, height)
        
        # 체스보드 코너 검출
        ret, corners = cv2.findChessboardCorners(
            gray, checkerboard_size,
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
        )
        
        if ret:
            # 서브픽셀 정확도로 코너 정밀화
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            
            obj_points.append(objp)
            img_points.append(corners_refined)
            successful += 1
    
    print(f"✅ 체스보드 검출: {successful}/{len(images)}개 성공")
    
    if successful < 10:
        print("❌ 최소 10개 이상의 이미지가 필요합니다.")
        return None, None, None, None, None
    
    # 캘리브레이션 실행
    print("🔄 캘리브레이션 진행 중...")
    
    ret, K, D, rvecs, tvecs = cv2.calibrateCamera(
        obj_points, img_points, img_shape, None, None,
        flags=cv2.CALIB_FIX_K3  # k3는 보통 0으로 고정
    )
    
    print(f"\n{'='*50}")
    print("📊 캘리브레이션 결과")
    print(f"{'='*50}")
    print(f"RMS 재투영 오차: {ret:.4f} 픽셀")
    print(f"\n내부 파라미터 행렬 K:")
    print(f"  fx = {K[0,0]:.2f}")
    print(f"  fy = {K[1,1]:.2f}")
    print(f"  cx = {K[0,2]:.2f}")
    print(f"  cy = {K[1,2]:.2f}")
    print(f"\n왜곡 계수 D:")
    print(f"  k1 = {D[0,0]:.6f}")
    print(f"  k2 = {D[0,1]:.6f}")
    print(f"  p1 = {D[0,2]:.6f}")
    print(f"  p2 = {D[0,3]:.6f}")
    print(f"  k3 = {D[0,4]:.6f}")
    
    return ret, K, D, rvecs, tvecs


if __name__ == "__main__":
    # 사용 예시
    ret, K, D, rvecs, tvecs = calibrate_single_camera(
        "calibration_images/left",
        checkerboard_size=(9, 6),
        square_size=25.0
    )
```

---

## 6. 스테레오 캘리브레이션

### 6.1 이론

스테레오 캘리브레이션은 두 카메라 간의 상대적 위치 관계를 구합니다:

```
오른쪽 카메라 좌표 = R × 왼쪽 카메라 좌표 + T

R: 3×3 회전 행렬 (두 카메라 간 회전)
T: 3×1 병진 벡터 (두 카메라 간 이동, 베이스라인 포함)

이상적인 경우 (완벽히 평행 정렬):
    ┌           ┐         ┌      ┐
R = │ 1  0  0   │    T =  │ -85  │  (베이스라인 85mm)
    │ 0  1  0   │         │  0   │
    │ 0  0  1   │         │  0   │
    └           ┘         └      ┘
```

### 6.2 구현

```python
"""
stereo_calibration.py
스테레오 카메라 캘리브레이션
"""

import cv2
import numpy as np
import glob
import yaml
import os


class StereoCalibrator:
    def __init__(self, checkerboard_size=(9, 6), square_size=25.0):
        """
        Parameters:
        - checkerboard_size: 내부 코너 수 (가로, 세로)
        - square_size: 정사각형 크기 (mm)
        """
        self.checkerboard_size = checkerboard_size
        self.square_size = square_size
        
        # 3D 객체 점 준비
        self.objp = np.zeros((checkerboard_size[0] * checkerboard_size[1], 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:checkerboard_size[0], 0:checkerboard_size[1]].T.reshape(-1, 2)
        self.objp *= square_size
        
        # 결과 저장용
        self.K1 = None
        self.D1 = None
        self.K2 = None
        self.D2 = None
        self.R = None
        self.T = None
        self.E = None
        self.F = None
        self.R1 = None
        self.R2 = None
        self.P1 = None
        self.P2 = None
        self.Q = None
        self.img_size = None
        
    def find_corners(self, left_dir, right_dir):
        """양쪽 이미지에서 체스보드 코너 검출"""
        
        left_images = sorted(glob.glob(f"{left_dir}/*.png"))
        right_images = sorted(glob.glob(f"{right_dir}/*.png"))
        
        if len(left_images) != len(right_images):
            print(f"❌ 이미지 수 불일치: Left={len(left_images)}, Right={len(right_images)}")
            return None, None, None
        
        obj_points = []
        left_points = []
        right_points = []
        
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        
        print(f"📁 이미지 처리 중: {len(left_images)}쌍")
        
        for i, (left_path, right_path) in enumerate(zip(left_images, right_images)):
            img_l = cv2.imread(left_path)
            img_r = cv2.imread(right_path)
            
            gray_l = cv2.cvtColor(img_l, cv2.COLOR_BGR2GRAY)
            gray_r = cv2.cvtColor(img_r, cv2.COLOR_BGR2GRAY)
            
            if self.img_size is None:
                self.img_size = gray_l.shape[::-1]
            
            # 코너 검출
            ret_l, corners_l = cv2.findChessboardCorners(
                gray_l, self.checkerboard_size,
                cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
            )
            ret_r, corners_r = cv2.findChessboardCorners(
                gray_r, self.checkerboard_size,
                cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
            )
            
            if ret_l and ret_r:
                # 서브픽셀 정밀화
                corners_l = cv2.cornerSubPix(gray_l, corners_l, (11, 11), (-1, -1), criteria)
                corners_r = cv2.cornerSubPix(gray_r, corners_r, (11, 11), (-1, -1), criteria)
                
                obj_points.append(self.objp)
                left_points.append(corners_l)
                right_points.append(corners_r)
                
        print(f"✅ 코너 검출 성공: {len(obj_points)}/{len(left_images)}쌍")
        
        return obj_points, left_points, right_points
    
    def calibrate(self, left_dir, right_dir):
        """스테레오 캘리브레이션 실행"""
        
        # 코너 검출
        obj_points, left_points, right_points = self.find_corners(left_dir, right_dir)
        
        if obj_points is None or len(obj_points) < 10:
            print("❌ 캘리브레이션을 위한 충분한 이미지가 없습니다.")
            return False
        
        print("\n🔄 단일 카메라 캘리브레이션...")
        
        # 왼쪽 카메라 캘리브레이션
        ret1, self.K1, self.D1, _, _ = cv2.calibrateCamera(
            obj_points, left_points, self.img_size, None, None
        )
        print(f"  왼쪽 카메라 RMS 오차: {ret1:.4f}")
        
        # 오른쪽 카메라 캘리브레이션
        ret2, self.K2, self.D2, _, _ = cv2.calibrateCamera(
            obj_points, right_points, self.img_size, None, None
        )
        print(f"  오른쪽 카메라 RMS 오차: {ret2:.4f}")
        
        print("\n🔄 스테레오 캘리브레이션...")
        
        # 스테레오 캘리브레이션
        flags = cv2.CALIB_FIX_INTRINSIC  # 내부 파라미터 고정
        
        ret, self.K1, self.D1, self.K2, self.D2, self.R, self.T, self.E, self.F = \
            cv2.stereoCalibrate(
                obj_points, left_points, right_points,
                self.K1, self.D1, self.K2, self.D2,
                self.img_size, flags=flags,
                criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-5)
            )
        
        print(f"\n{'='*60}")
        print("📊 스테레오 캘리브레이션 결과")
        print(f"{'='*60}")
        print(f"RMS 재투영 오차: {ret:.4f} 픽셀")
        
        # 베이스라인 계산
        baseline = np.linalg.norm(self.T)
        print(f"\n베이스라인: {baseline:.2f} mm")
        
        print(f"\n병진 벡터 T (mm):")
        print(f"  Tx = {self.T[0,0]:.2f}")
        print(f"  Ty = {self.T[1,0]:.2f}")
        print(f"  Tz = {self.T[2,0]:.2f}")
        
        # 회전 각도 계산
        rvec, _ = cv2.Rodrigues(self.R)
        angles = np.degrees(rvec.flatten())
        print(f"\n회전 각도 (degrees):")
        print(f"  Rx = {angles[0]:.3f}°")
        print(f"  Ry = {angles[1]:.3f}°")
        print(f"  Rz = {angles[2]:.3f}°")
        
        return True
    
    def rectify(self, alpha=0):
        """
        스테레오 정류 계산
        
        Parameters:
        - alpha: 0=유효픽셀만, 1=모든픽셀
        """
        
        if self.K1 is None:
            print("❌ 먼저 calibrate()를 실행하세요.")
            return False
        
        print("\n🔄 스테레오 정류 계산...")
        
        self.R1, self.R2, self.P1, self.P2, self.Q, roi1, roi2 = cv2.stereoRectify(
            self.K1, self.D1, self.K2, self.D2,
            self.img_size, self.R, self.T,
            flags=cv2.CALIB_ZERO_DISPARITY,
            alpha=alpha
        )
        
        print(f"  왼쪽 ROI: {roi1}")
        print(f"  오른쪽 ROI: {roi2}")
        
        # 정류 맵 생성
        self.map1_left, self.map2_left = cv2.initUndistortRectifyMap(
            self.K1, self.D1, self.R1, self.P1, self.img_size, cv2.CV_32FC1
        )
        self.map1_right, self.map2_right = cv2.initUndistortRectifyMap(
            self.K2, self.D2, self.R2, self.P2, self.img_size, cv2.CV_32FC1
        )
        
        print("✅ 정류 맵 생성 완료")
        
        return True
    
    def save_parameters(self, filename="stereo_params.yaml"):
        """캘리브레이션 파라미터 저장"""
        
        params = {
            'image_size': list(self.img_size),
            'K1': self.K1.tolist(),
            'D1': self.D1.tolist(),
            'K2': self.K2.tolist(),
            'D2': self.D2.tolist(),
            'R': self.R.tolist(),
            'T': self.T.tolist(),
            'E': self.E.tolist(),
            'F': self.F.tolist(),
            'R1': self.R1.tolist(),
            'R2': self.R2.tolist(),
            'P1': self.P1.tolist(),
            'P2': self.P2.tolist(),
            'Q': self.Q.tolist(),
            'baseline_mm': float(np.linalg.norm(self.T))
        }
        
        with open(filename, 'w') as f:
            yaml.dump(params, f, default_flow_style=False)
        
        print(f"\n✅ 파라미터 저장: {filename}")
        
    def load_parameters(self, filename="stereo_params.yaml"):
        """캘리브레이션 파라미터 로드"""
        
        with open(filename, 'r') as f:
            params = yaml.safe_load(f)
        
        self.img_size = tuple(params['image_size'])
        self.K1 = np.array(params['K1'])
        self.D1 = np.array(params['D1'])
        self.K2 = np.array(params['K2'])
        self.D2 = np.array(params['D2'])
        self.R = np.array(params['R'])
        self.T = np.array(params['T'])
        self.E = np.array(params['E'])
        self.F = np.array(params['F'])
        self.R1 = np.array(params['R1'])
        self.R2 = np.array(params['R2'])
        self.P1 = np.array(params['P1'])
        self.P2 = np.array(params['P2'])
        self.Q = np.array(params['Q'])
        
        # 정류 맵 재생성
        self.map1_left, self.map2_left = cv2.initUndistortRectifyMap(
            self.K1, self.D1, self.R1, self.P1, self.img_size, cv2.CV_32FC1
        )
        self.map1_right, self.map2_right = cv2.initUndistortRectifyMap(
            self.K2, self.D2, self.R2, self.P2, self.img_size, cv2.CV_32FC1
        )
        
        print(f"✅ 파라미터 로드: {filename}")
        
    def rectify_images(self, img_left, img_right):
        """이미지 정류 적용"""
        
        rectified_left = cv2.remap(img_left, self.map1_left, self.map2_left, cv2.INTER_LINEAR)
        rectified_right = cv2.remap(img_right, self.map1_right, self.map2_right, cv2.INTER_LINEAR)
        
        return rectified_left, rectified_right


if __name__ == "__main__":
    # 사용 예시
    calibrator = StereoCalibrator(checkerboard_size=(9, 6), square_size=25.0)
    
    # 캘리브레이션 실행
    if calibrator.calibrate("calibration_images/left", "calibration_images/right"):
        # 정류 계산
        calibrator.rectify(alpha=0)
        
        # 파라미터 저장
        calibrator.save_parameters("stereo_params.yaml")
```

---

## 7. 스테레오 정류

### 7.1 정류 결과 시각화

```python
"""
visualize_rectification.py
정류 결과 시각화
"""

import cv2
import numpy as np


def draw_epipolar_lines(img_left, img_right, num_lines=20):
    """
    정류된 이미지에 에피폴라 라인 그리기
    정류가 잘 되었으면 수평선이 양쪽 이미지에서 같은 높이를 지남
    """
    
    h, w = img_left.shape[:2]
    
    # 복사본 생성
    vis_left = img_left.copy()
    vis_right = img_right.copy()
    
    # 수평선 그리기
    colors = [(0, 0, 255), (0, 255, 0), (255, 0, 0), (255, 255, 0), (255, 0, 255)]
    
    for i in range(num_lines):
        y = int(h * (i + 1) / (num_lines + 1))
        color = colors[i % len(colors)]
        cv2.line(vis_left, (0, y), (w, y), color, 1)
        cv2.line(vis_right, (0, y), (w, y), color, 1)
    
    # 합치기
    combined = cv2.hconcat([vis_left, vis_right])
    
    # 연결선 (중앙)
    for i in range(num_lines):
        y = int(h * (i + 1) / (num_lines + 1))
        color = colors[i % len(colors)]
        cv2.line(combined, (w-5, y), (w+5, y), color, 2)
    
    return combined


def compare_before_after(img_left, img_right, calibrator):
    """정류 전후 비교"""
    
    # 정류 적용
    rect_left, rect_right = calibrator.rectify_images(img_left, img_right)
    
    # 에피폴라 라인 그리기
    before = draw_epipolar_lines(img_left, img_right, num_lines=15)
    after = draw_epipolar_lines(rect_left, rect_right, num_lines=15)
    
    # 크기 조정
    scale = 0.5
    before = cv2.resize(before, None, fx=scale, fy=scale)
    after = cv2.resize(after, None, fx=scale, fy=scale)
    
    # 레이블 추가
    cv2.putText(before, "BEFORE Rectification", (10, 30),
               cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
    cv2.putText(after, "AFTER Rectification", (10, 30),
               cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    
    # 세로로 합치기
    result = cv2.vconcat([before, after])
    
    return result


if __name__ == "__main__":
    from stereo_calibration import StereoCalibrator
    
    # 캘리브레이터 로드
    calibrator = StereoCalibrator()
    calibrator.load_parameters("stereo_params.yaml")
    
    # 테스트 이미지 로드
    img_left = cv2.imread("test_left.png")
    img_right = cv2.imread("test_right.png")
    
    # 비교 이미지 생성
    comparison = compare_before_after(img_left, img_right, calibrator)
    
    cv2.imshow("Rectification Comparison", comparison)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    cv2.imwrite("rectification_comparison.png", comparison)
    print("✅ 저장됨: rectification_comparison.png")
```

---

## 8. 캘리브레이션 검증

### 8.1 품질 지표

| 지표 | 양호 | 보통 | 불량 |
|------|------|------|------|
| RMS 재투영 오차 | < 0.5 px | 0.5-1.0 px | > 1.0 px |
| 에피폴라 라인 정렬 | < 0.5 px | 0.5-1.0 px | > 1.0 px |
| 베이스라인 오차 | < 1% | 1-3% | > 3% |

### 8.2 검증 코드

```python
"""
validate_calibration.py
캘리브레이션 품질 검증
"""

import cv2
import numpy as np


def compute_epipolar_error(calibrator, left_points, right_points):
    """
    에피폴라 라인 오차 계산
    정류 후 대응점들이 같은 행에 있는지 검증
    """
    
    errors = []
    
    for lp, rp in zip(left_points, right_points):
        # 정류 적용
        lp_rect = cv2.undistortPoints(lp, calibrator.K1, calibrator.D1, 
                                       R=calibrator.R1, P=calibrator.P1)
        rp_rect = cv2.undistortPoints(rp, calibrator.K2, calibrator.D2,
                                       R=calibrator.R2, P=calibrator.P2)
        
        # y좌표 차이 계산
        for l, r in zip(lp_rect.reshape(-1, 2), rp_rect.reshape(-1, 2)):
            error = abs(l[1] - r[1])  # y좌표 차이
            errors.append(error)
    
    errors = np.array(errors)
    
    print(f"\n{'='*50}")
    print("📊 에피폴라 오차 분석")
    print(f"{'='*50}")
    print(f"평균 오차: {np.mean(errors):.4f} 픽셀")
    print(f"최대 오차: {np.max(errors):.4f} 픽셀")
    print(f"표준편차: {np.std(errors):.4f} 픽셀")
    print(f"90% 이내: {np.percentile(errors, 90):.4f} 픽셀")
    
    return errors


def validate_baseline(calibrator, expected_baseline_mm):
    """베이스라인 검증"""
    
    measured_baseline = np.linalg.norm(calibrator.T)
    error_percent = abs(measured_baseline - expected_baseline_mm) / expected_baseline_mm * 100
    
    print(f"\n{'='*50}")
    print("📊 베이스라인 검증")
    print(f"{'='*50}")
    print(f"예상 베이스라인: {expected_baseline_mm:.2f} mm")
    print(f"측정 베이스라인: {measured_baseline:.2f} mm")
    print(f"오차: {error_percent:.2f}%")
    
    if error_percent < 1:
        print("✅ 베이스라인 정확도: 양호")
    elif error_percent < 3:
        print("⚠️ 베이스라인 정확도: 보통")
    else:
        print("❌ 베이스라인 정확도: 불량 - 재캘리브레이션 권장")
    
    return error_percent


def full_validation(calibrator, left_dir, right_dir, expected_baseline_mm=85.0):
    """전체 검증 실행"""
    
    print("\n" + "="*60)
    print("🔍 캘리브레이션 품질 검증")
    print("="*60)
    
    # 코너 검출
    obj_points, left_points, right_points = calibrator.find_corners(left_dir, right_dir)
    
    # 에피폴라 오차
    errors = compute_epipolar_error(calibrator, left_points, right_points)
    
    # 베이스라인 검증
    baseline_error = validate_baseline(calibrator, expected_baseline_mm)
    
    # 종합 평가
    print(f"\n{'='*60}")
    print("📋 종합 평가")
    print(f"{'='*60}")
    
    avg_error = np.mean(errors)
    
    if avg_error < 0.5 and baseline_error < 1:
        print("✅ 캘리브레이션 품질: 우수")
        print("   스테레오 매칭에 적합합니다.")
    elif avg_error < 1.0 and baseline_error < 3:
        print("⚠️ 캘리브레이션 품질: 양호")
        print("   일반적인 사용에 적합합니다.")
    else:
        print("❌ 캘리브레이션 품질: 불량")
        print("   재캘리브레이션을 권장합니다.")
        print("\n권장사항:")
        print("  1. 더 많은 이미지 사용 (30장 이상)")
        print("  2. 다양한 각도와 위치에서 촬영")
        print("  3. 이미지 가장자리에도 체스보드 배치")
        print("  4. 체스보드 평평한지 확인")
```

---

## 9. 실습 코드

### 9.1 전체 파이프라인

```python
"""
full_calibration_pipeline.py
스테레오 캘리브레이션 전체 파이프라인
"""

import cv2
import numpy as np
import os
from stereo_calibration import StereoCalibrator
from validate_calibration import full_validation


def run_full_pipeline():
    """캘리브레이션 전체 파이프라인 실행"""
    
    print("="*60)
    print("🎯 스테레오 카메라 캘리브레이션 파이프라인")
    print("="*60)
    
    # 설정
    CHECKERBOARD_SIZE = (9, 6)  # 내부 코너 수
    SQUARE_SIZE = 25.0          # mm
    EXPECTED_BASELINE = 85.0    # mm
    
    LEFT_DIR = "calibration_images/left"
    RIGHT_DIR = "calibration_images/right"
    OUTPUT_FILE = "stereo_params.yaml"
    
    # 1. 이미지 확인
    print("\n[1/5] 이미지 확인...")
    left_count = len([f for f in os.listdir(LEFT_DIR) if f.endswith('.png')])
    right_count = len([f for f in os.listdir(RIGHT_DIR) if f.endswith('.png')])
    print(f"  왼쪽 이미지: {left_count}개")
    print(f"  오른쪽 이미지: {right_count}개")
    
    if left_count < 15 or right_count < 15:
        print("❌ 최소 15쌍 이상의 이미지가 필요합니다.")
        return
    
    # 2. 캘리브레이터 초기화
    print("\n[2/5] 캘리브레이터 초기화...")
    calibrator = StereoCalibrator(
        checkerboard_size=CHECKERBOARD_SIZE,
        square_size=SQUARE_SIZE
    )
    
    # 3. 캘리브레이션 실행
    print("\n[3/5] 스테레오 캘리브레이션 실행...")
    if not calibrator.calibrate(LEFT_DIR, RIGHT_DIR):
        print("❌ 캘리브레이션 실패")
        return
    
    # 4. 정류 계산
    print("\n[4/5] 스테레오 정류 계산...")
    calibrator.rectify(alpha=0)
    
    # 5. 검증
    print("\n[5/5] 캘리브레이션 검증...")
    full_validation(calibrator, LEFT_DIR, RIGHT_DIR, EXPECTED_BASELINE)
    
    # 저장
    calibrator.save_parameters(OUTPUT_FILE)
    
    print("\n" + "="*60)
    print("✅ 캘리브레이션 완료!")
    print(f"📁 파라미터 저장: {OUTPUT_FILE}")
    print("="*60)
    
    # 결과 시각화
    print("\n정류 결과를 시각화하시겠습니까? (y/n): ", end="")
    if input().lower() == 'y':
        visualize_result(calibrator, LEFT_DIR, RIGHT_DIR)


def visualize_result(calibrator, left_dir, right_dir):
    """결과 시각화"""
    
    import glob
    
    left_images = sorted(glob.glob(f"{left_dir}/*.png"))
    right_images = sorted(glob.glob(f"{right_dir}/*.png"))
    
    # 첫 번째 이미지 쌍 사용
    img_left = cv2.imread(left_images[0])
    img_right = cv2.imread(right_images[0])
    
    # 정류 적용
    rect_left, rect_right = calibrator.rectify_images(img_left, img_right)
    
    # 에피폴라 라인 그리기
    h = rect_left.shape[0]
    for y in range(0, h, 50):
        color = tuple(np.random.randint(0, 255, 3).tolist())
        cv2.line(rect_left, (0, y), (rect_left.shape[1], y), color, 1)
        cv2.line(rect_right, (0, y), (rect_right.shape[1], y), color, 1)
    
    # 합치기
    combined = cv2.hconcat([
        cv2.resize(rect_left, (640, 360)),
        cv2.resize(rect_right, (640, 360))
    ])
    
    cv2.imshow("Rectified Stereo (press any key to close)", combined)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    cv2.imwrite("rectified_result.png", combined)
    print("✅ 저장됨: rectified_result.png")


if __name__ == "__main__":
    run_full_pipeline()
```

---

## 10. 트러블슈팅

### 10.1 일반적인 문제와 해결책

| 문제 | 원인 | 해결책 |
|------|------|--------|
| 체스보드 검출 실패 | 조명 부족, 반사 | 조명 개선, 무광 체스보드 |
| RMS 오차 > 1.0 | 이미지 부족, 다양성 부족 | 더 많은 이미지, 다양한 각도 |
| 베이스라인 오차 큼 | 캘리브레이션 이미지 품질 | 정사각형 크기 정확히 측정 |
| 정류 후 이미지 잘림 | alpha=0 설정 | alpha=1로 변경 또는 ROI 사용 |
| 카메라 동기화 문제 | 프레임 불일치 | 캡처 시 정지 상태 유지 |

### 10.2 체스보드 검출 개선

```python
def find_corners_robust(gray, checkerboard_size):
    """강건한 코너 검출"""
    
    # 방법 1: 기본
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
    ret, corners = cv2.findChessboardCorners(gray, checkerboard_size, flags)
    
    if ret:
        return ret, corners
    
    # 방법 2: 빠른 검색 추가
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK
    ret, corners = cv2.findChessboardCorners(gray, checkerboard_size, flags)
    
    if ret:
        return ret, corners
    
    # 방법 3: 이미지 전처리
    gray_eq = cv2.equalizeHist(gray)
    ret, corners = cv2.findChessboardCorners(gray_eq, checkerboard_size, flags)
    
    if ret:
        return ret, corners
    
    # 방법 4: 가우시안 블러
    gray_blur = cv2.GaussianBlur(gray, (5, 5), 0)
    ret, corners = cv2.findChessboardCorners(gray_blur, checkerboard_size, flags)
    
    return ret, corners
```

---

## 📝 학습 체크리스트

### 이론 이해

- [ ] 내부 파라미터와 외부 파라미터의 차이를 설명할 수 있다
- [ ] 왜곡 계수의 의미를 이해했다
- [ ] 스테레오 정류의 목적을 설명할 수 있다
- [ ] Q 행렬을 이용한 3D 복원 원리를 이해했다

### 실습 완료

- [ ] 체스보드 패턴 생성 및 인쇄
- [ ] 캘리브레이션 이미지 30쌍 이상 캡처
- [ ] 단일 카메라 캘리브레이션 완료
- [ ] 스테레오 캘리브레이션 완료
- [ ] 정류 맵 생성 및 적용
- [ ] 캘리브레이션 검증 (RMS < 1.0)
- [ ] 파라미터 파일 저장 (.yaml)

---

## ➡️ 다음 모듈

**[Module 03: 스테레오 매칭 & 깊이 추정](../Module_03_Stereo_Matching/README.md)**

다음 모듈에서는 캘리브레이션된 스테레오 카메라를 사용하여:
- Block Matching (BM) 알고리즘
- Semi-Global Block Matching (SGBM) 알고리즘
- Disparity Map 생성
- Depth Map 변환

을 실습합니다.

---

## 📄 라이선스

MIT License - 자유롭게 사용, 수정, 배포 가능
