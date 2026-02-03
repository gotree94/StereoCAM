# Module 04: 3D 포인트 클라우드 생성

[![난이도](https://img.shields.io/badge/난이도-⭐⭐⭐_중급-yellow.svg)]()
[![예상시간](https://img.shields.io/badge/예상시간-8--12시간-blue.svg)]()
[![선수지식](https://img.shields.io/badge/선수지식-Module_03_완료-orange.svg)]()

---

## 📋 모듈 개요

| 항목 | 내용 |
|------|------|
| **학습 목표** | 깊이 맵을 3D 포인트 클라우드로 변환하고 시각화/처리 |
| **핵심 키워드** | Point Cloud, Open3D, 필터링, 메쉬 생성, PLY/PCD |
| **산출물** | 3D 포인트 클라우드 뷰어, 포인트 클라우드 처리 파이프라인 |

---

## 📚 목차

1. [포인트 클라우드 개요](#1-포인트-클라우드-개요)
2. [깊이 맵에서 3D 변환](#2-깊이-맵에서-3d-변환)
3. [Open3D 기초](#3-open3d-기초)
4. [포인트 클라우드 필터링](#4-포인트-클라우드-필터링)
5. [법선 벡터 추정](#5-법선-벡터-추정)
6. [포인트 클라우드 정합](#6-포인트-클라우드-정합)
7. [메쉬 생성](#7-메쉬-생성)
8. [파일 입출력](#8-파일-입출력)
9. [실시간 3D 시각화](#9-실시간-3d-시각화)
10. [실습 프로젝트](#10-실습-프로젝트)

---

## 1. 포인트 클라우드 개요

### 1.1 포인트 클라우드란?

3D 공간의 점들의 집합으로, 각 점은 위치(X, Y, Z)와 선택적으로 색상(R, G, B), 법선 벡터 등의 속성을 가집니다.

```
포인트 클라우드 구조:
┌─────────────────────────────────────────────────────────────┐
│                                                             │
│   Point[0]: (X₀, Y₀, Z₀, R₀, G₀, B₀, Nx₀, Ny₀, Nz₀)       │
│   Point[1]: (X₁, Y₁, Z₁, R₁, G₁, B₁, Nx₁, Ny₁, Nz₁)       │
│   Point[2]: (X₂, Y₂, Z₂, R₂, G₂, B₂, Nx₂, Ny₂, Nz₂)       │
│   ...                                                       │
│   Point[N]: (Xₙ, Yₙ, Zₙ, Rₙ, Gₙ, Bₙ, Nxₙ, Nyₙ, Nzₙ)       │
│                                                             │
│   위치 (필수)    색상 (선택)    법선 (선택)                   │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### 1.2 스테레오 비전에서의 활용

```
┌─────────────────────────────────────────────────────────────┐
│                                                             │
│   스테레오 이미지     →     깊이 맵     →    포인트 클라우드   │
│   ┌─────┐ ┌─────┐         ┌─────┐           ·  · ·  ·      │
│   │ 📷 │ │ 📷 │    →     │▓▓▓▓▓│     →    ·  ·····  ·     │
│   └─────┘ └─────┘         │▓▓░░▓│          · ······· ·     │
│    Left   Right           │░░░░░│           ·········      │
│                           └─────┘            (3D 점들)      │
│                                                             │
│   2D 이미지 쌍        2.5D 표현           3D 표현           │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### 1.3 주요 라이브러리 비교

| 라이브러리 | 특징 | 용도 |
|-----------|------|------|
| **Open3D** | Python 친화적, 시각화 우수 | 연구, 프로토타이핑 |
| **PCL** | C++, 기능 풍부, 빠름 | 산업용, 로보틱스 |
| **PyVista** | VTK 기반, 과학 시각화 | 시뮬레이션 |
| **Trimesh** | 메쉬 처리 특화 | CAD, 3D 프린팅 |

---

## 2. 깊이 맵에서 3D 변환

### 2.1 변환 원리

```
이미지 좌표 (u, v) + 깊이 Z  →  3D 좌표 (X, Y, Z)

공식:
    X = (u - cx) × Z / fx
    Y = (v - cy) × Z / fy
    Z = depth[v, u]

여기서:
    (cx, cy): 주점 (이미지 중심)
    (fx, fy): 초점거리 (픽셀 단위)
```

### 2.2 OpenCV를 이용한 변환

```python
"""
depth_to_pointcloud.py
깊이 맵을 포인트 클라우드로 변환
"""

import cv2
import numpy as np
import open3d as o3d


class DepthToPointCloud:
    def __init__(self, fx, fy, cx, cy):
        """
        깊이 맵 → 포인트 클라우드 변환기
        
        Parameters:
        - fx, fy: 초점거리 (픽셀)
        - cx, cy: 주점 (픽셀)
        """
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy
        
    @classmethod
    def from_calibration(cls, calibration_file):
        """캘리브레이션 파일에서 생성"""
        import yaml
        
        with open(calibration_file, 'r') as f:
            params = yaml.safe_load(f)
        
        P1 = np.array(params['P1'])
        fx = P1[0, 0]
        fy = P1[1, 1]
        cx = P1[0, 2]
        cy = P1[1, 2]
        
        return cls(fx, fy, cx, cy)
    
    @classmethod
    def from_Q_matrix(cls, Q):
        """Q 행렬에서 생성"""
        # Q 행렬 구조:
        # [1  0  0  -cx ]
        # [0  1  0  -cy ]
        # [0  0  0   f  ]
        # [0  0 -1/B  0 ]
        
        cx = -Q[0, 3]
        cy = -Q[1, 3]
        f = Q[2, 3]
        
        return cls(f, f, cx, cy)
    
    def convert(self, depth, color_image=None, depth_scale=1.0, 
                max_depth=5000, min_depth=100):
        """
        깊이 맵을 포인트 클라우드로 변환
        
        Parameters:
        - depth: 깊이 맵 (mm 단위)
        - color_image: 컬러 이미지 (BGR, 선택)
        - depth_scale: 깊이 스케일 (기본 1.0)
        - max_depth: 최대 깊이 (mm)
        - min_depth: 최소 깊이 (mm)
        
        Returns:
        - points: Nx3 포인트 배열
        - colors: Nx3 컬러 배열 (0-1 범위) 또는 None
        """
        
        h, w = depth.shape
        
        # 유효한 깊이 마스크
        valid_mask = (depth > min_depth) & (depth < max_depth)
        
        # 픽셀 좌표 그리드 생성
        u = np.arange(w)
        v = np.arange(h)
        u, v = np.meshgrid(u, v)
        
        # 3D 좌표 계산
        z = depth * depth_scale
        x = (u - self.cx) * z / self.fx
        y = (v - self.cy) * z / self.fy
        
        # 유효한 점만 추출
        points = np.stack([x[valid_mask], y[valid_mask], z[valid_mask]], axis=-1)
        
        # 컬러 추출
        colors = None
        if color_image is not None:
            if len(color_image.shape) == 2:
                # 그레이스케일
                gray = color_image[valid_mask]
                colors = np.stack([gray, gray, gray], axis=-1) / 255.0
            else:
                # BGR → RGB, 정규화
                colors = color_image[valid_mask][:, ::-1] / 255.0
        
        return points, colors
    
    def convert_with_Q(self, disparity, Q, color_image=None, 
                       max_depth=5000, min_depth=100):
        """
        Q 행렬을 이용한 변환 (OpenCV reprojectImageTo3D 사용)
        
        Parameters:
        - disparity: 시차 맵
        - Q: 4x4 시차-깊이 매핑 행렬
        - color_image: 컬러 이미지 (선택)
        
        Returns:
        - points: Nx3 포인트 배열
        - colors: Nx3 컬러 배열 또는 None
        """
        
        # 3D 재투영
        points_3d = cv2.reprojectImageTo3D(disparity, Q)
        
        # 유효한 깊이 마스크
        z = points_3d[:, :, 2]
        valid_mask = (z > min_depth) & (z < max_depth) & (disparity > 0)
        
        # 유효한 점만 추출
        points = points_3d[valid_mask]
        
        # 컬러 추출
        colors = None
        if color_image is not None:
            colors = color_image[valid_mask][:, ::-1] / 255.0
        
        return points, colors
    
    def to_open3d(self, points, colors=None):
        """
        Open3D PointCloud 객체로 변환
        
        Parameters:
        - points: Nx3 포인트 배열
        - colors: Nx3 컬러 배열 (0-1 범위)
        
        Returns:
        - pcd: Open3D PointCloud 객체
        """
        
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        
        if colors is not None:
            pcd.colors = o3d.utility.Vector3dVector(colors)
        
        return pcd


def create_pointcloud_from_stereo(rect_left, disparity, calibration_file):
    """
    스테레오 결과에서 포인트 클라우드 생성
    
    Parameters:
    - rect_left: 정류된 왼쪽 이미지
    - disparity: 시차 맵
    - calibration_file: 캘리브레이션 파일
    
    Returns:
    - pcd: Open3D PointCloud
    """
    
    import yaml
    
    # 캘리브레이션 로드
    with open(calibration_file, 'r') as f:
        params = yaml.safe_load(f)
    
    Q = np.array(params['Q'])
    
    # 변환기 생성
    converter = DepthToPointCloud.from_Q_matrix(Q)
    
    # 포인트 클라우드 생성
    points, colors = converter.convert_with_Q(disparity, Q, rect_left)
    
    # Open3D 객체로 변환
    pcd = converter.to_open3d(points, colors)
    
    print(f"✅ 포인트 클라우드 생성: {len(points):,} points")
    
    return pcd


if __name__ == "__main__":
    # 테스트
    converter = DepthToPointCloud(fx=1317, fy=1317, cx=960, cy=540)
    
    # 더미 데이터
    depth = np.random.uniform(500, 3000, (1080, 1920)).astype(np.float32)
    color = np.random.randint(0, 255, (1080, 1920, 3), dtype=np.uint8)
    
    points, colors = converter.convert(depth, color)
    print(f"생성된 포인트 수: {len(points):,}")
    
    pcd = converter.to_open3d(points, colors)
    print(f"Open3D PointCloud: {pcd}")
```

---

## 3. Open3D 기초

### 3.1 설치

```bash
pip install open3d
```

### 3.2 기본 사용법

```python
"""
open3d_basics.py
Open3D 기본 사용법
"""

import open3d as o3d
import numpy as np


def create_sample_pointcloud():
    """샘플 포인트 클라우드 생성"""
    
    # 무작위 포인트 생성
    points = np.random.rand(10000, 3) * 100  # 0-100 범위
    colors = np.random.rand(10000, 3)  # 0-1 범위
    
    # Open3D PointCloud 생성
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    
    return pcd


def visualize_pointcloud(pcd, window_name="Point Cloud Viewer"):
    """포인트 클라우드 시각화"""
    
    # 좌표축 추가
    coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=50, origin=[0, 0, 0]
    )
    
    # 시각화
    o3d.visualization.draw_geometries(
        [pcd, coord_frame],
        window_name=window_name,
        width=1280,
        height=720,
        left=50,
        top=50,
        point_show_normal=False
    )


def visualize_with_custom_view(pcd):
    """커스텀 뷰 설정"""
    
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="Custom View", width=1280, height=720)
    
    # 포인트 클라우드 추가
    vis.add_geometry(pcd)
    
    # 렌더링 옵션
    opt = vis.get_render_option()
    opt.point_size = 2.0
    opt.background_color = np.array([0.1, 0.1, 0.1])  # 어두운 배경
    opt.show_coordinate_frame = True
    
    # 뷰 컨트롤
    ctr = vis.get_view_control()
    ctr.set_zoom(0.8)
    ctr.set_front([0, 0, -1])  # 카메라 방향
    ctr.set_up([0, -1, 0])      # 위쪽 방향
    
    vis.run()
    vis.destroy_window()


def get_pointcloud_info(pcd):
    """포인트 클라우드 정보 출력"""
    
    points = np.asarray(pcd.points)
    
    print("\n" + "="*50)
    print("포인트 클라우드 정보")
    print("="*50)
    print(f"포인트 수: {len(points):,}")
    print(f"색상 있음: {pcd.has_colors()}")
    print(f"법선 있음: {pcd.has_normals()}")
    
    if len(points) > 0:
        print(f"\n좌표 범위:")
        print(f"  X: {points[:, 0].min():.2f} ~ {points[:, 0].max():.2f}")
        print(f"  Y: {points[:, 1].min():.2f} ~ {points[:, 1].max():.2f}")
        print(f"  Z: {points[:, 2].min():.2f} ~ {points[:, 2].max():.2f}")
        
        # 바운딩 박스
        bbox = pcd.get_axis_aligned_bounding_box()
        print(f"\n바운딩 박스:")
        print(f"  Min: {bbox.min_bound}")
        print(f"  Max: {bbox.max_bound}")
        print(f"  크기: {bbox.get_extent()}")
    
    return points


if __name__ == "__main__":
    # 샘플 생성
    pcd = create_sample_pointcloud()
    
    # 정보 출력
    get_pointcloud_info(pcd)
    
    # 시각화
    print("\n시각화 창을 닫으면 프로그램이 종료됩니다.")
    visualize_pointcloud(pcd)
```

### 3.3 시각화 컨트롤

```
┌─────────────────────────────────────────────────────────────┐
│                Open3D 시각화 단축키                          │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  마우스 조작:                                                │
│    좌클릭 드래그  - 회전                                     │
│    휠 드래그      - 이동                                     │
│    휠 스크롤      - 확대/축소                                │
│    우클릭 드래그  - 이동                                     │
│                                                             │
│  키보드 단축키:                                              │
│    H             - 도움말 표시                               │
│    R             - 뷰 리셋                                   │
│    +/-           - 포인트 크기 조절                          │
│    N             - 법선 표시 토글                            │
│    L             - 조명 토글                                 │
│    Q / ESC       - 종료                                      │
│    S             - 스크린샷 저장                             │
│    P             - 현재 뷰 파라미터 출력                      │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

---

## 4. 포인트 클라우드 필터링

### 4.1 필터링 종류

```python
"""
pointcloud_filtering.py
포인트 클라우드 필터링
"""

import open3d as o3d
import numpy as np


class PointCloudFilter:
    """포인트 클라우드 필터링 도구"""
    
    @staticmethod
    def statistical_outlier_removal(pcd, nb_neighbors=20, std_ratio=2.0):
        """
        통계적 이상치 제거
        
        각 점의 이웃들과의 평균 거리를 계산하고,
        평균 + std_ratio * 표준편차 보다 먼 점을 제거
        
        Parameters:
        - nb_neighbors: 이웃 수
        - std_ratio: 표준편차 비율
        
        Returns:
        - filtered_pcd: 필터링된 포인트 클라우드
        - inlier_indices: 유효한 점의 인덱스
        """
        
        filtered_pcd, inlier_indices = pcd.remove_statistical_outlier(
            nb_neighbors=nb_neighbors,
            std_ratio=std_ratio
        )
        
        outlier_count = len(pcd.points) - len(filtered_pcd.points)
        print(f"통계적 이상치 제거: {outlier_count:,} points removed")
        
        return filtered_pcd, inlier_indices
    
    @staticmethod
    def radius_outlier_removal(pcd, nb_points=16, radius=5.0):
        """
        반경 기반 이상치 제거
        
        주어진 반경 내에 최소 개수의 이웃이 없는 점 제거
        
        Parameters:
        - nb_points: 최소 이웃 수
        - radius: 검색 반경
        
        Returns:
        - filtered_pcd: 필터링된 포인트 클라우드
        - inlier_indices: 유효한 점의 인덱스
        """
        
        filtered_pcd, inlier_indices = pcd.remove_radius_outlier(
            nb_points=nb_points,
            radius=radius
        )
        
        outlier_count = len(pcd.points) - len(filtered_pcd.points)
        print(f"반경 기반 이상치 제거: {outlier_count:,} points removed")
        
        return filtered_pcd, inlier_indices
    
    @staticmethod
    def voxel_downsample(pcd, voxel_size=5.0):
        """
        복셀 다운샘플링
        
        3D 공간을 복셀로 나누고, 각 복셀 내의 점들을 하나로 합침
        
        Parameters:
        - voxel_size: 복셀 크기 (mm)
        
        Returns:
        - downsampled_pcd: 다운샘플링된 포인트 클라우드
        """
        
        original_count = len(pcd.points)
        downsampled_pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
        new_count = len(downsampled_pcd.points)
        
        reduction = (1 - new_count / original_count) * 100
        print(f"복셀 다운샘플링: {original_count:,} → {new_count:,} ({reduction:.1f}% 감소)")
        
        return downsampled_pcd
    
    @staticmethod
    def uniform_downsample(pcd, every_k_points=5):
        """
        균일 다운샘플링
        
        매 k번째 점만 선택
        
        Parameters:
        - every_k_points: 선택 간격
        
        Returns:
        - downsampled_pcd: 다운샘플링된 포인트 클라우드
        """
        
        original_count = len(pcd.points)
        downsampled_pcd = pcd.uniform_down_sample(every_k_points=every_k_points)
        new_count = len(downsampled_pcd.points)
        
        print(f"균일 다운샘플링: {original_count:,} → {new_count:,}")
        
        return downsampled_pcd
    
    @staticmethod
    def crop_by_bbox(pcd, min_bound, max_bound):
        """
        바운딩 박스로 크롭
        
        Parameters:
        - min_bound: [x_min, y_min, z_min]
        - max_bound: [x_max, y_max, z_max]
        
        Returns:
        - cropped_pcd: 크롭된 포인트 클라우드
        """
        
        bbox = o3d.geometry.AxisAlignedBoundingBox(
            min_bound=np.array(min_bound),
            max_bound=np.array(max_bound)
        )
        
        cropped_pcd = pcd.crop(bbox)
        
        print(f"바운딩 박스 크롭: {len(pcd.points):,} → {len(cropped_pcd.points):,}")
        
        return cropped_pcd
    
    @staticmethod
    def remove_by_depth_range(pcd, min_depth, max_depth):
        """
        깊이 범위로 필터링 (Z축 기준)
        
        Parameters:
        - min_depth: 최소 깊이
        - max_depth: 최대 깊이
        
        Returns:
        - filtered_pcd: 필터링된 포인트 클라우드
        """
        
        points = np.asarray(pcd.points)
        colors = np.asarray(pcd.colors) if pcd.has_colors() else None
        
        # Z 범위로 필터링
        mask = (points[:, 2] >= min_depth) & (points[:, 2] <= max_depth)
        
        filtered_pcd = o3d.geometry.PointCloud()
        filtered_pcd.points = o3d.utility.Vector3dVector(points[mask])
        
        if colors is not None:
            filtered_pcd.colors = o3d.utility.Vector3dVector(colors[mask])
        
        print(f"깊이 필터링 ({min_depth}-{max_depth}): {len(points):,} → {np.sum(mask):,}")
        
        return filtered_pcd


def demo_filtering():
    """필터링 데모"""
    
    # 샘플 데이터 생성 (노이즈 포함)
    np.random.seed(42)
    
    # 메인 데이터 (평면)
    n_main = 5000
    x = np.random.uniform(-50, 50, n_main)
    y = np.random.uniform(-50, 50, n_main)
    z = 500 + np.random.normal(0, 5, n_main)  # 약간의 노이즈
    
    # 이상치 추가
    n_outliers = 200
    x_out = np.random.uniform(-100, 100, n_outliers)
    y_out = np.random.uniform(-100, 100, n_outliers)
    z_out = np.random.uniform(0, 1000, n_outliers)
    
    points = np.vstack([
        np.column_stack([x, y, z]),
        np.column_stack([x_out, y_out, z_out])
    ])
    
    # 포인트 클라우드 생성
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    
    # 색상 (원본: 파랑, 이상치: 빨강)
    colors = np.zeros((len(points), 3))
    colors[:n_main] = [0, 0, 1]  # 파랑
    colors[n_main:] = [1, 0, 0]  # 빨강
    pcd.colors = o3d.utility.Vector3dVector(colors)
    
    print(f"원본 포인트 수: {len(points):,}")
    
    # 필터링
    pcf = PointCloudFilter()
    
    # 1. 통계적 이상치 제거
    filtered_pcd, _ = pcf.statistical_outlier_removal(pcd, nb_neighbors=20, std_ratio=2.0)
    
    # 2. 복셀 다운샘플링
    downsampled_pcd = pcf.voxel_downsample(filtered_pcd, voxel_size=5.0)
    
    # 시각화
    print("\n원본 (왼쪽) vs 필터링됨 (오른쪽)")
    
    # 원본 이동
    pcd_copy = o3d.geometry.PointCloud(pcd)
    pcd_copy.translate([-150, 0, 0])
    
    o3d.visualization.draw_geometries([pcd_copy, downsampled_pcd])


if __name__ == "__main__":
    demo_filtering()
```

### 4.2 필터 비교

```
┌─────────────────────────────────────────────────────────────┐
│                    필터 비교                                 │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  원본              통계적 이상치 제거    복셀 다운샘플링      │
│  ┌─────────┐       ┌─────────┐         ┌─────────┐         │
│  │ · ·· ·  │       │  ···    │         │  · ·    │         │
│  │ ·····   │  →    │  ·····  │    →    │   · ·   │         │
│  │  ····   │       │   ····  │         │    ·    │         │
│  │ ·  · ·  │       │         │         │         │         │
│  └─────────┘       └─────────┘         └─────────┘         │
│  (이상치 포함)      (이상치 제거)        (포인트 수 감소)    │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

---

## 5. 법선 벡터 추정

### 5.1 법선 벡터란?

```
법선 벡터: 표면에 수직인 단위 벡터

        ↑ 법선 (N)
        │
        │
    ────●────── 표면
       /│\
      / │ \
     (점과 이웃들)

용도:
- 조명 계산
- 메쉬 생성
- 표면 특성 분석
```

### 5.2 법선 추정 코드

```python
"""
normal_estimation.py
법선 벡터 추정
"""

import open3d as o3d
import numpy as np


def estimate_normals(pcd, radius=10.0, max_nn=30):
    """
    법선 벡터 추정
    
    Parameters:
    - pcd: 포인트 클라우드
    - radius: 검색 반경
    - max_nn: 최대 이웃 수
    
    Returns:
    - pcd: 법선이 추가된 포인트 클라우드
    """
    
    # 법선 추정
    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=radius,
            max_nn=max_nn
        )
    )
    
    # 법선 방향 일관성 보정 (카메라 방향 기준)
    pcd.orient_normals_towards_camera_location(camera_location=[0, 0, 0])
    
    print(f"✅ 법선 추정 완료: {len(pcd.normals)} normals")
    
    return pcd


def estimate_normals_consistent(pcd, radius=10.0, max_nn=30):
    """
    일관된 법선 방향으로 추정 (전파 기반)
    
    Parameters:
    - pcd: 포인트 클라우드
    - radius: 검색 반경
    - max_nn: 최대 이웃 수
    
    Returns:
    - pcd: 법선이 추가된 포인트 클라우드
    """
    
    # 법선 추정
    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=radius,
            max_nn=max_nn
        )
    )
    
    # 법선 방향 일관성 보정 (그래프 전파 기반)
    pcd.orient_normals_consistent_tangent_plane(k=30)
    
    print(f"✅ 일관된 법선 추정 완료")
    
    return pcd


def visualize_normals(pcd, normal_length=10.0):
    """법선 시각화"""
    
    if not pcd.has_normals():
        print("❌ 법선이 없습니다. 먼저 estimate_normals()를 실행하세요.")
        return
    
    # 법선 표시와 함께 시각화
    o3d.visualization.draw_geometries(
        [pcd],
        window_name="Point Cloud with Normals",
        point_show_normal=True
    )


def demo_normal_estimation():
    """법선 추정 데모"""
    
    # 반구 형태의 샘플 데이터 생성
    n_points = 5000
    
    # 구면 좌표
    theta = np.random.uniform(0, np.pi, n_points)
    phi = np.random.uniform(0, 2 * np.pi, n_points)
    r = 100  # 반지름
    
    x = r * np.sin(theta) * np.cos(phi)
    y = r * np.sin(theta) * np.sin(phi)
    z = r * np.cos(theta) + 200  # Z 오프셋
    
    points = np.column_stack([x, y, z])
    
    # 포인트 클라우드 생성
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    
    # 깊이 기반 색상
    z_norm = (z - z.min()) / (z.max() - z.min())
    colors = np.column_stack([z_norm, 1 - z_norm, np.zeros_like(z_norm)])
    pcd.colors = o3d.utility.Vector3dVector(colors)
    
    print(f"포인트 수: {len(points):,}")
    
    # 법선 추정
    pcd = estimate_normals(pcd, radius=15, max_nn=30)
    
    # 시각화
    print("\n'N' 키를 눌러 법선 표시를 토글하세요.")
    visualize_normals(pcd)


if __name__ == "__main__":
    demo_normal_estimation()
```

---

## 6. 포인트 클라우드 정합

### 6.1 ICP (Iterative Closest Point)

```python
"""
pointcloud_registration.py
포인트 클라우드 정합 (Registration)
"""

import open3d as o3d
import numpy as np
import copy


class PointCloudRegistration:
    """포인트 클라우드 정합"""
    
    @staticmethod
    def icp_point_to_point(source, target, threshold=5.0, 
                           init_transform=np.eye(4), max_iteration=100):
        """
        Point-to-Point ICP
        
        Parameters:
        - source: 소스 포인트 클라우드 (이동할 것)
        - target: 타겟 포인트 클라우드 (고정)
        - threshold: 대응점 최대 거리
        - init_transform: 초기 변환 행렬
        - max_iteration: 최대 반복 수
        
        Returns:
        - result: 정합 결과
        """
        
        result = o3d.pipelines.registration.registration_icp(
            source, target,
            threshold,
            init_transform,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(
                max_iteration=max_iteration
            )
        )
        
        print(f"\nICP Point-to-Point 결과:")
        print(f"  적합도 (fitness): {result.fitness:.4f}")
        print(f"  RMSE: {result.inlier_rmse:.4f}")
        
        return result
    
    @staticmethod
    def icp_point_to_plane(source, target, threshold=5.0,
                           init_transform=np.eye(4), max_iteration=100):
        """
        Point-to-Plane ICP (법선 필요)
        
        Parameters:
        - source: 소스 포인트 클라우드
        - target: 타겟 포인트 클라우드 (법선 필요)
        - threshold: 대응점 최대 거리
        
        Returns:
        - result: 정합 결과
        """
        
        # 타겟에 법선이 없으면 추정
        if not target.has_normals():
            target.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(
                    radius=10, max_nn=30
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
        
        print(f"\nICP Point-to-Plane 결과:")
        print(f"  적합도 (fitness): {result.fitness:.4f}")
        print(f"  RMSE: {result.inlier_rmse:.4f}")
        
        return result
    
    @staticmethod
    def colored_icp(source, target, voxel_size=5.0, max_iteration=50):
        """
        Colored ICP (색상 정보 활용)
        
        Parameters:
        - source: 소스 포인트 클라우드 (색상 필요)
        - target: 타겟 포인트 클라우드 (색상 필요)
        - voxel_size: 복셀 크기
        
        Returns:
        - result: 정합 결과
        """
        
        # 법선 추정
        for pcd in [source, target]:
            if not pcd.has_normals():
                pcd.estimate_normals(
                    search_param=o3d.geometry.KDTreeSearchParamHybrid(
                        radius=voxel_size * 2, max_nn=30
                    )
                )
        
        result = o3d.pipelines.registration.registration_colored_icp(
            source, target,
            voxel_size,
            np.eye(4),
            o3d.pipelines.registration.TransformationEstimationForColoredICP(),
            o3d.pipelines.registration.ICPConvergenceCriteria(
                max_iteration=max_iteration
            )
        )
        
        print(f"\nColored ICP 결과:")
        print(f"  적합도 (fitness): {result.fitness:.4f}")
        print(f"  RMSE: {result.inlier_rmse:.4f}")
        
        return result
    
    @staticmethod
    def apply_transformation(pcd, transformation):
        """변환 적용"""
        pcd_transformed = copy.deepcopy(pcd)
        pcd_transformed.transform(transformation)
        return pcd_transformed


def demo_registration():
    """정합 데모"""
    
    # 샘플 포인트 클라우드 생성
    n_points = 3000
    
    # 타겟: 평면
    x = np.random.uniform(-50, 50, n_points)
    y = np.random.uniform(-50, 50, n_points)
    z = 500 + 0.1 * x + 0.05 * y + np.random.normal(0, 2, n_points)
    
    target_points = np.column_stack([x, y, z])
    target = o3d.geometry.PointCloud()
    target.points = o3d.utility.Vector3dVector(target_points)
    target.paint_uniform_color([0, 0, 1])  # 파랑
    
    # 소스: 타겟을 회전/이동
    source = copy.deepcopy(target)
    
    # 변환 행렬 (회전 + 이동)
    angle = np.radians(15)
    R = np.array([
        [np.cos(angle), -np.sin(angle), 0],
        [np.sin(angle), np.cos(angle), 0],
        [0, 0, 1]
    ])
    t = np.array([20, 10, 5])
    
    true_transform = np.eye(4)
    true_transform[:3, :3] = R
    true_transform[:3, 3] = t
    
    source.transform(true_transform)
    source.paint_uniform_color([1, 0, 0])  # 빨강
    
    print("Ground truth 변환:")
    print(f"  회전: {np.degrees(angle):.1f}°")
    print(f"  이동: {t}")
    
    # 정합 전 시각화
    print("\n정합 전 (빨강: 소스, 파랑: 타겟)")
    o3d.visualization.draw_geometries([source, target])
    
    # ICP 정합
    reg = PointCloudRegistration()
    result = reg.icp_point_to_point(source, target, threshold=10.0)
    
    # 변환 적용
    source_aligned = reg.apply_transformation(source, result.transformation)
    source_aligned.paint_uniform_color([0, 1, 0])  # 초록
    
    # 정합 후 시각화
    print("\n정합 후 (초록: 정합됨, 파랑: 타겟)")
    o3d.visualization.draw_geometries([source_aligned, target])


if __name__ == "__main__":
    demo_registration()
```

---

## 7. 메쉬 생성

### 7.1 포인트 클라우드에서 메쉬로

```python
"""
mesh_generation.py
포인트 클라우드에서 메쉬 생성
"""

import open3d as o3d
import numpy as np


class MeshGenerator:
    """포인트 클라우드에서 메쉬 생성"""
    
    @staticmethod
    def poisson_reconstruction(pcd, depth=9, width=0, scale=1.1):
        """
        Poisson Surface Reconstruction
        
        법선이 있는 포인트 클라우드에서 매끄러운 메쉬 생성
        
        Parameters:
        - pcd: 포인트 클라우드 (법선 필요)
        - depth: 옥트리 깊이 (높을수록 세밀)
        - width: 타겟 너비 (0이면 자동)
        - scale: 바운딩 박스 스케일
        
        Returns:
        - mesh: 생성된 메쉬
        - densities: 밀도 정보
        """
        
        if not pcd.has_normals():
            print("법선 추정 중...")
            pcd.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(
                    radius=10, max_nn=30
                )
            )
            pcd.orient_normals_consistent_tangent_plane(k=30)
        
        print("Poisson 재구성 중...")
        mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
            pcd,
            depth=depth,
            width=width,
            scale=scale
        )
        
        print(f"✅ 메쉬 생성 완료:")
        print(f"   정점 수: {len(mesh.vertices):,}")
        print(f"   삼각형 수: {len(mesh.triangles):,}")
        
        return mesh, np.asarray(densities)
    
    @staticmethod
    def ball_pivoting(pcd, radii=[5, 10, 20]):
        """
        Ball Pivoting Algorithm (BPA)
        
        구를 굴려서 표면 재구성
        
        Parameters:
        - pcd: 포인트 클라우드 (법선 필요)
        - radii: 구의 반지름 리스트
        
        Returns:
        - mesh: 생성된 메쉬
        """
        
        if not pcd.has_normals():
            print("법선 추정 중...")
            pcd.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(
                    radius=10, max_nn=30
                )
            )
        
        print("Ball Pivoting 재구성 중...")
        radii_o3d = o3d.utility.DoubleVector(radii)
        mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
            pcd, radii_o3d
        )
        
        print(f"✅ 메쉬 생성 완료:")
        print(f"   정점 수: {len(mesh.vertices):,}")
        print(f"   삼각형 수: {len(mesh.triangles):,}")
        
        return mesh
    
    @staticmethod
    def alpha_shape(pcd, alpha=30.0):
        """
        Alpha Shape (2D Delaunay의 3D 확장)
        
        Parameters:
        - pcd: 포인트 클라우드
        - alpha: 알파 값 (작을수록 세밀, 구멍 많음)
        
        Returns:
        - mesh: 생성된 메쉬
        """
        
        print("Alpha Shape 재구성 중...")
        mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(
            pcd, alpha
        )
        
        print(f"✅ 메쉬 생성 완료:")
        print(f"   정점 수: {len(mesh.vertices):,}")
        print(f"   삼각형 수: {len(mesh.triangles):,}")
        
        return mesh
    
    @staticmethod
    def clean_mesh(mesh, remove_density_threshold=None, densities=None):
        """
        메쉬 정리
        
        Parameters:
        - mesh: 입력 메쉬
        - remove_density_threshold: 밀도 임계값 (Poisson용)
        - densities: 밀도 배열
        
        Returns:
        - cleaned_mesh: 정리된 메쉬
        """
        
        mesh_cleaned = mesh
        
        # 저밀도 영역 제거 (Poisson 결과용)
        if densities is not None and remove_density_threshold is not None:
            vertices_to_remove = densities < np.quantile(densities, remove_density_threshold)
            mesh_cleaned.remove_vertices_by_mask(vertices_to_remove)
            print(f"저밀도 영역 제거: {np.sum(vertices_to_remove):,} vertices")
        
        # 중복 정점 제거
        mesh_cleaned.remove_duplicated_vertices()
        mesh_cleaned.remove_duplicated_triangles()
        
        # 퇴화 삼각형 제거
        mesh_cleaned.remove_degenerate_triangles()
        
        # 비다양체 엣지 제거
        mesh_cleaned.remove_non_manifold_edges()
        
        # 정점 법선 재계산
        mesh_cleaned.compute_vertex_normals()
        
        print(f"✅ 메쉬 정리 완료")
        
        return mesh_cleaned


def demo_mesh_generation():
    """메쉬 생성 데모"""
    
    # 반구 형태의 샘플 데이터
    n_points = 10000
    
    # 반구
    theta = np.random.uniform(0, np.pi/2, n_points)
    phi = np.random.uniform(0, 2 * np.pi, n_points)
    r = 100
    
    x = r * np.sin(theta) * np.cos(phi)
    y = r * np.sin(theta) * np.sin(phi)
    z = r * np.cos(theta)
    
    points = np.column_stack([x, y, z])
    
    # 포인트 클라우드 생성
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    
    # 색상 (높이 기반)
    colors = np.zeros((n_points, 3))
    colors[:, 0] = (z - z.min()) / (z.max() - z.min())
    colors[:, 2] = 1 - colors[:, 0]
    pcd.colors = o3d.utility.Vector3dVector(colors)
    
    print(f"포인트 수: {n_points:,}")
    
    # 메쉬 생성기
    mg = MeshGenerator()
    
    # Poisson 재구성
    mesh, densities = mg.poisson_reconstruction(pcd, depth=8)
    
    # 저밀도 영역 제거
    mesh_cleaned = mg.clean_mesh(mesh, remove_density_threshold=0.1, densities=densities)
    
    # 색상 적용
    mesh_cleaned.paint_uniform_color([0.7, 0.7, 0.9])
    
    # 시각화
    print("\n포인트 클라우드 (왼쪽) vs 메쉬 (오른쪽)")
    
    pcd_display = o3d.geometry.PointCloud(pcd)
    pcd_display.translate([-150, 0, 0])
    
    o3d.visualization.draw_geometries([pcd_display, mesh_cleaned])


if __name__ == "__main__":
    demo_mesh_generation()
```

---

## 8. 파일 입출력

### 8.1 지원 포맷

| 포맷 | 확장자 | 특징 |
|------|--------|------|
| **PLY** | .ply | 범용, ASCII/Binary |
| **PCD** | .pcd | PCL 표준, Binary |
| **XYZ** | .xyz | 단순 텍스트 |
| **OBJ** | .obj | 메쉬용, 텍스처 지원 |
| **STL** | .stl | 3D 프린팅 |

### 8.2 파일 입출력 코드

```python
"""
pointcloud_io.py
포인트 클라우드 파일 입출력
"""

import open3d as o3d
import numpy as np
import os


class PointCloudIO:
    """포인트 클라우드 파일 입출력"""
    
    @staticmethod
    def save_ply(pcd, filename, write_ascii=False):
        """
        PLY 파일 저장
        
        Parameters:
        - pcd: 포인트 클라우드
        - filename: 파일명
        - write_ascii: ASCII 형식 여부 (False면 Binary)
        """
        
        success = o3d.io.write_point_cloud(
            filename, pcd,
            write_ascii=write_ascii,
            compressed=True
        )
        
        if success:
            size = os.path.getsize(filename) / (1024 * 1024)
            print(f"✅ PLY 저장: {filename} ({size:.2f} MB)")
        else:
            print(f"❌ PLY 저장 실패: {filename}")
        
        return success
    
    @staticmethod
    def save_pcd(pcd, filename, write_ascii=False):
        """
        PCD 파일 저장
        
        Parameters:
        - pcd: 포인트 클라우드
        - filename: 파일명
        - write_ascii: ASCII 형식 여부
        """
        
        success = o3d.io.write_point_cloud(
            filename, pcd,
            write_ascii=write_ascii,
            compressed=True
        )
        
        if success:
            size = os.path.getsize(filename) / (1024 * 1024)
            print(f"✅ PCD 저장: {filename} ({size:.2f} MB)")
        else:
            print(f"❌ PCD 저장 실패: {filename}")
        
        return success
    
    @staticmethod
    def save_xyz(pcd, filename):
        """
        XYZ 텍스트 파일 저장
        
        Parameters:
        - pcd: 포인트 클라우드
        - filename: 파일명
        """
        
        points = np.asarray(pcd.points)
        
        if pcd.has_colors():
            colors = (np.asarray(pcd.colors) * 255).astype(np.uint8)
            data = np.hstack([points, colors])
            header = "# X Y Z R G B"
            fmt = "%.6f %.6f %.6f %d %d %d"
        else:
            data = points
            header = "# X Y Z"
            fmt = "%.6f %.6f %.6f"
        
        np.savetxt(filename, data, fmt=fmt, header=header)
        
        size = os.path.getsize(filename) / (1024 * 1024)
        print(f"✅ XYZ 저장: {filename} ({size:.2f} MB)")
    
    @staticmethod
    def load(filename):
        """
        포인트 클라우드 파일 로드
        
        Parameters:
        - filename: 파일명 (확장자로 포맷 자동 감지)
        
        Returns:
        - pcd: 포인트 클라우드
        """
        
        if not os.path.exists(filename):
            print(f"❌ 파일 없음: {filename}")
            return None
        
        pcd = o3d.io.read_point_cloud(filename)
        
        if pcd.is_empty():
            print(f"❌ 빈 포인트 클라우드: {filename}")
            return None
        
        print(f"✅ 로드 완료: {filename}")
        print(f"   포인트 수: {len(pcd.points):,}")
        print(f"   색상: {pcd.has_colors()}")
        print(f"   법선: {pcd.has_normals()}")
        
        return pcd
    
    @staticmethod
    def save_mesh(mesh, filename):
        """
        메쉬 파일 저장
        
        Parameters:
        - mesh: 메쉬
        - filename: 파일명 (.obj, .ply, .stl)
        """
        
        success = o3d.io.write_triangle_mesh(filename, mesh)
        
        if success:
            size = os.path.getsize(filename) / (1024 * 1024)
            print(f"✅ 메쉬 저장: {filename} ({size:.2f} MB)")
        else:
            print(f"❌ 메쉬 저장 실패: {filename}")
        
        return success
    
    @staticmethod
    def load_mesh(filename):
        """
        메쉬 파일 로드
        
        Parameters:
        - filename: 파일명
        
        Returns:
        - mesh: 메쉬
        """
        
        if not os.path.exists(filename):
            print(f"❌ 파일 없음: {filename}")
            return None
        
        mesh = o3d.io.read_triangle_mesh(filename)
        
        print(f"✅ 메쉬 로드: {filename}")
        print(f"   정점 수: {len(mesh.vertices):,}")
        print(f"   삼각형 수: {len(mesh.triangles):,}")
        
        return mesh


def demo_io():
    """파일 입출력 데모"""
    
    # 샘플 포인트 클라우드 생성
    n_points = 10000
    points = np.random.rand(n_points, 3) * 100
    colors = np.random.rand(n_points, 3)
    
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    
    io = PointCloudIO()
    
    # 다양한 포맷으로 저장
    print("\n" + "="*50)
    print("파일 저장 테스트")
    print("="*50)
    
    io.save_ply(pcd, "test_cloud.ply")
    io.save_ply(pcd, "test_cloud_ascii.ply", write_ascii=True)
    io.save_pcd(pcd, "test_cloud.pcd")
    io.save_xyz(pcd, "test_cloud.xyz")
    
    # 로드 테스트
    print("\n" + "="*50)
    print("파일 로드 테스트")
    print("="*50)
    
    pcd_loaded = io.load("test_cloud.ply")
    
    # 정리
    import os
    for f in ["test_cloud.ply", "test_cloud_ascii.ply", 
              "test_cloud.pcd", "test_cloud.xyz"]:
        if os.path.exists(f):
            os.remove(f)
    
    print("\n✅ 테스트 완료 (임시 파일 삭제됨)")


if __name__ == "__main__":
    demo_io()
```

---

## 9. 실시간 3D 시각화

### 9.1 실시간 뷰어

```python
"""
realtime_pointcloud_viewer.py
실시간 포인트 클라우드 뷰어
"""

import cv2
import numpy as np
import open3d as o3d
import yaml
import threading
import time


class RealtimePointCloudViewer:
    def __init__(self, calibration_file, left_idx=0, right_idx=2):
        """
        실시간 포인트 클라우드 뷰어
        
        Parameters:
        - calibration_file: 캘리브레이션 파일
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
        
        # SGBM 매처
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
        
        # 포인트 클라우드
        self.pcd = o3d.geometry.PointCloud()
        self.pcd_lock = threading.Lock()
        
        # 실행 플래그
        self.running = False
        
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
        
        # 정류 맵
        self.map1_left, self.map2_left = cv2.initUndistortRectifyMap(
            self.K1, self.D1, self.R1, self.P1, self.img_size, cv2.CV_32FC1
        )
        self.map1_right, self.map2_right = cv2.initUndistortRectifyMap(
            self.K2, self.D2, self.R2, self.P2, self.img_size, cv2.CV_32FC1
        )
        
    def process_stereo(self, frame_left, frame_right):
        """스테레오 처리 및 포인트 클라우드 생성"""
        
        # 정류
        rect_left = cv2.remap(frame_left, self.map1_left, self.map2_left, cv2.INTER_LINEAR)
        rect_right = cv2.remap(frame_right, self.map1_right, self.map2_right, cv2.INTER_LINEAR)
        
        # 시차 계산
        disparity = self.stereo.compute(rect_left, rect_right).astype(np.float32) / 16.0
        
        # 3D 재투영
        points_3d = cv2.reprojectImageTo3D(disparity, self.Q)
        
        # 유효한 점만 추출
        mask = (disparity > 0) & (points_3d[:, :, 2] > 100) & (points_3d[:, :, 2] < 5000)
        
        # 다운샘플링 (성능을 위해)
        mask_downsampled = np.zeros_like(mask)
        mask_downsampled[::4, ::4] = mask[::4, ::4]
        
        points = points_3d[mask_downsampled]
        colors = rect_left[mask_downsampled][:, ::-1] / 255.0  # BGR → RGB
        
        return points, colors
    
    def capture_thread(self):
        """캡처 스레드"""
        
        while self.running:
            ret1, frame_left = self.cap_left.read()
            ret2, frame_right = self.cap_right.read()
            
            if not ret1 or not ret2:
                continue
            
            # 포인트 클라우드 생성
            points, colors = self.process_stereo(frame_left, frame_right)
            
            if len(points) > 0:
                with self.pcd_lock:
                    self.pcd.points = o3d.utility.Vector3dVector(points)
                    self.pcd.colors = o3d.utility.Vector3dVector(colors)
            
            time.sleep(0.03)  # ~30fps
    
    def run(self):
        """메인 실행"""
        
        print("="*60)
        print("실시간 포인트 클라우드 뷰어")
        print("="*60)
        print("조작:")
        print("  마우스 드래그: 회전")
        print("  휠: 확대/축소")
        print("  Q: 종료")
        print("="*60)
        
        self.running = True
        
        # 캡처 스레드 시작
        capture_thread = threading.Thread(target=self.capture_thread)
        capture_thread.start()
        
        # Open3D 시각화
        vis = o3d.visualization.Visualizer()
        vis.create_window("Realtime Point Cloud", width=1280, height=720)
        
        # 좌표축 추가
        coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=100)
        vis.add_geometry(coord_frame)
        vis.add_geometry(self.pcd)
        
        # 렌더링 옵션
        opt = vis.get_render_option()
        opt.point_size = 2.0
        opt.background_color = np.array([0.1, 0.1, 0.1])
        
        # 초기 뷰
        ctr = vis.get_view_control()
        ctr.set_zoom(0.5)
        ctr.set_front([0, 0, -1])
        ctr.set_up([0, -1, 0])
        
        # 메인 루프
        while self.running:
            with self.pcd_lock:
                vis.update_geometry(self.pcd)
            
            if not vis.poll_events():
                self.running = False
                break
            
            vis.update_renderer()
        
        # 정리
        self.running = False
        capture_thread.join()
        vis.destroy_window()
        self.cap_left.release()
        self.cap_right.release()
        
        print("\n✅ 종료됨")


if __name__ == "__main__":
    viewer = RealtimePointCloudViewer("stereo_params.yaml", left_idx=0, right_idx=2)
    viewer.run()
```

---

## 10. 실습 프로젝트

### 10.1 전체 파이프라인 예제

```python
"""
pointcloud_pipeline.py
포인트 클라우드 생성 전체 파이프라인
"""

import cv2
import numpy as np
import open3d as o3d
import yaml
import os


def stereo_to_pointcloud_pipeline(left_image, right_image, calibration_file, 
                                   output_prefix="output"):
    """
    스테레오 이미지에서 포인트 클라우드 생성 전체 파이프라인
    
    Parameters:
    - left_image: 왼쪽 이미지 경로
    - right_image: 오른쪽 이미지 경로
    - calibration_file: 캘리브레이션 파일 경로
    - output_prefix: 출력 파일 접두사
    """
    
    print("="*60)
    print("포인트 클라우드 생성 파이프라인")
    print("="*60)
    
    # 1. 이미지 로드
    print("\n[1/7] 이미지 로드...")
    img_left = cv2.imread(left_image)
    img_right = cv2.imread(right_image)
    
    if img_left is None or img_right is None:
        print("❌ 이미지 로드 실패")
        return None
    
    print(f"  이미지 크기: {img_left.shape}")
    
    # 2. 캘리브레이션 로드
    print("\n[2/7] 캘리브레이션 로드...")
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
    Q = np.array(params['Q'])
    
    print(f"  베이스라인: {params['baseline_mm']:.1f} mm")
    
    # 3. 정류
    print("\n[3/7] 스테레오 정류...")
    map1_left, map2_left = cv2.initUndistortRectifyMap(
        K1, D1, R1, P1, img_size, cv2.CV_32FC1
    )
    map1_right, map2_right = cv2.initUndistortRectifyMap(
        K2, D2, R2, P2, img_size, cv2.CV_32FC1
    )
    
    rect_left = cv2.remap(img_left, map1_left, map2_left, cv2.INTER_LINEAR)
    rect_right = cv2.remap(img_right, map1_right, map2_right, cv2.INTER_LINEAR)
    
    print("  ✅ 정류 완료")
    
    # 4. 스테레오 매칭
    print("\n[4/7] 스테레오 매칭 (SGBM)...")
    stereo = cv2.StereoSGBM_create(
        minDisparity=0,
        numDisparities=128,
        blockSize=5,
        P1=8 * 3 * 5 ** 2,
        P2=32 * 3 * 5 ** 2,
        disp12MaxDiff=1,
        uniquenessRatio=10,
        speckleWindowSize=100,
        speckleRange=2,
        mode=cv2.STEREO_SGBM_MODE_SGBM
    )
    
    disparity = stereo.compute(rect_left, rect_right).astype(np.float32) / 16.0
    
    valid_count = np.sum(disparity > 0)
    print(f"  유효 시차: {valid_count:,} / {disparity.size:,}")
    
    # 5. 3D 재투영
    print("\n[5/7] 3D 포인트 클라우드 생성...")
    points_3d = cv2.reprojectImageTo3D(disparity, Q)
    
    # 유효한 점 필터링
    mask = (disparity > 0) & (points_3d[:, :, 2] > 100) & (points_3d[:, :, 2] < 10000)
    
    points = points_3d[mask]
    colors = rect_left[mask][:, ::-1] / 255.0  # BGR → RGB
    
    print(f"  포인트 수: {len(points):,}")
    
    # Open3D 포인트 클라우드 생성
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    
    # 6. 필터링
    print("\n[6/7] 포인트 클라우드 필터링...")
    
    # 통계적 이상치 제거
    pcd_filtered, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    print(f"  이상치 제거 후: {len(pcd_filtered.points):,}")
    
    # 복셀 다운샘플링
    pcd_downsampled = pcd_filtered.voxel_down_sample(voxel_size=5.0)
    print(f"  다운샘플링 후: {len(pcd_downsampled.points):,}")
    
    # 법선 추정
    pcd_downsampled.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=15, max_nn=30)
    )
    pcd_downsampled.orient_normals_towards_camera_location([0, 0, 0])
    print("  ✅ 법선 추정 완료")
    
    # 7. 파일 저장
    print("\n[7/7] 결과 저장...")
    
    # 포인트 클라우드 저장
    ply_file = f"{output_prefix}_pointcloud.ply"
    o3d.io.write_point_cloud(ply_file, pcd_downsampled)
    print(f"  ✅ {ply_file}")
    
    # 시차 맵 저장
    disp_vis = np.zeros_like(disparity)
    disp_vis[mask[:, :, 0] if len(mask.shape) == 3 else mask] = disparity[mask[:, :, 0] if len(mask.shape) == 3 else mask]
    disp_norm = cv2.normalize(disp_vis, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    disp_color = cv2.applyColorMap(disp_norm, cv2.COLORMAP_JET)
    cv2.imwrite(f"{output_prefix}_disparity.png", disp_color)
    print(f"  ✅ {output_prefix}_disparity.png")
    
    # 정류 이미지 저장
    cv2.imwrite(f"{output_prefix}_rectified.png", rect_left)
    print(f"  ✅ {output_prefix}_rectified.png")
    
    print("\n" + "="*60)
    print("✅ 파이프라인 완료!")
    print("="*60)
    
    # 시각화
    print("\n포인트 클라우드를 시각화합니다...")
    
    # 좌표축 추가
    coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=100)
    
    o3d.visualization.draw_geometries(
        [pcd_downsampled, coord_frame],
        window_name="Generated Point Cloud",
        width=1280,
        height=720
    )
    
    return pcd_downsampled


if __name__ == "__main__":
    # 사용 예시
    pcd = stereo_to_pointcloud_pipeline(
        "test_left.png",
        "test_right.png",
        "stereo_params.yaml",
        output_prefix="result"
    )
```

---

## 📝 학습 체크리스트

### 이론 이해

- [ ] 깊이 맵에서 3D 좌표로 변환하는 공식을 알고 있다
- [ ] Q 행렬의 역할을 설명할 수 있다
- [ ] 복셀 다운샘플링의 원리를 이해했다
- [ ] Poisson 재구성과 Ball Pivoting의 차이를 안다

### 실습 완료

- [ ] 깊이 맵에서 포인트 클라우드 생성
- [ ] Open3D로 포인트 클라우드 시각화
- [ ] 필터링 (이상치 제거, 다운샘플링)
- [ ] 법선 벡터 추정
- [ ] PLY/PCD 파일 저장 및 로드
- [ ] 실시간 포인트 클라우드 뷰어 실행

---

## ➡️ 다음 모듈

**[Module 05: 딥러닝 기반 스테레오 (RAFT-Stereo, AANet)](../Module_05_DeepLearning/README.md)**

다음 모듈에서는:
- 딥러닝 기반 스테레오 매칭 원리
- RAFT-Stereo 모델 사용법
- AANet 모델 사용법
- 전통적 방법과 성능 비교

를 학습합니다.

---

## 📄 라이선스

MIT License - 자유롭게 사용, 수정, 배포 가능
