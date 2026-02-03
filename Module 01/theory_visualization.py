"""
theory_visualization.py
Module 01: 스테레오 비전 이론 기초 - 시각화 코드

핀홀 카메라 모델, 시차-깊이 관계, 에피폴라 기하학을 시각화합니다.
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def visualize_pinhole_projection():
    """핀홀 카메라 투영 원리 시각화"""
    
    # 카메라 파라미터
    f = 500  # 초점거리 (픽셀)
    cx, cy = 320, 240  # 주점
    
    # 3D 점들 (정육면체)
    cube_size = 100
    cube_center = np.array([0, 0, 500])  # 카메라 앞 500mm
    
    # 정육면체 꼭짓점
    vertices = np.array([
        [-1, -1, -1], [1, -1, -1], [1, 1, -1], [-1, 1, -1],
        [-1, -1, 1], [1, -1, 1], [1, 1, 1], [-1, 1, 1]
    ]) * cube_size / 2 + cube_center
    
    # 투영 함수
    def project(points_3d, f, cx, cy):
        """3D 점을 2D 이미지로 투영"""
        X, Y, Z = points_3d[:, 0], points_3d[:, 1], points_3d[:, 2]
        u = f * X / Z + cx
        v = f * Y / Z + cy
        return np.column_stack([u, v])
    
    projected = project(vertices, f, cx, cy)
    
    # 시각화
    fig = plt.figure(figsize=(14, 5))
    
    # 3D 뷰
    ax1 = fig.add_subplot(131, projection='3d')
    ax1.set_title('3D Scene', fontsize=12)
    
    # 정육면체 모서리
    edges = [
        [0,1], [1,2], [2,3], [3,0],  # 앞면
        [4,5], [5,6], [6,7], [7,4],  # 뒷면
        [0,4], [1,5], [2,6], [3,7]   # 연결
    ]
    
    for edge in edges:
        ax1.plot3D(*zip(vertices[edge[0]], vertices[edge[1]]), 'b-', linewidth=2)
    
    # 카메라 위치
    ax1.scatter([0], [0], [0], color='red', s=100, label='Camera')
    ax1.set_xlabel('X (mm)')
    ax1.set_ylabel('Y (mm)')
    ax1.set_zlabel('Z (mm)')
    ax1.legend()
    
    # 2D 투영 결과
    ax2 = fig.add_subplot(132)
    ax2.set_title('2D Projection', fontsize=12)
    
    for edge in edges:
        ax2.plot(*zip(projected[edge[0]], projected[edge[1]]), 'b-', linewidth=2)
    
    ax2.scatter(projected[:, 0], projected[:, 1], color='red', s=50)
    ax2.set_xlim(0, 640)
    ax2.set_ylim(480, 0)  # 이미지 좌표계 (y 반전)
    ax2.set_xlabel('u (pixels)')
    ax2.set_ylabel('v (pixels)')
    ax2.set_aspect('equal')
    ax2.grid(True, alpha=0.3)
    
    # 투영 수식
    ax3 = fig.add_subplot(133)
    ax3.axis('off')
    ax3.set_title('Projection Equations', fontsize=12)
    
    equations = [
        r'$u = f \cdot \frac{X}{Z} + c_x$',
        r'$v = f \cdot \frac{Y}{Z} + c_y$',
        '',
        f'Parameters:',
        f'  f = {f} pixels',
        f'  (cx, cy) = ({cx}, {cy})',
        f'  Cube center Z = {cube_center[2]}mm'
    ]
    
    for i, eq in enumerate(equations):
        ax3.text(0.1, 0.9 - i*0.12, eq, fontsize=14, 
                transform=ax3.transAxes, family='monospace')
    
    plt.tight_layout()
    plt.savefig('pinhole_projection.png', dpi=150, bbox_inches='tight')
    plt.show()
    print("✅ 저장됨: pinhole_projection.png")


def visualize_disparity_depth_relationship():
    """시차-깊이 관계 시각화"""
    
    # 파라미터 (USB 웹캠 기준)
    f = 1317  # 초점거리 (픽셀)
    B = 85    # 베이스라인 (mm)
    
    # 거리 범위
    Z = np.linspace(300, 5000, 100)  # 0.3m ~ 5m
    
    # 시차 계산
    d = f * B / Z
    
    # 깊이 분해능 (시차 1픽셀 변화 시)
    delta_Z = Z**2 / (f * B)
    
    fig, axes = plt.subplots(1, 3, figsize=(15, 4))
    
    # 시차 vs 깊이
    ax1 = axes[0]
    ax1.plot(Z/1000, d, 'b-', linewidth=2)
    ax1.set_xlabel('Depth Z (m)', fontsize=11)
    ax1.set_ylabel('Disparity d (pixels)', fontsize=11)
    ax1.set_title('Disparity vs Depth', fontsize=12)
    ax1.grid(True, alpha=0.3)
    ax1.axhline(y=1, color='r', linestyle='--', label='Min disparity (1 pixel)')
    ax1.legend()
    
    # 깊이 vs 시차 (역관계)
    ax2 = axes[1]
    d_range = np.linspace(10, 400, 100)
    Z_from_d = f * B / d_range
    ax2.plot(d_range, Z_from_d/1000, 'g-', linewidth=2)
    ax2.set_xlabel('Disparity d (pixels)', fontsize=11)
    ax2.set_ylabel('Depth Z (m)', fontsize=11)
    ax2.set_title('Depth vs Disparity', fontsize=12)
    ax2.grid(True, alpha=0.3)
    
    # 깊이 분해능
    ax3 = axes[2]
    ax3.semilogy(Z/1000, delta_Z/10, 'r-', linewidth=2)
    ax3.set_xlabel('Depth Z (m)', fontsize=11)
    ax3.set_ylabel('Depth Resolution ΔZ (cm)', fontsize=11)
    ax3.set_title('Depth Resolution (per 1px disparity change)', fontsize=12)
    ax3.grid(True, alpha=0.3)
    
    plt.suptitle(f'Stereo Parameters: f={f}px, B={B}mm', fontsize=13, y=1.02)
    plt.tight_layout()
    plt.savefig('disparity_depth_analysis.png', dpi=150, bbox_inches='tight')
    plt.show()
    
    # 수치 테이블 출력
    print("\n" + "="*60)
    print(f"거리별 시차 및 깊이 분해능 (f={f}px, B={B}mm)")
    print("="*60)
    print(f"{'거리(m)':<10} {'시차(px)':<12} {'분해능(cm)':<12}")
    print("-"*60)
    
    for z_m in [0.3, 0.5, 1.0, 2.0, 3.0, 5.0]:
        z_mm = z_m * 1000
        disp = f * B / z_mm
        resolution = (z_mm**2) / (f * B) / 10  # cm로 변환
        print(f"{z_m:<10.1f} {disp:<12.1f} {resolution:<12.2f}")
    
    print("\n✅ 저장됨: disparity_depth_analysis.png")


def visualize_epipolar_geometry():
    """에피폴라 기하학 시각화"""
    
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    
    # 왼쪽 이미지
    ax1 = axes[0]
    ax1.set_xlim(0, 640)
    ax1.set_ylim(480, 0)
    ax1.set_title('Left Image', fontsize=12)
    ax1.set_xlabel('u (pixels)')
    ax1.set_ylabel('v (pixels)')
    
    # 점 pL 표시
    pL = np.array([200, 200])
    ax1.scatter(*pL, color='red', s=100, zorder=5, label='Point pL')
    
    # 에피폴 (오른쪽 카메라 중심의 투영)
    eL = np.array([640, 240])
    ax1.scatter(*eL, color='blue', s=100, marker='x', zorder=5, label='Epipole eL')
    
    # 에피폴라 라인
    ax1.plot([0, 640], [200, 200], 'g--', linewidth=2, alpha=0.7, label='Epipolar line')
    ax1.legend(loc='lower right')
    ax1.grid(True, alpha=0.3)
    
    # 오른쪽 이미지
    ax2 = axes[1]
    ax2.set_xlim(0, 640)
    ax2.set_ylim(480, 0)
    ax2.set_title('Right Image', fontsize=12)
    ax2.set_xlabel('u (pixels)')
    ax2.set_ylabel('v (pixels)')
    
    # 에피폴라 라인 (수평) - 검색 영역 표시
    ax2.axhspan(195, 205, alpha=0.3, color='green', label='Search region')
    ax2.plot([0, 640], [200, 200], 'g-', linewidth=3, label='Epipolar line')
    
    # 가능한 대응점들
    possible_pR = [(150, 200), (250, 200), (350, 200)]
    for p in possible_pR:
        ax2.scatter(*p, color='orange', s=80, alpha=0.5)
    
    # 실제 대응점
    pR = np.array([150, 200])
    ax2.scatter(*pR, color='red', s=100, zorder=5, label='Matching point pR')
    
    # 에피폴
    eR = np.array([0, 240])
    ax2.scatter(*eR, color='blue', s=100, marker='x', zorder=5, label='Epipole eR')
    
    ax2.legend(loc='lower right')
    ax2.grid(True, alpha=0.3)
    
    # 시차 표시
    disparity = pL[0] - pR[0]
    ax2.annotate(f'Disparity = {disparity} pixels', 
                xy=(200, 170), fontsize=12, color='purple',
                bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.8))
    
    plt.suptitle('Epipolar Geometry: Search Constraint', fontsize=13, y=1.02)
    plt.tight_layout()
    plt.savefig('epipolar_geometry.png', dpi=150, bbox_inches='tight')
    plt.show()
    print("✅ 저장됨: epipolar_geometry.png")


def visualize_baseline_effect():
    """베이스라인이 측정 성능에 미치는 영향"""
    
    f = 1317  # 고정 초점거리
    baselines = [50, 85, 120, 200]  # mm
    Z = np.linspace(300, 5000, 100)
    
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    
    # 시차 비교
    ax1 = axes[0]
    colors = ['#e74c3c', '#3498db', '#27ae60', '#9b59b6']
    for B, color in zip(baselines, colors):
        d = f * B / Z
        ax1.plot(Z/1000, d, linewidth=2, color=color, label=f'B={B}mm')
    
    ax1.set_xlabel('Depth Z (m)', fontsize=11)
    ax1.set_ylabel('Disparity d (pixels)', fontsize=11)
    ax1.set_title('Disparity for Different Baselines', fontsize=12)
    ax1.axhline(y=5, color='gray', linestyle=':', label='Min reliable (5px)')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    ax1.set_ylim(0, 400)
    
    # 깊이 분해능 비교
    ax2 = axes[1]
    for B, color in zip(baselines, colors):
        delta_Z = Z**2 / (f * B) / 10  # cm
        ax2.semilogy(Z/1000, delta_Z, linewidth=2, color=color, label=f'B={B}mm')
    
    ax2.set_xlabel('Depth Z (m)', fontsize=11)
    ax2.set_ylabel('Depth Resolution ΔZ (cm)', fontsize=11)
    ax2.set_title('Depth Resolution for Different Baselines', fontsize=12)
    ax2.axhline(y=5, color='gray', linestyle=':', label='5cm threshold')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    plt.suptitle('Effect of Baseline on Stereo Performance', fontsize=13, y=1.02)
    plt.tight_layout()
    plt.savefig('baseline_effect.png', dpi=150, bbox_inches='tight')
    plt.show()
    print("✅ 저장됨: baseline_effect.png")


def visualize_stereo_rectification():
    """스테레오 정류 전후 비교"""
    
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    
    # 정류 전 - 왼쪽
    ax1 = axes[0, 0]
    ax1.set_xlim(0, 640)
    ax1.set_ylim(480, 0)
    ax1.set_title('Before Rectification - Left', fontsize=11)
    
    # 기울어진 에피폴라 라인들
    for y in range(50, 450, 50):
        x1, y1 = 0, y - 30
        x2, y2 = 640, y + 30
        ax1.plot([x1, x2], [y1, y2], 'g--', alpha=0.5, linewidth=1)
    
    ax1.scatter([200], [200], color='red', s=100, zorder=5)
    ax1.set_xlabel('u')
    ax1.set_ylabel('v')
    
    # 정류 전 - 오른쪽
    ax2 = axes[0, 1]
    ax2.set_xlim(0, 640)
    ax2.set_ylim(480, 0)
    ax2.set_title('Before Rectification - Right', fontsize=11)
    
    for y in range(50, 450, 50):
        x1, y1 = 0, y + 20
        x2, y2 = 640, y - 20
        ax2.plot([x1, x2], [y1, y2], 'g--', alpha=0.5, linewidth=1)
    
    ax2.scatter([150], [215], color='red', s=100, zorder=5)
    ax2.set_xlabel('u')
    ax2.set_ylabel('v')
    
    # 정류 후 - 왼쪽
    ax3 = axes[1, 0]
    ax3.set_xlim(0, 640)
    ax3.set_ylim(480, 0)
    ax3.set_title('After Rectification - Left', fontsize=11)
    
    for y in range(50, 450, 50):
        ax3.axhline(y=y, color='g', linestyle='-', alpha=0.5, linewidth=1)
    
    ax3.scatter([200], [200], color='red', s=100, zorder=5)
    ax3.set_xlabel('u')
    ax3.set_ylabel('v')
    
    # 정류 후 - 오른쪽
    ax4 = axes[1, 1]
    ax4.set_xlim(0, 640)
    ax4.set_ylim(480, 0)
    ax4.set_title('After Rectification - Right', fontsize=11)
    
    for y in range(50, 450, 50):
        ax4.axhline(y=y, color='g', linestyle='-', alpha=0.5, linewidth=1)
    
    # 같은 행에 대응점
    ax4.scatter([150], [200], color='red', s=100, zorder=5)
    ax4.axhline(y=200, color='orange', linewidth=3, alpha=0.7)
    ax4.set_xlabel('u')
    ax4.set_ylabel('v')
    
    # 연결선 (같은 행임을 강조)
    ax4.annotate('Same row!', xy=(300, 200), fontsize=12, color='orange',
                bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
    
    for ax in axes.flat:
        ax.grid(True, alpha=0.3)
    
    plt.suptitle('Stereo Rectification: Before vs After', fontsize=13)
    plt.tight_layout()
    plt.savefig('stereo_rectification.png', dpi=150, bbox_inches='tight')
    plt.show()
    print("✅ 저장됨: stereo_rectification.png")


def calculate_depth_from_disparity(d, f=1317, B=85):
    """
    시차로부터 깊이 계산
    
    Parameters:
    - d: 시차 (픽셀)
    - f: 초점거리 (픽셀, default: 1317)
    - B: 베이스라인 (mm, default: 85)
    
    Returns:
    - Z: 깊이 (mm)
    """
    if d <= 0:
        return float('inf')
    return f * B / d


def calculate_disparity_from_depth(Z, f=1317, B=85):
    """
    깊이로부터 시차 계산
    
    Parameters:
    - Z: 깊이 (mm)
    - f: 초점거리 (픽셀, default: 1317)
    - B: 베이스라인 (mm, default: 85)
    
    Returns:
    - d: 시차 (픽셀)
    """
    if Z <= 0:
        return float('inf')
    return f * B / Z


def print_stereo_parameters_analysis():
    """스테레오 파라미터 분석 결과 출력"""
    
    print("\n" + "="*70)
    print("USB 웹캠 스테레오 시스템 파라미터 분석")
    print("="*70)
    
    # 기본 파라미터
    resolution = (1920, 1080)
    sensor_size = (3.6, 2.7)  # 1/4" 센서 (mm)
    fov_h = 72  # 수평 FOV (도)
    baseline = 85  # mm
    
    # 초점거리 계산
    f_mm = (sensor_size[0] / 2) / np.tan(np.radians(fov_h / 2))
    f_pixel = f_mm * (resolution[0] / sensor_size[0])
    
    print(f"\n[카메라 스펙]")
    print(f"  해상도: {resolution[0]} x {resolution[1]}")
    print(f"  센서 크기: {sensor_size[0]} x {sensor_size[1]} mm")
    print(f"  수평 FOV: {fov_h}°")
    print(f"  베이스라인: {baseline} mm")
    
    print(f"\n[계산된 파라미터]")
    print(f"  초점거리: {f_mm:.2f} mm = {f_pixel:.0f} pixels")
    
    print(f"\n[거리별 성능]")
    print(f"{'거리(m)':<10} {'시차(px)':<12} {'분해능(cm)':<12} {'최대 오차(cm)':<15}")
    print("-"*50)
    
    for z_m in [0.3, 0.5, 1.0, 2.0, 3.0, 5.0]:
        z_mm = z_m * 1000
        disp = f_pixel * baseline / z_mm
        resolution_cm = (z_mm**2) / (f_pixel * baseline) / 10
        max_error = resolution_cm  # 1픽셀 오차 가정
        print(f"{z_m:<10.1f} {disp:<12.1f} {resolution_cm:<12.2f} ±{max_error:<14.2f}")
    
    print("\n[권장 사용 범위]")
    print(f"  최적 범위: 0.5m ~ 2.0m")
    print(f"  사용 가능: 0.3m ~ 3.0m")
    print(f"  참고용: 3.0m ~ 5.0m")


if __name__ == "__main__":
    print("=" * 60)
    print("Module 01: 스테레오 비전 이론 시각화")
    print("=" * 60)
    
    # 파라미터 분석
    print_stereo_parameters_analysis()
    
    print("\n" + "-"*60)
    print("시각화 생성 중...")
    print("-"*60)
    
    print("\n[1/5] 핀홀 카메라 투영 시각화...")
    visualize_pinhole_projection()
    
    print("\n[2/5] 시차-깊이 관계 분석...")
    visualize_disparity_depth_relationship()
    
    print("\n[3/5] 에피폴라 기하학 시각화...")
    visualize_epipolar_geometry()
    
    print("\n[4/5] 베이스라인 영향 분석...")
    visualize_baseline_effect()
    
    print("\n[5/5] 스테레오 정류 시각화...")
    visualize_stereo_rectification()
    
    print("\n" + "="*60)
    print("✅ 모든 시각화 완료!")
    print("="*60)
    print("\n생성된 파일:")
    print("  - pinhole_projection.png")
    print("  - disparity_depth_analysis.png")
    print("  - epipolar_geometry.png")
    print("  - baseline_effect.png")
    print("  - stereo_rectification.png")
