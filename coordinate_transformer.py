#!/usr/bin/env python3
"""
坐标转换工具类
============

提供相机坐标系和机械臂基座坐标系之间的转换功能
"""

import numpy as np
import json
import pyrealsense2 as rs
from pathlib import Path
from typing import Tuple


class CoordinateTransformer:
    """坐标转换器"""
    
    def __init__(
        self,
        calibration_file: str = "outputs/camera_to_base_calibration.json"
    ):
        """
        初始化转换器
        
        Args:
            calibration_file: 手眼标定结果文件
        """
        # 加载手眼标定结果
        with open(calibration_file) as f:
            calib_data = json.load(f)
            self.T_cam_to_base = np.array(calib_data["T_cam_to_base"])
            self.T_base_to_cam = np.linalg.inv(self.T_cam_to_base)
            
        print("✅ 坐标转换器已初始化")
        print(f"   相机位置 (相对于基座): {calib_data['translation_m']}")
        
    def pixel_to_camera_3d(
        self,
        pixel_x: int,
        pixel_y: int,
        depth: float,
        camera_intrinsics: rs.intrinsics
    ) -> np.ndarray:
        """
        将像素坐标+深度转换为相机坐标系的3D坐标
        
        Args:
            pixel_x, pixel_y: 像素坐标
            depth: 深度值（米）
            camera_intrinsics: 相机内参
            
        Returns:
            [x, y, z] 在相机坐标系中的坐标（米）
        """
        # 使用RealSense SDK的反投影函数
        point_3d_camera = rs.rs2_deproject_pixel_to_point(
            camera_intrinsics,
            [pixel_x, pixel_y],
            depth
        )
        
        return np.array(point_3d_camera)
        
    def camera_to_base(self, point_camera: np.ndarray) -> np.ndarray:
        """
        将相机坐标系的点转换到机械臂基座坐标系
        
        Args:
            point_camera: [x, y, z] 在相机坐标系中的坐标（米）
            
        Returns:
            [x, y, z] 在机械臂基座坐标系中的坐标（米）
        """
        # 转换为齐次坐标
        point_camera_homo = np.array([*point_camera, 1.0])
        
        # 应用变换矩阵
        point_base_homo = self.T_cam_to_base @ point_camera_homo
        
        return point_base_homo[:3]
        
    def base_to_camera(self, point_base: np.ndarray) -> np.ndarray:
        """
        将机械臂基座坐标系的点转换到相机坐标系
        
        Args:
            point_base: [x, y, z] 在机械臂基座坐标系中的坐标（米）
            
        Returns:
            [x, y, z] 在相机坐标系中的坐标（米）
        """
        # 转换为齐次坐标
        point_base_homo = np.array([*point_base, 1.0])
        
        # 应用逆变换矩阵
        point_camera_homo = self.T_base_to_cam @ point_base_homo
        
        return point_camera_homo[:3]
        
    def pixel_to_base(
        self,
        pixel_x: int,
        pixel_y: int,
        depth: float,
        camera_intrinsics: rs.intrinsics
    ) -> np.ndarray:
        """
        一步到位：像素坐标+深度 → 机械臂基座坐标系
        
        Args:
            pixel_x, pixel_y: 像素坐标
            depth: 深度值（米）
            camera_intrinsics: 相机内参
            
        Returns:
            [x, y, z] 在机械臂基座坐标系中的坐标（米）
        """
        # 第一步：像素→相机3D
        point_camera = self.pixel_to_camera_3d(
            pixel_x, pixel_y, depth, camera_intrinsics
        )
        
        # 第二步：相机3D→基座3D
        point_base = self.camera_to_base(point_camera)
        
        return point_base
        
    def visualize_transform(
        self,
        point_camera: np.ndarray,
        point_base: np.ndarray
    ):
        """可视化坐标转换"""
        print("\n坐标转换:")
        print(f"  相机坐标系: X={point_camera[0]:.3f}m, Y={point_camera[1]:.3f}m, Z={point_camera[2]:.3f}m")
        print(f"  基座坐标系: X={point_base[0]:.3f}m, Y={point_base[1]:.3f}m, Z={point_base[2]:.3f}m")


# 使用示例
def example_usage():
    """使用示例"""
    
    # 1. 初始化转换器
    transformer = CoordinateTransformer("outputs/camera_to_base_calibration.json")
    
    # 2. 模拟相机内参（实际使用时从RealSense获取）
    class MockIntrinsics:
        def __init__(self):
            self.fx = 900.0
            self.fy = 900.0
            self.ppx = 640.0
            self.ppy = 360.0
    
    intrinsics = MockIntrinsics()
    
    # 3. 示例：检测到地鼠在像素 (320, 240)，深度 0.5m
    pixel_x, pixel_y = 320, 240
    depth = 0.5  # 米
    
    # 4. 转换到基座坐标系
    point_base = transformer.pixel_to_base(pixel_x, pixel_y, depth, intrinsics)
    
    print(f"\n地鼠位置（基座坐标系）: {point_base}")
    print(f"  X: {point_base[0]:.3f} m")
    print(f"  Y: {point_base[1]:.3f} m")
    print(f"  Z: {point_base[2]:.3f} m")
    
    # 5. 现在可以用这个坐标控制机械臂
    print("\n可以发送给机械臂的目标位置已准备好！")


if __name__ == "__main__":
    example_usage()
