#!/usr/bin/env python3
"""
坐标转换工具类
============

提供像素坐标到机械臂基座坐标系之间的转换功能。
使用 OpenCV 相机内参 + 手眼标定 + 已知桌面高度，无需深度相机。

方法: 第一阶段的直接推广
  - 已知: 相机内参(fx,fy,cx,cy), 手眼标定(T_cam_to_base), 桌面高度(table_height)
  - 对于任意像素(u,v):
    1. 计算深度 depth = camera_height - table_height（相机近似垂直朝下）
    2. 相机坐标系中的3D点: [(u-cx)/fx * depth, (v-cy)/fy * depth, depth]
    3. 用 T_cam_to_base 变换到基座坐标系
    4. 将Z坐标设为桌面高度
"""

import numpy as np
import json
from pathlib import Path


class CoordinateTransformer:
    """像素坐标到机器人基座坐标的转换器（无需深度相机）"""

    def __init__(
        self,
        intrinsics_file: str = "outputs/cheap_camera_intrinsics.json",
        calibration_file: str = "outputs/camera_to_base_calibration.json",
        table_height_file: str = "outputs/table_height.json",
    ):
        """
        初始化转换器，加载所有标定数据。

        Args:
            intrinsics_file: 相机内参文件
            calibration_file: 手眼标定结果文件
            table_height_file: 桌面高度标定文件
        """
        # 1. 加载相机内参
        with open(intrinsics_file) as f:
            intrinsics = json.load(f)
        camera_matrix = np.array(intrinsics["camera_matrix"])
        self.fx = camera_matrix[0, 0]
        self.fy = camera_matrix[1, 1]
        self.cx = camera_matrix[0, 2]
        self.cy = camera_matrix[1, 2]
        self.image_width = intrinsics["image_width"]
        self.image_height = intrinsics["image_height"]

        # 2. 加载手眼标定
        with open(calibration_file) as f:
            calib_data = json.load(f)
        self.T_cam_to_base = np.array(calib_data["T_cam_to_base"])
        self.T_base_to_cam = np.linalg.inv(self.T_cam_to_base)

        # 3. 加载桌面高度
        with open(table_height_file) as f:
            table_data = json.load(f)
        self.table_height = table_data["table_height_base"]

        # 4. 计算相机到桌面的深度（相机Z方向距离的近似值）
        camera_pos_base = self.T_cam_to_base[:3, 3]
        self.depth_camera = camera_pos_base[2] - self.table_height

        print("✅ 坐标转换器已初始化")
        print(f"   相机内参: fx={self.fx:.1f}, fy={self.fy:.1f}, cx={self.cx:.1f}, cy={self.cy:.1f}")
        print(f"   相机位置(基座系): [{camera_pos_base[0]:.3f}, {camera_pos_base[1]:.3f}, {camera_pos_base[2]:.3f}] m")
        print(f"   桌面高度: {self.table_height * 1000:.1f} mm")
        print(f"   相机到桌面深度: {self.depth_camera * 1000:.1f} mm")

    def pixel_to_base_3d(self, pixel_x: float, pixel_y: float) -> np.ndarray:
        """
        将像素坐标转换为机械臂基座坐标系的3D坐标。

        使用第一阶段验证过的方法:
          1. 将像素(u,v)通过内参反投影到相机坐标系的3D点
          2. 用 T_cam_to_base 变换到基座坐标系
          3. Z坐标设为桌面高度

        Args:
            pixel_x: 像素X坐标 (u)
            pixel_y: 像素Y坐标 (v)

        Returns:
            [x, y, z] 在机械臂基座坐标系中的3D坐标（米）
        """
        depth = self.depth_camera

        # 相机坐标系中的3D点（与第一阶段 compute_camera_center_3d 方法一致）
        x_cam = (pixel_x - self.cx) / self.fx * depth
        y_cam = (pixel_y - self.cy) / self.fy * depth
        z_cam = depth

        point_cam = np.array([x_cam, y_cam, z_cam, 1.0])

        # 变换到基座坐标系
        point_base = self.T_cam_to_base @ point_cam

        # Z坐标设为桌面高度（确保在桌面上）
        point_base[2] = self.table_height
        # 注意: 原先此处有手动补偿 CORRECTION_X/Y，
        # 现已改为用 FKCorrector（基于 check_urdf_error 数据拟合的仿射校正）
        # 补偿在 whack_a_mole.py / tap_one_mole.py 中调用 fk_corr.target_for_ik() 实现
        return point_base[:3]

    def update_depth_from_apriltag(self, depth_m: float):
        """
        用 AprilTag 实时测量的深度更新转换器参数。

        Args:
            depth_m: 相机到底板表面的深度（米），来自 AprilTagLocator.locate() 的第二个返回值

        注意：
          - 仅需在每次「底板位置可能改变」时调用（如每局游戏开始时）
          - 相机与机械臂基座的关系（T_cam_to_base）不变，无需重新标定
        """
        old_depth = self.depth_camera
        camera_pos_base = self.T_cam_to_base[:3, 3]
        self.depth_camera = float(depth_m)
        self.table_height = camera_pos_base[2] - self.depth_camera
        print(f"   深度更新: {old_depth * 1000:.1f} mm → {self.depth_camera * 1000:.1f} mm  "
              f"(桌面高度: {self.table_height * 1000:.1f} mm)")

    def pixel_to_base_3d_batch(self, pixels: np.ndarray) -> np.ndarray:
        """
        批量将像素坐标转换为基座坐标系3D坐标。

        Args:
            pixels: Nx2 数组，每行是 [pixel_x, pixel_y]

        Returns:
            Nx3 数组，每行是 [x, y, z] 在基座坐标系中（米）
        """
        results = np.zeros((len(pixels), 3))
        for i, (px, py) in enumerate(pixels):
            results[i] = self.pixel_to_base_3d(px, py)
        return results

    def base_to_pixel(self, point_base: np.ndarray) -> tuple[float, float]:
        """
        将基座坐标系的3D点投影到像素坐标。
        （反向转换，用于可视化验证）

        Args:
            point_base: [x, y, z] 在机械臂基座坐标系中的坐标（米）

        Returns:
            (pixel_x, pixel_y) 像素坐标
        """
        # 基座 -> 相机坐标系
        point_base_homo = np.array([*point_base, 1.0])
        point_cam = self.T_base_to_cam @ point_base_homo

        # 相机坐标系 -> 像素
        if abs(point_cam[2]) < 1e-6:
            return float("nan"), float("nan")

        pixel_x = self.fx * point_cam[0] / point_cam[2] + self.cx
        pixel_y = self.fy * point_cam[1] / point_cam[2] + self.cy

        return float(pixel_x), float(pixel_y)

    def verify_center_pixel(self) -> dict:
        """
        验证: 对相机中心像素(cx, cy)的转换结果应与第一阶段一致。

        Returns:
            包含验证结果的字典
        """
        center_pos = self.pixel_to_base_3d(self.cx, self.cy)

        print("\n🔍 验证: 相机中心像素转换")
        print(f"   中心像素: ({self.cx:.1f}, {self.cy:.1f})")
        print(f"   基座坐标: [{center_pos[0] * 1000:.1f}, {center_pos[1] * 1000:.1f}, {center_pos[2] * 1000:.1f}] mm")
        print(f"   (应与第一阶段目标位置 [230.6, 45.2, -7.9] mm 一致)")

        return {
            "center_pixel": (self.cx, self.cy),
            "base_position_m": center_pos.tolist(),
            "base_position_mm": (center_pos * 1000).tolist(),
        }


def main():
    """验证坐标转换器"""
    print("=" * 60)
    print("坐标转换器验证")
    print("=" * 60)

    transformer = CoordinateTransformer()

    # 验证1: 相机中心像素
    result = transformer.verify_center_pixel()

    # 验证2: 几个关键点
    print("\n" + "=" * 60)
    print("多点转换测试")
    print("=" * 60)

    test_pixels = [
        (transformer.cx, transformer.cy),   # 中心
        (0, 0),                               # 左上角
        (transformer.image_width, 0),         # 右上角
        (0, transformer.image_height),        # 左下角
        (transformer.image_width, transformer.image_height),  # 右下角
        (640, 360),                           # 图像中心（若不是主点）
    ]

    for px, py in test_pixels:
        pos = transformer.pixel_to_base_3d(px, py)
        print(f"  像素 ({px:7.1f}, {py:7.1f}) -> 基座 [{pos[0] * 1000:7.1f}, {pos[1] * 1000:7.1f}, {pos[2] * 1000:7.1f}] mm")


if __name__ == "__main__":
    main()
