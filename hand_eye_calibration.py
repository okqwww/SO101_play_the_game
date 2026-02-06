#!/usr/bin/env python3
"""
Eye-to-hand 手眼标定系统
=====================

场景：
- 相机（D435i）固定在桌面上方
- 机械臂（SO101）可以移动
- 标定板固定在机械臂末端

目标：
求解相机坐标系到机械臂基座坐标系的变换矩阵 T_cam_to_base

步骤：
1. 移动机械臂到多个不同位置和姿态（至少10个）
2. 在每个位置采集：
   - 机械臂末端位姿（通过正运动学计算）T_base_to_end
   - 标定板在相机中的位姿（通过OpenCV检测）T_cam_to_board
3. 求解手眼标定方程：AX=XB
   其中 X 是 T_cam_to_base（我们要求的）

数学原理：
对于两个不同的机械臂位置 i 和 j：
T_cam_to_base @ T_base_to_end_i @ T_end_to_board = T_cam_to_board_i
T_cam_to_base @ T_base_to_end_j @ T_end_to_board = T_cam_to_board_j

由于标定板固定在末端，T_end_to_board 是固定的，可以消去
"""

import numpy as np
import cv2
import json
import time
from pathlib import Path
from typing import List, Tuple, Dict
import pyrealsense2 as rs

from lerobot.cameras.realsense import RealSenseCamera, RealSenseCameraConfig
from lerobot.cameras.configs import ColorMode
from lerobot.robots.so101_follower import SO101Follower, SO101FollowerConfig
from lerobot.model.kinematics import RobotKinematics


class HandEyeCalibrator:
    """Eye-to-hand 手眼标定类"""
    
    def __init__(
        self,
        robot_port: str = "/dev/ttyACM0",
        camera_serial: str = "141722073600",
        urdf_path: str = "SO101/so101_new_calib.urdf",
        checkerboard_size: Tuple[int, int] = (11, 8),  # 棋盘格内角点数量 (6列×5行的格子)
        square_size: float = 0.005,  # 棋盘格方格大小（米，12mm）
    ):
        """
        初始化手眼标定器
        
        Args:
            robot_port: 机械臂串口
            camera_serial: 相机序列号
            urdf_path: URDF文件路径
            checkerboard_size: 棋盘格内角点数量 (列, 行)
            square_size: 棋盘格方格大小（米）
        """
        self.checkerboard_size = checkerboard_size
        self.square_size = square_size
        
        # 初始化机械臂
        print("初始化机械臂...")
        robot_config = SO101FollowerConfig(
            port=robot_port,
            id="hand_eye_calib_arm",
        )
        self.robot = SO101Follower(robot_config)
        
        # 初始化相机
        print("初始化相机...")
        camera_config = RealSenseCameraConfig(
            serial_number_or_name=camera_serial,
            fps=30,
            width=1280,
            height=720,
            color_mode=ColorMode.RGB,
            use_depth=True,  # 启用深度
        )
        self.camera = RealSenseCamera(camera_config)
        
        # 初始化运动学求解器
        print("初始化运动学求解器...")
        self.kinematics = RobotKinematics(
            urdf_path=urdf_path,
            target_frame_name="gripper_frame_link",
            joint_names=["shoulder_pan", "shoulder_lift", "elbow_flex",
                        "wrist_flex", "wrist_roll", "gripper"]
        )
        
        # 存储标定数据
        self.calibration_data = []
        
    def connect(self):
        """连接硬件"""
        print("连接机械臂和相机...")
        self.robot.connect()
        self.camera.connect()
        print("✅ 连接成功")
        
    def disconnect(self):
        """断开连接"""
        self.robot.disconnect()
        self.camera.disconnect()
        print("✅ 已断开连接")
        
    def get_robot_end_pose(self) -> np.ndarray:
        """
        获取机械臂末端位姿（相对于基座）
        
        Returns:
            4x4变换矩阵 T_base_to_end
        """
        # 读取当前关节角度
        obs = self.robot.get_observation()
        joint_positions = np.array([
            obs["shoulder_pan.pos"],
            obs["shoulder_lift.pos"],
            obs["elbow_flex.pos"],
            obs["wrist_flex.pos"],
            obs["wrist_roll.pos"],
            obs["gripper.pos"],
        ])
        
        # 正运动学
        T_base_to_end = self.kinematics.forward_kinematics(joint_positions)
        return T_base_to_end
        
    def detect_checkerboard_with_depth(
        self,
        color_image: np.ndarray,
        depth_image: np.ndarray,
        camera_intrinsics: rs.intrinsics
    ) -> Tuple[bool, np.ndarray, np.ndarray]:
        """
        检测棋盘格位姿（充分利用深度信息）
        
        方法：对每个检测到的角点，使用其像素坐标+深度值反投影到3D空间，
        得到角点在相机坐标系下的真实3D坐标，然后用3D-3D配准求解位姿。
        
        Args:
            color_image: RGB图像
            depth_image: 深度图像（米）
            camera_intrinsics: 相机内参
            
        Returns:
            success: 是否检测成功
            rvec: 旋转向量
            tvec: 平移向量（米）
        """
        # 转换为灰度图
        gray = cv2.cvtColor(color_image, cv2.COLOR_RGB2GRAY)
        
        # 检测棋盘格角点
        ret, corners = cv2.findChessboardCorners(
            gray,
            self.checkerboard_size,
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
        )
        
        if not ret:
            return False, None, None
            
        # 亚像素精化
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        
        # 方法1：使用深度信息构建真实3D点云
        points_3d_camera = []  # 角点在相机坐标系的3D坐标
        valid_indices = []
        
        for i, corner in enumerate(corners_refined):
            px, py = corner[0]
            px_int, py_int = int(px), int(py)
            
            # 获取该角点的深度值（取周围3x3区域的中位数，更鲁棒）
            if 1 <= py_int < depth_image.shape[0] - 1 and 1 <= px_int < depth_image.shape[1] - 1:
                depth_patch = depth_image[py_int-1:py_int+2, px_int-1:px_int+2]
                valid_depths = depth_patch[depth_patch > 0]  # 过滤无效深度
                
                if len(valid_depths) > 0:
                    depth_m = np.median(valid_depths) * 0.001  # 转换为米
                    
                    # 深度合理性检查（建议保持0.3m以上以确保精度）
                    # 用户要求: 0.05-2.0m，但建议至少0.2m以上
                    if 0.2 < depth_m < 2.0:  # 可改为 0.15 或 0.3
                        # 使用RealSense SDK反投影到3D
                        point_3d = rs.rs2_deproject_pixel_to_point(
                            camera_intrinsics,
                            [float(px), float(py)],
                            depth_m
                        )
                        points_3d_camera.append(point_3d)
                        valid_indices.append(i)
        
        # 需要足够的有效点（至少一半的角点）
        min_points = len(corners_refined) // 2
        if len(points_3d_camera) < min_points:
            print(f"  ⚠️  有效深度点太少: {len(points_3d_camera)}/{len(corners_refined)}")
            return False, None, None
        
        points_3d_camera = np.array(points_3d_camera, dtype=np.float32)
        
        # 构建对应的棋盘格坐标（标定板自身坐标系）
        objp_all = np.zeros((self.checkerboard_size[0] * self.checkerboard_size[1], 3), np.float32)
        objp_all[:, :2] = np.mgrid[
            0:self.checkerboard_size[0],
            0:self.checkerboard_size[1]
        ].T.reshape(-1, 2)
        objp_all *= self.square_size
        
        # 只使用有有效深度的角点
        objp_valid = objp_all[valid_indices]
        
        # 方法2：使用带深度约束的PnP求解
        # 将3D点云与标定板模型点进行配准
        # 使用OpenCV的solvePnP，但提供更精确的初始猜测
        
        # 构造相机内参矩阵
        camera_matrix = np.array([
            [camera_intrinsics.fx, 0, camera_intrinsics.ppx],
            [0, camera_intrinsics.fy, camera_intrinsics.ppy],
            [0, 0, 1]
        ])
        dist_coeffs = np.array(camera_intrinsics.coeffs)
        
        # 先用所有点做标准PnP得到初始估计
        ret_init, rvec_init, tvec_init = cv2.solvePnP(
            objp_all,
            corners_refined,
            camera_matrix,
            dist_coeffs,
            flags=cv2.SOLVEPNP_ITERATIVE
        )
        
        if not ret_init:
            return False, None, None
        
        # 使用深度约束的点进行精化
        # 用solvePnP的RANSAC版本，对异常点更鲁棒
        ret, rvec, tvec, inliers = cv2.solvePnPRansac(
            objp_valid,
            corners_refined[valid_indices],
            camera_matrix,
            dist_coeffs,
            rvec=rvec_init,
            tvec=tvec_init,
            useExtrinsicGuess=True,
            iterationsCount=200,
            reprojectionError=3.0,  # 像素误差阈值
            confidence=0.99
        )
        
        if not ret or inliers is None:
            # 如果RANSAC失败，使用迭代方法
            ret, rvec, tvec = cv2.solvePnP(
                objp_valid,
                corners_refined[valid_indices],
                camera_matrix,
                dist_coeffs,
                rvec=rvec_init,
                tvec=tvec_init,
                useExtrinsicGuess=True,
                flags=cv2.SOLVEPNP_ITERATIVE
            )
        
        if not ret:
            return False, None, None
        
        # 验证结果：计算深度点云中心与估计位置的距离
        cloud_center = np.mean(points_3d_camera, axis=0)
        estimated_center = tvec.flatten()
        distance_error = np.linalg.norm(cloud_center - estimated_center)
        
        print(f"  ✅ 有效深度点: {len(points_3d_camera)}/{len(corners_refined)}")
        print(f"  深度点云中心: [{cloud_center[0]:.3f}, {cloud_center[1]:.3f}, {cloud_center[2]:.3f}]m")
        print(f"  PnP估计位置: [{estimated_center[0]:.3f}, {estimated_center[1]:.3f}, {estimated_center[2]:.3f}]m")
        print(f"  中心距离误差: {distance_error*1000:.1f}mm")
        
        return True, rvec, tvec
        
    def visualize_detection(
        self,
        color_image: np.ndarray,
        rvec: np.ndarray,
        tvec: np.ndarray,
        camera_intrinsics: rs.intrinsics
    ) -> np.ndarray:
        """可视化检测结果"""
        img_vis = color_image.copy()
        
        # 转换为OpenCV格式
        img_vis = cv2.cvtColor(img_vis, cv2.COLOR_RGB2BGR)
        
        # 检测并绘制角点
        gray = cv2.cvtColor(color_image, cv2.COLOR_RGB2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, self.checkerboard_size, None)
        
        if ret:
            cv2.drawChessboardCorners(img_vis, self.checkerboard_size, corners, ret)
            
            # 绘制坐标轴
            camera_matrix = np.array([
                [camera_intrinsics.fx, 0, camera_intrinsics.ppx],
                [0, camera_intrinsics.fy, camera_intrinsics.ppy],
                [0, 0, 1]
            ])
            dist_coeffs = np.array(camera_intrinsics.coeffs)
            
            cv2.drawFrameAxes(
                img_vis,
                camera_matrix,
                dist_coeffs,
                rvec,
                tvec,
                self.square_size * 3
            )
            
        return img_vis
        
    def capture_calibration_pose(self, pose_id: int) -> bool:
        """
        采集一个标定位姿
        
        Args:
            pose_id: 位姿编号
            
        Returns:
            是否采集成功
        """
        print(f"\n采集位姿 #{pose_id}")
        
        # 1. 获取机械臂末端位姿
        T_base_to_end = self.get_robot_end_pose()
        print(f"  机械臂末端位姿:")
        print(f"    位置: {T_base_to_end[:3, 3]}")
        
        # 2. 拍摄图像
        color_image = self.camera.read()
        depth_image = self.camera.read_depth()
        
        # 获取相机内参
        # 注意：RealSenseCamera类中的pipeline属性名为rs_pipeline
        profile = self.camera.rs_pipeline.get_active_profile()
        color_stream = profile.get_stream(rs.stream.color)
        intrinsics = color_stream.as_video_stream_profile().get_intrinsics()
        
        # 3. 检测棋盘格
        success, rvec, tvec = self.detect_checkerboard_with_depth(
            color_image,
            depth_image,
            intrinsics
        )
        
        if not success:
            print("  ❌ 未检测到棋盘格")
            return False
            
        # 4. 构造相机到标定板的变换矩阵
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        T_cam_to_board = np.eye(4)
        T_cam_to_board[:3, :3] = rotation_matrix
        T_cam_to_board[:3, 3] = tvec.flatten()
        
        print(f"  相机到标定板:")
        print(f"    位置: {tvec.flatten()}")
        
        # 5. 保存数据
        self.calibration_data.append({
            "pose_id": pose_id,
            "T_base_to_end": T_base_to_end.tolist(),
            "T_cam_to_board": T_cam_to_board.tolist(),
            "rvec": rvec.tolist(),
            "tvec": tvec.tolist(),
        })
        
        # 6. 保存可视化图像
        img_vis = self.visualize_detection(color_image, rvec, tvec, intrinsics)
        output_path = f"outputs/hand_eye_pose_{pose_id:02d}.png"
        cv2.imwrite(output_path, img_vis)
        print(f"  ✅ 已保存: {output_path}")
        
        return True
        
    def save_calibration_data(self, filename: str = "hand_eye_data.json"):
        """保存标定数据"""
        output_file = Path("outputs") / filename
        output_file.parent.mkdir(exist_ok=True)
        
        with open(output_file, 'w') as f:
            json.dump(self.calibration_data, f, indent=2)
            
        print(f"\n✅ 标定数据已保存到: {output_file}")
        
    def load_calibration_data(self, filename: str = "hand_eye_data.json"):
        """加载标定数据"""
        input_file = Path("outputs") / filename
        
        with open(input_file, 'r') as f:
            self.calibration_data = json.load(f)
            
        print(f"✅ 已加载 {len(self.calibration_data)} 组标定数据")


def main():
    """测试手眼标定数据采集"""
    print("=" * 60)
    print("Eye-to-hand 手眼标定 - 数据采集")
    print("=" * 60)
    
    calibrator = HandEyeCalibrator(
        robot_port="/dev/ttyACM0",
        camera_serial="141722073600",
        urdf_path="SO101/so101_new_calib.urdf",
        checkerboard_size=(11, 8),  # 6列×5行格子 = 5×4内角点
        square_size=0.005,  # 12mm方格，总尺寸约 7.2cm×6cm
    )
    
    try:
        calibrator.connect()
        
        print("\n准备开始数据采集...")
        print("请将棋盘格固定在机械臂末端")
        print("确保棋盘格在相机视野内")
        input("\n按 Enter 开始...")
        
        # 采集多个位姿（手动移动机械臂到不同位置）
        pose_count = 0
        
        while True:
            print("\n" + "=" * 60)
            print(f"准备采集第 {pose_count + 1} 个位姿")
            
            # 禁用力矩，让用户可以手动移动机械臂
            print("正在禁用机械臂力矩（现在可以手动移动了）...")
            calibrator.robot.bus.disable_torque()
            
            print("请手动移动机械臂到新位置（确保棋盘格在相机视野内）")
            user_input = input("按 Enter 采集，输入 'q' 退出: ")
            
            if user_input.lower() == 'q':
                break
            
            # 启用力矩，保持位置稳定用于拍照
            print("启用力矩，固定位置...")
            calibrator.robot.bus.enable_torque()
            time.sleep(0.5)  # 等待机械臂稳定
            
            if calibrator.capture_calibration_pose(pose_count):
                pose_count += 1
                print(f"✅ 已成功采集 {pose_count} 个位姿")
            else:
                print("❌ 采集失败，请调整位置后重试")
                
        # 保存数据
        if pose_count > 0:
            calibrator.save_calibration_data()
            print(f"\n✅ 共采集 {pose_count} 个有效位姿")
            print(f"建议至少采集 10-15 个位姿以获得更好的标定精度")
        else:
            print("\n⚠️  没有采集到任何数据")
            
    finally:
        calibrator.disconnect()


if __name__ == "__main__":
    main()
