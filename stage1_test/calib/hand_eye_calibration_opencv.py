#!/usr/bin/env python3
"""
Eye-to-hand 手眼标定系统 - OpenCV相机版本
"""

import numpy as np
import cv2
import json
import time
from pathlib import Path
from typing import List, Tuple, Dict

# 使用OpenCV相机替代RealSense
from lerobot.cameras.opencv import OpenCVCamera, OpenCVCameraConfig
from lerobot.cameras.configs import ColorMode
from lerobot.robots.so101_follower import SO101Follower, SO101FollowerConfig
from lerobot.model.kinematics import RobotKinematics


class HandEyeCalibratorOpenCV:
    """Eye-to-hand 手眼标定类 - OpenCV相机版本"""
    
    def __init__(
        self,
        robot_port: str = "/dev/ttyACM0",
        camera_index: int = 0,  # 相机索引
        camera_intrinsics_file: str = "outputs/cheap_camera_intrinsics.json",  # 内参文件
        urdf_path: str = "SO101/so101_5dof_stylus.urdf",
        checkerboard_size: Tuple[int, int] = (11, 8),
        square_size: float = 0.005,
    ):
        self.checkerboard_size = checkerboard_size
        self.square_size = square_size
        
        # 加载相机内参
        print(f"加载相机内参: {camera_intrinsics_file}")
        with open(camera_intrinsics_file, 'r') as f:
            calib_data = json.load(f)
        
        self.camera_matrix = np.array(calib_data["camera_matrix"])
        self.dist_coeffs = np.array(calib_data["dist_coeffs"])
        self.image_width = calib_data["image_width"]
        self.image_height = calib_data["image_height"]
        
        print(f"  相机内参矩阵:\n{self.camera_matrix}")
        print(f"  畸变系数: {self.dist_coeffs}")
        
        # 初始化机械臂
        print("初始化机械臂...")
        robot_config = SO101FollowerConfig(
            port=robot_port,
            id="hand_eye_calib_arm",
        )
        self.robot = SO101Follower(robot_config)
        
        # 初始化OpenCV相机
        print("初始化OpenCV相机...")
        camera_config = OpenCVCameraConfig(
            index_or_path=camera_index,
            fps=10,  # 廉价相机在1280x720分辨率下只支持10fps
            width=self.image_width,
            height=self.image_height,
            color_mode=ColorMode.RGB,
        )
        self.camera = OpenCVCamera(camera_config)
        
        # 初始化运动学求解器
        print("初始化运动学求解器...")
        self.kinematics = RobotKinematics(
            urdf_path=urdf_path,
            target_frame_name = "stylus_tip_link",
            joint_names=["shoulder_pan", "shoulder_lift", "elbow_flex",
                        "wrist_flex", "wrist_roll"]
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
        """获取机械臂末端位姿"""
        obs = self.robot.get_observation()
        joint_positions = np.array([
            obs["shoulder_pan.pos"],
            obs["shoulder_lift.pos"],
            obs["elbow_flex.pos"],
            obs["wrist_flex.pos"],
            obs["wrist_roll.pos"],
            # obs["gripper.pos"],
        ])
        
        T_base_to_end = self.kinematics.forward_kinematics(joint_positions)
        return T_base_to_end
        
    def detect_checkerboard_pnp(
        self,
        color_image: np.ndarray
    ) -> Tuple[bool, np.ndarray, np.ndarray]:
        """
        检测棋盘格位姿（使用纯PnP方法，无深度信息）
        
        Args:
            color_image: RGB图像
            
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
        
        # 构建棋盘格3D坐标
        objp = np.zeros((self.checkerboard_size[0] * self.checkerboard_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[
            0:self.checkerboard_size[0],
            0:self.checkerboard_size[1]
        ].T.reshape(-1, 2)
        objp *= self.square_size
        
        # PnP求解
        ret, rvec, tvec = cv2.solvePnP(
            objp,
            corners_refined,
            self.camera_matrix,
            self.dist_coeffs,
            flags=cv2.SOLVEPNP_ITERATIVE
        )
        
        if not ret:
            return False, None, None
        
        print(f"  ✅ 检测到棋盘格")
        print(f"  PnP估计位置: [{tvec[0][0]:.3f}, {tvec[1][0]:.3f}, {tvec[2][0]:.3f}]m")
        
        return True, rvec, tvec
    
    def visualize_detection(
        self,
        color_image: np.ndarray,
        rvec: np.ndarray,
        tvec: np.ndarray
    ) -> np.ndarray:
        """可视化检测结果"""
        img_vis = color_image.copy()
        
        # 绘制坐标轴
        cv2.drawFrameAxes(
            img_vis,
            self.camera_matrix,
            self.dist_coeffs,
            rvec,
            tvec,
            self.square_size * 3
        )
        
        return img_vis
        
    def capture_calibration_pose(self, pose_id: int) -> bool:
        """采集一个标定位姿"""
        print(f"\n采集位姿 #{pose_id}")
        
        # 1. 获取机械臂末端位姿
        T_base_to_end = self.get_robot_end_pose()
        print(f"  机械臂末端位姿:")
        print(f"    位置: {T_base_to_end[:3, 3]}")
        
        # 2. 清空相机缓冲区，确保读取最新图像
        print("  正在读取相机图像...")
        # 读取并丢弃几帧，清空缓冲区（相机是10fps，需要等待0.1秒/帧）
        for i in range(10):
            img = self.camera.read()
            time.sleep(0.12)  # 等待超过1帧时间（10fps = 0.1秒/帧）
            if i == 9:
                print(f"  最新图像尺寸: {img.shape}")
        
        # 读取用于检测的图像
        color_image = self.camera.read()
        
        # 3. 检测棋盘格
        success, rvec, tvec = self.detect_checkerboard_pnp(color_image)
        
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
        img_vis = self.visualize_detection(color_image, rvec, tvec)
        # OpenCV相机返回RGB，需要转BGR保存
        img_bgr = cv2.cvtColor(img_vis, cv2.COLOR_RGB2BGR)
        output_path = f"outputs/hand_eye_pose_{pose_id:02d}.png"
        cv2.imwrite(output_path, img_bgr)
        print(f"  ✅ 已保存: {output_path}")
        
        return True
        
    def save_calibration_data(self, filename: str = "hand_eye_data.json"):
        """保存标定数据"""
        output_file = Path("outputs") / filename
        output_file.parent.mkdir(exist_ok=True)
        
        with open(output_file, 'w') as f:
            json.dump(self.calibration_data, f, indent=2)
            
        print(f"\n✅ 标定数据已保存到: {output_file}")


def main():
    """测试手眼标定数据采集"""
    print("=" * 60)
    print("Eye-to-hand 手眼标定 - 数据采集（OpenCV相机）")
    print("=" * 60)
    
    calibrator = HandEyeCalibratorOpenCV(
        robot_port="/dev/ttyACM0",
        camera_index=0,  # 修改为你的相机索引
        camera_intrinsics_file="outputs/cheap_camera_intrinsics.json",
    )
    
    try:
        calibrator.connect()
        
        print("\n请将标定板固定在机械臂末端")
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
            time.sleep(1.0)  # 等待机械臂稳定
            
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