#!/usr/bin/env python3
"""
验证手眼标定精度
===============

测试方法：
1. 移动机械臂到一个位置
2. 通过正运动学计算末端在基座坐标系的位置
3. 通过相机+标定结果计算末端位置（检测标定板）
4. 比较两者差异
"""

import numpy as np
import json
from pathlib import Path

from hand_eye_calibration import HandEyeCalibrator


class CalibrationValidator:
    """标定验证器"""
    
    def __init__(self, calibration_file: str = "outputs/camera_to_base_calibration.json"):
        """
        Args:
            calibration_file: 标定结果文件
        """
        # 加载标定结果
        with open(calibration_file) as f:
            calib_data = json.load(f)
            self.T_cam_to_base = np.array(calib_data["T_cam_to_base"])
            
        print("✅ 已加载标定结果")
        print(f"   相机位置: {calib_data['translation_m']}")
        
        # 初始化标定器（复用数据采集功能）
        self.calibrator = HandEyeCalibrator()
        
    def test_single_pose(self) -> dict:
        """测试单个位姿的精度"""
        print("\n测试位姿精度...")
        
        # 1. 通过正运动学获取末端位姿
        T_base_to_end = self.calibrator.get_robot_end_pose()
        end_pos_fk = T_base_to_end[:3, 3]
        
        print(f"  正运动学计算的末端位置: {end_pos_fk}")
        
        # 2. 通过相机观测标定板
        color_image = self.calibrator.camera.read()
        depth_image = self.calibrator.camera.read_depth()
        
        # 获取相机内参
        import pyrealsense2 as rs
        profile = self.calibrator.camera.rs_pipeline.get_active_profile()
        color_stream = profile.get_stream(rs.stream.color)
        intrinsics = color_stream.as_video_stream_profile().get_intrinsics()
        
        success, rvec, tvec = self.calibrator.detect_checkerboard_with_depth(
            color_image,
            depth_image,
            intrinsics
        )
        
        if not success:
            print("  ❌ 未检测到标定板")
            return None
            
        # 3. 构造相机到标定板的变换
        import cv2
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        T_cam_to_board = np.eye(4)
        T_cam_to_board[:3, :3] = rotation_matrix
        T_cam_to_board[:3, 3] = tvec.flatten()
        
        # 4. 计算标定板在基座坐标系的位置
        T_base_to_board = self.T_cam_to_base @ T_cam_to_board
        board_pos_calib = T_base_to_board[:3, 3]
        
        print(f"  标定结果计算的标定板位置: {board_pos_calib}")
        
        # 5. 计算误差
        # 注意：这里比较的是"末端"和"标定板"的位置
        # 它们有一个固定偏移 T_end_to_board（我们不知道具体值）
        # 所以我们比较的是相对变化
        
        result = {
            "end_pos_fk": end_pos_fk.tolist(),
            "board_pos_calib": board_pos_calib.tolist(),
            "T_base_to_end": T_base_to_end.tolist(),
            "T_cam_to_board": T_cam_to_board.tolist(),
        }
        
        return result
        
    def test_multiple_poses(self, num_poses: int = 5):
        """测试多个位姿的一致性"""
        print(f"\n测试 {num_poses} 个位姿的一致性...")
        
        results = []
        
        for i in range(num_poses):
            print(f"\n位姿 #{i+1}")
            print("请移动机械臂到新位置，按 Enter 继续...")
            input()
            
            import time
            time.sleep(0.5)  # 等待稳定
            
            result = self.test_single_pose()
            if result:
                results.append(result)
                print(f"  ✅ 采集成功")
            else:
                print(f"  ❌ 采集失败")
                
        # 分析一致性
        if len(results) >= 2:
            print("\n" + "=" * 60)
            print("一致性分析：")
            
            # 计算相邻位姿之间的关系是否一致
            for i in range(len(results) - 1):
                T_base_end_i = np.array(results[i]["T_base_to_end"])
                T_base_end_j = np.array(results[i+1]["T_base_to_end"])
                
                T_cam_board_i = np.array(results[i]["T_cam_to_board"])
                T_cam_board_j = np.array(results[i+1]["T_cam_to_board"])
                
                # 机械臂移动距离
                delta_robot = np.linalg.norm(T_base_end_j[:3, 3] - T_base_end_i[:3, 3])
                
                # 标定板在相机中的移动距离
                delta_camera = np.linalg.norm(T_cam_board_j[:3, 3] - T_cam_board_i[:3, 3])
                
                print(f"  位姿{i+1}→{i+2}:")
                print(f"    机械臂移动: {delta_robot*1000:.1f} mm")
                print(f"    相机观测移动: {delta_camera*1000:.1f} mm")
                print(f"    比率: {delta_camera/delta_robot:.3f}")
                
        # 保存测试结果
        output_file = Path("outputs") / "calibration_validation.json"
        with open(output_file, 'w') as f:
            json.dump(results, f, indent=2)
        print(f"\n✅ 测试结果已保存到: {output_file}")


def main():
    """主程序"""
    print("=" * 60)
    print("手眼标定验证")
    print("=" * 60)
    
    try:
        validator = CalibrationValidator()
        validator.calibrator.connect()
        
        print("\n选择测试模式:")
        print("  1. 测试单个位姿")
        print("  2. 测试多个位姿的一致性")
        
        choice = input("\n请选择 (1/2): ")
        
        if choice == "1":
            result = validator.test_single_pose()
            if result:
                print("\n✅ 测试完成")
        elif choice == "2":
            num_poses = int(input("请输入测试位姿数量 (建议5-10): "))
            validator.test_multiple_poses(num_poses)
        else:
            print("无效选择")
            
    except FileNotFoundError:
        print("\n❌ 错误：找不到标定结果文件")
        print("   请先运行 hand_eye_solver.py 完成标定")
    except Exception as e:
        print(f"\n❌ 错误：{e}")
        import traceback
        traceback.print_exc()
    finally:
        try:
            validator.calibrator.disconnect()
        except:
            pass


if __name__ == "__main__":
    main()
