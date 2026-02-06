#!/usr/bin/env python3
"""
验证手眼标定精度 - OpenCV相机版本
=====================================

测试方法：
1. 移动机械臂到一个位置
2. 通过正运动学计算末端在基座坐标系的位置
3. 通过相机+标定结果计算末端位置（检测标定板）
4. 比较两者差异
"""

import numpy as np
import cv2
import json
import time
from pathlib import Path

from hand_eye_calibration_opencv import HandEyeCalibratorOpenCV


class CalibrationValidatorOpenCV:
    """标定验证器 - OpenCV相机版本"""
    
    def __init__(
        self,
        calibration_file: str = "outputs/camera_to_base_calibration.json",
        camera_index: int = 0,
        camera_intrinsics_file: str = "outputs/cheap_camera_intrinsics.json"
    ):
        """
        Args:
            calibration_file: 手眼标定结果文件
            camera_index: 相机索引
            camera_intrinsics_file: 相机内参文件
        """
        # 加载手眼标定结果
        with open(calibration_file) as f:
            calib_data = json.load(f)
            self.T_cam_to_base = np.array(calib_data["T_cam_to_base"])
            
        print("✅ 已加载手眼标定结果")
        print(f"   相机位置: {calib_data['translation_m']}")
        
        # 初始化标定器（复用数据采集功能）
        self.calibrator = HandEyeCalibratorOpenCV(
            camera_index=camera_index,
            camera_intrinsics_file=camera_intrinsics_file
        )
        
    def test_single_pose(self) -> dict:
        """测试单个位姿的精度"""
        print("\n测试位姿精度...")
        
        # 1. 通过正运动学获取末端位姿
        T_base_to_end = self.calibrator.get_robot_end_pose()
        end_pos_fk = T_base_to_end[:3, 3]
        
        print(f"  正运动学计算的末端位置: {end_pos_fk}")
        
        # 2. 清空相机缓冲区，读取最新图像
        print("  正在读取相机图像...")
        for _ in range(5):
            self.calibrator.camera.read()
            time.sleep(0.12)
        
        color_image = self.calibrator.camera.read()
        
        # 3. 检测棋盘格
        success, rvec, tvec = self.calibrator.detect_checkerboard_pnp(color_image)
        
        if not success:
            print("  ❌ 未检测到标定板")
            return None
            
        # 4. 构造相机到标定板的变换
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        T_cam_to_board = np.eye(4)
        T_cam_to_board[:3, :3] = rotation_matrix
        T_cam_to_board[:3, 3] = tvec.flatten()
        
        print(f"  相机观测的标定板位置: {tvec.flatten()}")
        
        # 5. 计算标定板在基座坐标系的位置
        T_base_to_board = self.T_cam_to_base @ T_cam_to_board
        board_pos_calib = T_base_to_board[:3, 3]
        
        print(f"  标定结果计算的标定板位置（基座坐标系）: {board_pos_calib}")
        
        # 6. 计算误差
        # 注意：这里比较的是"末端"和"标定板"的位置
        # 它们有一个固定偏移 T_end_to_board（标定板相对末端的安装位置）
        # 所以我们主要关注相对变化的一致性
        
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
        print("这个测试会验证手眼标定的准确性")
        print("如果标定正确，机械臂移动多少，相机观测也应该变化相应的距离\n")
        
        results = []
        
        for i in range(num_poses):
            print("\n" + "=" * 60)
            print(f"位姿 #{i+1}/{num_poses}")
            
            # 禁用力矩，让用户移动机械臂
            print("正在禁用机械臂力矩...")
            self.calibrator.robot.bus.disable_torque()
            
            print("请移动机械臂到新位置")
            input("按 Enter 继续...")
            
            # 启用力矩，固定位置
            print("启用力矩，固定位置...")
            self.calibrator.robot.bus.enable_torque()
            time.sleep(1.0)  # 等待稳定
            
            result = self.test_single_pose()
            if result:
                results.append(result)
                print(f"  ✅ 采集成功")
            else:
                print(f"  ❌ 采集失败（未检测到标定板）")
                
        # 分析一致性
        if len(results) >= 2:
            print("\n" + "=" * 60)
            print("一致性分析：")
            print("=" * 60)
            
            # 计算相邻位姿之间的关系是否一致
            errors = []
            for i in range(len(results) - 1):
                T_base_end_i = np.array(results[i]["T_base_to_end"])
                T_base_end_j = np.array(results[i+1]["T_base_to_end"])
                
                T_cam_board_i = np.array(results[i]["T_cam_to_board"])
                T_cam_board_j = np.array(results[i+1]["T_cam_to_board"])
                
                # 机械臂移动距离
                delta_robot = np.linalg.norm(T_base_end_j[:3, 3] - T_base_end_i[:3, 3])
                
                # 标定板在相机中的移动距离
                delta_camera = np.linalg.norm(T_cam_board_j[:3, 3] - T_cam_board_i[:3, 3])
                
                # 计算差异
                ratio = delta_camera / delta_robot if delta_robot > 0.001 else 0
                error = abs(delta_camera - delta_robot)
                errors.append(error)
                
                print(f"\n  位姿 {i+1} → {i+2}:")
                print(f"    机械臂末端移动: {delta_robot*1000:.2f} mm")
                print(f"    相机观测移动:   {delta_camera*1000:.2f} mm")
                print(f"    比率: {ratio:.3f} (理想值: 1.000)")
                print(f"    误差: {error*1000:.2f} mm")
                
                if abs(ratio - 1.0) < 0.1:
                    print(f"    状态: ✅ 好")
                elif abs(ratio - 1.0) < 0.2:
                    print(f"    状态: ⚠️  一般")
                else:
                    print(f"    状态: ❌ 差")
            
            print("\n" + "=" * 60)
            print("总体评估：")
            print("=" * 60)
            mean_error = np.mean(errors) * 1000
            max_error = np.max(errors) * 1000
            
            print(f"  平均误差: {mean_error:.2f} mm")
            print(f"  最大误差: {max_error:.2f} mm")
            
            if mean_error < 5:
                print(f"  评级: ✅ 优秀 (误差<5mm)")
            elif mean_error < 10:
                print(f"  评级: ✅ 良好 (误差<10mm)")
            elif mean_error < 20:
                print(f"  评级: ⚠️  一般 (误差<20mm)")
            else:
                print(f"  评级: ❌ 需要改进 (误差>20mm)")
                print(f"\n  建议：")
                print(f"    1. 检查相机内参标定质量")
                print(f"    2. 增加手眼标定采集的位姿数量")
                print(f"    3. 确保标定板平整且固定牢固")
                
        # 保存测试结果
        output_file = Path("outputs") / "calibration_validation_opencv.json"
        with open(output_file, 'w') as f:
            json.dump({
                "results": results,
                "mean_error_mm": float(np.mean(errors) * 1000) if errors else 0,
                "max_error_mm": float(np.max(errors) * 1000) if errors else 0
            }, f, indent=2)
        print(f"\n✅ 测试结果已保存到: {output_file}")


def main():
    """主程序"""
    print("=" * 60)
    print("手眼标定验证 - OpenCV相机版本")
    print("=" * 60)
    
    try:
        validator = CalibrationValidatorOpenCV()
        validator.calibrator.connect()
        
        print("\n选择测试模式:")
        print("  1. 测试单个位姿")
        print("  2. 测试多个位姿的一致性（推荐）")
        
        choice = input("\n请选择 (1/2): ")
        
        if choice == "1":
            # 禁用力矩，让用户移动机械臂
            print("\n正在禁用机械臂力矩...")
            validator.calibrator.robot.bus.disable_torque()
            print("请移动机械臂到测试位置")
            input("按 Enter 开始测试...")
            
            # 启用力矩
            validator.calibrator.robot.bus.enable_torque()
            time.sleep(1.0)
            
            result = validator.test_single_pose()
            if result:
                print("\n✅ 测试完成")
        elif choice == "2":
            num_poses = int(input("请输入测试位姿数量 (建议5-10): "))
            validator.test_multiple_poses(num_poses)
        else:
            print("无效选择")
            
    except FileNotFoundError as e:
        print(f"\n❌ 错误：找不到文件")
        print(f"   {e}")
        print("\n请确保：")
        print("   1. 已完成相机内参标定 (cheap_camera_intrinsics.json)")
        print("   2. 已完成手眼标定 (camera_to_base_calibration.json)")
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
