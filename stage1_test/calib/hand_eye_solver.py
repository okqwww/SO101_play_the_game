#!/usr/bin/env python3
"""
手眼标定求解器 - 求解 AX=XB 问题
================================

给定多组观测：
- A_i: 机械臂从位置1到位置i的变换
- B_i: 相机观测到的标定板从位置1到位置i的变换
求解：
- X: 相机到机械臂基座的变换矩阵

使用方法：Tsai-Lenz 算法或 OpenCV 的 calibrateHandEye
"""

import numpy as np
import cv2
import json
from pathlib import Path
from scipy.spatial.transform import Rotation
from typing import List, Tuple


class HandEyeSolver:
    """手眼标定求解器"""
    
    def __init__(self, data_file: str = "outputs/hand_eye_data.json"):
        """
        初始化求解器
        
        Args:
            data_file: 标定数据文件路径
        """
        self.data_file = data_file
        self.calibration_data = []
        
    def load_data(self):
        """加载标定数据"""
        with open(self.data_file, 'r') as f:
            self.calibration_data = json.load(f)
        print(f"✅ 已加载 {len(self.calibration_data)} 组标定数据")
        
    def prepare_hand_eye_matrices(
        self
    ) -> Tuple[List[np.ndarray], List[np.ndarray]]:
        """
        准备手眼标定所需的矩阵对
        
        对于 Eye-to-hand：
        - R_gripper2base, t_gripper2base: 机械臂末端到基座的变换
        - R_target2cam, t_target2cam: 标定板到相机的变换
        
        Returns:
            R_gripper2base_list: 旋转矩阵列表
            t_gripper2base_list: 平移向量列表
            R_target2cam_list: 旋转矩阵列表  
            t_target2cam_list: 平移向量列表
        """
        if len(self.calibration_data) < 3:
            raise ValueError(f"至少需要3组数据，当前只有{len(self.calibration_data)}组")
            
        R_gripper2base_list = []
        t_gripper2base_list = []
        R_target2cam_list = []
        t_target2cam_list = []
        
        for data in self.calibration_data:
            # 机械臂末端到基座 (已经是base到end，需要求逆)
            T_base_to_end = np.array(data["T_base_to_end"])
            T_end_to_base = np.linalg.inv(T_base_to_end)  # 求逆得到end到base
            # T_end_to_base = T_base_to_end
            R_gripper2base_list.append(T_end_to_base[:3, :3])
            t_gripper2base_list.append(T_end_to_base[:3, 3:4])
            
            # 标定板到相机 (需要求逆得到相机到标定板)
            T_cam_to_board = np.array(data["T_cam_to_board"])
            # T_board_to_cam = np.linalg.inv(T_cam_to_board)
            T_board_to_cam = T_cam_to_board    #相机和标定板之间的这个转移矩阵不需要取逆（因为睿尔曼就是正运动学取逆PnP结果不取逆，我照着来，就对了）
            
            R_target2cam_list.append(T_board_to_cam[:3, :3])
            t_target2cam_list.append(T_board_to_cam[:3, 3:4])
            
        return R_gripper2base_list, t_gripper2base_list, R_target2cam_list, t_target2cam_list
        
    def solve_hand_eye(self, method: str = "tsai") -> Tuple[np.ndarray, np.ndarray]:
        """
        求解手眼标定
        
        Args:
            method: 求解方法
                - "tsai": Tsai-Lenz方法
                - "park": Park方法
                - "horaud": Horaud方法
                - "andreff": Andreff方法
                - "daniilidis": Daniilidis方法
                
        Returns:
            R_cam2base: 相机到基座的旋转矩阵
            t_cam2base: 相机到基座的平移向量
        """
        print(f"\n使用 {method} 方法求解手眼标定...")
        
        # 准备数据
        R_gripper, t_gripper, R_target, t_target = self.prepare_hand_eye_matrices()
        
        # 选择OpenCV方法
        method_map = {
            "tsai": cv2.CALIB_HAND_EYE_TSAI,
            "park": cv2.CALIB_HAND_EYE_PARK,
            "horaud": cv2.CALIB_HAND_EYE_HORAUD,
            "andreff": cv2.CALIB_HAND_EYE_ANDREFF,
            "daniilidis": cv2.CALIB_HAND_EYE_DANIILIDIS,
        }
        
        cv_method = method_map.get(method, cv2.CALIB_HAND_EYE_TSAI)
        
        # 求解
        R_cam2base, t_cam2base = cv2.calibrateHandEye(
            R_gripper2base=R_gripper,
            t_gripper2base=t_gripper,
            R_target2cam=R_target,
            t_target2cam=t_target,
            method=cv_method
        )
        
        return R_cam2base, t_cam2base
        
    def compute_reprojection_error(
        self,
        R_cam2base: np.ndarray,
        t_cam2base: np.ndarray
    ) -> float:
        """
        计算重投影误差
        
        Args:
            R_cam2base: 相机到基座的旋转矩阵
            t_cam2base: 相机到基座的平移向量
            
        Returns:
            平均误差（米）
        """
        T_cam2base = np.eye(4)
        T_cam2base[:3, :3] = R_cam2base
        T_cam2base[:3, 3:4] = t_cam2base
        
        errors = []
        
        for data in self.calibration_data:
            T_base_to_end = np.array(data["T_base_to_end"])
            T_cam_to_board = np.array(data["T_cam_to_board"])
            
            # 计算预测的标定板位姿
            # T_cam_to_board_pred = T_cam_to_base @ T_base_to_end @ T_end_to_board
            # 但我们不知道T_end_to_board，所以用不同方法
            
            # 使用：T_cam_to_base @ T_base_to_end 应该在同一个相对关系
            T_cam_to_end_pred = T_cam2base @ T_base_to_end
            
            # 位置误差
            pos_error = np.linalg.norm(T_cam_to_end_pred[:3, 3] - T_cam_to_board[:3, 3])
            errors.append(pos_error)
            
        mean_error = np.mean(errors)
        return mean_error
        
    def solve_and_evaluate(self) -> dict:
        """求解并评估多种方法"""
        methods = ["tsai", "park", "horaud", "andreff", "daniilidis"]
        results = {}
        
        for method in methods:
            try:
                R, t = self.solve_hand_eye(method)
                error = self.compute_reprojection_error(R, t)
                
                results[method] = {
                    "R": R.tolist(),
                    "t": t.tolist(),
                    "error": float(error)
                }
                
                print(f"  {method}: 误差 = {error:.4f} m")
                
            except Exception as e:
                print(f"  {method}: 求解失败 - {e}")
                results[method] = {"error": "failed"}
                
        # 选择误差最小的方法
        best_method = min(
            [m for m in methods if "error" in results[m] and isinstance(results[m]["error"], float)],
            key=lambda m: results[m]["error"]
        )
        
        print(f"\n✅ 最佳方法: {best_method}")
        print(f"   平均误差: {results[best_method]['error']:.4f} m")
        
        return results, best_method
        
    def save_calibration_result(
        self,
        R_cam2base: np.ndarray,
        t_cam2base: np.ndarray,
        filename: str = "camera_to_base_calibration.json"
    ):
        """保存标定结果"""
        T_cam2base = np.eye(4)
        T_cam2base[:3, :3] = R_cam2base
        T_cam2base[:3, 3:4] = t_cam2base
        
        # 转换为欧拉角（方便理解）
        euler = Rotation.from_matrix(R_cam2base).as_euler('xyz', degrees=True)
        
        result = {
            "T_cam_to_base": T_cam2base.tolist(),
            "rotation_matrix": R_cam2base.tolist(),
            "translation_vector": t_cam2base.flatten().tolist(),
            "euler_angles_deg": {
                "roll": float(euler[0]),
                "pitch": float(euler[1]),
                "yaw": float(euler[2])
            },
            "translation_m": {
                "x": float(t_cam2base[0][0]),
                "y": float(t_cam2base[1][0]),
                "z": float(t_cam2base[2][0])
            }
        }
        
        output_file = Path("outputs") / filename
        with open(output_file, 'w') as f:
            json.dump(result, f, indent=2)
            
        print(f"\n✅ 标定结果已保存到: {output_file}")
        print("\n标定结果摘要:")
        print(f"  相机相对于机械臂基座的位置:")
        print(f"    X: {result['translation_m']['x']:.3f} m")
        print(f"    Y: {result['translation_m']['y']:.3f} m")
        print(f"    Z: {result['translation_m']['z']:.3f} m")
        print(f"  旋转角度:")
        print(f"    Roll:  {result['euler_angles_deg']['roll']:.1f}°")
        print(f"    Pitch: {result['euler_angles_deg']['pitch']:.1f}°")
        print(f"    Yaw:   {result['euler_angles_deg']['yaw']:.1f}°")


def main():
    """主程序"""
    print("=" * 60)
    print("手眼标定求解")
    print("=" * 60)
    
    solver = HandEyeSolver("outputs/hand_eye_data.json")
    
    try:
        # 加载数据
        solver.load_data()
        
        if len(solver.calibration_data) < 3:
            print("\n❌ 错误：至少需要3组标定数据")
            print("   请先运行 python stage1_test/calib/hand_eye_calibration_opencv.py 采集数据")
            return
            
        # 求解并评估
        results, best_method = solver.solve_and_evaluate()
        
        # 保存最佳结果
        best_result = results[best_method]
        R_cam2base = np.array(best_result["R"])
        t_cam2base = np.array(best_result["t"])
        
        solver.save_calibration_result(R_cam2base, t_cam2base)
        
        print("\n✅ 手眼标定完成！")
        print("   可以使用标定结果进行坐标转换")
        
    except FileNotFoundError:
        print("\n❌ 错误：找不到标定数据文件")
        print("   请先运行 python stage1_test/calib/hand_eye_calibration_opencv.py 采集数据")
    except Exception as e:
        print(f"\n❌ 错误：{e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
