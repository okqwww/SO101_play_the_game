#!/usr/bin/env python3
"""
手眼标定求解器 - 使用过滤后的数据

专门用于处理过滤后的高质量数据
"""

import json
import numpy as np
import cv2
from pathlib import Path
from typing import List, Tuple
from scipy.spatial.transform import Rotation

class HandEyeSolverFiltered:
    """使用过滤后数据的手眼标定求解器"""
    
    def __init__(self, data_file: str = "outputs/hand_eye_data_filtered.json"):
        """
        初始化求解器
        
        Args:
            data_file: 过滤后的标定数据文件
        """
        self.data_file = data_file
        self.calibration_data = []
        self.load_data()
        
    def load_data(self):
        """加载标定数据"""
        print("=" * 60)
        print("手眼标定求解（使用过滤后的高质量数据）")
        print("=" * 60)
        
        with open(self.data_file, 'r') as f:
            self.calibration_data = json.load(f)
        
        print(f"✅ 已加载 {len(self.calibration_data)} 组高质量标定数据\n")
        
    def prepare_hand_eye_matrices(
        self
    ) -> Tuple[List[np.ndarray], List[np.ndarray], List[np.ndarray], List[np.ndarray]]:
        """
        准备手眼标定所需的矩阵对
        """
        if len(self.calibration_data) < 3:
            raise ValueError(f"至少需要3组数据，当前只有{len(self.calibration_data)}组")
            
        R_gripper2base_list = []
        t_gripper2base_list = []
        R_target2cam_list = []
        t_target2cam_list = []
        
        for data in self.calibration_data:
            # 机械臂末端到基座
            T_base_to_end = np.array(data["T_base_to_end"])
            T_end_to_base = np.linalg.inv(T_base_to_end)
            
            R_gripper2base_list.append(T_end_to_base[:3, :3])
            t_gripper2base_list.append(T_end_to_base[:3, 3:4])
            
            # 标定板到相机
            T_cam_to_board = np.array(data["T_cam_to_board"])
            T_board_to_cam = np.linalg.inv(T_cam_to_board)
            
            R_target2cam_list.append(T_board_to_cam[:3, :3])
            t_target2cam_list.append(T_board_to_cam[:3, 3:4])
            
        return R_gripper2base_list, t_gripper2base_list, R_target2cam_list, t_target2cam_list
        
    def solve_hand_eye(self, method: str = "tsai") -> Tuple[np.ndarray, np.ndarray]:
        """求解手眼标定"""
        print(f"使用 {method} 方法求解手眼标定...")
        
        R_gripper, t_gripper, R_target, t_target = self.prepare_hand_eye_matrices()
        
        method_map = {
            "tsai": cv2.CALIB_HAND_EYE_TSAI,
            "park": cv2.CALIB_HAND_EYE_PARK,
            "horaud": cv2.CALIB_HAND_EYE_HORAUD,
            "andreff": cv2.CALIB_HAND_EYE_ANDREFF,
            "daniilidis": cv2.CALIB_HAND_EYE_DANIILIDIS,
        }
        
        cv_method = method_map.get(method, cv2.CALIB_HAND_EYE_TSAI)
        
        R_cam2base, t_cam2base = cv2.calibrateHandEye(
            R_gripper2base=R_gripper,
            t_gripper2base=t_gripper,
            R_target2cam=R_target,
            t_target2cam=t_target,
            method=cv_method
        )
        
        return R_cam2base, t_cam2base
    
    def compute_reprojection_error_relative(
        self,
        R_cam2base: np.ndarray,
        t_cam2base: np.ndarray
    ) -> float:
        """
        计算重投影误差 - 使用相对变换验证法（更准确）
        """
        T_cam2base = np.eye(4)
        T_cam2base[:3, :3] = R_cam2base
        T_cam2base[:3, 3:4] = t_cam2base
        T_base2cam = np.linalg.inv(T_cam2base)
        
        errors = []
        n = len(self.calibration_data)
        
        # 使用相对变换验证
        for i in range(n - 1):
            T_base_to_end_i = np.array(self.calibration_data[i]["T_base_to_end"])
            T_cam_to_board_i = np.array(self.calibration_data[i]["T_cam_to_board"])
            
            T_base_to_end_j = np.array(self.calibration_data[i+1]["T_base_to_end"])
            T_cam_to_board_j = np.array(self.calibration_data[i+1]["T_cam_to_board"])
            
            # 计算相对变换
            T_end_to_base_i = np.linalg.inv(T_base_to_end_i)
            A_ij = T_end_to_base_i @ T_base_to_end_j
            
            T_board_to_cam_i = np.linalg.inv(T_cam_to_board_i)
            B_ij = T_board_to_cam_i @ T_cam_to_board_j
            
            # 验证一致性
            pred = T_base2cam @ B_ij @ T_cam2base
            
            pos_error = np.linalg.norm(A_ij[:3, 3] - pred[:3, 3])
            errors.append(pos_error)
        
        return np.mean(errors)
        
    def solve_and_evaluate(self) -> dict:
        """求解并评估多种方法"""
        methods = ["tsai", "park", "horaud", "andreff", "daniilidis"]
        results = {}
        
        for method in methods:
            try:
                R, t = self.solve_hand_eye(method)
                error = self.compute_reprojection_error_relative(R, t)
                
                results[method] = {
                    "R": R.tolist(),
                    "t": t.tolist(),
                    "error": float(error)
                }
                
                print(f"  {method}: 误差 = {error:.4f} m")
                
            except Exception as e:
                print(f"  {method}: 求解失败 - {e}")
                results[method] = None
        
        # 找最佳方法
        valid_results = {k: v for k, v in results.items() if v is not None}
        if not valid_results:
            raise RuntimeError("所有方法都失败了")
        
        best_method = min(valid_results.keys(), key=lambda k: valid_results[k]["error"])
        best_result = valid_results[best_method]
        
        print(f"\n✅ 最佳方法: {best_method}")
        print(f"   平均误差: {best_result['error']:.4f} m")
        
        return best_result, best_method
        
    def save_calibration(self, R: np.ndarray, t: np.ndarray, output_file: str):
        """保存标定结果"""
        T_cam2base = np.eye(4)
        T_cam2base[:3, :3] = R
        T_cam2base[:3, 3:4] = t
        
        # 提取欧拉角（处理可能不正交的旋转矩阵）
        try:
            # 尝试正交化旋转矩阵
            U, _, Vt = np.linalg.svd(R)
            R_ortho = U @ Vt
            rot = Rotation.from_matrix(R_ortho)
            euler_deg = rot.as_euler('xyz', degrees=True)
        except Exception as e:
            print(f"⚠️  警告：无法提取欧拉角 - {e}")
            euler_deg = [0, 0, 0]
        
        result = {
            "T_camera_to_base": T_cam2base.tolist(),
            "rotation_matrix": R.tolist(),
            "translation_vector": t.flatten().tolist(),
            "euler_angles_deg": {
                "roll": float(euler_deg[0]),
                "pitch": float(euler_deg[1]),
                "yaw": float(euler_deg[2])
            },
            "num_poses_used": len(self.calibration_data),
            "data_quality": "filtered_high_quality"
        }
        
        with open(output_file, 'w') as f:
            json.dump(result, f, indent=2)
        
        print(f"\n✅ 标定结果已保存到: {output_file}")
        
        # 打印摘要
        print(f"\n标定结果摘要:")
        print(f"  相机相对于机械臂基座的位置:")
        print(f"    X: {t[0,0]:.3f} m")
        print(f"    Y: {t[1,0]:.3f} m")
        print(f"    Z: {t[2,0]:.3f} m")
        print(f"  旋转角度:")
        print(f"    Roll:  {euler_deg[0]:.1f}°")
        print(f"    Pitch: {euler_deg[1]:.1f}°")
        print(f"    Yaw:   {euler_deg[2]:.1f}°")
        
def main():
    solver = HandEyeSolverFiltered("outputs/hand_eye_data_filtered.json")
    
    best_result, best_method = solver.solve_and_evaluate()
    
    R = np.array(best_result["R"])
    t = np.array(best_result["t"])
    
    solver.save_calibration(
        R, t,
        "outputs/camera_to_base_calibration_filtered.json"
    )
    
    print(f"\n✅ 手眼标定完成！（使用过滤后的高质量数据）")
    print(f"   使用位姿数: {len(solver.calibration_data)}")
    print(f"   标定误差: {best_result['error']*1000:.1f} mm")
    
    if best_result['error'] < 0.01:
        print(f"   🎉 标定质量: 优秀！（< 10mm）")
    elif best_result['error'] < 0.05:
        print(f"   ✅ 标定质量: 良好（< 50mm）")
    else:
        print(f"   ⚠️  标定质量: 一般（可能需要重新采集）")

if __name__ == "__main__":
    main()
