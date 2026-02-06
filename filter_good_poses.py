#!/usr/bin/env python3
"""
过滤手眼标定数据 - 只保留高质量位姿

根据中心距离误差过滤数据，只保留误差 < 50mm 的位姿。
"""

import json
import numpy as np
from pathlib import Path

def filter_calibration_data(
    input_file: str = "outputs/hand_eye_data.json",
    output_file: str = "outputs/hand_eye_data_filtered.json",
    max_error_mm: float = 50.0
):
    """
    过滤标定数据
    
    Args:
        input_file: 输入数据文件
        output_file: 输出过滤后的数据文件
        max_error_mm: 最大允许的中心距离误差（毫米）
    """
    # 从终端输出提取的中心距离误差
    # 实际应该在采集时保存到JSON中，这里手动输入
    center_errors_mm = {
        0: 256.9, 1: 223.0, 2: 151.7, 3: 30.7, 4: 30.4, 5: 30.4,
        6: 39.8, 7: 79.7, 8: 69.5, 9: 30.2, 10: 30.2, 11: 32.8,
        12: 105.8, 13: 175.8, 14: 30.2, 15: 29.5, 16: 27.0, 17: 29.3,
        18: 262.9, 19: 237.3, 20: 34.5, 21: 31.1, 22: 133.8, 23: 34.1
    }
    
    # 加载原始数据
    print("=" * 60)
    print("手眼标定数据过滤")
    print("=" * 60)
    
    with open(input_file, 'r') as f:
        data = json.load(f)
    
    print(f"\n✅ 已加载 {len(data)} 组原始标定数据")
    
    # 过滤数据
    filtered_data = []
    good_indices = []
    bad_indices = []
    
    for i, pose_data in enumerate(data):
        error = center_errors_mm.get(i, 999.0)
        
        if error <= max_error_mm:
            filtered_data.append(pose_data)
            good_indices.append(i)
        else:
            bad_indices.append((i, error))
    
    print(f"\n📊 过滤结果:")
    print(f"  阈值: {max_error_mm}mm")
    print(f"  ✅ 保留: {len(filtered_data)} 组位姿")
    print(f"  ❌ 丢弃: {len(bad_indices)} 组位姿")
    
    if good_indices:
        print(f"\n✅ 保留的位姿编号:")
        print(f"  {good_indices}")
    
    if bad_indices:
        print(f"\n❌ 丢弃的位姿（误差太大）:")
        for idx, err in bad_indices:
            print(f"  位姿 #{idx}: 中心误差 {err:.1f}mm")
    
    # 分析保留数据的分布
    if filtered_data:
        cam_positions = []
        for d in filtered_data:
            T = np.array(d['T_cam_to_board'])
            pos = T[:3, 3]
            cam_positions.append(pos)
        
        cam_positions = np.array(cam_positions)
        
        print(f"\n📍 保留数据的分布:")
        print(f"  相机观测距离范围: [{cam_positions[:,2].min():.3f}, {cam_positions[:,2].max():.3f}] m")
        print(f"  平均距离: {cam_positions[:,2].mean():.3f} m")
        print(f"  X范围: [{cam_positions[:,0].min():.3f}, {cam_positions[:,0].max():.3f}] m")
        print(f"  Y范围: [{cam_positions[:,1].min():.3f}, {cam_positions[:,1].max():.3f}] m")
    
    # 保存过滤后的数据
    if filtered_data:
        with open(output_file, 'w') as f:
            json.dump(filtered_data, f, indent=2)
        
        print(f"\n✅ 已保存过滤后的数据到: {output_file}")
        print(f"\n🚀 下一步: 运行 python hand_eye_solver_filtered.py")
    else:
        print(f"\n❌ 警告：没有符合条件的数据！请降低阈值或重新采集。")
    
    return filtered_data

if __name__ == "__main__":
    filter_calibration_data()
