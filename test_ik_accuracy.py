#!/usr/bin/env python3
"""
SO101 IK精度自动化测试程序

功能：
1. 自动执行N轮测试
2. 记录目标位置、IK计算位置、实际到达位置
3. 生成统计分析报告和可视化图表
4. 分析FK效果和电机执行精度
"""

import numpy as np
import time
import json
from pathlib import Path
from datetime import datetime
import pandas as pd

from lerobot.robots.so101_follower import SO101Follower
from lerobot.robots.so101_follower.config_so101_follower import SO101FollowerConfig
from lerobot.model.kinematics import RobotKinematics


class IKAccuracyTester:
    def __init__(self, port: str = "/dev/ttyACM0"):
        print("=" * 70)
        print("SO101 逆运动学精度自动化测试")
        print("=" * 70)
        
        # 初始化机械臂
        print("\n[1/3] 初始化机械臂...")
        config = SO101FollowerConfig(
            port=port,
            use_degrees=True,
            max_relative_target=None,  # 不限制移动速度
        )
        self.robot = SO101Follower(config)
        self.robot.connect(calibrate=False)  # 使用已有校准
        print("✅ 机械臂已连接")
        
        # 初始化运动学
        print("\n[2/3] 初始化运动学求解器...")
        urdf_path = "/home/zyj/lerobot/SO101/so101_new_calib.urdf"
        self.kinematics = RobotKinematics(
            urdf_path=urdf_path,
            target_frame_name="gripper_frame_link",
            joint_names=["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
        )
        print("✅ 运动学求解器已初始化")
        
        # 测试结果存储
        self.results = []
        
        # 休息位置（用于每次测试前复位）
        self.home_position = np.array([0.20, 0.00, 0.04])
        
        print("\n[3/3] 准备就绪！")
    
    def get_current_pose(self):
        """获取当前机械臂位姿"""
        obs = self.robot.get_observation()
        joint_names = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
        joint_positions = np.array([obs[f"{name}.pos"] for name in joint_names])
        T = self.kinematics.forward_kinematics(joint_positions)
        position = T[:3, 3]
        return joint_positions, T, position
    
    def move_to_position_silent(self, target_xyz, timeout=5):
        """
        静默移动到目标位置（无交互）
        返回：(success, ik_pos, actual_pos, ik_error, actual_error, move_time)
        """
        start_time = time.time()
        
        # 获取当前状态
        current_joints, current_T, current_pos = self.get_current_pose()
        
        # 构造目标位姿
        target_T = current_T.copy()
        target_T[:3, 3] = target_xyz
        
        # 计算IK
        try:
            target_joints = self.kinematics.inverse_kinematics(
                current_joint_pos=current_joints,
                desired_ee_pose=target_T,
                position_weight=1.0,
                orientation_weight=0.0
            )
        except Exception as e:
            return False, None, None, None, None, 0, str(e)
        
        # 验证IK（FK检查）
        verify_T = self.kinematics.forward_kinematics(target_joints)
        ik_pos = verify_T[:3, 3]
        ik_error = np.linalg.norm(ik_pos - target_xyz)
        
        # 发送动作
        try:
            joint_names = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
            action = {f"{name}.pos": target_joints[i] for i, name in enumerate(joint_names)}
            self.robot.send_action(action)
            time.sleep(2.5)  # 等待移动完成
        except Exception as e:
            return False, ik_pos, None, ik_error, None, 0, str(e)
        
        # 读取实际到达位置
        _, _, actual_pos = self.get_current_pose()
        actual_error = np.linalg.norm(actual_pos - target_xyz)
        
        move_time = time.time() - start_time
        
        return True, ik_pos, actual_pos, ik_error, actual_error, move_time, None
    
    def move_to_home(self):
        """移动到休息位置"""
        self.move_to_position_silent(self.home_position, timeout=5)
        time.sleep(1)
    
    def generate_test_positions(self, num_tests=100):
        """
        生成测试位置
        在安全工作空间内随机生成，同时包含一些固定测试点
        """
        positions = []
        
        # 固定测试点（10个）
        fixed_points = [
            [0.20, 0.00, 0.04],  # 休息位置
            [0.22, 0.00, 0.04],  # 中心
            [0.25, 0.00, 0.05],  # 正前方低位
            [0.25, 0.00, 0.03],  # 更低
            [0.20, 0.03, 0.04],  # 左前
            [0.20, -0.03, 0.04], # 右前
            [0.23, 0.04, 0.05],  # 左前上
            [0.23, -0.04, 0.05], # 右前上
            [0.18, 0.00, 0.04],  # 近距离
            [0.27, 0.00, 0.04],  # 远距离
        ]
        
        # 添加固定点
        for point in fixed_points:
            positions.append(np.array(point))
        
        # 随机生成剩余的测试点
        remaining = num_tests - len(fixed_points)
        if remaining > 0:
            # 定义安全工作空间
            x_range = (0.18, 0.35)  # 18-35cm
            y_range = (-0.09, 0.09) # ±5cm
            z_range = (0.01, 0.06)  # 2-6cm
            
            for _ in range(remaining):
                x = np.random.uniform(*x_range)
                y = np.random.uniform(*y_range)
                z = np.random.uniform(*z_range)
                positions.append(np.array([x, y, z]))
        
        # 打乱顺序
        np.random.shuffle(positions)
        
        return positions
    
    def run_batch_test(self, num_tests=100):
        """执行批量测试"""
        print(f"\n{'='*70}")
        print(f"开始批量测试 - 共 {num_tests} 次")
        print(f"{'='*70}\n")
        
        # 生成测试位置
        test_positions = self.generate_test_positions(num_tests)
        
        # 先移动到休息位置
        print("移动到初始休息位置...")
        self.move_to_home()
        print("✅ 准备完成\n")
        
        # 执行测试
        for i, target in enumerate(test_positions):
            print(f"[{i+1}/{num_tests}] 测试目标: ({target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f})", end=" ")
            
            # 执行移动并记录
            success, ik_pos, actual_pos, ik_error, actual_error, move_time, error_msg = \
                self.move_to_position_silent(target)
            
            if success:
                # 记录结果
                result = {
                    'test_id': i + 1,
                    'target_x': target[0],
                    'target_y': target[1],
                    'target_z': target[2],
                    'ik_x': ik_pos[0],
                    'ik_y': ik_pos[1],
                    'ik_z': ik_pos[2],
                    'actual_x': actual_pos[0],
                    'actual_y': actual_pos[1],
                    'actual_z': actual_pos[2],
                    'ik_error_mm': ik_error * 1000,
                    'actual_error_mm': actual_error * 1000,
                    'motor_error_mm': np.linalg.norm(actual_pos - ik_pos) * 1000,
                    'move_time_s': move_time,
                    'success': True,
                    'error_msg': None
                }
                self.results.append(result)
                
                # 简洁输出
                print(f"✓ IK:{ik_error*1000:.1f}mm Act:{actual_error*1000:.1f}mm Mot:{result['motor_error_mm']:.1f}mm")
            else:
                # 记录失败
                result = {
                    'test_id': i + 1,
                    'target_x': target[0],
                    'target_y': target[1],
                    'target_z': target[2],
                    'success': False,
                    'error_msg': error_msg
                }
                self.results.append(result)
                print(f"✗ 失败: {error_msg}")
            
            # 每10次测试后回到休息位置
            if (i + 1) % 10 == 0:
                print(f"  [暂停] 返回休息位置...")
                self.move_to_home()
        
        print(f"\n{'='*70}")
        print("测试完成！")
        print(f"{'='*70}\n")
    
    def analyze_results(self):
        """分析测试结果"""
        print("\n" + "="*70)
        print("测试结果分析")
        print("="*70 + "\n")
        
        # 转换为DataFrame
        df = pd.DataFrame(self.results)
        
        # 只分析成功的测试
        df_success = df[df['success'] == True].copy()
        
        if len(df_success) == 0:
            print("❌ 没有成功的测试！")
            return
        
        # 基本统计
        print("📊 基本统计：")
        print(f"  总测试次数: {len(df)}")
        print(f"  成功次数: {len(df_success)}")
        print(f"  失败次数: {len(df) - len(df_success)}")
        print(f"  成功率: {len(df_success)/len(df)*100:.1f}%\n")
        
        # IK误差统计
        print("📐 IK误差统计（目标 vs IK计算）：")
        print(f"  平均误差: {df_success['ik_error_mm'].mean():.2f} mm")
        print(f"  中位数误差: {df_success['ik_error_mm'].median():.2f} mm")
        print(f"  标准差: {df_success['ik_error_mm'].std():.2f} mm")
        print(f"  最小误差: {df_success['ik_error_mm'].min():.2f} mm")
        print(f"  最大误差: {df_success['ik_error_mm'].max():.2f} mm")
        print(f"  < 10mm: {(df_success['ik_error_mm'] < 10).sum()} ({(df_success['ik_error_mm'] < 10).sum()/len(df_success)*100:.1f}%)")
        print(f"  < 20mm: {(df_success['ik_error_mm'] < 20).sum()} ({(df_success['ik_error_mm'] < 20).sum()/len(df_success)*100:.1f}%)")
        print(f"  < 30mm: {(df_success['ik_error_mm'] < 30).sum()} ({(df_success['ik_error_mm'] < 30).sum()/len(df_success)*100:.1f}%)\n")
        
        # 实际误差统计
        print("🎯 实际误差统计（目标 vs 实际到达）：")
        print(f"  平均误差: {df_success['actual_error_mm'].mean():.2f} mm")
        print(f"  中位数误差: {df_success['actual_error_mm'].median():.2f} mm")
        print(f"  标准差: {df_success['actual_error_mm'].std():.2f} mm")
        print(f"  最小误差: {df_success['actual_error_mm'].min():.2f} mm")
        print(f"  最大误差: {df_success['actual_error_mm'].max():.2f} mm")
        print(f"  < 15mm: {(df_success['actual_error_mm'] < 15).sum()} ({(df_success['actual_error_mm'] < 15).sum()/len(df_success)*100:.1f}%)")
        print(f"  < 25mm: {(df_success['actual_error_mm'] < 25).sum()} ({(df_success['actual_error_mm'] < 25).sum()/len(df_success)*100:.1f}%)")
        print(f"  < 40mm: {(df_success['actual_error_mm'] < 40).sum()} ({(df_success['actual_error_mm'] < 40).sum()/len(df_success)*100:.1f}%)\n")
        
        # 电机执行误差统计
        print("⚙️  电机执行误差统计（IK计算 vs 实际到达）：")
        print(f"  平均误差: {df_success['motor_error_mm'].mean():.2f} mm")
        print(f"  中位数误差: {df_success['motor_error_mm'].median():.2f} mm")
        print(f"  标准差: {df_success['motor_error_mm'].std():.2f} mm")
        print(f"  最小误差: {df_success['motor_error_mm'].min():.2f} mm")
        print(f"  最大误差: {df_success['motor_error_mm'].max():.2f} mm\n")
        
        # 区域分析
        print("📍 工作空间区域分析：")
        
        # X轴分析
        # 按照新的安全工作空间区间进行分段分析
        # 对应区间: x_range = (0.18, 0.35), y_range = (-0.09, 0.09), z_range = (0.01, 0.06)

        print("\n  X轴（距离）分析：")
        # 分 0.18-0.23(近), 0.23-0.28(中), 0.28-0.35(远)
        for x_min, x_max, label in [
            (0.18, 0.23, "近距离"),
            (0.23, 0.28, "中距离"),
            (0.28, 0.35, "远距离")
        ]:
            mask = (df_success['target_x'] >= x_min) & (df_success['target_x'] < x_max)
            subset = df_success[mask]
            if len(subset) > 0:
                print(f"    {label} ({x_min:.2f}-{x_max:.2f}m): n={len(subset)}, "
                      f"IK={subset['ik_error_mm'].mean():.1f}mm, "
                      f"实际={subset['actual_error_mm'].mean():.1f}mm")

        print("\n  Y轴（左右）分析：")
        # 分 -0.09到-0.03(右), -0.03到0.03(中心), 0.03到0.09(左)
        for y_min, y_max, label in [
            (-0.09, -0.03, "右侧"),
            (-0.03,  0.03, "中心"),
            ( 0.03,  0.09, "左侧")
        ]:
            mask = (df_success['target_y'] >= y_min) & (df_success['target_y'] < y_max)
            subset = df_success[mask]
            if len(subset) > 0:
                print(f"    {label} ({y_min:.2f}-{y_max:.2f}m): n={len(subset)}, "
                      f"IK={subset['ik_error_mm'].mean():.1f}mm, "
                      f"实际={subset['actual_error_mm'].mean():.1f}mm")

        print("\n  Z轴（高度）分析：")
        # 分 0.01-0.03(低位), 0.03-0.045(中位), 0.045-0.06(高位)
        for z_min, z_max, label in [
            (0.01, 0.03, "低位"),
            (0.03, 0.045, "中位"),
            (0.045, 0.06, "高位")
        ]:
            mask = (df_success['target_z'] >= z_min) & (df_success['target_z'] < z_max)
            subset = df_success[mask]
            if len(subset) > 0:
                print(f"    {label} ({z_min:.3f}-{z_max:.3f}m): n={len(subset)}, "
                      f"IK={subset['ik_error_mm'].mean():.1f}mm, "
                      f"实际={subset['actual_error_mm'].mean():.1f}mm")

        print("\n" + "="*70)
        # 找出最佳区域
        print("\n🌟 最佳精度区域（实际误差 < 20mm的区域）：")
        best_tests = df_success[df_success['actual_error_mm'] < 20]
        if len(best_tests) > 0:
            print(f"  数量: {len(best_tests)} ({len(best_tests)/len(df_success)*100:.1f}%)")
            print(f"  X范围: {best_tests['target_x'].min():.3f} - {best_tests['target_x'].max():.3f}m")
            print(f"  Y范围: {best_tests['target_y'].min():.3f} - {best_tests['target_y'].max():.3f}m")
            print(f"  Z范围: {best_tests['target_z'].min():.3f} - {best_tests['target_z'].max():.3f}m")
            print(f"  平均误差: {best_tests['actual_error_mm'].mean():.2f}mm")
        
        print("\n" + "="*70)
    
    def save_results(self, output_dir="outputs/ik_accuracy_test"):
        """保存测试结果"""
        output_path = Path(output_dir)
        output_path.mkdir(parents=True, exist_ok=True)
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # 保存原始数据（JSON）
        json_file = output_path / f"test_results_{timestamp}.json"
        with open(json_file, 'w') as f:
            json.dump(self.results, f, indent=2)
        print(f"\n✅ 原始数据已保存: {json_file}")
        
        # 保存表格数据（CSV）
        df = pd.DataFrame(self.results)
        csv_file = output_path / f"test_results_{timestamp}.csv"
        df.to_csv(csv_file, index=False)
        print(f"✅ CSV表格已保存: {csv_file}")
        
        # 保存统计摘要
        summary_file = output_path / f"test_summary_{timestamp}.txt"
        df_success = df[df['success'] == True]
        
        with open(summary_file, 'w') as f:
            f.write("SO101 IK精度测试摘要\n")
            f.write("="*70 + "\n\n")
            f.write(f"测试时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"总测试次数: {len(df)}\n")
            f.write(f"成功次数: {len(df_success)}\n")
            f.write(f"成功率: {len(df_success)/len(df)*100:.1f}%\n\n")
            
            if len(df_success) > 0:
                f.write("IK误差统计:\n")
                f.write(f"  平均: {df_success['ik_error_mm'].mean():.2f} mm\n")
                f.write(f"  中位数: {df_success['ik_error_mm'].median():.2f} mm\n")
                f.write(f"  标准差: {df_success['ik_error_mm'].std():.2f} mm\n")
                f.write(f"  范围: {df_success['ik_error_mm'].min():.2f} - {df_success['ik_error_mm'].max():.2f} mm\n\n")
                
                f.write("实际误差统计:\n")
                f.write(f"  平均: {df_success['actual_error_mm'].mean():.2f} mm\n")
                f.write(f"  中位数: {df_success['actual_error_mm'].median():.2f} mm\n")
                f.write(f"  标准差: {df_success['actual_error_mm'].std():.2f} mm\n")
                f.write(f"  范围: {df_success['actual_error_mm'].min():.2f} - {df_success['actual_error_mm'].max():.2f} mm\n\n")
                
                f.write("电机执行误差统计:\n")
                f.write(f"  平均: {df_success['motor_error_mm'].mean():.2f} mm\n")
                f.write(f"  中位数: {df_success['motor_error_mm'].median():.2f} mm\n")
                f.write(f"  标准差: {df_success['motor_error_mm'].std():.2f} mm\n")
        
        print(f"✅ 统计摘要已保存: {summary_file}")
        
        return json_file, csv_file, summary_file
    
    def cleanup(self):
        """清理资源"""
        print("\n正在断开机械臂连接...")
        self.robot.disconnect()
        print("✅ 已断开连接")


def main():
    """主程序"""
    import sys
    
    # 参数
    port = "/dev/ttyACM0"
    num_tests = 30  # 默认100次测试
    
    # 解析命令行参数
    if len(sys.argv) > 1:
        try:
            num_tests = int(sys.argv[1])
        except:
            print(f"⚠️  无效的测试次数，使用默认值: {num_tests}")
    
    if len(sys.argv) > 2:
        port = sys.argv[2]
    
    tester = None
    
    try:
        # 创建测试器
        tester = IKAccuracyTester(port=port)
        
        # 执行批量测试
        tester.run_batch_test(num_tests=num_tests)
        
        # 分析结果
        tester.analyze_results()
        
        # 保存结果
        tester.save_results()
        
    except KeyboardInterrupt:
        print("\n\n⚠️  用户中断测试（Ctrl+C）")
        if tester and len(tester.results) > 0:
            print("\n保存已完成的测试结果...")
            tester.analyze_results()
            tester.save_results()
    
    except Exception as e:
        print(f"\n❌ 程序出错: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        if tester is not None:
            tester.cleanup()
        
        print("\n程序结束")


if __name__ == "__main__":
    main()
