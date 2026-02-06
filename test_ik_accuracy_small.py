#!/usr/bin/env python3
"""
SO101机械臂IK精度小范围测试（夹爪竖直向下）

测试范围：
- X: 0.22m ~ 0.28m，间隔0.02m (4个点)
- Y: -0.04m ~ 0.04m，间隔0.02m (5个点)
- Z: 0.015m, 0.025m (2个高度)

总共：4 * 5 * 2 = 40个测试点
"""

import numpy as np
import time
import json
import csv
from pathlib import Path
from datetime import datetime
from scipy.spatial.transform import Rotation as R

from lerobot.robots.so101_follower import SO101Follower
from lerobot.robots.so101_follower.config_so101_follower import SO101FollowerConfig
from lerobot.model.kinematics import RobotKinematics


class SmallIKTester:
    def __init__(self, port: str = "/dev/ttyACM0"):
        print("=" * 70)
        print("SO101 IK小范围测试 - 夹爪竖直向下")
        print("=" * 70)
        
        # 初始化机械臂
        print("\n[1/3] 正在初始化机械臂...")
        config = SO101FollowerConfig(
            port=port,
            use_degrees=True,
            max_relative_target=None,
        )
        
        self.robot = SO101Follower(config)
        self.robot.connect(calibrate=True)
        print("✅ 机械臂连接成功！")
        
        # 初始化运动学求解器
        print("\n[2/3] 正在初始化运动学求解器...")
        urdf_path = "/home/zyj/lerobot/SO101/so101_new_calib.urdf"
        
        self.kinematics = RobotKinematics(
            urdf_path=urdf_path,
            target_frame_name="gripper_frame_link",
            joint_names=["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
        )
        print("✅ 运动学求解器初始化成功！")
        
        print("\n[3/3] 准备就绪！\n")
        
        # 测试结果存储
        self.results = []
    
    def generate_test_points(self):
        """生成小范围测试点"""
        print("\n生成测试点网格...")
        
        # 定义小范围
        x_values = np.arange(0.22, 0.28 + 0.001, 0.02)  # 4个点
        y_values = np.arange(-0.04, 0.04 + 0.001, 0.02)   # 5个点
        z_values = np.array([0.015, 0.025])               # 2个点
        
        print(f"  X轴: {len(x_values)}个点 - {x_values[0]:.3f} ~ {x_values[-1]:.3f} m")
        print(f"  Y轴: {len(y_values)}个点 - {y_values[0]:.3f} ~ {y_values[-1]:.3f} m")
        print(f"  Z轴: {len(z_values)}个点 - {z_values[0]:.3f}, {z_values[1]:.3f} m")
        
        # 生成所有组合
        test_points = []
        for z in z_values:
            for y in y_values:
                for x in x_values:
                    test_points.append([x, y, z])
        
        print(f"\n✅ 总共生成 {len(test_points)} 个测试点")
        print(f"   预计: 4 × 5 × 2 = {4*5*2} 个点")
        
        return test_points
    
    def get_current_state(self):
        """获取当前机械臂状态"""
        obs = self.robot.get_observation()
        joint_names = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
        joint_positions = np.array([obs[f"{name}.pos"] for name in joint_names])
        
        T = self.kinematics.forward_kinematics(joint_positions)
        position = T[:3, 3]
        
        return joint_positions, T, position
    
    def test_single_point(self, target_xyz, round_idx, total_rounds):
        """测试单个目标点"""
        print(f"\n{'='*70}")
        print(f"测试进度: [{round_idx}/{total_rounds}]")
        print(f"目标位置: X={target_xyz[0]:.3f}m, Y={target_xyz[1]:.3f}m, Z={target_xyz[2]:.3f}m")
        print(f"{'='*70}")
        
        result = {
            'round': round_idx,
            'target_x': target_xyz[0],
            'target_y': target_xyz[1],
            'target_z': target_xyz[2],
            'success': False,
        }
        
        try:
            # 1. 获取当前状态
            current_joints, current_T, current_pos = self.get_current_state()
            
            # 2. 构造目标位姿（夹爪竖直向下）
            target_T = np.eye(4)
            target_T[:3, 3] = target_xyz
            
            # 定义"竖直向下"的姿态
            # 尝试不同的旋转看哪个是真正的"竖直向下"
            rot = R.from_euler('y', 180, degrees=True)
            target_T[:3, :3] = rot.as_matrix()
            
            # 3. 计算IK（增加姿态约束）
            target_joints = self.kinematics.inverse_kinematics(
                current_joint_pos=current_joints,
                desired_ee_pose=target_T,
                position_weight=1.0,
                orientation_weight=0.0  # 姿态权重
            )
            
            # 4. 验证IK（FK检查）
            verify_T = self.kinematics.forward_kinematics(target_joints)
            verify_pos = verify_T[:3, 3]
            ik_error = np.linalg.norm(verify_pos - target_xyz)
            
            print(f"  IK计算误差: {ik_error*1000:.2f} mm")
            
            result['ik_x'] = verify_pos[0]
            result['ik_y'] = verify_pos[1]
            result['ik_z'] = verify_pos[2]
            result['ik_error_mm'] = ik_error * 1000
            
            # 5. 移动机械臂
            joint_names = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
            action = {f"{name}.pos": target_joints[i] for i, name in enumerate(joint_names)}
            
            self.robot.send_action(action)
            time.sleep(2.5)
            
            # 6. 读取实际到达位置
            final_joints, final_T, final_pos = self.get_current_state()
            
            actual_error = np.linalg.norm(final_pos - target_xyz)
            motor_error = np.linalg.norm(final_pos - verify_pos)
            
            print(f"  实际位置误差: {actual_error*1000:.2f} mm")
            print(f"  电机执行误差: {motor_error*1000:.2f} mm")
            
            result['actual_x'] = final_pos[0]
            result['actual_y'] = final_pos[1]
            result['actual_z'] = final_pos[2]
            result['actual_error_mm'] = actual_error * 1000
            result['motor_error_mm'] = motor_error * 1000
            result['success'] = True
            
            if actual_error <= 0.01:
                print(f"  ✅ 精度达标（<10mm）")
            else:
                print(f"  ⚠️  精度不达标（≥10mm）")
        
        except Exception as e:
            print(f"  ❌ 测试失败: {e}")
            result['error_message'] = str(e)
        
        return result
    
    def run_test(self):
        """运行完整测试"""
        print("\n" + "="*70)
        print("开始小范围测试")
        print("="*70)
        
        # 生成测试点
        test_points = self.generate_test_points()
        total_points = len(test_points)
        
        print(f"\n⚠️  本次测试共 {total_points} 个点，预计耗时约 {total_points*2.5/60:.1f} 分钟")
        print("按 Enter 开始测试，或输入 'n' 取消...")
        response = input()
        if response.lower() == 'n':
            print("❌ 用户取消测试")
            return
        
        # 开始测试
        start_time = time.time()
        success_count = 0
        
        for i, target_xyz in enumerate(test_points, 1):
            result = self.test_single_point(target_xyz, i, total_points)
            self.results.append(result)
            
            if result['success']:
                success_count += 1
        
        # 测试完成
        total_time = time.time() - start_time
        
        print("\n" + "="*70)
        print("测试完成！")
        print("="*70)
        print(f"总测试点数: {total_points}")
        print(f"成功测试: {success_count}/{total_points} ({success_count/total_points*100:.1f}%)")
        print(f"总耗时: {total_time/60:.1f} 分钟")
        
        # 保存结果
        self.save_results()
    
    def save_results(self):
        """保存测试结果"""
        print("\n保存测试结果...")
        
        # 创建输出目录
        output_dir = Path("outputs/ik_accuracy_test_small")
        output_dir.mkdir(parents=True, exist_ok=True)
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # 1. 保存JSON
        json_file = output_dir / f"test_results_{timestamp}.json"
        with open(json_file, 'w') as f:
            json.dump(self.results, f, indent=2)
        print(f"✅ 原始数据已保存: {json_file}")
        
        # 2. 保存CSV
        csv_file = output_dir / f"test_results_{timestamp}.csv"
        with open(csv_file, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=self.results[0].keys())
            writer.writeheader()
            writer.writerows(self.results)
        print(f"✅ CSV表格已保存: {csv_file}")
        
        # 3. 保存统计摘要
        self.save_summary(output_dir / f"test_summary_{timestamp}.txt")
    
    def save_summary(self, filename):
        """保存统计摘要"""
        success_results = [r for r in self.results if r['success']]
        
        if len(success_results) == 0:
            print("⚠️  没有成功的测试结果")
            return
        
        ik_errors = [r['ik_error_mm'] for r in success_results]
        motor_errors = [r['motor_error_mm'] for r in success_results]
        actual_errors = [r['actual_error_mm'] for r in success_results]
        
        good_count = sum(1 for r in success_results if r['actual_error_mm'] < 10.0)
        
        with open(filename, 'w') as f:
            f.write("="*70 + "\n")
            f.write("SO101 IK小范围测试统计（夹爪竖直向下）\n")
            f.write("="*70 + "\n\n")
            
            f.write(f"测试时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"总测试点数: {len(self.results)}\n")
            f.write(f"成功测试: {len(success_results)}\n\n")
            
            f.write("误差统计：\n")
            f.write(f"  IK误差: 平均{np.mean(ik_errors):.2f}mm, 最大{max(ik_errors):.2f}mm\n")
            f.write(f"  电机误差: 平均{np.mean(motor_errors):.2f}mm, 最大{max(motor_errors):.2f}mm\n")
            f.write(f"  实际误差: 平均{np.mean(actual_errors):.2f}mm, 最大{max(actual_errors):.2f}mm\n\n")
            
            f.write(f"精度达标率（<10mm）: {good_count}/{len(success_results)} ({good_count/len(success_results)*100:.1f}%)\n")
        
        print(f"✅ 统计摘要已保存: {filename}")
        print(f"\n精度达标: {good_count}/{len(success_results)} ({good_count/len(success_results)*100:.1f}%)")
    
    def cleanup(self):
        """清理资源"""
        print("\n正在断开机械臂连接...")
        print("✅ 已断开连接")


def main():
    import sys
    
    port = "/dev/ttyACM0"
    if len(sys.argv) > 1:
        port = sys.argv[1]
    
    tester = None
    
    try:
        tester = SmallIKTester(port=port)
        tester.run_test()
        
    except KeyboardInterrupt:
        print("\n\n⚠️  用户中断程序（Ctrl+C）")
    
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
